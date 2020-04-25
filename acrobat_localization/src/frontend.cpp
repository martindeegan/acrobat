#include "acrobat_localization/map_point.hpp"
#include "acrobat_localization/visual_odometry.hpp"
#include <acrobat_common/time/rate.hpp>
#include <acrobat_localization/frame.hpp>
#include <acrobat_localization/frontend.hpp>
#include <algorithm>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <shared_mutex>

using namespace std::chrono_literals;
using namespace gtsam;

namespace acrobat::localization {

Frontend::Frontend(rclcpp::Logger         logger,
                   Visualizer::SharedPtr  visualizer,
                   cv::Ptr<cv::Feature2D> detector)
    : running_(true),
      state_(TrackingState::Uninitialized),
      logger_(logger),
      detector_(detector),
      matcher_(
          cv::DescriptorMatcher::create(cv::DescriptorMatcher::MatcherType::BRUTEFORCE_HAMMING)),
      visualizer_(visualizer),
      camera_calibration_(new Calibration(278.66,
                                          278.48,
                                          0.0,
                                          319.75,
                                          241.96,
                                          -0.013721808247486035,
                                          0.020727425669427896,
                                          -0.012786476702685545,
                                          0.0025242267320687625)),
      meas_noise_(NoiseModel::Sigma(2, 1.0)),
      prior_noise_(noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished())) {}

Frontend::~Frontend() {}

void Frontend::run() {
    gtsam::Values initial_estimate;

    time::Rate loop_rate(150);
    while (rclcpp::ok() && running_) {
        // Add frames

        std::shared_ptr<gtsam::Pose3> new_pose_prior;
        Frame::SharedPtr              new_frame;
        // Get pose and pose prior
        {
            std::scoped_lock lock{new_pose_priors_mutex_, new_frames_mutex_};
            if (!new_frames_.empty() && !new_pose_priors_.empty()) {
                new_frame = new_frames_.front();
                new_frames_.pop();
                // Get latest pose prior
                while (!new_pose_priors_.empty()) {
                    new_pose_prior = new_pose_priors_.front();
                    new_pose_priors_.pop();
                }
            }
        }

        if (new_frame) {
            static size_t pose_id = 1;
            new_frame->frame_id_  = pose_id;
            pose_id++;

            StaticGetters::get_factor_graph()->emplace_shared<PriorFactor<Pose3>>(
                Symbol('x', new_frame->frame_id_), *new_pose_prior, prior_noise_);
            initial_estimate.insert(Symbol('x', new_frame->frame_id_), *new_pose_prior);

            frames_.push_back(new_frame);

            construct_descriptors();

            RCLCPP_INFO(logger_,
                        "Frame Dims: %i x %i\tType: %i",
                        new_frame->descriptors().rows,
                        new_frame->descriptors().cols,
                        new_frame->descriptors().type());

            RCLCPP_INFO(logger_,
                        "Constr Dims: %i x %i\tType: %i",
                        descriptors_.rows,
                        descriptors_.cols,
                        descriptors_.type());

            cv::BFMatcher           matcher(cv::NORM_HAMMING);
            std::vector<cv::DMatch> matches;
            matcher.match(new_frame->descriptors(), descriptors_, matches);
            // Sort based on best matches
            const auto comp = [](const auto& match_a, const auto& match_b) {
                return match_a.distance < match_b.distance;
            };

            float min_distance = std::numeric_limits<float>::infinity();
            for (const auto& match : matches) {
                if (match.distance < min_distance) { min_distance = match.distance; }
            }

            float thresh = 2.0 * min_distance;

            const auto test_thresh = [&](const auto& match) { return match.distance <= thresh; };
            const auto part_it     = std::partition(matches.begin(), matches.end(), test_thresh);

            size_t num_valid = std::distance(matches.begin(), part_it);
            RCLCPP_INFO(logger_, "%i valid matches", num_valid);

            frames_.push_back(new_frame);

            if (!matches.empty()) {
                // Add observations for valid
                const auto insertion = [&](const auto& match) {
                    const size_t combined_index = match.trainIdx;
                    map_point_index_map_[combined_index]->add_observation(new_frame,
                                                                          match.queryIdx);
                };
                std::for_each(matches.begin(), part_it, insertion);

                const auto map_point_creation = [&](const auto& match) {
                    candidate_points_.emplace(new MapPoint(new_frame->stamp(),
                                                           new_frame,
                                                           match.queryIdx,
                                                           meas_noise_,
                                                           camera_calibration_));
                };
                std::for_each(part_it, matches.end(), map_point_creation);
            } else {
                // If this is the first frame or there are no map points, then
                // turn all features
                // into candidates
                const size_t num_keypoints = new_frame->keypoints().size();
                for (size_t i = 0; i < num_keypoints; i++) {
                    candidate_points_.emplace(new MapPoint(
                        new_frame->stamp(), new_frame, i, meas_noise_, camera_calibration_));
                }
            }

            std::vector<bool> valid(new_frame->keypoints().size());
            std::fill(valid.begin(), valid.begin() + num_valid, true);
            if (visualizer_) { visualizer_->display_frame(new_frame, matches, valid); }
            new_frame->release_image();

            check_candidates();
            clean_candidates();
        }

        LevenbergMarquardtOptimizer optimizer(*StaticGetters::get_factor_graph(), initial_estimate);

        Values result      = optimizer.optimize();
        double total_error = StaticGetters::get_factor_graph()->error(result);

        for (auto& frame : frames_) {
            frame->pose_ = result.at(Symbol('x', frame->frame_id_)).cast<Pose3>();
        }
        visualizer_->display_poses(new_frame);

        // Reinitialize the estimate with the previous results
        initial_estimate = result;

        loop_rate.sleep();
    }
} // namespace acrobat::localization

void Frontend::stop() noexcept { running_ = false; }

void Frontend::add_image(Frame::SharedPtr& frame) {
    frame->extract_features(detector_);
    {
        std::lock_guard<std::mutex> lock(new_frames_mutex_);
        new_frames_.push(frame);
    }
}

void Frontend::add_prior(std::shared_ptr<gtsam::Pose3>& pose) {
    std::lock_guard<std::mutex> lock(new_pose_priors_mutex_);
    new_pose_priors_.push(pose);
}

void Frontend::construct_descriptors() {
    RCLCPP_INFO(logger_, "Num tracking features: %i", tracking_points_.size());
    RCLCPP_INFO(logger_, "Num candidate features: %i", candidate_points_.size());

    const size_t     num_points         = tracking_points_.size() + candidate_points_.size();
    constexpr size_t ORB_feature_length = 32;
    descriptors_                        = cv::Mat(num_points, ORB_feature_length, 0);
    // Create a big list of points
    // Tracking points are points that have enough observations to deproject
    // Candidate points are newly observed or old and untracked (waiting for a match)
    std::vector<MapPoint::SharedPtr> map_points_(tracking_points_.begin(), tracking_points_.end());
    map_points_.insert(map_points_.end(), candidate_points_.begin(), candidate_points_.end());

    RCLCPP_INFO(logger_, "Combining map points");

    size_t point_idx = 0;
    for (const auto& map_point : map_points_) {
        cv::Mat row = descriptors_.row(point_idx);
        if (map_point->need_recompute_descriptor) {
            // TODO find out how to average ORB descriptors
            for (std::pair<Frame::SharedPtr, size_t> obs : map_point->observations_) {
                row = obs.first->descriptors().row(obs.second);
            }
            // col = col / static_cast<double>(map_point->observations_.size());
        } else {
            map_point->descriptor_.copyTo(row);
        }

        // Make a two way map between points and indices
        map_point_index_map_[point_idx] = map_point;

        // Make map point descriptor point to col
        map_point->descriptor_ = row;
        point_idx++;
    }
    RCLCPP_INFO(logger_, "Combined map points");
}

void Frontend::check_candidates() {
    if (frames_.empty()) return;
    const size_t latest_frame_id = frames_.back()->frame_id_;
    for (auto& candidate : candidate_points_) {
        // Cull if it hasne't been seen in the last M frames
        constexpr size_t freq_thresh = 3;
        RCLCPP_INFO(logger_, "%i", latest_frame_id - candidate->last_frame_id_);
        if (latest_frame_id - candidate->last_frame_id_ > freq_thresh) {
            candidate->mark_erase = true;
            continue;
        }

        // Cull if it was created more than N frames ago
        constexpr size_t start_thresh = 20;
        if (latest_frame_id - candidate->first_frame_id_ > start_thresh) {
            candidate->mark_erase = true;
            continue;
        }

        if (candidate->mark_deproject) {
            tracking_points_.insert(candidate);
            candidate_points_.erase(candidate);
            // candidate->add_to_graph();
        }
    }
}

void Frontend::clean_candidates() {
    RCLCPP_INFO(logger_, "Erasing candidates");
    // Remove candidates that are marked for erasure
    size_t num_candidates = candidate_points_.size();
    for (auto& candidate : candidate_points_) {
        if (candidate->mark_erase) { candidate_points_.erase(candidate); }
    }
    size_t diff = num_candidates - candidate_points_.size();
    RCLCPP_INFO(logger_, "Erased %i candidates", num_candidates);
}

} // namespace acrobat::localization
