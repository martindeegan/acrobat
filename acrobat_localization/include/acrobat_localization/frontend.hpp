#pragma once

#include <atomic>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <unordered_set>

#include <opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <acrobat_localization/frame.hpp>
#include <acrobat_localization/map_point.hpp>
#include <acrobat_localization/visualizer.hpp>

#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/linear/NoiseModel.h>

namespace acrobat::localization {

enum class TrackingState { Uninitialized, Tracking, LostTracking };

class Frontend {
  public:
    using SharedPtr = std::shared_ptr<Frontend>;

    using NoiseModel  = gtsam::noiseModel::Isotropic;
    using Calibration = gtsam::Cal3Fisheye;

    Frontend(rclcpp::Logger         logger,
             Visualizer::SharedPtr  visualizer,
             cv::Ptr<cv::Feature2D> detector);
    virtual ~Frontend();

    template<class DetectorType>
    static SharedPtr create(rclcpp::Logger        logger,
                            Visualizer::SharedPtr visualizer,
                            cv::Ptr<DetectorType> detector = DetectorType::create());

    virtual void run();
    virtual void stop() noexcept;

    virtual void add_image(Frame::SharedPtr& frame);
    using PosePtr = std::shared_ptr<gtsam::Pose3>;
    virtual void add_prior(PosePtr& pose);

  private:
    // Combines all map point descriptors into a single descriptor array
    void construct_descriptors();
    void check_candidates();
    void clean_candidates();

    std::atomic_bool running_;
    TrackingState    state_;

    std::vector<PosePtr> pose_priors_;
    std::queue<PosePtr>  new_pose_priors_;
    std::mutex           new_pose_priors_mutex_;

    std::queue<Frame::SharedPtr> new_frames_;
    std::mutex                   new_frames_mutex_;

    std::vector<Frame::SharedPtr> frames_;
    std::shared_mutex             frames_mutex_;

    // Candidate points are the points which have not had enough observations to deproject them.
    std::unordered_set<MapPoint::SharedPtr> candidate_points_;

    // Tracking points are the points with enough observations at different viewing angles to have a
    // valid position estimated
    std::unordered_set<MapPoint::SharedPtr> tracking_points_;

    // index_map maps the index in the descriptors_ back to the map point object
    std::unordered_map<size_t, MapPoint::SharedPtr> map_point_index_map_;

    // Combined descriptors from all tracked map points
    cv::Mat descriptors_;

    rclcpp::Logger logger_;

    cv::Ptr<cv::Feature2D>         detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    Visualizer::SharedPtr          visualizer_;

    Calibration::shared_ptr                 camera_calibration_;
    NoiseModel::shared_ptr                  meas_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
};

template<class DetectorType>
Frontend::SharedPtr Frontend::create(rclcpp::Logger        logger,
                                     Visualizer::SharedPtr visualizer,
                                     cv::Ptr<DetectorType> detector) {
    Frontend::SharedPtr frontend = std::make_shared<Frontend>(logger, visualizer, detector);
    return frontend;
}

} // namespace acrobat::localization