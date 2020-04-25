#include "acrobat_localization/visual_odometry.hpp"
#include <acrobat_localization/anms/anms.h>
#include <acrobat_localization/frame.hpp>
#include <numeric>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>

namespace acrobat::localization {

Frame::Frame(const cv_bridge::CvImageConstPtr& image, size_t frame_id)
    : frame_id_(frame_id), stamp_(image->header.stamp), image_(image) {}

const rclcpp::Time&              Frame::stamp() const noexcept { return stamp_; }
const cv::Mat&                   Frame::image() const noexcept { return image_->image; }
const cv::Mat&                   Frame::descriptors() const noexcept { return descriptors_; }
const std::vector<cv::KeyPoint>& Frame::keypoints() const noexcept { return keypoints_; }

void Frame::extract_features(const cv::Ptr<cv::Feature2D>& detector) {
    // TODO create new cv::Feature2D class
    cv::FAST(image_->image, keypoints_, 5, true);

    auto comp = [](const auto& a, const auto& b) { return a.response > b.response; };
    std::sort(keypoints_.begin(), keypoints_.end(), comp);

    size_t num_retaining_points = 250;
    keypoints_ =
        anms::Ssc(keypoints_, num_retaining_points, 0.1, image_->image.cols, image_->image.rows);
    detector->compute(image_->image, keypoints_, descriptors_);
}

void Frame::release_image() { image_.reset(); }

} // namespace acrobat::localization