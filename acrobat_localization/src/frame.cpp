#include <acrobat_localization/frame.hpp>

namespace acrobat::localization {

Frame::Frame(const cv_bridge::CvImageConstPtr& image)
    : stamp_(image->header.stamp), image_(image) {}

const rclcpp::Time&              Frame::stamp() const noexcept { return stamp_; }
const cv::Mat&                   Frame::image() const noexcept { return image_->image; }
const cv::Mat&                   Frame::descriptors() const noexcept { return descriptors_; }
const std::vector<cv::KeyPoint>& Frame::keypoints() const noexcept { return keypoints_; }

void Frame::extract_features(const cv::Ptr<cv::Feature2D>& detector) {
    cv::Mat mask;
    detector->detectAndCompute(image_->image, mask, keypoints_, descriptors_);
}

void Frame::release_image() { image_.reset(); }

} // namespace acrobat::localization