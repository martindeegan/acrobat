#pragma once

#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>

namespace acrobat::localization {

class Frame {
  public:
    using SharedPtr = std::shared_ptr<Frame>;

    Frame() = delete;

    ///
    /// Construct a frame from a cv_bridge shared image
    ///
    explicit Frame(const cv_bridge::CvImageConstPtr& image);

    ///
    /// Get timestamp of the frame as provided by the image message
    ///
    const rclcpp::Time& stamp() const noexcept;

    ///
    /// Get reference to the cv::Mat image within the shared image
    ///
    const cv::Mat& image() const noexcept;

    ///
    /// Get a const reference to the keypoint descriptors
    ///
    const cv::Mat& descriptors() const noexcept;

    ///
    /// Get a const reference to the vector of keypoints
    ///
    const std::vector<cv::KeyPoint>& keypoints() const noexcept;

    ///
    /// Extract features from shared image using the given detector.
    ///
    void extract_features(const cv::Ptr<cv::Feature2D>& detector);

    ///
    /// Releases image_ shared pointer to free up space
    ///
    void release_image();

  private:
    rclcpp::Time               stamp_;
    cv_bridge::CvImageConstPtr image_;
    cv::Mat                    descriptors_;
    std::vector<cv::KeyPoint>  keypoints_;
};

} // namespace acrobat::localization