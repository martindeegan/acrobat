#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace acrobat::image_viewer {
class ImageViewer : public rclcpp::Node {
  public:
    explicit ImageViewer(const rclcpp::NodeOptions& options);
    ~ImageViewer();

  private:
    const std::string                                        window_name_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    void    receiveImage(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat image_;
};

} // namespace acrobat::image_viewer
