#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/core.hpp>

class ImageViewer : public rclcpp::Node {
  public:
    explicit ImageViewer(const rclcpp::NodeOptions& options);

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    void    receiveImage(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat image_;
};