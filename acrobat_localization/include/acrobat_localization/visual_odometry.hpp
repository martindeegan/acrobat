#pragma once

#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace acrobat::localization {

class VisualOdometry : public rclcpp::Node {
  public:
    VisualOdometry(const rclcpp::NodeOptions& options);
    ~VisualOdometry();

  private:
    void run_frontend();
    void run_backend();

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg);

    std::atomic_bool running_;
    std::thread      frontend_thread_;
    std::thread      backend_thread_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

} // namespace acrobat::localization