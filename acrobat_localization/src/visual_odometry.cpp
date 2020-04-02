#include <chrono>

#include <acrobat_common/constants/topics.hpp>
#include <acrobat_localization/visual_odometry.hpp>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;

using acrobat::common::constants::camera_topic;
using acrobat::common::constants::imu_topic;

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace acrobat::localization {

VisualOdometry::VisualOdometry(const rclcpp::NodeOptions& options)
    : Node("acrobat_vio", options), running_(true) {
    imu_subscription_ =
        create_subscription<Imu>(imu_topic, 10, std::bind(&VisualOdometry::imu_callback, this, _1));
    image_subscription_ = create_subscription<Image>(
        camera_topic, 1, std::bind(&VisualOdometry::image_callback, this, _1));

    // Launch both threads
    RCLCPP_INFO(get_logger(), "Launching frontend and backend threads.");
    frontend_thread_ = std::thread(std::bind(&VisualOdometry::run_frontend, this));
    backend_thread_  = std::thread(std::bind(&VisualOdometry::run_backend, this));
}

VisualOdometry::~VisualOdometry() {
    RCLCPP_INFO(get_logger(), "Finishing VIO.");
    running_ = false;
    frontend_thread_.join();
    backend_thread_.join();
}

void VisualOdometry::run_frontend() {
    while (running_) {
        std::this_thread::sleep_for(1s);
    }
}

void VisualOdometry::run_backend() {
    while (running_) {
        std::this_thread::sleep_for(1s);
    }
}

void VisualOdometry::imu_callback(const Imu::SharedPtr imu_msg) {
}

void VisualOdometry::image_callback(const Image::SharedPtr image_msg) {
}

} // namespace acrobat::localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::localization::VisualOdometry)
