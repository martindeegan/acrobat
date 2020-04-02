#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <acrobat_common/constants/reference_frames.hpp>
#include <acrobat_common/constants/topics.hpp>
#include <acrobat_common/lie_groups/lie_group_conversions.hpp>
#include <acrobat_common/lie_groups/lie_groups.hpp>
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
    imu_subscription_ = create_subscription<Imu>(
        "/dvs/imu", 10, std::bind(&VisualOdometry::imu_callback, this, _1));
    image_subscription_ = create_subscription<Image>(
        "/dvs/image_raw", 1, std::bind(&VisualOdometry::image_callback, this, _1));

    orb_detector_ = cv::ORB::create(1000);
    cv::namedWindow(window_name);

    pose_msg_stamped_.child_frame_id  = rf::acrobat_body;
    pose_msg_stamped_.header.frame_id = rf::world;
    publish_timer_ = create_wall_timer(10ms, std::bind(&VisualOdometry::publish_pose, this));

    tf_broadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(tf2_ros::TransformBroadcaster{this});

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

void VisualOdometry::publish_pose() {
    // Publish identity pose for now
    using Tangent  = lie_groups::SE3::Tangent;
    Tangent change = Tangent::Zero();
    change(5)      = 0.01;
    change(0)      = 0.005;
    robot_pose_ *= lie_groups::SE3::exp(change);
    geometry_msgs::msg::Transform pose_msg;
    lie_group_conversions::convert(robot_pose_, pose_msg);

    pose_msg_stamped_.transform    = pose_msg;
    pose_msg_stamped_.header.stamp = rclcpp::Time();
    tf_broadcaster_->sendTransform(pose_msg_stamped_);
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
    auto shared_img = cv_bridge::toCvShare(image_msg, image_msg->encoding);

    cv::Mat                   mask;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;
    orb_detector_->detectAndCompute(shared_img->image, mask, keypoints, descriptors);

    cv::Mat image_with_keypoints;
    cv::cvtColor(shared_img->image, image_with_keypoints, cv::COLOR_GRAY2RGB);
    for (const auto& kp : keypoints) {
        cv::circle(image_with_keypoints, kp.pt, 1, {0.0, 255.0, 0.0}, 1);
    }
    cv::imshow(window_name, image_with_keypoints);
    cv::waitKey(1);
}

} // namespace acrobat::localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::localization::VisualOdometry)
