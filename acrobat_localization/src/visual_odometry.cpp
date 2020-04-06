#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>

#include <acrobat_common/constants/reference_frames.hpp>
#include <acrobat_common/constants/topics.hpp>
#include <acrobat_common/lie_groups/lie_group_conversions.hpp>
#include <acrobat_common/lie_groups/lie_groups.hpp>
#include <acrobat_localization/visual_odometry.hpp>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;

using acrobat::common::constants::camera_topic;
using acrobat::common::constants::imu_topic;

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace acrobat::localization {

VisualOdometry::VisualOdometry(const rclcpp::NodeOptions& options) : Node("acrobat_vio", options) {
    // Variable Topics
    declare_parameter<std::string>("camera_topic", "/acrobat/camera");
    declare_parameter<std::string>("imu_topic", "/acrobat/imu");
    declare_parameter<std::string>("ground_truth_pose_topic", "/acrobat/gt_pose");
    declare_parameter<std::string>("ground_truth_transform_topic", "/acrobat/gt_transform");

    // Toggles
    declare_parameter<bool>("display", false);

    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    RCLCPP_INFO(get_logger(), "Initializing VIO modules");
    // Create modules and launch threads
    if (parameter_client->get_parameter<bool>("display")) {
        visualizer_ = Visualizer::create(get_logger());
    }
    imu_propagator_ = ImuPropagator::create(get_logger());
    frontend_       = Frontend::create<cv::ORB>(get_logger(), visualizer_);
    backend_        = Backend::create(get_logger());

    RCLCPP_INFO(get_logger(), "Launching frontend and backend threads.");
    frontend_thread_ = std::thread(std::bind(&Frontend::run, frontend_));
    backend_thread_  = std::thread(std::bind(&Backend::run, backend_));

    // Build pose message
    pose_msg_stamped_.child_frame_id  = rf::acrobat_body;
    pose_msg_stamped_.header.frame_id = rf::world;
    tf_broadcaster_                   = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // TODO: add velocity publish

    // Subscribe to sensor topics
    imu_subscription_ =
        create_subscription<Imu>(parameter_client->get_parameter<std::string>("imu_topic"),
                                 10,
                                 std::bind(&VisualOdometry::imu_callback, this, _1));
    image_subscription_ =
        create_subscription<Image>(parameter_client->get_parameter<std::string>("camera_topic"),
                                   1,
                                   std::bind(&VisualOdometry::image_callback, this, _1));

    // Subscribe to ground truth topics
    ground_truth_pose_subscription_ = create_subscription<PoseStamped>(
        parameter_client->get_parameter<std::string>("ground_truth_pose_topic"),
        10,
        std::bind(&VisualOdometry::ground_truth_pose_callback, this, _1));
    ground_truth_transform_subscription_ = create_subscription<TransformStamped>(
        parameter_client->get_parameter<std::string>("ground_truth_transform_topic"),
        10,
        std::bind(&VisualOdometry::ground_truth_transform_callback, this, _1));
}

VisualOdometry::~VisualOdometry() {
    RCLCPP_INFO(get_logger(), "Finishing VIO.");

    backend_->stop();
    frontend_->stop();
    backend_thread_.join();
    frontend_thread_.join();

    RCLCPP_INFO(get_logger(), "VIO finished execution.");
}

void VisualOdometry::imu_callback(const Imu::SharedPtr imu_msg) {
    lie_groups::SE3::Tangent imu_sample;
    imu_sample(0) = imu_msg->linear_acceleration.x;
    imu_sample(1) = imu_msg->linear_acceleration.y;
    imu_sample(2) = imu_msg->linear_acceleration.z;

    imu_sample(3) = imu_msg->angular_velocity.x;
    imu_sample(4) = imu_msg->angular_velocity.y;
    imu_sample(5) = imu_msg->angular_velocity.z;
}

void VisualOdometry::image_callback(const Image::SharedPtr image_msg) {
    Frame::SharedPtr frame =
        std::make_shared<Frame>(cv_bridge::toCvShare(image_msg, image_msg->encoding));
    frontend_->add_image(frame);
}

void VisualOdometry::ground_truth_pose_callback(const PoseStamped::SharedPtr gt_msg) {
    // geometry_msgs::msg::TransformStamped pose_msg;
    // pose_msg.child_frame_id = rf::ground_truth;
    // pose_msg.header         = gt_msg->header;

    // RCLCPP_INFO(get_logger(), "got ground_truth");

    // pose_msg.transform.rotation      = gt_msg->pose.orientation;
    // pose_msg.transform.translation.x = gt_msg->pose.position.x;
    // pose_msg.transform.translation.y = gt_msg->pose.position.y;
    // pose_msg.transform.translation.z = gt_msg->pose.position.z;
    // tf_broadcaster_->sendTransform(pose_msg);
}

void VisualOdometry::ground_truth_transform_callback(const TransformStamped::SharedPtr gt_msg) {
    // geometry_msgs::msg::TransformStamped pose_msg = *gt_msg;
    // pose_msg.child_frame_id                       = rf::ground_truth;
    // pose_msg.header.frame_id                      = rf::world;

    // RCLCPP_INFO(get_logger(), "got ground_truth");

    // tf_broadcaster_->sendTransform(pose_msg);
}

} // namespace acrobat::localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::localization::VisualOdometry)
