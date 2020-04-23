#include <chrono>
#include <functional>

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

namespace acrobat::localization {

VisualOdometry::VisualOdometry(const rclcpp::NodeOptions& options) : Node("acrobat_vio", options) {
    // Variable Topics
    declare_parameter<std::string>("camera_topic", "/acrobat/camera");
    declare_parameter<std::string>("imu_topic", "/acrobat/imu");
    declare_parameter<std::string>("ground_truth_pose_topic", "/acrobat/gt_pose");
    declare_parameter<std::string>("ground_truth_transform_topic", "/acrobat/gt_transform");

    declare_parameter<int>("ORB_nfeatures", 500);
    declare_parameter<float>("ORB_scalefactor", 1.2);
    declare_parameter<int>("ORB_nlevels", 8);
    declare_parameter<int>("ORB_edgethresh", 31);
    declare_parameter<int>("ORB_firstlevel", 0);
    declare_parameter<int>("ORB_patchsize", 31);
    declare_parameter<int>("ORB_fastthresh", 20);

    // Toggles
    declare_parameter<bool>("display", false);

    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    RCLCPP_INFO(get_logger(), "Initializing VIO modules");
    // Create modules and launch threads
    if (parameter_client->get_parameter<bool>("display")) {
        visualizer_ = Visualizer::create(get_logger());
    }
    imu_propagator_ = ImuPropagator::create(get_logger());
    auto feature_detector =
        cv::ORB::create(parameter_client->get_parameter<int>("ORB_nfeatures"),
                        parameter_client->get_parameter<float>("ORB_scalefactor"),
                        parameter_client->get_parameter<int>("ORB_nlevels"),
                        parameter_client->get_parameter<int>("ORB_edgethresh"),
                        parameter_client->get_parameter<int>("ORB_firstlevel"),
                        2,
                        cv::ORB::ScoreType::HARRIS_SCORE,
                        parameter_client->get_parameter<int>("ORB_patchsize"),
                        parameter_client->get_parameter<int>("ORB_fastthresh"));
    frontend_ = Frontend::create<cv::ORB>(get_logger(), visualizer_, feature_detector);
    backend_  = Backend::create(get_logger());

    RCLCPP_INFO(get_logger(), "Launching frontend and backend threads.");
    frontend_thread_ = std::thread(std::bind(&Frontend::run, frontend_));
    backend_thread_  = std::thread(std::bind(&Backend::run, backend_));

    // Build pose message
    pose_msg_stamped_.child_frame_id  = rf::acrobat_body;
    pose_msg_stamped_.header.frame_id = rf::world;
    tf_broadcaster_                   = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    std::this_thread::sleep_for(1s);
    tf2::Transform identity;
    identity.setIdentity();
    geometry_msgs::msg::TransformStamped gt_to_world;
    tf2::convert(identity, gt_to_world.transform);
    gt_to_world.header.frame_id = rf::world;
    gt_to_world.child_frame_id  = rf::ground_truth;
    gt_to_world.header.stamp    = rclcpp::Time();
    tf_broadcaster_->sendTransform(gt_to_world);

    geometry_msgs::msg::TransformStamped imu_to_gt_msg;
    tf2::convert(identity, imu_to_gt_msg.transform);
    imu_to_gt_msg.header.frame_id = rf::ground_truth;
    imu_to_gt_msg.child_frame_id  = rf::ground_truth_imu;
    imu_to_gt_msg.header.stamp    = rclcpp::Time();
    tf_broadcaster_->sendTransform(imu_to_gt_msg);

    tf2::Matrix3x3 cam_to_imu_rot;
    cam_to_imu_rot[0][0] = -0.01182306;
    cam_to_imu_rot[0][1] = 0.01155299;
    cam_to_imu_rot[0][2] = 0.99986336;
    cam_to_imu_rot[1][0] = -0.99987014;
    cam_to_imu_rot[1][1] = 0.01081377;
    cam_to_imu_rot[1][2] = -0.01194809;
    cam_to_imu_rot[2][0] = -0.01095033;
    cam_to_imu_rot[2][1] = -0.99987479;
    cam_to_imu_rot[2][2] = 0.01142364;
    tf2::Vector3                         cam_to_imu_trans{-0.00029028, -0.05790695, -0.0001919};
    tf2::Transform                       cam_to_imu(cam_to_imu_rot, cam_to_imu_trans);
    geometry_msgs::msg::TransformStamped cam_to_imu_msg;
    tf2::convert(cam_to_imu.inverse(), cam_to_imu_msg.transform);

    cam_to_imu_msg.header.stamp    = rclcpp::Time();
    cam_to_imu_msg.header.frame_id = rf::ground_truth_imu;
    cam_to_imu_msg.child_frame_id  = rf::ground_truth_camera;
    tf_broadcaster_->sendTransform(cam_to_imu_msg);

    // Subscribe to sensor topics
    imu_subscription_ = create_subscription<Imu>(
        parameter_client->get_parameter<std::string>("imu_topic"),
        10,
        std::bind(&VisualOdometry::imu_callback, this, std::placeholders::_1));
    image_subscription_ = create_subscription<Image>(
        parameter_client->get_parameter<std::string>("camera_topic"),
        1,
        std::bind(&VisualOdometry::image_callback, this, std::placeholders::_1));

    // Subscribe to ground truth topics
    ground_truth_pose_subscription_ = create_subscription<PoseStamped>(
        parameter_client->get_parameter<std::string>("ground_truth_pose_topic"),
        10,
        std::bind(&VisualOdometry::ground_truth_pose_callback, this, std::placeholders::_1));
    ground_truth_transform_subscription_ = create_subscription<TransformStamped>(
        parameter_client->get_parameter<std::string>("ground_truth_transform_topic"),
        10,
        std::bind(&VisualOdometry::ground_truth_transform_callback, this, std::placeholders::_1));
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
    geometry_msgs::msg::TransformStamped pose_msg;
    pose_msg.child_frame_id = rf::ground_truth;
    pose_msg.header         = gt_msg->header;

    static bool            first_message = true;
    static lie_groups::SE3 first_pose;
    if (first_message) {
        lie_group_conversions::convert(gt_msg->pose, first_pose);
        first_pose    = first_pose.inverse();
        first_message = false;
    }

    // Remove starting offset
    lie_groups::SE3 pose;
    lie_group_conversions::convert(gt_msg->pose, pose);
    pose = first_pose * pose;

    lie_group_conversions::convert(pose, pose_msg.transform);

    tf_broadcaster_->sendTransform(pose_msg);
}

void VisualOdometry::ground_truth_transform_callback(const TransformStamped::SharedPtr gt_msg) {
    geometry_msgs::msg::TransformStamped pose_msg = *gt_msg;
    pose_msg.child_frame_id                       = rf::ground_truth;
    pose_msg.header.frame_id                      = rf::world;

    tf_broadcaster_->sendTransform(pose_msg);
}

} // namespace acrobat::localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::localization::VisualOdometry)
