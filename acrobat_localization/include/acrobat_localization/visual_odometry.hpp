#pragma once

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <acrobat_common/composition/visibility_control.hpp>
#include <acrobat_common/lie_groups/lie_groups.hpp>
#include <acrobat_localization/backend.hpp>
#include <acrobat_localization/frontend.hpp>
#include <acrobat_localization/imu_propagator.hpp>
#include <acrobat_localization/visualizer.hpp>

namespace acrobat::localization {

class VisualOdometry : public rclcpp::Node {
  public:
    ACROBAT_COMPOSITION_PUBLIC

    explicit VisualOdometry(const rclcpp::NodeOptions& options);
    ~VisualOdometry();

  private:
    ImuPropagator::SharedPtr imu_propagator_;
    Frontend::SharedPtr      frontend_;
    Backend::SharedPtr       backend_;
    Visualizer::SharedPtr    visualizer_;

    std::atomic_bool running_;
    std::thread      frontend_thread_;
    std::thread      backend_thread_;

    geometry_msgs::msg::TransformStamped           pose_msg_stamped_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticBroadcasterQoS> tf_static_broadcaster_;
    std::unique_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::unique_ptr<tf2_ros::Buffer>               tf_buffer_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg);
    void ground_truth_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr gt_msg);
    void ground_truth_transform_callback(
        const geometry_msgs::msg::TransformStamped::SharedPtr gt_msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        ground_truth_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
        ground_truth_transform_subscription_;
};

} // namespace acrobat::localization