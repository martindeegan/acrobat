#pragma once

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>

#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <acrobat_common/composition/visibility_control.hpp>
#include <acrobat_common/lie_groups/lie_groups.hpp>

namespace acrobat::localization {

class VisualOdometry : public rclcpp::Node {
  public:
    ACROBAT_COMPOSITION_PUBLIC

    explicit VisualOdometry(const rclcpp::NodeOptions& options);
    ~VisualOdometry();

  private:
    void run_frontend();
    void run_backend();

    void                                           publish_pose();
    rclcpp::TimerBase::SharedPtr                   publish_timer_;
    lie_groups::SE3                                robot_pose_;
    geometry_msgs::msg::TransformStamped           pose_msg_stamped_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::atomic_bool running_;
    std::thread      frontend_thread_;
    std::thread      backend_thread_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    cv::Ptr<cv::ORB> orb_detector_;

    const std::string window_name = "/acrobat/camera";
};

} // namespace acrobat::localization