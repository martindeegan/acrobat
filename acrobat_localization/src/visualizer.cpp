#include "acrobat_localization/map_point.hpp"
#include "acrobat_localization/visual_odometry.hpp"
#include "geometry_msgs/msg/point__struct.hpp"
#include "geometry_msgs/msg/pose__struct.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/marker__struct.hpp"
#include "visualization_msgs/msg/marker_array__struct.hpp"
#include <acrobat_common/constants/reference_frames.hpp>
#include <acrobat_common/lie_groups/lie_group_conversions.hpp>
#include <acrobat_localization/visualizer.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

using namespace visualization_msgs::msg;

namespace acrobat::localization {

Visualizer::~Visualizer() { cv::destroyWindow(window_name_); }

Visualizer::SharedPtr Visualizer::create(rclcpp::Logger logger, std::string window_name) {
    Visualizer::SharedPtr viz = std::make_shared<Visualizer>(logger, window_name);
    return viz;
}

Visualizer::Visualizer(rclcpp::Logger logger, std::string window_name)
    : logger_(logger), window_name_(window_name) {
    cv::namedWindow(window_name_);
}

void Visualizer::display_frame(const Frame::SharedPtr&  frame,
                               std::vector<cv::DMatch>& matches,
                               std::vector<bool>&       valid) const {
    cv::Mat image_with_keypoints;
    cv::cvtColor(frame->image(), image_with_keypoints, cv::COLOR_GRAY2RGB);

    for (size_t i = 0; i < frame->keypoints().size(); i++) {
        cv::Scalar color;
        if (valid[i]) {
            color = {0.0, 255.0, 0.0};
        } else {
            color = {255.0, 0.0, 0.0};
        }
        cv::circle(image_with_keypoints, frame->keypoints()[i].pt, 3, color);
    }
    cv::imshow(window_name_, image_with_keypoints);
    cv::waitKey(1);
}

void Visualizer::display_poses(const Frame::SharedPtr new_frame) {
    bool first = true;
    if (VisualOdometry::viz_pub_ && new_frame) {
        MarkerArray markers;
        Marker      pose_marker;
        pose_marker.action          = Marker::ADD;
        pose_marker.header.frame_id = acrobat::rf::world;
        pose_marker.header.stamp    = new_frame->received_stamp_;
        pose_marker.type            = Marker::ARROW;
        pose_marker.id              = new_frame->frame_id_;
        pose_marker.lifetime        = rclcpp::Duration(1, 0);
        pose_marker.frame_locked    = false;
        pose_marker.color.r         = 0.0;
        pose_marker.color.g         = 1.0;
        pose_marker.color.b         = 0.0;
        pose_marker.color.a         = 0.8;
        pose_marker.scale.x         = 0.1;
        pose_marker.scale.y         = 0.04;
        pose_marker.scale.z         = 0.04;
        geometry_msgs::msg::Pose pose;

        pose.position.x    = new_frame->pose_.x();
        pose.position.y    = new_frame->pose_.y();
        pose.position.z    = new_frame->pose_.z();
        pose.orientation.w = new_frame->pose_.rotation().toQuaternion().w();
        pose.orientation.x = new_frame->pose_.rotation().toQuaternion().x();
        pose.orientation.y = new_frame->pose_.rotation().toQuaternion().y();
        pose.orientation.z = new_frame->pose_.rotation().toQuaternion().z();
        pose_marker.pose   = pose;
        markers.markers.push_back(pose_marker);
        positions.push_back(pose.position);

        Marker connection_marker;
        if (first) {
            first                    = false;
            connection_marker.action = Marker::ADD;
        } else {
            connection_marker.action = Marker::MODIFY;
        }

        connection_marker.header.frame_id    = acrobat::rf::world;
        connection_marker.header.stamp       = new_frame->received_stamp_;
        connection_marker.type               = Marker::LINE_STRIP;
        connection_marker.id                 = -1;
        connection_marker.lifetime           = rclcpp::Duration::max();
        connection_marker.frame_locked       = false;
        connection_marker.color.r            = 0.0;
        connection_marker.color.g            = 0.0;
        connection_marker.color.b            = 1.0;
        connection_marker.color.a            = 1.0;
        connection_marker.scale.x            = 0.01;
        connection_marker.points             = positions;
        connection_marker.pose.position.x    = 0.0;
        connection_marker.pose.position.y    = 0.0;
        connection_marker.pose.position.z    = 0.0;
        connection_marker.pose.orientation.w = 1.0;
        connection_marker.pose.orientation.x = 0.0;
        connection_marker.pose.orientation.y = 0.0;
        connection_marker.pose.orientation.z = 0.0;
        markers.markers.push_back(connection_marker);

        VisualOdometry::viz_pub_->publish(markers);
    }
}

void Visualizer::display_points(const std::vector<MapPoint::SharedPtr>& points) {}

} // namespace acrobat::localization
