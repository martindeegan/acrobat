#pragma once

#include "acrobat_localization/frame.hpp"
#include "acrobat_localization/map_point.hpp"
#include "rclcpp/publisher.hpp"
#include "visualization_msgs/msg/marker__struct.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <acrobat_localization/frame.hpp>

namespace acrobat::localization {

class Visualizer {
  public:
    Visualizer(rclcpp::Logger logger, std::string window_name);
    ~Visualizer();

    using SharedPtr = std::shared_ptr<Visualizer>;

    static SharedPtr create(rclcpp::Logger logger, std::string window_name = "vio_camera");

    void display_frame(const Frame::SharedPtr&  frame,
                       std::vector<cv::DMatch>& matches,
                       std::vector<bool>&       valid) const;

    void display_poses(const Frame::SharedPtr new_frame);
    void display_points(const std::vector<MapPoint::SharedPtr>& points);

  private:
    rclcpp::Logger                                                logger_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualization_publisher;

    std::vector<geometry_msgs::msg::Point> positions;
    const cv::String                       window_name_;
};

} // namespace acrobat::localization