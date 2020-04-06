#pragma once

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <acrobat_localization/frame.hpp>

namespace acrobat::localization {

class Visualizer {
  public:
    Visualizer(rclcpp::Logger logger, std::string window_name);
    ~Visualizer();

    using SharedPtr = std::shared_ptr<Visualizer>;

    static SharedPtr create(rclcpp::Logger logger, std::string window_name = "vio_camera");

    void display_frame(const Frame::SharedPtr& frame) const;

  private:
    rclcpp::Logger logger_;

    const cv::String window_name_;
};

} // namespace acrobat::localization