#pragma once

#include <termios.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <arducam_config_parser/arducam_config_parser.h>
#include <arducam_sdk/ArduCamLib.h>

namespace acrobat::arducam {

class ArducamDriver : public rclcpp::Node {
  public:
    explicit ArducamDriver(const rclcpp::NodeOptions& options);
    ~ArducamDriver();

  private:
    bool camera_init(const std::string& filename);

    void configure_board(const Config& config);

    void capture_image();

    struct termios oldt, newt;

    ArduCamCfg    camera_config_;
    ArduCamHandle camera_handle_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher;
    size_t                                                      image_size_;
    sensor_msgs::msg::Image                                     msg_;

    double      capture_frequency_;
    std::thread capture_thread_;
};

} // namespace acrobat::arducam
