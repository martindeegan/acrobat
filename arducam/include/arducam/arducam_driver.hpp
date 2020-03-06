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
    ArduCamCfg     cameraCfg;
    ArduCamHandle  cameraHandle;
    struct termios oldt, newt;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher;

    bool cameraInit(const std::string& filename);

    void configBoard(const Config& config);

    void capture_image();
    void read_image();

    sensor_msgs::msg::Image msg;

    std::mutex                   read_mutex_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
};

} // namespace acrobat::arducam
