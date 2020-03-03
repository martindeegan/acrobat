#pragma once

#include <termios.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <arducam_config_parser/arducam_config_parser.h>
#include <arducam_sdk/ArduCamLib.h>

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

    void captureImage_callback();

    void convert_frame_to_message(uint8_t*                 frame_data,
                                  size_t                   frame_id,
                                  sensor_msgs::msg::Image& msg);

    rclcpp::TimerBase::SharedPtr timer_;
};
