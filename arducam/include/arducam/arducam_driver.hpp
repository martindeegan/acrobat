#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "arducam/ArduCamLib.h"
#include "arducam_config_parser.h"
#include <termios.h>
#include <thread>

class ArducamDriver : public rclcpp::Node {
  public:
    explicit ArducamDriver(const rclcpp::NodeOptions& options);
    ~ArducamDriver();

  private:
    ArduCamCfg     cameraCfg;
    ArduCamHandle  cameraHandle;
    struct termios oldt, newt;

    bool save_raw   = false;
    bool save_flag  = false;
    int  color_mode = 0;
    bool running    = true;

    std::thread capture_thread_;
    std::thread read_thread_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher;

    bool cameraInit(const std::string& filename);

    void configBoard(const Config& config);

    void captureImage_thread();
    void readImage_thread();

    void convert_frame_to_message(uint8_t*                 frame_data,
                                  size_t                   frame_id,
                                  sensor_msgs::msg::Image& msg);
};
