#include <chrono>
#include <ctime>
#include <iostream>
#include <istream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <arducam/arducam_driver.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace acrobat::arducam {
ArducamDriver::ArducamDriver(const rclcpp::NodeOptions& options) : Node("arducam_driver", options) {
    declare_parameter("config_name");
    declare_parameter("camera_delay");
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(),
                        "Interrupted while waiting for the parameters client. Exiting.");
            rclcpp::shutdown();
        }
    }

    const auto config_name = parameters_client->get_parameter<std::string>("config_name");
    const auto config_filepath =
        ament_index_cpp::get_package_share_directory("arducam") + "/config/" + config_name;
    RCLCPP_ERROR(get_logger(), "Loading config file from %s", config_filepath.c_str());

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    if (!camera_init(config_filepath)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize the camera driver");
        rclcpp::shutdown();
    }

    rclcpp::sleep_for(1s);

    ArduCam_setMode(camera_handle_, CONTINUOUS_MODE);

    uint32_t rtn_val = ArduCam_beginCaptureImage(camera_handle_);
    if (rtn_val == USB_CAMERA_USB_TASK_ERROR) {
        RCLCPP_ERROR(get_logger(), "Error beginning capture, rtn_val = %zd", rtn_val);
    }

    RCLCPP_INFO(get_logger(), "Beginning capture");

    publisher               = create_publisher<sensor_msgs::msg::Image>("/acrobat/camera", 10);
    const auto camera_delay = parameters_client->get_parameter<double>("camera_delay");
    timer_                  = create_wall_timer(duration<double, std::ratio<1, 1000>>(camera_delay),
                               std::bind(&ArducamDriver::capture_image, this));
}

ArducamDriver::~ArducamDriver() {
    ArduCam_endCaptureImage(camera_handle_);
    ArduCam_close(camera_handle_);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

bool ArducamDriver::camera_init(const std::string& filename) {
    CameraConfigs cam_cfgs;
    memset(&cam_cfgs, 0x00, sizeof(CameraConfigs));
    if (arducam_parse_config(filename.c_str(), &cam_cfgs)) {
        RCLCPP_ERROR(get_logger(), "Cannot find configuration file.");
        return false;
    }

    CameraParam* cam_param      = &cam_cfgs.camera_param;
    Config*      configs        = &cam_cfgs.configs[0];
    int          configs_length = cam_cfgs.configs_length;

    switch (cam_param->i2c_mode) {
    case 0:
        camera_config_.emI2cMode = I2C_MODE_8_8;
        break;
    case 1:
        camera_config_.emI2cMode = I2C_MODE_8_16;
        break;
    case 2:
        camera_config_.emI2cMode = I2C_MODE_16_8;
        break;
    case 3:
        camera_config_.emI2cMode = I2C_MODE_16_16;
        break;
    default:
        break;
    }

    switch (cam_param->format >> 8) {
    case 0:
        camera_config_.emImageFmtMode = FORMAT_MODE_RAW;
        break;
    case 1:
        camera_config_.emImageFmtMode = FORMAT_MODE_RGB;
        break;
    case 2:
        camera_config_.emImageFmtMode = FORMAT_MODE_YUV;
        break;
    case 3:
        camera_config_.emImageFmtMode = FORMAT_MODE_JPG;
        break;
    case 4:
        camera_config_.emImageFmtMode = FORMAT_MODE_MON;
        break;
    case 5:
        camera_config_.emImageFmtMode = FORMAT_MODE_RAW_D;
        break;
    case 6:
        camera_config_.emImageFmtMode = FORMAT_MODE_MON_D;
        break;
    default:
        break;
    }

    camera_config_.u32Width  = cam_param->width;
    camera_config_.u32Height = cam_param->height;

    camera_config_.u32I2cAddr  = cam_param->i2c_addr;
    camera_config_.u8PixelBits = cam_param->bit_width;
    camera_config_.u32TransLvl = cam_param->trans_lvl;

    if (camera_config_.u8PixelBits <= 8) {
        camera_config_.u8PixelBytes = 1;
    } else if (camera_config_.u8PixelBits > 8 && camera_config_.u8PixelBits <= 16) {
        camera_config_.u8PixelBytes = 2;
    }

    int ret_val = ArduCam_autoopen(camera_handle_, &camera_config_);
    if (ret_val == USB_CAMERA_NO_ERROR) {
        // ArduCam_enableForceRead(camera_handle_);	//Force display image
        // Uint8 u8Buf[8];
        for (size_t i = 0; i < configs_length; ++i) {
            uint32_t type = configs[i].type;
            if (((type >> 16) & 0xFF) && ((type >> 16) & 0xFF) != camera_config_.usbType)
                continue;
            switch (type & 0xFFFF) {
            case CONFIG_TYPE_REG:
                ArduCam_writeSensorReg(camera_handle_, configs[i].params[0], configs[i].params[1]);
                break;
            case CONFIG_TYPE_DELAY:
                rclcpp::sleep_for(microseconds(configs[i].params[0]));
                break;
            case CONFIG_TYPE_VRCMD:
                configure_board(configs[i]);
                break;
            }
        }
        unsigned char u8TmpData[16];
        ArduCam_readUserData(camera_handle_, 0x400 - 16, 16, u8TmpData);
        RCLCPP_INFO(get_logger(),
                    "Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c\n",
                    u8TmpData[0],
                    u8TmpData[1],
                    u8TmpData[2],
                    u8TmpData[3],
                    u8TmpData[4],
                    u8TmpData[5],
                    u8TmpData[6],
                    u8TmpData[7],
                    u8TmpData[8],
                    u8TmpData[9],
                    u8TmpData[10],
                    u8TmpData[11]);
    } else {
        RCLCPP_ERROR(get_logger(), "Cannot open camera.rtn_val = %zd\n", ret_val);
        return false;
    }

    return true;
}

void ArducamDriver::configure_board(const Config& config) {
    uint8_t u8Buf[10];
    for (unsigned n = 0; n < config.params[3]; n++) {
        u8Buf[n] = config.params[4 + n];
    }
    ArduCam_setboardConfig(camera_handle_,
                           config.params[0],
                           config.params[1],
                           config.params[2],
                           config.params[3],
                           u8Buf);
}

void ArducamDriver::convert_frame_to_message(uint8_t*                 frame_data,
                                             size_t                   frame_id,
                                             sensor_msgs::msg::Image& msg) {
    // copy cv information into ros message
    msg.height          = camera_config_.u32Height;
    msg.width           = camera_config_.u32Width;
    msg.is_bigendian    = false;
    size_t size         = msg.height * msg.width;
    msg.data            = std::vector<uint8_t>(frame_data, frame_data + size);
    msg.header.frame_id = std::to_string(frame_id);
    msg.encoding        = sensor_msgs::image_encodings::TYPE_8UC1; // opencv type
}

void ArducamDriver::capture_image() {
    ArduCamOutData* frameData;

    ArduCam_captureImage(camera_handle_);

    uint32_t rtn_val = ArduCam_readImage(camera_handle_, frameData);
    if (rtn_val == USB_CAMERA_NO_ERROR) {
        static size_t frame_id = 0;
        auto          msg      = std::make_unique<sensor_msgs::msg::Image>();
        convert_frame_to_message(frameData->pu8ImageData, frame_id, *msg);
        publisher->publish(std::move(msg));
        ArduCam_del(camera_handle_);

        ++frame_id;
    }
}

} // namespace acrobat::arducam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::arducam::ArducamDriver)
