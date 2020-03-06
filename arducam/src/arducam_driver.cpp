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
            RCLCPP_ERROR(this->get_logger(),
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

    publisher = create_publisher<sensor_msgs::msg::Image>("/acrobat/camera", 10);

    if (!cameraInit(config_filepath)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize the camera driver\n");
    }

    rclcpp::sleep_for(1s);

    ArduCam_setMode(cameraHandle, CONTINUOUS_MODE);

    uint32_t rtn_val = ArduCam_beginCaptureImage(cameraHandle);
    if (rtn_val == USB_CAMERA_USB_TASK_ERROR) {
        RCLCPP_ERROR(get_logger(), "Error beginning capture, rtn_val = %zd\n", rtn_val);
    }
    RCLCPP_INFO(get_logger(), "Capture began, rtn_val = %zd\n", rtn_val);

    const auto camera_delay = parameters_client->get_parameter<double>("camera_delay");
    capture_timer_          = create_wall_timer(duration<double, std::ratio<1, 1000>>(camera_delay),
                                       std::bind(&ArducamDriver::capture_image, this));
    read_timer_ = create_wall_timer(duration<double, std::ratio<1, 1000>>(camera_delay / 8.0),
                                    std::bind(&ArducamDriver::read_image, this));

    msg.height          = cameraCfg.u32Height;
    msg.width           = cameraCfg.u32Width;
    msg.header.frame_id = "acrobat_camera";
    msg.encoding        = sensor_msgs::image_encodings::TYPE_8UC1; // opencv type
    msg.is_bigendian    = false;
}

ArducamDriver::~ArducamDriver() {
    RCLCPP_INFO(get_logger(), "Closing arducam...");

    ArduCam_endCaptureImage(cameraHandle);
    ArduCam_close(cameraHandle);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    RCLCPP_INFO(get_logger(), "Arducam closed.");
    rclcpp::sleep_for(1s);
}

void ArducamDriver::capture_image() {
    msg.header.stamp = rclcpp::Time();
    ArduCam_captureImage(cameraHandle);
}

void ArducamDriver::read_image() {
    std::lock_guard<std::mutex> guard(read_mutex_);
    if (ArduCam_availableImage(cameraHandle)) {
        ArduCamOutData* frameData;

        uint32_t rtn_val = ArduCam_readImage(cameraHandle, frameData);
        if (rtn_val == USB_CAMERA_NO_ERROR) {
            static size_t frame_id = 0;
            const size_t  size     = msg.height * msg.width;
            msg.data =
                std::vector<uint8_t>(frameData->pu8ImageData, frameData->pu8ImageData + size);
            publisher->publish(msg);
            ArduCam_del(cameraHandle);

            ++frame_id;
        }
    }
}

bool ArducamDriver::cameraInit(const std::string& filename) {
    CameraConfigs cam_cfgs;
    memset(&cam_cfgs, 0x00, sizeof(CameraConfigs));
    if (arducam_parse_config(filename.c_str(), &cam_cfgs)) {
        std::cout << "Cannot find configuration file." << std::endl << std::endl;
        return false;
    }

    CameraParam* cam_param      = &cam_cfgs.camera_param;
    Config*      configs        = &cam_cfgs.configs[0];
    int          configs_length = cam_cfgs.configs_length;

    switch (cam_param->i2c_mode) {
    case 0:
        cameraCfg.emI2cMode = I2C_MODE_8_8;
        break;
    case 1:
        cameraCfg.emI2cMode = I2C_MODE_8_16;
        break;
    case 2:
        cameraCfg.emI2cMode = I2C_MODE_16_8;
        break;
    case 3:
        cameraCfg.emI2cMode = I2C_MODE_16_16;
        break;
    default:
        break;
    }

    switch (cam_param->format >> 8) {
    case 0:
        cameraCfg.emImageFmtMode = FORMAT_MODE_RAW;
        break;
    case 1:
        cameraCfg.emImageFmtMode = FORMAT_MODE_RGB;
        break;
    case 2:
        cameraCfg.emImageFmtMode = FORMAT_MODE_YUV;
        break;
    case 3:
        cameraCfg.emImageFmtMode = FORMAT_MODE_JPG;
        break;
    case 4:
        cameraCfg.emImageFmtMode = FORMAT_MODE_MON;
        break;
    case 5:
        cameraCfg.emImageFmtMode = FORMAT_MODE_RAW_D;
        break;
    case 6:
        cameraCfg.emImageFmtMode = FORMAT_MODE_MON_D;
        break;
    default:
        break;
    }

    cameraCfg.u32Width  = cam_param->width;
    cameraCfg.u32Height = cam_param->height;

    cameraCfg.u32I2cAddr  = cam_param->i2c_addr;
    cameraCfg.u8PixelBits = cam_param->bit_width;
    cameraCfg.u32TransLvl = cam_param->trans_lvl;

    if (cameraCfg.u8PixelBits <= 8) {
        cameraCfg.u8PixelBytes = 1;
    } else if (cameraCfg.u8PixelBits > 8 && cameraCfg.u8PixelBits <= 16) {
        cameraCfg.u8PixelBytes = 2;
    }

    int ret_val = ArduCam_autoopen(cameraHandle, &cameraCfg);
    if (ret_val == USB_CAMERA_NO_ERROR) {
        // ArduCam_enableForceRead(cameraHandle);	//Force display image
        // Uint8 u8Buf[8];
        for (size_t i = 0; i < configs_length; ++i) {
            uint32_t type = configs[i].type;
            if (((type >> 16) & 0xFF) && ((type >> 16) & 0xFF) != cameraCfg.usbType)
                continue;
            switch (type & 0xFFFF) {
            case CONFIG_TYPE_REG:
                ArduCam_writeSensorReg(cameraHandle, configs[i].params[0], configs[i].params[1]);
                break;
            case CONFIG_TYPE_DELAY:
                rclcpp::sleep_for(microseconds(configs[i].params[0]));
                break;
            case CONFIG_TYPE_VRCMD:
                configBoard(configs[i]);
                break;
            }
        }
        unsigned char u8TmpData[16];
        ArduCam_readUserData(cameraHandle, 0x400 - 16, 16, u8TmpData);
        printf("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c\n",
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

void ArducamDriver::configBoard(const Config& config) {
    uint8_t u8Buf[10];
    for (unsigned n = 0; n < config.params[3]; n++) {
        u8Buf[n] = config.params[4 + n];
    }
    ArduCam_setboardConfig(cameraHandle,
                           config.params[0],
                           config.params[1],
                           config.params[2],
                           config.params[3],
                           u8Buf);
}

} // namespace acrobat::arducam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::arducam::ArducamDriver)
