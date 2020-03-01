#include "arducam/arducam_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <ctime>
#include <iostream>
#include <istream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

ArducamDriver::ArducamDriver(const rclcpp::NodeOptions& options) : Node("arducam_driver", options) {
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    publisher = create_publisher<sensor_msgs::msg::Image>("/acrobat/camera", 10);

    if (cameraInit("/usr/local/ArduCam/AR0135_MONO_8b_1280x964_51fps.cfg")) {
        ArduCam_setMode(cameraHandle, CONTINUOUS_MODE);
        capture_thread = std::thread(&ArducamDriver::captureImage_thread, this);
        read_thread    = std::thread(&ArducamDriver::readImage_thread, this);
    }
}

ArducamDriver::~ArducamDriver() {
    running = false;
    capture_thread.join();
    read_thread.join();

    ArduCam_close(cameraHandle);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void ArducamDriver::readImage_thread() {
    static size_t   frame_id = 0;
    ArduCamOutData* frameData; // Uint8* data = frameData->pu8ImageData;

    while (running) {
        if (ArduCam_availableImage(cameraHandle) > 0) {
            Uint32 rtn_val = ArduCam_readImage(cameraHandle, frameData);
            if (rtn_val == USB_CAMERA_NO_ERROR) {
                auto msg          = std::make_unique<sensor_msgs::msg::Image>();
                msg->is_bigendian = false;

                convert_frame_to_message(frameData->pu8ImageData, frame_id, *msg);

                RCLCPP_INFO(get_logger(), "Publishing image #%zd", frame_id);
                publisher->publish(std::move(msg));

                ++frame_id;

                ArduCam_del(cameraHandle);
            }
        }
    }
    // RCLCPP_INFO()
    std::cout << "Read thread stopped" << std::endl;
}

void ArducamDriver::convert_frame_to_message(uint8_t*                 frame_data,
                                             size_t                   frame_id,
                                             sensor_msgs::msg::Image& msg) {
    // copy cv information into ros message
    msg.height = cameraCfg.u32Height;
    msg.width  = cameraCfg.u32Width;
    // msg.encoding = ?
    // msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step); ?
    size_t size = msg.height * msg.width;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame_data, size);
    msg.header.frame_id = std::to_string(frame_id);
    msg.encoding        = sensor_msgs::image_encodings::TYPE_8UC1;
}

void ArducamDriver::captureImage_thread() {
    Uint32 rtn_val = ArduCam_beginCaptureImage(cameraHandle);

    if (rtn_val == USB_CAMERA_USB_TASK_ERROR) {
        std::cout << "Error beginning capture, rtn_val = " << rtn_val << std::endl;
        return;
    }
    std::cout << "Capture began, rtn_val = " << rtn_val << std::endl;

    while (running) {
        rtn_val = ArduCam_captureImage(cameraHandle);
        if (rtn_val == USB_CAMERA_USB_TASK_ERROR) {
            std::cout << "Error capture image, rtn_val = " << rtn_val << std::endl;
            break;
        }

        if (rtn_val > 0xFF) {
            std::cout << "Error capture image, rtn_val = " << rtn_val << std::endl;
        }
    }
    running = false;
    ArduCam_endCaptureImage(cameraHandle);
    std::cout << "Capture thread stopped." << std::endl;
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

    color_mode = cam_param->format & 0xFF;
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
        save_raw               = true;
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
                usleep(1000 * configs[i].params[0]);
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
        std::cout << "Cannot open camera.rtn_val = " << ret_val << std::endl;
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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ArducamDriver)