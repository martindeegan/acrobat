#include "arducam/arducam_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <ctime>
#include <iostream>
#include <istream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

using namespace std::chrono;

ArducamDriver::ArducamDriver(const rclcpp::NodeOptions& options) : Node("arducam_driver", options) {
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    publisher = create_publisher<sensor_msgs::msg::Image>("/acrobat/camera", 10);

    if (!cameraInit("/usr/local/ArduCam/AR0135_MONO_8b_1280x964_51fps.cfg")) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize the camera driver\n");
    }

    ArduCam_setMode(cameraHandle, CONTINUOUS_MODE);
    capture_thread_ = std::thread(&ArducamDriver::captureImage_thread, this);
    read_thread_    = std::thread(&ArducamDriver::readImage_thread, this);
}

ArducamDriver::~ArducamDriver() {
    running = false;
    capture_thread_.join();
    read_thread_.join();

    ArduCam_close(cameraHandle);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void ArducamDriver::readImage_thread() {
    static size_t   frame_id = 1;
    const size_t    n_frames = 30;
    ArduCamOutData* frameData; // Uint8* data = frameData->pu8ImageData;

    time_point<system_clock, duration<double>> t_start = system_clock::now();
    while (running) {
        if (ArduCam_availableImage(cameraHandle) > 0) {
            uint32_t rtn_val = ArduCam_readImage(cameraHandle, frameData);
            if (rtn_val == USB_CAMERA_NO_ERROR) {
                auto msg = std::make_unique<sensor_msgs::msg::Image>();
                convert_frame_to_message(frameData->pu8ImageData, frame_id, *msg);
                publisher->publish(std::move(msg));
                ArduCam_del(cameraHandle);

                // RCLCPP_INFO(get_logger(), "Frame recieved %zd\n", frame_id);

                ++frame_id;
                if (frame_id >= n_frames) {
                    duration<double> dt = system_clock::now() - t_start;
                    t_start             = system_clock::now(); // reset t_start
                    RCLCPP_INFO(get_logger(), "Camera fps = %lf\n", n_frames / dt.count());
                    frame_id = 1;
                }
            }
        }
    }
    // RCLCPP_INFO()
    RCLCPP_INFO(get_logger(), "Read thread stopped\n");
}

void ArducamDriver::convert_frame_to_message(uint8_t*                 frame_data,
                                             size_t                   frame_id,
                                             sensor_msgs::msg::Image& msg) {
    // copy cv information into ros message
    msg.height       = cameraCfg.u32Height;
    msg.width        = cameraCfg.u32Width;
    msg.is_bigendian = false;
    size_t size      = msg.height * msg.width;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame_data, size);
    msg.header.frame_id = std::to_string(frame_id);
    msg.encoding        = sensor_msgs::image_encodings::TYPE_8UC1; // opencv type
}

void ArducamDriver::captureImage_thread() {
    uint32_t rtn_val = ArduCam_beginCaptureImage(cameraHandle);

    if (rtn_val == USB_CAMERA_USB_TASK_ERROR) {
        RCLCPP_ERROR(get_logger(), "Error beginning capture, rtn_val = %zd\n", rtn_val);
        return;
    }
    RCLCPP_INFO(get_logger(), "Capture began, rtn_val = %zd\n", rtn_val);

    while (running) {
        rtn_val = ArduCam_captureImage(cameraHandle);
        if (rtn_val == USB_CAMERA_USB_TASK_ERROR) {
            RCLCPP_ERROR(get_logger(), "Error capture image, rtn_val = %zd\n", rtn_val);
            break;
        }

        if (rtn_val > 0xFF) {
            RCLCPP_ERROR(get_logger(), "Error capture image, rtn_val = %zd\n", rtn_val);
        }
    }
    running = false;
    ArduCam_endCaptureImage(cameraHandle);
    RCLCPP_INFO(get_logger(), "Capture thread stopped.\n");
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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ArducamDriver)