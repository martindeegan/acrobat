#include <array>
#include <chrono>
#include <queue>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <acrobat_common/composition/visibility_control.hpp>
#include <serial_bridge/buffer.hpp>

using namespace std::chrono_literals;

namespace acrobat::serial_bridge {

enum class MessageType : uint8_t { Pose = 0 };

class SerialBridge : public rclcpp::Node {
  public:
    ACROBAT_COMPOSITION_PUBLIC

    SerialBridge(const rclcpp::NodeOptions& options) : Node("serial_bridge", options) {
        declare_parameter("ground_station");
        declare_parameter("device");
        declare_parameter("baudrate");

        auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this);
        ground_station_       = parameter_client->get_parameter<bool>("ground_station");
        device_               = parameter_client->get_parameter<std::string>("device");
        baudrate_             = parameter_client->get_parameter<uint32_t>("baudrate");

        serial_port_.setPort(device_);
        serial_port_.setBaudrate(baudrate_);
        serial_port_.open();

        reset_buffer();

        handshake_timer_ = create_wall_timer(100ms, std::bind(&SerialBridge::handshake, this));
    }

    ~SerialBridge() {
        serial_port_.close();
    }

  private:
    void handshake() {
        serial_port_.write("Handshake\n");
        std::string response = serial_port_.readline();

        if (response == "Handshake\n") {
        }
    }

    void initialize_timers() {
        write_timer_ = create_wall_timer(20ms, std::bind(&SerialBridge::write, this));
        read_timer_  = create_wall_timer(20ms, std::bind(&SerialBridge::read, this));
    }

    void reset_buffer() {
        clear_buffer(buffer_);
        buffer_index_ = checksum_size;
    }

    void write() {
        std::lock_guard<std::mutex> guard(buffer_mutex_);
        serial_port_.write(buffer_, buffer_size);
    }

    void read() {
        uint8_t read_buffer[buffer_size];
        serial_port_.read(read_buffer, buffer_size);
    }

    void transform_callback(geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(buffer_mutex_);
    }

    bool ground_station_;

    std::string                  device_;
    uint32_t                     baudrate_;
    serial::Serial               serial_port_;
    rclcpp::TimerBase::SharedPtr handshake_timer_;

    std::mutex                   buffer_mutex_;
    uint8_t*                     buffer_;
    size_t                       buffer_index_;
    rclcpp::TimerBase::SharedPtr write_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
};

} // namespace acrobat::serial_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::serial_bridge::SerialBridge)
