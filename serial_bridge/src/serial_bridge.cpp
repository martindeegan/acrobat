#include <array>
#include <chrono>
#include <queue>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <acrobat_common/composition/visibility_control.hpp>
#include <serial_bridge/encodings.hpp>

using namespace std::chrono_literals;

namespace acrobat::serial_bridge {

enum class MessageType : uint8_t { Pose = 0 };

class SerialBridge : public rclcpp::Node {
  public:
    ACROBAT_COMPOSITION_PUBLIC

    SerialBridge(const rclcpp::NodeOptions& options) : Node("serial_bridge", options) {
        declare_parameter("device");

        auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this);
        device_               = parameter_client->get_parameter<std::string>("device");

        write_timer_ = create_wall_timer(10ms, std::bind(&SerialBridge::write, this));
        read_timer_  = create_wall_timer(10ms, std::bind(&SerialBridge::read, this));

        buffer_       = Buffer{0};
        buffer_begin_ = buffer_.begin() + 2;
    }

    ~SerialBridge() {
    }

  private:
    void write() {
    }

    void read() {
    }

    void transform_callback(geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(buffer_mutex_);
        encode(buffer_, buffer_begin_, msg);
    }

    std::string device_;

    Buffer                       buffer_;
    Buffer::iterator             buffer_begin_;
    std::mutex                   buffer_mutex_;
    rclcpp::TimerBase::SharedPtr write_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
};

} // namespace acrobat::serial_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::serial_bridge::SerialBridge)
