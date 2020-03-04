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

    SerialBridge(const rclcpp::NodeOptions& options)
        : Node("serial_bridge", options), num_messages_(0), buffer_index_(1) {
        declare_parameter("device");

        auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this);
        device_               = parameter_client->get_parameter<std::string>("device");

        // pose_buffer_   = std::make_shared<tf2_ros::Buffer>();
        // pose_listener_ = std::make_shared<tf2_ros::TransformListener>(*pose_buffer_);

        write_timer_ = create_wall_timer(10ms, std::bind(&SerialBridge::write, this));
        read_timer_  = create_wall_timer(10ms, std::bind(&SerialBridge::read, this));
    }

    ~SerialBridge() {
    }

  private:
    void write() {
        std::array<char, buffer_size_> temp_buffer;
        {
            std::lock_guard<std::mutex> guard(buffer_mutex_);
            buffer_[0] = num_messages_;
            std::swap(temp_buffer, buffer_);

            num_messages_ = 0;
            buffer_index_ = 1;
        }
    }

    void read() {
    }

    bool write_to_buffer(MessageType msg_type, const char* data, size_t len) {
        if (buffer_index_ + 1 + len >= 256)
            return false;

        {
            std::lock_guard<std::mutex> guard(buffer_mutex_);
            buffer_[buffer_index_] = static_cast<uint8_t>(msg_type);
            memcpy(buffer_.data() + buffer_index_, data, len);
            buffer_index_ += len;
            num_messages_++;
        }

        return true;
    }

    void transform_callback(geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        std::array<double, 8> data;
        rclcpp::Time          stamp(msg->header.stamp);
        data[0] = stamp.seconds();
        data[1] = msg->transform.rotation.w;
        data[2] = msg->transform.rotation.x;
        data[3] = msg->transform.rotation.y;
        data[4] = msg->transform.rotation.z;
        data[5] = msg->transform.translation.x;
        data[6] = msg->transform.translation.y;
        data[7] = msg->transform.translation.z;

        if (!write_to_buffer(MessageType::Pose,
                             reinterpret_cast<const char*>(data.data()),
                             data.size() * sizeof(double))) {
            RCLCPP_ERROR(get_logger(), "Not enough space in buffer!");
        }
    }

    std::string device_;

    constexpr static size_t        buffer_size_ = 256;
    char                           num_messages_;
    size_t                         buffer_index_;
    std::array<char, buffer_size_> buffer_;
    std::mutex                     buffer_mutex_;
    rclcpp::TimerBase::SharedPtr   write_timer_;
    rclcpp::TimerBase::SharedPtr   read_timer_;

    std::shared_ptr<tf2_ros::Buffer>               pose_buffer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> pose_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener>    pose_listener_;
};

} // namespace acrobat::serial_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::serial_bridge::SerialBridge)
