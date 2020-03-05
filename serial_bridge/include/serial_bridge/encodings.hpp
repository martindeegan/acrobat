#pragma once

#include <array>
#include <memory>

#include <boost/crc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace acrobat::serial_bridge {

constexpr size_t checksum_size         = sizeof(short);
constexpr size_t end_of_message_length = sizeof(char);
constexpr size_t buffer_size           = 256;
using Buffer                           = std::array<char, buffer_size>;

// Templated message information
template<class Message> struct message_information {
    static constexpr char   id     = 0;
    static constexpr size_t length = 0;
};

template<> struct message_information<geometry_msgs::msg::TransformStamped> {
    static constexpr char   id     = 0;
    static constexpr size_t length = 8 * sizeof(double);
};

// Compute CRC16 checksum
short compute_checksum(const Buffer& buffer);

// Add checksum and end of message data
void finalize_message(Buffer& buffer, Buffer::iterator buffer_begin);

// Check that the checksum matches the message contents
bool check_message(const Buffer& buffer);

template<class Message>
void encode_impl(Buffer::iterator         buffer_begin,
                 Buffer::iterator         buffer_end,
                 std::shared_ptr<Message> msg) {
    // Empty implementation
}

template<>
void encode_impl(Buffer::iterator                                      buffer_begin,
                 Buffer::iterator                                      buffer_end,
                 std::shared_ptr<geometry_msgs::msg::TransformStamped> msg) {
    std::array<double, 8> message_data;

    rclcpp::Time stamp(msg->header.stamp);
    message_data[0] = stamp.seconds();
    message_data[1] = msg->transform.rotation.w;
    message_data[2] = msg->transform.rotation.x;
    message_data[3] = msg->transform.rotation.y;
    message_data[4] = msg->transform.rotation.z;
    message_data[5] = msg->transform.translation.x;
    message_data[6] = msg->transform.translation.y;
    message_data[7] = msg->transform.translation.z;

    const char* message_bytes = reinterpret_cast<const char*>(message_data.data());
    std::copy(message_bytes,
              message_bytes + message_information<geometry_msgs::msg::TransformStamped>::length,
              buffer_begin);
}

// Encodes a message into a byte array
// data - the data to write the encoded bytes to
// msg - the ros message
// returns the size of the message in bytes
template<class Message>
bool encode(Buffer buffer, Buffer::iterator& buffer_begin, std::shared_ptr<Message> msg) {
    // Typical message structure
    // | id |  message data |
    if (std::distance(buffer_begin, buffer.end()) < 1 + message_information<Message>::length)
        return false;

    *buffer_begin = message_information<Message>::id;
    buffer_begin++;
    auto buffer_end = buffer_begin + message_information<Message>::length;
    encode_impl<Message>(buffer_begin, buffer_end, msg);

    buffer_begin = buffer_end;

    return true;
}

// Decodes a message into its ros message type
// data - the data contained in the messsage
template<typename Message> Message decode(const char* data) {
}

} // namespace acrobat::serial_bridge
