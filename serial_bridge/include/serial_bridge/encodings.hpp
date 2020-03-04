#pragma once

#include <array>
#include <memory>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>

// namespace acrobat::serial_bridge {

// using TransformStamped = geometry_msgs::msg::TransformStamped;

// constexpr size_t checksum_length = 32;
// constexpr size_t buffer_size     = 256;
// using Buffer                     = std::array<char, buffer_size>;

// // Templated message information
// template<class Message> struct message_information {
//     static constexpr char   id;
//     static constexpr size_t length;
// };

// // Encodes a message into a byte array
// // data - the data to write the encoded bytes to
// // msg - the ros message
// // returns the size of the message in bytes
// template<typename Message>
// bool encode(Buffer buffer, Buffer::iterator& buffer_begin, typename std::shared_ptr<Message>
// msg);

// template<typename Message>
// void encode_impl(Buffer                            buffer,
//                  Buffer::iterator&                 buffer_begin,
//                  typename std::shared_ptr<Message> msg);

// // Decodes a message into its ros message type
// // data - the data contained in the messsage
// template<typename Message> Message decode(const char* data);

// } // namespace acrobat::serial_bridge

// // Custom implementation for poses since they have dynamic sized frame names
// template<>
// bool acrobat::serial_bridge::encode<geometry_msgs::msg::TransformStamped>(
//     Buffer                                                         buffer,
//     Buffer::iterator&                                              buffer_begin,
//     typename std::shared_ptr<geometry_msgs::msg::TransformStamped> msg) {
//     const size_t message_length = message_information<TransformStamped>::length +
//                                   msg->child_frame_id.size() + msg->header.frame_id.size();
//     if (std::distance(buffer_begin, buffer.end()) < message_length)
//         return false;
//     // clang-format off
//     // Transform structure
//     // | id |  timestamp | rotation | translation | child_frame_name_length | child_frame_name |
//     frame_name_length | frame_name |
//     //clang-format on

//     // Set message id
//     *buffer_begin = message_information<TransformStamped>::id;
//     buffer_begin++;

//     // Copy pose data into buffer
//     std::array<double, message_information<TransformStamped>::length> message_data;
//     rclcpp::Time                                                      stamp(msg->header.stamp);
//     message_data[0] = stamp.seconds();
//     message_data[1] = msg->transform.rotation.w;
//     message_data[2] = msg->transform.rotation.x;
//     message_data[3] = msg->transform.rotation.y;
//     message_data[4] = msg->transform.rotation.z;
//     message_data[5] = msg->transform.translation.x;
//     message_data[6] = msg->transform.translation.y;
//     message_data[7] = msg->transform.translation.z;

//     const char* encoded_bytes = reinterpret_cast<const char*>(message_data.data());
//     std::copy(
//         encoded_bytes, encoded_bytes + message_information<TransformStamped>::length,
//         buffer_begin);
//     buffer_begin += message_information<TransformStamped>::length;

//     // copy child frame name
//     *buffer_begin = static_cast<char>(msg->child_frame_id.size());
//     buffer_begin++;
//     std::copy(msg->child_frame_id.begin(), msg->child_frame_id.end(), buffer_begin);

//     // copy frame name
//     *buffer_begin = static_cast<char>(msg->child_frame_id.size());
//     buffer_begin++;
//     std::copy(msg->child_frame_id.begin(), msg->child_frame_id.end(), buffer_begin);
// }

// template<typename Message>
// bool acrobat::serial_bridge::encode(Buffer                            buffer,
//                                     Buffer::iterator&                 buffer_begin,
//                                     typename std::shared_ptr<Message> msg) {
//     // clang-format off
//     // Typical message structure
//     // | id |  timestamp | rotation | translation | child_frame_name_length | child_frame_name |
//     frame_name_length | frame_name |
//     //clang-format on

//     if (std::distance(buffer_begin, buffer.end()) < 1 + message_information<Message>::length())
//         return false;

//     *buffer_begin = message_information<Message>::id();
//     buffer_begin++;
//     encode_impl(buffer, buffer_begin, msg);

//     return true;
// }

// template<>
// struct acrobat::serial_bridge::message_information<geometry_msgs::msg::TransformStamped> {
//     static constexpr char   id     = 0;
//     static constexpr size_t length = 8 * sizeof(double);
// };
