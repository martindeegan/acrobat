#pragma once

#include <string>

namespace acrobat::topics {

constexpr std::string_view camera_topic   = "/acrobat/camera";
constexpr std::string_view imu_topic      = "/acrobat/imu";
constexpr std::string_view fc_imu_topic   = "/acrobat/fc_imu";
constexpr std::string_view fc_motor_topic = "/acrobat/fc_motor";
constexpr std::string_view control_topic  = "/acrobat/control";

} // namespace acrobat::topics