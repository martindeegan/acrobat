#pragma once

#include <string>

namespace acrobat::reference_frames {

// Gravity aligned world reference frame. Arbitrary point on Earth
constexpr std::string_view rf_world = "world";

// Acrobat reference frames
// body       - Center of mass
// fc         - Betaflight board IMU
// imu        - Jetson carrier board IMU
// camera     - Arducam
// ir_markers - Rigid body defined by motion caputre IR markers
constexpr std::string_view rf_acrobat_body       = "acrobat_body";
constexpr std::string_view rf_acrobat_fc         = "acrobat_fc";
constexpr std::string_view rf_acrobat_imu        = "acrobat_imu";
constexpr std::string_view rf_acrobat_camera     = "acrobat_camera";
constexpr std::string_view rf_acrobat_ir_markers = "acrobat_IR_markers";

// Motion capture reference frames
constexpr std::string_view rf_motion_capture_origin     = "motion_capture_origin";
constexpr std::string_view rf_motion_capture_ir_markers = "motion_capture_ir_markers";

} // namespace acrobat::reference_frames