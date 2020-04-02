#pragma once

#include <geometry_msgs/msg/transform.hpp>

#include <acrobat_common/lie_groups/lie_groups.hpp>

namespace acrobat::lie_group_conversions {

// Converts between the TF2 quaternion message the Sophus SO3 object
void convert(const geometry_msgs::msg::Quaternion& tf2_quaternion, lie_groups::SO3& acrobat_so3);
void convert(const lie_groups::SO3& acrobat_so3, geometry_msgs::msg::Quaternion& tf2_quaternion);

// Converts between the TF2 Vector3 message the Eigen Vector3 object
void convert(const geometry_msgs::msg::Vector3& tf2_vector3, lie_groups::Vector3& acrobat_vector3);
void convert(const lie_groups::Vector3& acrobat_vector3, geometry_msgs::msg::Vector3& tf2_vector3);

// Converts between the TF2 quaternion message the Sophus SO3 object
void convert(const geometry_msgs::msg::Transform& tf2_transform, lie_groups::SE3& acrobat_se3);
void convert(const lie_groups::SE3& acrobat_se3, geometry_msgs::msg::Transform& tf2_transform);

} // namespace acrobat::lie_group_conversions
