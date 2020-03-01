#pragma once

#include <acrobat_common/lie_groups/lie_groups.hpp>

namespace tf2 {

class Quaternion;
class Vector3;
class Transform;

} // namespace tf2

namespace acrobat::lie_group_conversions {

// Converts between the TF2 quaternion message the Sophus SO3 object
void convert(const tf2::Quaternion& tf2_quaternion, lie_groups::SO3& acrobat_so3);
void convert(const lie_groups::SO3& acrobat_so3, tf2::Quaternion& tf2_quaternion);

// Converts between the TF2 Vector3 message the Eigen Vector3 object
void convert(const tf2::Vector3& tf2_vector3, lie_groups::Vector3& acrobat_vector3);
void convert(const lie_groups::Vector3& acrobat_vector3, tf2::Vector3& tf2_vector3);

// Converts between the TF2 quaternion message the Sophus SO3 object
void convert(const tf2::Transform& tf2_transform, lie_groups::SE3& acrobat_se3);
void convert(const lie_groups::SE3& acrobat_se3, tf2::Transform& tf2_transform);

} // namespace acrobat::lie_group_conversions
