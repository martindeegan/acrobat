#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <acrobat_common/lie_groups/lie_group_conversions.hpp>

using acrobat::lie_groups::SE3;
using acrobat::lie_groups::SO3;
using acrobat::lie_groups::Vector3;

namespace acrobat::lie_group_conversions {

void convert(const tf2::Quaternion& tf2_quaternion, SO3& acrobat_so3) {
    acrobat_so3.setQuaternion(
        {tf2_quaternion.w(), tf2_quaternion.x(), tf2_quaternion.y(), tf2_quaternion.z()});
}

void convert(const SO3& acrobat_so3, tf2::Quaternion& tf2_quaternion) {
    tf2_quaternion.setW(acrobat_so3.unit_quaternion().w());
    tf2_quaternion.setX(acrobat_so3.unit_quaternion().x());
    tf2_quaternion.setY(acrobat_so3.unit_quaternion().y());
    tf2_quaternion.setZ(acrobat_so3.unit_quaternion().z());
}

void convert(const tf2::Vector3& tf2_vector3, Vector3& acrobat_vector3) {
    acrobat_vector3.x() = tf2_vector3.x();
    acrobat_vector3.y() = tf2_vector3.y();
    acrobat_vector3.z() = tf2_vector3.z();
}

void convert(const Vector3& acrobat_vector3, tf2::Vector3& tf2_vector3) {
    tf2_vector3.setX(acrobat_vector3.x());
    tf2_vector3.setY(acrobat_vector3.y());
    tf2_vector3.setZ(acrobat_vector3.z());
}

void convert(const tf2::Transform& tf2_transform, SE3& acrobat_se3) {
    convert(tf2_transform.getRotation(), acrobat_se3.so3());
    convert(tf2_transform.getOrigin(), acrobat_se3.translation());
}
void convert(const SE3& acrobat_se3, tf2::Transform& tf2_transform) {
    tf2::Quaternion intermediate;
    convert(acrobat_se3.so3(), intermediate);
    tf2_transform.setRotation(intermediate);
    convert(acrobat_se3.translation(), tf2_transform.getOrigin());
}

} // namespace acrobat::lie_group_conversions
