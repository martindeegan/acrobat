
#include <acrobat_common/lie_groups/lie_group_conversions.hpp>

using acrobat::lie_groups::SE3;
using acrobat::lie_groups::SO3;
using acrobat::lie_groups::Vector3;

namespace acrobat::lie_group_conversions {

void convert(const geometry_msgs::msg::Quaternion& tf2_quaternion, SO3& acrobat_so3) {
    acrobat_so3.setQuaternion(
        {tf2_quaternion.w, tf2_quaternion.x, tf2_quaternion.y, tf2_quaternion.z});
}

void convert(const SO3& acrobat_so3, geometry_msgs::msg::Quaternion& tf2_quaternion) {
    tf2_quaternion.w = acrobat_so3.unit_quaternion().w();
    tf2_quaternion.x = acrobat_so3.unit_quaternion().x();
    tf2_quaternion.y = acrobat_so3.unit_quaternion().y();
    tf2_quaternion.z = acrobat_so3.unit_quaternion().z();
}

void convert(const geometry_msgs::msg::Vector3& tf2_vector3, Vector3& acrobat_vector3) {
    acrobat_vector3.x() = tf2_vector3.x;
    acrobat_vector3.y() = tf2_vector3.y;
    acrobat_vector3.z() = tf2_vector3.z;
}

void convert(const Vector3& acrobat_vector3, geometry_msgs::msg::Vector3& tf2_vector3) {
    tf2_vector3.x = acrobat_vector3.x();
    tf2_vector3.y = acrobat_vector3.y();
    tf2_vector3.z = acrobat_vector3.z();
}

void convert(const geometry_msgs::msg::Transform& tf2_transform, SE3& acrobat_se3) {
    convert(tf2_transform.rotation, acrobat_se3.so3());
    convert(tf2_transform.translation, acrobat_se3.translation());
}
void convert(const SE3& acrobat_se3, geometry_msgs::msg::Transform& tf2_transform) {
    convert(acrobat_se3.so3(), tf2_transform.rotation);
    convert(acrobat_se3.translation(), tf2_transform.translation);
}

void convert(const geometry_msgs::msg::Point& point_msg, Vector3& acrobat_vector3) {
    acrobat_vector3.x() = point_msg.x;
    acrobat_vector3.y() = point_msg.y;
    acrobat_vector3.z() = point_msg.z;
}

void convert(const Vector3& acrobat_vector3, geometry_msgs::msg::Point& point_msg) {
    point_msg.x = acrobat_vector3.x();
    point_msg.y = acrobat_vector3.y();
    point_msg.z = acrobat_vector3.z();
}

void convert(const geometry_msgs::msg::Pose& pose_msg, SE3& acrobat_se3) {
    convert(pose_msg.orientation, acrobat_se3.so3());
    convert(pose_msg.position, acrobat_se3.translation());
}

void convert(const SE3& acrobat_se3, geometry_msgs::msg::Pose& pose_msg) {
    convert(acrobat_se3.so3(), pose_msg.orientation);
    convert(acrobat_se3.translation(), pose_msg.position);
}

} // namespace acrobat::lie_group_conversions
