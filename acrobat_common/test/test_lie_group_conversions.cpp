#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <gtest/gtest.h>

#include <acrobat_common/lie_groups/lie_group_conversions.hpp>

using acrobat::lie_group_conversions::convert;
using acrobat::lie_groups::SE3;
using acrobat::lie_groups::SO3;
using acrobat::lie_groups::Vector3;

namespace {

void compare_quaternions(const geometry_msgs::msg::Quaternion& q1,
                         const geometry_msgs::msg::Quaternion& q2) {
    constexpr double tol = 5e-5;
    EXPECT_NEAR(q1.w, q2.w, tol);
    EXPECT_NEAR(q1.x, q2.x, tol);
    EXPECT_NEAR(q1.y, q2.y, tol);
    EXPECT_NEAR(q1.z, q2.z, tol);
}

void compare_vectors(const geometry_msgs::msg::Vector3& t1, const geometry_msgs::msg::Vector3& t2) {
    constexpr double tol = 5e-5;
    EXPECT_NEAR(t1.x, t2.x, tol);
    EXPECT_NEAR(t1.y, t2.y, tol);
    EXPECT_NEAR(t1.z, t2.z, tol);
}

void compare_points(const geometry_msgs::msg::Point& t1, const geometry_msgs::msg::Point& t2) {
    constexpr double tol = 5e-5;
    EXPECT_NEAR(t1.x, t2.x, tol);
    EXPECT_NEAR(t1.y, t2.y, tol);
    EXPECT_NEAR(t1.z, t2.z, tol);
}

geometry_msgs::msg::Quaternion create_quat() {
    geometry_msgs::msg::Quaternion r1;
    double norm_inv = 1.0 / (0.692 * 0.692 + -0.002 * -0.002 + -0.685 * -0.685 + 0.228 * 0.228);
    r1.w            = 0.692 * norm_inv;
    r1.x            = -0.002 * norm_inv;
    r1.y            = -0.685 * norm_inv;
    r1.z            = 0.228 * norm_inv;
    return r1;
}

geometry_msgs::msg::Vector3 create_vector() {
    geometry_msgs::msg::Vector3 t1;
    t1.x = 100.0;
    t1.y = 231231.1231231;
    t1.z = -2837182731.123123;
    return t1;
}

geometry_msgs::msg::Point create_point() {
    geometry_msgs::msg::Point t1;
    t1.x = 100.0;
    t1.y = 231231.1231231;
    t1.z = -2837182731.123123;
    return t1;
}

TEST(LieGroupConversionsTest, Quaternion) {
    geometry_msgs::msg::Quaternion r1 = create_quat();
    SO3                            r2;

    geometry_msgs::msg::Quaternion result;
    convert(r1, r2);
    convert(r2, result);

    compare_quaternions(r1, result);
} // namespace

TEST(LieGroupConversionsTest, Vector) {
    EXPECT_TRUE(true);

    geometry_msgs::msg::Vector3 t1 = create_vector();
    Vector3                     t2;

    geometry_msgs::msg::Vector3 result;
    convert(t1, t2);
    convert(t2, result);

    compare_vectors(t1, result);
}

TEST(LieGroupConversionsTest, Point) {
    EXPECT_TRUE(true);

    geometry_msgs::msg::Point t1 = create_point();
    Vector3                   t2;

    geometry_msgs::msg::Point result;
    convert(t1, t2);
    convert(t2, result);

    compare_points(t1, result);
}

TEST(LieGroupConversionsTest, Transform) {
    geometry_msgs::msg::Quaternion r1 = create_quat();

    geometry_msgs::msg::Vector3 t1 = create_vector();

    geometry_msgs::msg::Transform p1;
    p1.rotation    = r1;
    p1.translation = t1;
    SE3 p2;

    geometry_msgs::msg::Transform result;
    convert(p1, p2);
    convert(p2, result);

    compare_quaternions(p1.rotation, result.rotation);
    compare_vectors(p1.translation, result.translation);
}

TEST(LieGroupConversionsTest, Pose) {
    geometry_msgs::msg::Quaternion r1 = create_quat();

    geometry_msgs::msg::Point t1 = create_point();

    geometry_msgs::msg::Pose p1;
    p1.orientation = r1;
    p1.position    = t1;
    SE3 p2;

    geometry_msgs::msg::Pose result;
    convert(p1, p2);
    convert(p2, result);

    compare_quaternions(p1.orientation, result.orientation);
    compare_points(p1.position, result.position);
}

} // namespace
