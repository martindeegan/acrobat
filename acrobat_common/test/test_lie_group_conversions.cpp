#include <gtest/gtest.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <acrobat_common/lie_groups/lie_group_conversions.hpp>

using acrobat::lie_group_conversions::convert;
using acrobat::lie_groups::SE3;
using acrobat::lie_groups::SO3;
using acrobat::lie_groups::Vector3;

namespace {

void compare_quaternions(const tf2::Quaternion& q1, const tf2::Quaternion& q2) {
    EXPECT_NEAR(q1.getW(), q2.getW(), 1e-6);
    EXPECT_NEAR(q1.getX(), q2.getX(), 1e-6);
    EXPECT_NEAR(q1.getY(), q2.getY(), 1e-6);
    EXPECT_NEAR(q1.getZ(), q2.getZ(), 1e-6);
}

void compare_vectors(const tf2::Vector3& t1, const tf2::Vector3& t2) {
    EXPECT_NEAR(t1.getX(), t2.getX(), 1e-6);
    EXPECT_NEAR(t1.getY(), t2.getY(), 1e-6);
    EXPECT_NEAR(t1.getZ(), t2.getZ(), 1e-6);
}

TEST(LieGroupConversionsTest, Quaternion) {
    tf2::Quaternion r1{0.692, -0.002, -0.685, 0.228};
    r1.normalize();
    SO3 r2;

    tf2::Quaternion result;
    convert(r1, r2);
    convert(r2, result);

    compare_quaternions(r1, result);
}

TEST(LieGroupConversionsTest, Vector) {
    EXPECT_TRUE(true);

    tf2::Vector3 t1{100.0, 231231.1231231, -2837182731.123123};
    Vector3      t2;

    tf2::Vector3 result;
    convert(t1, t2);
    convert(t2, result);

    compare_vectors(t1, result);
}

TEST(LieGroupConversionsTest, Pose) {
    tf2::Quaternion r1{0.692, -0.002, -0.685, 0.228};
    tf2::Vector3    t1{100.0, 231231.1231231, -2837182731.123123};
    tf2::Transform  p1{r1, t1};
    SE3             p2;

    tf2::Transform result;
    convert(p1, p2);
    convert(p2, result);

    compare_quaternions(p1.getRotation(), result.getRotation());
    compare_vectors(p1.getOrigin(), result.getOrigin());
}

} // namespace
