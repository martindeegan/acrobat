#pragma once

#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>
#include <sophus/so3.hpp>

#include <acrobat_common/math/math_defines.hpp>

namespace acrobat::lie_groups {

using Scalar = math::Scalar;

using Vector3    = Eigen::Matrix<Scalar, 3, 1>;
using Quaternion = Eigen::Quaternion<Scalar>;

using SO3  = Sophus::SO3<Scalar>;
using SE3  = Sophus::SE3<Scalar>;
using SIM3 = Sophus::Sim3<Scalar>;

} // namespace acrobat::lie_groups
