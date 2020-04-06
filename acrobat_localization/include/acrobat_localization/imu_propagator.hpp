#pragma once

#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <acrobat_common/lie_groups/lie_groups.hpp>

namespace acrobat::localization {

struct Sample {
    rclcpp::Time             stamp;
    lie_groups::SE3          pose;
    lie_groups::SE3::Tangent velocity;
    lie_groups::SE3::Tangent imu_sample;
};

using Samples = std::vector<Sample>;

class ImuPropagator {
  public:
    using SharedPtr = std::shared_ptr<ImuPropagator>;

    static SharedPtr create();

    void set_bias(const lie_groups::SE3::Tangent& bias);
    void set_gravity(const lie_groups::Vector3& gravity);

    const Sample& add_measurement(const rclcpp::Time&        stamp,
                                  lie_groups::SE3::Tangent&& measurement);

    void update_latest_pose(const rclcpp::Time&             stamp,
                            const lie_groups::SE3&          pose,
                            const lie_groups::SE3::Tangent& velocity);

  private:
    ImuPropagator();

    void integrate(Samples::iterator prev, Samples::iterator next);

    bool first_sample_;

    lie_groups::SE3::Tangent bias_;
    lie_groups::Vector3      gravity_;

    Samples samples_;
};

} // namespace acrobat::localization