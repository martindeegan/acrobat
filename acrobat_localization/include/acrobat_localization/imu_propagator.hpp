#pragma once

#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <acrobat_common/lie_groups/lie_groups.hpp>

namespace acrobat::localization {

using ImuSample = lie_groups::SE3::Tangent;

struct State {
    rclcpp::Time             stamp;
    lie_groups::SE3          state;
    lie_groups::SE3::Tangent velocity;
};

class ImuPropagator {
  public:
    using SharedPtr = std::shared_ptr<ImuPropagator>;

    explicit ImuPropagator(rclcpp::Logger logger);
    static SharedPtr create(rclcpp::Logger logger);

    State add_sample(rclcpp::Time&& stamp, ImuSample&& sample);

  private:
    rclcpp::Logger logger_;
};

} // namespace acrobat::localization