#include <acrobat_localization/imu_propagator.hpp>

using acrobat::lie_groups::SE3;
using acrobat::lie_groups::Vector3;
using Tangent = acrobat::lie_groups::SE3::Tangent;

namespace acrobat::localization {

ImuPropagator::SharedPtr ImuPropagator::create(rclcpp::Logger logger) {
    return std::make_shared<ImuPropagator>(logger);
}

ImuPropagator::ImuPropagator(rclcpp::Logger logger) : logger_(logger) {}

State ImuPropagator::add_sample(rclcpp::Time&& stamp, ImuSample&& sample) {
    State state;
    return state;
}

} // namespace acrobat::localization