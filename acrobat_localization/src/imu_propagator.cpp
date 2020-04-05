#include <acrobat_localization/imu_propagator.hpp>

using acrobat::lie_groups::SE3;
using acrobat::lie_groups::Vector3;
using Tangent = acrobat::lie_groups::SE3::Tangent;

namespace acrobat::localization {

ImuPropagator::ImuPropagator(const Tangent& bias, const Vector3& gravity)
    : first_sample_(true), bias_(bias), gravity_(gravity) {
    Sample first_sample;
    first_sample.stamp      = rclcpp::Time(0.0);
    first_sample.pose       = SE3{};
    first_sample.velocity   = Tangent::Zero();
    first_sample.imu_sample = Tangent::Zero();
}

const Sample& ImuPropagator::add_measurement(const rclcpp::Time&        stamp,
                                             lie_groups::SE3::Tangent&& imu_sample) {
    Sample new_sample;
    new_sample.stamp      = stamp;
    new_sample.imu_sample = imu_sample;

    if (first_sample_) {
        new_sample.pose     = SE3{};
        new_sample.velocity = Tangent::Zero();
        first_sample_       = false;
    }

    samples_.push_back(new_sample);
    if (samples_.size() >= 2) {
        auto next = samples_.end() - 1;
        auto prev = next - 1;
        integrate(prev, next);
    }

    return samples_.back();
}

void ImuPropagator::update_latest_pose(const rclcpp::Time& stamp,
                                       const SE3&          pose,
                                       const Tangent&      velocity) {
    // Find location in sample vector
    auto it = samples_.begin();
    for (; it != samples_.end() && it->stamp < stamp; it++)
        ;

    it->pose     = pose;
    it->velocity = velocity;
    it->stamp    = stamp;

    Samples new_samples(it, samples_.end());

    auto prev_it = new_samples.begin();
    auto next_it = new_samples.begin() + 1;
    for (; next_it != new_samples.end(); prev_it++, next_it++) {
        integrate(prev_it, next_it);
    }
}

void ImuPropagator::integrate(Samples::iterator prev, Samples::iterator next) {
    const double dt = (next->stamp - prev->stamp).seconds();

    const auto&   R = prev->pose.so3();
    const Tangent acceleration;
    acceleration.topRows<3>() =
        prev->imu_sample.topRows<3>() - bias_.topRows<3>() - R.inverse() * gravity_;

    next->velocity = prev->velocity + acceleration * dt;
    next->pose     = prev->pose * SE3::exp(prev->velocity * dt + acceleration * 0.5 * dt * dt);
}

} // namespace acrobat::localization