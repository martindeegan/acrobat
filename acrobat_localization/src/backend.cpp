#include <acrobat_common/time/rate.hpp>
#include <acrobat_localization/backend.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <memory>

namespace acrobat::localization {

Backend::Backend(rclcpp::Logger logger)
    : logger_(logger), graph_(new gtsam::NonlinearFactorGraph) {}

Backend::SharedPtr Backend::create(rclcpp::Logger logger) {
    return std::make_shared<Backend>(logger);
}

Backend::~Backend() {}

void Backend::run() {
    time::Rate loop_rate(100);
    while (rclcpp::ok() && running_) { loop_rate.sleep(); }
}

void Backend::stop() noexcept { running_ = false; }

} // namespace acrobat::localization
