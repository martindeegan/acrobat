#include <acrobat_common/time/rate.hpp>
#include <acrobat_localization/backend.hpp>

namespace acrobat::localization {

Backend::Backend(rclcpp::Logger logger) : logger_(logger) {}

Backend::SharedPtr Backend::create(rclcpp::Logger logger) {
    return std::make_shared<Backend>(logger);
}

void Backend::run() {
    time::Rate loop_rate(100);
    while (rclcpp::ok() && running_) { loop_rate.sleep(); }
}

void Backend::stop() noexcept { running_ = false; }

} // namespace acrobat::localization
