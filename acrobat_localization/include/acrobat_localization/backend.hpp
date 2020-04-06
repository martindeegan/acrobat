#pragma once

#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace acrobat::localization {

class Backend {
  public:
    explicit Backend(rclcpp::Logger logger);

    using SharedPtr = std::shared_ptr<Backend>;

    static SharedPtr create(rclcpp::Logger logger_);

    virtual void run();
    virtual void stop() noexcept;

  private:
    std::atomic_bool running_;

    rclcpp::Logger logger_;
};

} // namespace acrobat::localization