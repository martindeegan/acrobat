#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <rclcpp/rclcpp.hpp>

namespace acrobat::localization {

class Backend {
  public:
    explicit Backend(rclcpp::Logger logger);
    virtual ~Backend();

    using SharedPtr = std::shared_ptr<Backend>;

    static SharedPtr create(rclcpp::Logger logger_);

    virtual void run();
    virtual void stop() noexcept;

    void add_frame();
    void add_map_point();

  private:
    std::atomic_bool running_;

    rclcpp::Logger logger_;

  public:
    gtsam::NonlinearFactorGraph::shared_ptr graph_;
    gtsam::NonlinearOptimizer::shared_ptr   optimizer_;
};

} // namespace acrobat::localization