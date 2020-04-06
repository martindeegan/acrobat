#include <rclcpp/rclcpp.hpp>

#include <acrobat_localization/visual_odometry.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions                              options;
    rclcpp::executors::SingleThreadedExecutor        exec;
    acrobat::localization::VisualOdometry::SharedPtr odometry(
        new acrobat::localization::VisualOdometry(options));

    exec.add_node(odometry);
    exec.spin();
}