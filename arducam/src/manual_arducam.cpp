#include <memory>

#include "arducam/arducam_driver.hpp"
#include "arducam/image_viewer.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions                       options;

    auto arducam_driver = std::make_shared<ArducamDriver>(options);
    exec.add_node(arducam_driver);

    auto image_viewer = std::make_shared<ImageViewer>(options);
    exec.add_node(image_viewer);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}