#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <acrobat_msgs/msg/betaflight_motor.hpp>
#include <arducam/arducam_driver.hpp>
#include <arducam/image_viewer.hpp>
#include <msp_bridge/msp_bridge.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace {

class AcrobatParameters : public rclcpp::Node {
  public:
    explicit AcrobatParameters(const rclcpp::NodeOptions& options)
        : Node("acrobat_parameters", options) {
        declare_parameter<bool>("use_arducam", false);
        declare_parameter<bool>("use_image_viewer", false);
        declare_parameter<bool>("use_msp_bridge", false);
    }
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions                      options;

    auto acrobat_parameters = std::make_shared<AcrobatParameters>(options);
    auto parameter_client   = std::make_shared<rclcpp::SyncParametersClient>(acrobat_parameters);

    bool use_arducam      = parameter_client->get_parameter<bool>("use_arducam");
    bool use_image_viewer = parameter_client->get_parameter<bool>("use_image_viewer");
    bool use_msp_bridge   = parameter_client->get_parameter<bool>("use_msp_bridge");

    std::shared_ptr<acrobat::arducam::ArducamDriver>    arducam_driver = nullptr;
    std::shared_ptr<acrobat::image_viewer::ImageViewer> image_viewer   = nullptr;
    std::shared_ptr<acrobat::msp_bridge::MspBridge>     msp_bridge     = nullptr;

    if (use_arducam) {
        arducam_driver = std::make_shared<acrobat::arducam::ArducamDriver>(options);
        exec.add_node(arducam_driver);
    }

    if (use_image_viewer) {
        image_viewer = std::make_shared<acrobat::image_viewer::ImageViewer>(options);
        exec.add_node(image_viewer);
    }

    if (use_msp_bridge) {
        msp_bridge = std::make_shared<acrobat::msp_bridge::MspBridge>(options);
        exec.add_node(msp_bridge);
    }

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
