#pragma once

#include <msp/Client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <acrobat_common/composition/visibility_control.hpp>
#include <acrobat_msgs/msg/acrobat_control.hpp>
#include <acrobat_msgs/msg/betaflight_motor.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace msp {

namespace msg {

class RawImu;
class Motor;

} // namespace msg

} // namespace msp

namespace acrobat::msp_bridge {

class MspBridge : public rclcpp::Node {
  public:
    ACROBAT_COMPOSITION_PUBLIC

    explicit MspBridge(const rclcpp::NodeOptions& options);

    ~MspBridge();

  private:
    void on_imu(const msp::msg::RawImu& imu);
    void on_motor(const msp::msg::Motor& motor);
    void on_control(const acrobat_msgs::msg::AcrobatControl::SharedPtr msg);

    sensor_msgs::msg::Imu              imu_msg_;
    acrobat_msgs::msg::BetaflightMotor motor_msg_;

    std::string         device_;
    size_t              baudrate_;
    msp::client::Client msp_client_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                imu_pub_;
    rclcpp::Publisher<acrobat_msgs::msg::BetaflightMotor>::SharedPtr   motor_pub_;
    rclcpp::Subscription<acrobat_msgs::msg::AcrobatControl>::SharedPtr control_sub_;
};

} // namespace acrobat::msp_bridge
