#include <chrono>
#include <functional>

#include <msp/Client.hpp>
#include <msp/msp_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <msp_bridge/msp_bridge.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace acrobat::msp_bridge {

MspBridge::MspBridge(const rclcpp::NodeOptions& options) : Node("msp_bridge", options) {
    declare_parameter("device");
    declare_parameter("baudrate");
    declare_parameter("imu_frequency");
    declare_parameter("motor_frequency");

    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    device_      = parameter_client->get_parameter<std::string>("device");
    baudrate_    = parameter_client->get_parameter<size_t>("baudrate");
    bool success = msp_client_.start(device_, baudrate_);
    if (!success) {
        RCLCPP_ERROR(get_logger(), "Could not connect to flight controller through MSP.");
        rclcpp::shutdown();
    }

    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>("/acrobat/fc_imu", 2);
    motor_pub_   = create_publisher<acrobat_msgs::msg::BetaflightMotor>("/acrobat/motors", 2);
    control_sub_ = create_subscription<acrobat_msgs::msg::AcrobatControl>(
        "/acrobat/control", 2, std::bind(&MspBridge::on_control, this, _1));

    auto imu_frequency   = parameter_client->get_parameter<double>("imu_frequency");
    auto motor_frequency = parameter_client->get_parameter<double>("motor_frequency");
    msp_client_.subscribe(&MspBridge::on_motor, this, 1.0 / motor_frequency);
    msp_client_.subscribe(&MspBridge::on_imu, this, 1.0 / imu_frequency);

    imu_msg_.header.frame_id = "/acrobat/fc";
}

MspBridge::~MspBridge() {
    msp_client_.stop();
}

void MspBridge::on_imu(const msp::msg::RawImu& imu) {
    imu_msg_.header.stamp = rclcpp::Time();

    msp::msg::ImuSI converted(imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);

    imu_msg_.angular_velocity.x = converted.gyro[0];
    imu_msg_.angular_velocity.y = converted.gyro[1];
    imu_msg_.angular_velocity.z = converted.gyro[2];

    imu_msg_.linear_acceleration.x = converted.acc[0];
    imu_msg_.linear_acceleration.y = converted.acc[1];
    imu_msg_.linear_acceleration.z = converted.acc[2];

    imu_pub_->publish(imu_msg_);
}

void MspBridge::on_motor(const msp::msg::Motor& motor) {
    motor_msg_.stamp = rclcpp::Time();

    motor_msg_.thrust[0] = static_cast<float>(motor.motor[0]);
    motor_msg_.thrust[1] = static_cast<float>(motor.motor[1]);
    motor_msg_.thrust[2] = static_cast<float>(motor.motor[2]);
    motor_msg_.thrust[3] = static_cast<float>(motor.motor[3]);

    motor_pub_->publish(motor_msg_);
}

void MspBridge::on_control(const acrobat_msgs::msg::AcrobatControl::SharedPtr msg) {
    // Do nothing for now
}

} // namespace acrobat::msp_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::msp_bridge::MspBridge)
