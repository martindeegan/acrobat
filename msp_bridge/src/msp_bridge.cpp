#include <chrono>

#include <msp/Client.hpp>
#include <msp/msp_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <acrobat_common/composition/visibility_control.hpp>

using namespace std::chrono_literals;

namespace acrobat::msp_bridge {

class MspBridge : public rclcpp::Node {
  public:
    ACROBAT_COMPOSITION_PUBLIC

    MspBridge() : Node("msp_bridge") {
        std::string device   = "/dev/ttyUSB0";
        size_t      baudrate = 115200;
        msp_client_.start(device, baudrate);

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/acrobat/fc_imu", 10);

        msp_client_.subscribe(&MspBridge::on_motor, this, 0.01);
        msp_client_.subscribe(&MspBridge::on_imu, this, 0.005);

        imu_msg_.header.frame_id = "/acrobat/fc";
    }

    ~MspBridge() {
        msp_client_.stop();
    }

  private:
    void on_imu(const msp::msg::RawImu& imu) {
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

    void on_motor(const msp::msg::Motor& motor) {
        // Do nothing for now
    }

    sensor_msgs::msg::Imu                               imu_msg_;
    msp::client::Client                                 msp_client_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

} // namespace acrobat::msp_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    acrobat::msp_bridge::MspBridge bridge;

    rclcpp::WallRate loop_rate(100ms);
    while (rclcpp::ok()) {
        loop_rate.sleep();
    }
}