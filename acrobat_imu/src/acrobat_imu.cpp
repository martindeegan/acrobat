#include <chrono>

#include <IMUDrivers/RTIMUMPU9250.h>
#include <RTIMULib.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;
using namespace std::chrono;

namespace acrobat::imu {
class AcrobatImu : public rclcpp::Node {
  public:
    AcrobatImu(const rclcpp::NodeOptions& options)
        : Node("acrobat_imu", options), sample_count_(0) {
        declare_parameter<int>("sample_rate", 100);
        declare_parameter<unsigned char>("/acrobat_imu/SPIBus", 1);
        declare_parameter<unsigned char>("/acrobat_imu/SPISelect", 0);
        declare_parameter<int>("/acrobat_imu/SPISpeed", 1000000);

        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

        const auto             sample_rate = parameters_client->get_parameter<int>("sample_rate");
        const duration<double> sample_period =
            duration<double>(1) / static_cast<double>(sample_rate);
        RCLCPP_INFO(get_logger(), "Sampling at %i Hz", sample_rate);
        RCLCPP_INFO(get_logger(), "Sampling period: %f s", sample_period.count());

        settings_ = new RTIMUSettings();
        settings_->setDefaults();
        settings_->m_busIsI2C = false;
        settings_->m_SPIBus =
            parameters_client->get_parameter<unsigned char>("/acrobat_imu/SPIBus");
        settings_->m_SPISelect =
            parameters_client->get_parameter<unsigned char>("/acrobat_imu/SPISelect");
        settings_->m_SPISpeed   = parameters_client->get_parameter<int>("/acrobat_imu/SPISpeed");
        settings_->m_imuType    = RTIMU_TYPE_MPU9250;
        settings_->m_fusionType = 0;
        settings_->m_MPU9250GyroAccelSampleRate = (8 * sample_rate) / 10;
        settings_->m_MPU9250GyroFsr             = MPU9250_GYROFSR_1000;
        settings_->m_MPU9250AccelFsr            = MPU9250_ACCELFSR_8;
        settings_->m_MPU9250GyroLpf             = MPU9250_GYRO_LPF_250;
        settings_->m_MPU9250GyroLpf             = MPU9250_ACCEL_LPF_460;

        imu_ = RTIMU::createIMU(settings_);
        if (!imu_ || (imu_->IMUType() == RTIMU_TYPE_NULL)) {
            RCLCPP_ERROR(get_logger(), "Could not create IMU");
            rclcpp::shutdown();
        }

        if (!imu_->IMUInit()) {
            RCLCPP_ERROR(get_logger(), "Could not initialize the IMU");
            rclcpp::shutdown();
        }

        imu_->setAccelEnable(true);
        imu_->setGyroEnable(true);

        imu_msg_.header.frame_id = "acrobat_imu";
        publisher_               = create_publisher<sensor_msgs::msg::Imu>("/acrobat/imu", 10);
        timer_ = create_wall_timer(sample_period, std::bind(&AcrobatImu::read_imu, this));
    }

    ~AcrobatImu() {
        delete imu_;
        delete settings_;
    }

    void read_imu() {
        imu_msg_.header.stamp = rclcpp::Time();
        imu_->IMURead();

        imu_msg_.linear_acceleration.x = imu_->getAccel().x();
        imu_msg_.linear_acceleration.y = imu_->getAccel().y();
        imu_msg_.linear_acceleration.z = imu_->getAccel().z();

        imu_msg_.angular_velocity.x = imu_->getGyro().x();
        imu_msg_.angular_velocity.y = imu_->getGyro().y();
        imu_msg_.angular_velocity.z = imu_->getGyro().z();

        publisher_->publish(imu_msg_);
        sample_count_++;
    }

  private:
    RTIMUSettings*                                      settings_;
    RTIMU*                                              imu_;
    sensor_msgs::msg::Imu                               imu_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr                        timer_;

    size_t sample_count_;
}; // namespace acrobat::imu
} // namespace acrobat::imu

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    // Initialize SPI
    rclcpp::executors::SingleThreadedExecutor exec;
    acrobat::imu::AcrobatImu::SharedPtr       imu_node =
        std::make_shared<acrobat::imu::AcrobatImu>(options);

    exec.add_node(imu_node);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}