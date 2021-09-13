#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
// #include 

class OdometrydListener : public rclcpp::Node
{
public:
    explicit OdometrydListener() : Node("Odometryd_listener")
    {
        auto cb = [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
        {
            std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
            std::cout << "=============================" << std::endl;
            std::cout << "vx: " << msg->vx << std::endl;
            std::cout << "vy: " << msg->vy << std::endl;
            std::cout << "vz: " << msg->vz << std::endl;
            // std::cout << "gyro_rad[2]: " << msg->gyro_rad[2] << std::endl;
            // std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
            // std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
            // std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
            // std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
            // std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
            // std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
        };
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/vehicle_odometry/out", 10, cb);
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    std::cout << "Starting listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometrydListener>());

    rclcpp::shutdown();
    return 0;
}
