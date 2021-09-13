#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/actuator_controls4.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Eigen>

#include <px4_controller/util_quad.hpp>

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;
class SetMotor : public rclcpp::Node
{
public:
    typedef Eigen::Matrix<double, 13, 1> State;
    typedef Eigen::Matrix<double, 12, 1> DState;

    SetMotor() : Node("motor_advertiser")
    {
        index = 0;
        startTime = std::chrono::system_clock::now();

        K << -229.40442495852892,-153.5242667670563,-476.0910219001551,-1738.9369303675435,2617.0233234129782,438.66427451972805,-337.37936166503926,-225.35231878094083,-515.2054775098936,-172.06403386068504,260.7585757551678,450.8248294928117,
              229.40442495852497,153.5242667666147,-476.09102190033786,1738.936930366137,-2617.023323412826,438.66427451901376,337.3793616650198,225.3523187806968,-515.2054775099037,172.06403386067677,-260.75857575516693,450.8248294927914,
             -229.4044249584433,153.52426676693858,-476.09102190057985,1738.936930365213,2617.023323412525,-438.6642745191296,-337.3793616649597,225.35231878075982,-515.2054775099245,172.0640338606714,260.7585757551646,-450.82482949279154,
              229.40442495848998,-153.5242667665879,-476.09102189991535,-1738.9369303651924,-2617.0233234128095,-438.66427451970213,337.37936166502016,-225.3523187806546,-515.2054775098709,-172.0640338606711,-260.75857575516676,-450.82482949280904;



        publisher_actuators_ = this->create_publisher<px4_msgs::msg::ActuatorControls4>("/fmu/actuator_controls4/in", 10);
        auto cb = [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
        {
            std::lock_guard<std::mutex> lock(state_mut);

            cur_state(0) = static_cast<double>(msg->x);
            cur_state(1) = static_cast<double>(msg->y);
            cur_state(2) = static_cast<double>(msg->z);
            cur_state(3) = static_cast<double>(msg->q[0]);
            cur_state(4) = static_cast<double>(msg->q[1]);
            cur_state(5) = static_cast<double>(msg->q[2]);
            cur_state(6) = static_cast<double>(msg->q[3]);
            cur_state(7) = static_cast<double>(msg->vx);
            cur_state(8) = static_cast<double>(msg->vy);
            cur_state(9) = static_cast<double>(msg->vz);
            cur_state(10) = static_cast<double>(msg->rollspeed);
            cur_state(11) = static_cast<double>(msg->pitchspeed);
            cur_state(12) = static_cast<double>(msg->yawspeed);
        };
        auto timer_callback =
            [this]() -> void
        {
            {
                std::lock_guard<std::mutex> lock(state_mut);
                delta_state.segment<3>(0) = cur_state.segment<3>(0) - Eigen::Vector3d(0., 0., -2.);
                
                delta_state.segment<3>(3) = std::isnan(cur_state(3)) ? Eigen::Vector3d::Zero() :
                                            UtilQuad::qtorp(cur_state.segment<4>(3));
                delta_state.segment<6>(6) = cur_state.segment<6>(7);
            }

            Eigen::Vector4d controls = Eigen::Vector4d(798.9505102371, 798.9505102371605, 798.9505102371605, 798.9505102371605) - K*delta_state;
            
            if (std::chrono::system_clock::now() - startTime < 3s)
            {
                controls = Eigen::Vector4d(700, 700, 700, 700);
            }
            auto set_actuators = px4_msgs::msg::ActuatorControls4();

            set_actuators.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

            set_actuators.control = {normalize(controls(0)), normalize(controls(1)), normalize(controls(2)), normalize(controls(3))};

            // RCLCPP_INFO(this->get_logger(), "\033[97m Publishing : w1: %f w2: %f w3: %f w4: %f \033[0m",
            //             controls(0), controls(1), controls(2), controls(3));
            RCLCPP_INFO_STREAM(this->get_logger(), "\n" << delta_state);
            
            this->publisher_actuators_->publish(set_actuators);
            // this->index++;
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/vehicle_odometry/out", 1, cb);
    }

    string readFileIntoString(const string &path)
    {
        auto ss = ostringstream{};
        ifstream input_file(path);
        if (!input_file.is_open())
        {
            cerr << "Could not open the file - '" << path << "'" << endl;
            exit(EXIT_FAILURE);
        }
        ss << input_file.rdbuf();
        return ss.str();
    }

    void read_record(const string &file_name)
    {
        string filename(file_name);
        string file_contents;

        file_contents = readFileIntoString(filename);

        istringstream sstream(file_contents);
        string record;

        while (std::getline(sstream, record))
        {
            istringstream line(record);
            std::vector<double> control_input;

            while (std::getline(line, record, ','))
            {
                control_input.push_back(stod(record));
            }

            control_inputs.push_back(control_input);
            control_input.clear();
        }
    }

    float normalize(double input)
    {
        return static_cast<float>(std::max(std::min(input, 1100.0), 0.0) / 1100.0 * 2.0 - 1.0);
    }

public:
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorControls4>::SharedPtr publisher_actuators_;
    unsigned long index;
    std::vector<std::vector<double>> control_inputs;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    State cur_state;
    DState delta_state;
    std::mutex state_mut;

    Eigen::Matrix<double, 4, 12> K;

    std::chrono::time_point<std::chrono::system_clock> startTime;

};

int main(int argc, char *argv[])
{
    std::cout << "Starting advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetMotor>());

    rclcpp::shutdown();
    return 0;
}
