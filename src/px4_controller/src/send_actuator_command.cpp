#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/actuator_controls4.hpp>

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;
using namespace std;

class SetMotor : public rclcpp::Node
{
public:
    SetMotor() : Node("motor_advertiser")
    {
        index = 0;
        read_record("/workspaces/ros2_ws/controls.csv");

        publisher_actuators_ = this->create_publisher<px4_msgs::msg::ActuatorControls4>("/fmu/actuator_controls4/in", 10);
        auto timer_callback =
            [this]() -> void
        {
            if (this->index >= this->control_inputs.size())
            {
                RCLCPP_INFO(this->get_logger(),"End");
                rclcpp::shutdown();
            }
            auto set_actuators = px4_msgs::msg::ActuatorControls4();

            set_actuators.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

            set_actuators.control = {normalize(this->control_inputs[index][0]), normalize(this->control_inputs[index][1]), normalize(control_inputs[index][2]), normalize(control_inputs[index][3])};

            RCLCPP_INFO(this->get_logger(), "\033[97m Publishing : time: %llu x: %f y: %f z: %f \033[0m",
                        set_actuators.timestamp, set_actuators.control[0], set_actuators.control[1], set_actuators.control[2]);
            this->publisher_actuators_->publish(set_actuators);
            this->index++;
        };
        timer_ = this->create_wall_timer(15ms, timer_callback);
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
        return static_cast<float>(std::min(input, 1100.0) / 1100.0 * 2.0 - 1.0);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorControls4>::SharedPtr publisher_actuators_;
    unsigned long index;
    std::vector<std::vector<double>> control_inputs;
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
