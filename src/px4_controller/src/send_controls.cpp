#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

using namespace std::chrono_literals;
using namespace std;

class SendControl : public rclcpp::Node
{
public:
    SendControl() : Node("control_advertiser")
    {

        publisher_thrust_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("VehicleThrustSetpoint_PubSubTopic", 10);
        publisher_torque_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>("VehicleTorqueSetpoint_PubSubTopic", 10);
        auto timer_callback =
            [this]() -> void
        {
            if (index >= control_inputs.size()) 
            {
                rclcpp::shutdown();
            }

            auto set_thrust = px4_msgs::msg::VehicleThrustSetpoint();
            auto set_torque = px4_msgs::msg::VehicleTorqueSetpoint();

            set_torque.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
            set_thrust.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
            
            std::array<float, 3> thrust{0.0, 0.0, static_cast<float>(control_inputs[index][3])};
            set_thrust.set__xyz(thrust);

            std::array<float, 3> torque{static_cast<float>(control_inputs[index][0]), static_cast<float>(control_inputs[index][1]), static_cast<float>(control_inputs[index][2])};
            set_torque.set__xyz(torque);

            // RCLCPP_INFO(this->get_logger(), "\033[97m Publishing : time: %llu x: %f y: %f z: %f \033[0m",
            //             set_thrust.timestamp, thrust[0], thrust[1], thrust[2]);
            this->publisher_thrust_->publish(set_thrust);
            this->publisher_torque_->publish(set_torque);
            index++;
        };

        read_record("/workspaces/ros2_ws/controls.csv");
        for (const std::vector<double> &i : control_inputs)
        {
            for (double c : i)
            {
                cout << c << ",";
            }
            cout << endl;
        }
        
        index = 0;
        timer_ = this->create_wall_timer(4ms, timer_callback);
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

private:
    long unsigned int index;
    std::vector<std::vector<double>> control_inputs;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr publisher_thrust_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr publisher_torque_;
};

int main(int argc, char *argv[])
{
    std::cout << "Starting advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SendControl>());

    rclcpp::shutdown();
    return 0;
}
