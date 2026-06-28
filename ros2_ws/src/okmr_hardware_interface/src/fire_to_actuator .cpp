#include <cmath>
#include <lazycsv.hpp>
#include <okmr_msgs/msg/battery_voltage.hpp>
#include <okmr_msgs/msg/actuator_command.hpp>
#include <okmr_msgs/msg/fire_torpedo_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

class FireToActuator : public rclcpp::Node {
    public:
    FireToActuator () : Node ("actuator_to_fire") {
        this->declar_parameter<std::string> ("actuator_curve_file", "");

        std::string csv_file_path = this->get_parameter ("actuator_curve_file").as_string();
        if(csv_file_path.empty()) {
            RCLCPP_ERROR (this->get_logger (), "actuator_curve_file parameter not set");
            return;
        }

        load_actuator_curve (csv_file_path);

        fire_torpedo_sub = this->create_subscription<okmr_msgs::msg::FireTorpedoCommand> (
            "/fire_torpedo_command", 10,
            std::bind (&FireToActuator::fire_torpedo_callback, this, std::placeholders::_1)
        );

        voltage_sub_ = this->create_subscription<okmr_msgs::msg::BatteryVoltage> (
            "/voltage", 10,
            std::bind (&FireToActuator::voltage_callback, this, std::placeholders::_1)
        );

        actuator_pub_ = this->create_publisher<okmr_msgs::msg::ActuatorCommand> (
            "/actuator_command", 10
        );

        RCLCPP_INFO (this->get_logger (), "FireToActuator node initialized");
    }

    std::map<float, std::map<float>> actuator_curve_map_;
    float current_voltage_ = 16.0f;

    rclcpp::Subscription<okmr_msgs::msg::FireTorpedoCommand>::SharedPtr fire_torpedo_sub_;
    rclcpp::Subscription<okmr_msgs::msg::BatteryVoltage>::SharedPtr voltage_sub_;
    
}