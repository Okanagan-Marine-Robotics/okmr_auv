#include <okmr_msgs/msg/actuator_command.hpp>
#include <okmr_msgs/msg/fire_torpedo_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

u_int8_t NUM_ACTUATORS = 2;

class FireToActuatorNode : public rclcpp::Node {
    public:
    FireToActuatorNode () : Node ("actuator_to_fire") {
        fire_torpedo_sub_ = this->create_subscription<okmr_msgs::msg::FireTorpedoCommand> (
            "/fire_torpedo_command", 10,
            std::bind (&FireToActuatorNode::fire_torpedo_callback, this, std::placeholders::_1)
        );

        actuator_pub_ = this->create_publisher<okmr_msgs::msg::ActuatorCommand> (
            "/actuator_command", 10
        );

        RCLCPP_INFO (this->get_logger (), "FireToActuator node initialized");
    }
    private:

    rclcpp::Subscription<okmr_msgs::msg::FireTorpedoCommand>::SharedPtr fire_torpedo_sub_;
    rclcpp::Publisher<okmr_msgs::msg::ActuatorCommand>::SharedPtr actuator_pub_;

    void fire_torpedo_callback (const okmr_msgs::msg::FireTorpedoCommand::SharedPtr msg) {
        auto actuator_msg = std::make_shared<okmr_msgs::msg::ActuatorCommand> ();
        actuator_msg->state = true;

        u_int8_t tube_number = msg->tube_number;
        if (tube_number > NUM_ACTUATORS) {
            RCLCPP_ERROR (this->get_logger (), "Failed to fire torpedo, invalid tube number: %d", tube_number);
            hrow std::runtime_error("Invalid tube number requested!");
        }
        if (tube_number < NUM_ACTUATORS) {
            actuator_msg->index = msg->tube_number;
            actuator_pub_->publish (*actuator_msg);
            return;
        }
        for (u_int8_t i = 0; i < NUM_ACTUATORS; ++i) {
            actuator_msg->index = i;
            actuator_pub_->publish (*actuator_msg);
        }
        RCLCPP_DEBUG (this->get_logger (), "Published actuator commands");
    }
};

int main (int argc, char** argv) {
    rclcpp::init (argc, argv);

    try {
        auto node = std::make_shared<FireToActuatorNode> ();
        rclcpp::spin (node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR (rclcpp::get_logger ("fire_to_actuator"), "Node failed: %s", e.what ());
        rclcpp::shutdown ();
        return 1;
    }

    rclcpp::shutdown ();
    return 0;
}