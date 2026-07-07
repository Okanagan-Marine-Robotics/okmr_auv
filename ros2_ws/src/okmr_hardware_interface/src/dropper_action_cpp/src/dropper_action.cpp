/*
This is the Dropper Action Server
tomi shittu (main reference: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
6/29/2026
*/

#include <functional>
#include <memory>
#include <thread>

#include "okmr_msgs/action/dropper.hpp" // okmr_msgs::action::Dropper
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "dropper_action_cpp/visibility_control.h"


// ------------ CONFIDENCE ABOVE. REVISION PENDING BELOW ------------


class DropNode : public rclcpp::Node
{
public:
    DropNode() : Node("drop_node")
    {
        actuator_pub_ = this->create_publisher<okmr_msgs::msg::ActuatorCommand>("/actuator_command", 10);

        drop_sub_ = this->create_subscription<okmr_msgs::msg::DropCmd>(
            "/drop", 10, std::bind(&DropNode::drop_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dropper node initialized");
    }

private:
    rclcpp::Subscription<okmr_msgs::msg::DropCmd>::SharedPtr drop_sub_;
    rclcpp::Publisher<okmr_msgs::msg::ActuatorCommand>::SharedPtr actuator_pub_;

    void drop_callback(const okmr_msgs::msg::DropCmd::SharedPtr msg)
    {
        auto actuator_msg = std::make_shared<okmr_msgs::msg::ActuatorCommand>();
        actuator_msg->state = true;
        actuator_msg->index = 0; // Assuming the dropper is servo 0. TO CONFIRM

        actuator_pub_->publish(*actuator_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published actuator commands");

        sleep(1); // LAZY AND BAD, NEEDS IMPROVEMENT
        actuator_msg->state = false;
        return;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<DropNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("drop"), "Node failed: %s", e.what());
        rclcpp::shutdown();
    }
    rclcpp::shutdown();
    return 0;
}