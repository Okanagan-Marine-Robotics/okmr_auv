/*
This is the Dropper Action Server
tomi shittu (main reference: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
07/07/2026

SOMETHING TO NOTE
> indecision (or arguably, foresight,) led to a convoluted code structure that implements both a traditional
  ROS2 action server structure, AND one that calls the execute function directly from a topic callback.
  The definitive desired structure should be discussed, and then the code revised as needed.
  The tradtional action server structure is better because it allows for feedback and cancellation, but the topic 
  callback structure is simpler and more specific to our implementation.

QUESTIONS
> ask where /drop publisher is; make sure communication protocol is consistent
> confirm that the dropper is servo 0
> make sure that redundancy with execution function doesn't cause problems

NEXT STEPS
> mitigate that lazy sleep timer part (likely with parallel thread?)
*/

#include <functional>
#include <memory>
#include <thread>

#include "okmr_msgs/action/dropper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "dropper_action_cpp/visibility_control.h"

namespace dropper_action_cpp
{
    class DropNode : public rclcpp::Node
    {
    public:
        using Dropper = okmr_msgs::action::Dropper;
        using GoalHandleDropper = rclcpp_action::ServerGoalHandle<Dropper>;

        ACTION_TUTORIALS_CPP_PUBLIC
        explicit DropNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("drop_node", options)
        {
            using namespace std::placeholders;
            
            // familiar ROS2 pub/sub structure that we've been using
            
            actuator_pub_ = this->create_publisher<okmr_msgs::msg::ActuatorCommand>("/actuator_command", 10); // subscriber is in clutch_mega_driver
            
            drop_sub_ = this->create_subscription<okmr_msgs::msg::DropCmd>( 
                "/drop", 10, std::bind(&DropNode::drop_topic_callback, this, std::placeholders::_1)); // publisher is probably in the automated planner (?)

            // expected structure for an action server; goal handling, cancellation, and execution 
            this->action_server_ = rclcpp_action::create_server<Dropper>(
                this,
                "dropper",
                std::bind(&DropNode::handle_goal, this, _1, _2),
                std::bind(&DropNode::handle_cancel, this, _1),
                std::bind(&DropNode::handle_accepted, this, _1));

            RCLCPP_INFO(this->get_logger(), "Dropper node initialized");
        }

    private:

        rclcpp_action::Server<Dropper>::SharedPtr action_server_;

        rclcpp::Subscription<okmr_msgs::msg::DropCmd>::SharedPtr drop_sub_;
        rclcpp::Publisher<okmr_msgs::msg::ActuatorCommand>::SharedPtr actuator_pub_;
        
        // GOAL RESPONSE - TO BE REVISED FOR CONSISTENCY
        // Currently accepts all goals
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Dropper::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // CANCEL RESPONSE
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleDropper> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // GOAL HANDLING/EXECUTION
        void handle_accepted(const std::shared_ptr<GoalHandleDropper> goal_handle)
        {
            using namespace std::placeholders;
            // "this needs to return quickly to avoid blocking the executor, so spin up a new thread"
            std::thread{std::bind(&DropNode::execute, this, _1), goal_handle}.detach();
        }

        // this is the main part we care about (the execute function)
        // logic needs to be confirmed and tested
        void execute(const std::shared_ptr<GoalHandleDropper> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(1);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Dropper::Feedback>();
            auto result = std::make_shared<Dropper::Result>();

            DROP_IT(); // helper function that actually sends the actuator command. see below

            result->result = true;
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            goal_handle->succeed(result);
        }

        void drop_topic_callback(const okmr_msgs::msg::DropCmd::SharedPtr msg)
        {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Received /drop topic message");
            DROP_IT();
        }

        /* 
        now, this is the part that actually sends the state and index message. since this program (indecisively) combines the traditional
        action server structure AND the more familiar structure we've been using, this helper function will allow us to 
        execute the actuator command from either the topic callback or the action server callback, whichever one we want!
        */
        void DROP_IT()
        {
            auto actuator_msg = std::make_shared<okmr_msgs::msg::ActuatorCommand>();
            actuator_msg->state = true;
            actuator_msg->index = 0; // Assuming the dropper is servo 0. TO CONFIRM

            actuator_pub_->publish(*actuator_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published actuator commands");

            sleep(1); // LAZY AND BAD, NEEDS IMPROVEMENT
            actuator_msg->state = false;
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
}