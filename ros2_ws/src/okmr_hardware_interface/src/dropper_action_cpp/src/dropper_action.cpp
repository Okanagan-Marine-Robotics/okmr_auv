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
    TLDR; WE CAN FREELY ACTUATE THE DROPPER THROUGH EITHER THE ACTION SERVER OR TOPIC CALLBACK,
    INTERCHANGEABLY WITHOUT CONCERN FOR RACE CONDITIONS OR REDUNDACY

> i kinda superhard referenced the visibility file and the CMakeLists; if there are compilation issues during testing,
  these are the first places id re-assess

QUESTIONS
> ask where /drop publisher is; make sure communication protocol is consistent
> confirm that the dropper is servo 0

NEXT STEPS
> consider replacing sleep(1) with ROS2 TimerBase (not very necessary given parallel thread implementation)
> feedback publishing could be improved if desired
*/

#include <functional>
#include <memory>
#include <thread>
#include <atomic>

#include "okmr_msgs/action/dropper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "dropper_action_cpp/visibility_control.h"

u_int8_t SERVO_INDEX = 0;


namespace dropper_action_cpp
{
    class DropNode : public rclcpp::Node
    {
    public:
        using Dropper = okmr_msgs::action::Dropper;
        using GoalHandleDropper = rclcpp_action::ServerGoalHandle<Dropper>;

        DROPPER_ACTION_CPP_PUBLIC
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
        
        std::atomic<bool> dropping_{false}; // used for atomicity

        // GOAL RESPONSE - TO BE REVISED FOR CONSISTENCY
        // Currently accepts all goals
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Dropper::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->dropper_state);
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

            DROP_IT();

            result->exit_status = Dropper::Result::SUCCESS;
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            goal_handle->succeed(result);
        }

        
        void drop_topic_callback(const okmr_msgs::msg::DropCmd::SharedPtr msg)
        {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Received /drop topic message");
            std::thread(&DropNode::DROP_IT, this).detach();
        }

        /* 
        now, this is the part that actually sends the state and index message. since this program (indecisively) combines the traditional
        action server structure AND the more familiar structure we've been using, this helper function will allow us to 
        execute the actuator command from either the topic callback or the action server callback, whichever one we want!
        */
        void DROP_IT()
        {
            // implementing mutex to prevent race condition between threads
            bool expected = false;
            if (!dropping_.compare_exchange_strong(expected, true)) {
                RCLCPP_WARN(this->get_logger(), "Drop already in progress, ignoring");
                return;
            }

            auto actuator_msg = std::make_shared<okmr_msgs::msg::ActuatorCommand>();
            actuator_msg->state = true;
            actuator_msg->index = SERVO_INDEX;

            actuator_pub_->publish(*actuator_msg);
            RCLCPP_DEBUG(this->get_logger(), "DROPPER ON - Command Published");

            sleep(1);
            actuator_msg->state = false;
            actuator_pub_->publish(*actuator_msg);
            RCLCPP_DEBUG(this->get_logger(), "DROPPER OFF - Command Published");

            dropping_ = false;
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