/*
This is the Dropper Action Server
tomi shittu (main reference: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
07/07/2026

SOME NOTES
> subscriber actuator removed (commented out); you have to use the traditional action server interface now
> the REAL goal handling is done at the beginning of the execution function. this way we can receive descriptive exit statuses
  that tell us why we arent dropping
> no goal canceling cuz we'd never need it. can be implemented later if some need is found 
>>>> IF goal canceling is implemented, then we have to replace the sleep(1) with ROS2 TimerBase
> i kinda superhard referenced the visibility file and the CMakeLists; if there are compilation issues during testing,
  these are the first places id re-assess

REAL NEXT STEPS - COMP
> DONE - DROP_IT should return false if anything fails, and then the bool should be passed to the result msg
> DONE - implement a temp variable for ball count; whenever sub turns back on it resets to 0. if count = 2, NOBALLS
> DONE -  put 'already firing' block in goal handling instead of drop_it()
> NVM - implement real goal acceptance and cancellation
> DONE & NVM - implement feedback and response
*/

#include <functional>
#include <memory>
#include <thread>
#include <cstdint>
#include <mutex>

#include "okmr_msgs/action/dropper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" 

#include "dropper_action_cpp/visibility_control.h"

constexpr uint8_t SERVO_INDEX = 4; // constexpr = basically a static variable
                                   // servo index matches firmware.ino
constexpr uint8_t MAX_BALL_COUNT = 2;


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
            
            actuator_pub_ = this->create_publisher<okmr_msgs::msg::ActuatorCommand>("/actuator_command", 10); // subscriber is in clutch_mega_driver
            //drop_sub_ = this->create_subscription<okmr_msgs::msg::DropCmd>( 
            //    "/drop", 10, std::bind(&DropNode::drop_topic_callback, this, std::placeholders::_1)); // publisher is in the automated planner

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
        int balls_dropped = 0; // pause asff
        rclcpp_action::Server<Dropper>::SharedPtr action_server_;

        //rclcpp::Subscription<okmr_msgs::msg::DropCmd>::SharedPtr drop_sub_;
        rclcpp::Publisher<okmr_msgs::msg::ActuatorCommand>::SharedPtr actuator_pub_;
        
        // atomic friend
        std::mutex drop_mutex_;  

        // GOAL RESPONSE - Currently accepts all goals
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Dropper::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->dropper_state);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // CANCEL RESPONSE - Rejects all cancel requests (no cancellation logic implemented)
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleDropper> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::REJECT;
        }

        // GOAL HANDLING/EXECUTION
        void handle_accepted(const std::shared_ptr<GoalHandleDropper> goal_handle)
        {
            using namespace std::placeholders;
            // "this needs to return quickly to avoid blocking the executor, so spin up a new thread"
            std::thread{std::bind(&DropNode::execute, this, _1), goal_handle}.detach();
        }

        // this is the main part we care about (paired with the DROP_IT helper function)
        void execute(const std::shared_ptr<GoalHandleDropper> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing Dropper Action");
            const auto goal = goal_handle->get_goal();

            auto feedback = std::make_shared<Dropper::Feedback>();
            auto result = std::make_shared<Dropper::Result>();

            // implementing atomicity (RAII WRAPPER) to prevent race condition between threads
            std::unique_lock<std::mutex> lock(drop_mutex_, std::try_to_lock);
            if (!lock.owns_lock()) {
                RCLCPP_WARN(this->get_logger(), "Drop already in progress. Rejecting goal");
                result->exit_status = Dropper::Result::TIMEOUT;
                goal_handle->abort(result);
                return;
            }

            if (balls_dropped >= MAX_BALL_COUNT) {
                result->exit_status = Dropper::Result::NOBALLS;
                feedback->current_status = Dropper::Feedback::idle;
                RCLCPP_WARN(this->get_logger(), "No more balls");
                goal_handle->abort(result);
                goal_handle->publish_feedback(feedback);
                return;
            } 

            feedback->current_status = Dropper::Feedback::dropping;
            goal_handle->publish_feedback(feedback);
            
            if (!DROP_IT()) {
                result->exit_status = Dropper::Result::TIMEOUT;
                RCLCPP_ERROR(this->get_logger(), "Goal failed");
                goal_handle->abort(result);
                return;
            } 

            result->exit_status = Dropper::Result::SUCCESS;
            feedback->current_status = Dropper::Feedback::idle;
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            goal_handle->succeed(result);
            goal_handle->publish_feedback(feedback); 
            balls_dropped++;
        }

        /*  SUBSCRIBER ACUATOR; DONT USE THIS ANYMORE! but keeping it in case of testing etc
        void drop_topic_callback(const okmr_msgs::msg::DropCmd::SharedPtr msg)
        {
            // CHECK BALL COUNT
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Received /drop topic message");
            std::thread(&DropNode::DROP_IT, this).detach();
        }
        */ 

        // the part that actually sends the state and index message
        bool DROP_IT()
        {
            auto actuator_msg = std::make_shared<okmr_msgs::msg::ActuatorCommand>();
            actuator_msg->state = true;
            actuator_msg->index = SERVO_INDEX;

            actuator_pub_->publish(*actuator_msg);
            RCLCPP_DEBUG(this->get_logger(), "DROPPER ON - Command Published");

            sleep(1);
            actuator_msg->state = false;
            actuator_pub_->publish(*actuator_msg);
            RCLCPP_DEBUG(this->get_logger(), "DROPPER OFF - Command Published");

            return true;
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