#include <okmr_msgs/msg/actuator_command.hpp>
#include <okmr_msgs/action/torpedo.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <mutex>
#include <cstdint>

// Local visibility macro until we decide how to resolve visibility
#ifndef FIRE_TO_TORPEDO_CPP_PUBLIC
  #if defined _WIN32 || defined __CYGWIN__
    #define FIRE_TO_TORPEDO_CPP_PUBLIC __declspec(dllexport)
  #else
    #define FIRE_TO_TORPEDO_CPP_PUBLIC __attribute__ ((visibility("default")))
  #endif
#endif

constexpr uint8_t NUM_ACTUATORS = 2;

class FireToActuatorNode : public rclcpp::Node {
public:
    using FireTorpedo = okmr_msgs::action::Torpedo;
    using GoalHandleFireTorpedo = rclcpp_action::ServerGoalHandle<FireTorpedo>;

    FIRE_TO_TORPEDO_CPP_PUBLIC
    explicit FireToActuatorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : Node("fire_to_actuator_node", options) 
    {
        using namespace std::placeholders;

        actuator_pub_ = this->create_publisher<okmr_msgs::msg::ActuatorCommand>("/actuator_command", 10);

        this->action_server_ = rclcpp_action::create_server<FireTorpedo>(
            this,
            "fire_torpedo",
            std::bind(&FireToActuatorNode::handle_goal, this, _1, _2),
            std::bind(&FireToActuatorNode::handle_cancel, this, _1),
            std::bind(&FireToActuatorNode::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "FireToActuator Action Server initialized");
    }

private:
    rclcpp_action::Server<FireTorpedo>::SharedPtr action_server_;
    rclcpp::Publisher<okmr_msgs::msg::ActuatorCommand>::SharedPtr actuator_pub_;
    std::mutex fire_mutex_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FireTorpedo::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received torpedo fire request for tube: %d", goal->tube_number);
        (void)uuid;

        if (goal->tube_number > NUM_ACTUATORS) {
            RCLCPP_WARN(this->get_logger(), "Rejected: Invalid tube number requested: %d", goal->tube_number);
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::SUCCESS;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFireTorpedo> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel torpedo fire sequence");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /*
        This detached thread can currently cause undefined behaviour if Node is destrudcted mid-sequence.
        Should not be a major issue but will be refactored once i firgure out a better way.
    */ 
    void handle_accepted(const std::shared_ptr<GoalHandleFireTorpedo> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&FireToActuatorNode::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFireTorpedo> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing torpedo firing sequence");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FireTorpedo::Feedback>();
        auto result = std::make_shared<FireTorpedo::Result>();

        std::unique_lock<std::mutex> lock(fire_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            RCLCPP_WARN(this->get_logger(), "Firing sequence already in progress! Aborting request.");
            result->exit_status = FireTorpedo::Result::TIMEOUT;
            goal_handle->abort(result);
            return;
        }

        uint8_t tube_number = goal->tube_number;

        auto fire_tube = [&](uint8_t index) -> bool {
            if (goal_handle->is_canceling()) {
                return false;
            }

            auto actuator_msg = std::make_shared<okmr_msgs::msg::ActuatorCommand>();
            
            actuator_msg->index = index;
            actuator_msg->state = true;
            actuator_pub_->publish(*actuator_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            actuator_msg->state = false;
            actuator_pub_->publish(*actuator_msg);

            return true;
        };

        bool success = true;
        
        if (tube_number < NUM_ACTUATORS) {
            feedback->current_status = FireTorpedo::Feedback::FIRING;
            goal_handle->publish_feedback(feedback);
            success = fire_tube(tube_number);
        } else {
            for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) {
                feedback->current_status = FireTorpedo::Feedback::FIRING;
                goal_handle->publish_feedback(feedback);
                
                if (!fire_tube(i)) {
                    success = false;
                    break;
                }
            }
        }

        if (success) {
            result->exit_status = FireTorpedo::Result::SUCCESS;
            feedback->current_status = FireTorpedo::Feedback::IDLE;
            RCLCPP_INFO(this->get_logger(), "Torpedo firing sequence succeeded");
            goal_handle->succeed(result);
        } else {
            result->exit_status = FireTorpedo::Result::TIMEOUT;
            feedback->current_status = FireTorpedo::Feedback::IDLE;
            RCLCPP_WARN(this->get_logger(), "Torpedo firing sequence was cancelled or failed");
            goal_handle->canceled(result);
        }
        
        goal_handle->publish_feedback(feedback);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<FireToActuatorNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("fire_to_actuator"), "Node failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}