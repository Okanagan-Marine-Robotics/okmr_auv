#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "okmr_msgs/msg/control_mode.hpp"
#include "okmr_msgs/msg/goal_pose.hpp"
#include "okmr_msgs/msg/movement_command.hpp"
#include "okmr_msgs/msg/relative_pose.hpp"
#include "okmr_msgs/msg/sensor_reading.hpp"
#include "okmr_msgs/srv/distance_from_goal.hpp"
#include "okmr_msgs/srv/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RelativePoseTargetServer : public rclcpp::Node {
   public:
    RelativePoseTargetServer () : Node ("relative_pose_target_server") {
        // Declare parameters
        this->declare_parameter ("update_frequency", 100.0);
        this->declare_parameter ("holding_radius", 0.5);
        this->declare_parameter ("yaw_tolerance", 15.0);  // degrees
        update_frequency_ = this->get_parameter ("update_frequency").as_double ();
        holding_radius_ = this->get_parameter ("holding_radius").as_double ();
        yaw_tolerance_ = this->get_parameter ("yaw_tolerance").as_double ();

        // Parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback (std::bind (
            &RelativePoseTargetServer::on_parameter_change, this, std::placeholders::_1));

        goal_pose_subscription_ = this->create_subscription<okmr_msgs::msg::GoalPose> (
            "/current_goal_pose", 10,
            std::bind (&RelativePoseTargetServer::goal_pose_callback, this, _1));

        current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped> (
            "/pose", 10, std::bind (&RelativePoseTargetServer::current_pose_callback, this, _1));

        // Create separate callback groups
        control_mode_callback_group_ =
            this->create_callback_group (rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ =
            this->create_callback_group (rclcpp::CallbackGroupType::MutuallyExclusive);

        // Subscribe to control mode with separate callback group
        auto sub_options = rclcpp::SubscriptionOptions ();
        sub_options.callback_group = control_mode_callback_group_;

        control_mode_subscription_ = this->create_subscription<okmr_msgs::msg::ControlMode> (
            "/control_mode", 10,
            std::bind (&RelativePoseTargetServer::control_mode_callback, this, _1), sub_options);

        distance_from_goal_service_ = this->create_service<okmr_msgs::srv::DistanceFromGoal> (
            "distance_from_pose_goal",
            std::bind (&RelativePoseTargetServer::distance_from_goal_callback, this,
                       std::placeholders::_1, std::placeholders::_2));

        // Publisher for relative pose target
        relative_pose_pub_ =
            this->create_publisher<okmr_msgs::msg::RelativePose> ("/relative_pose_target", 10);

        // Timer for regular updates
        auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds> (
            std::chrono::duration<double> (1.0 / update_frequency_));

        timer_ = this->create_wall_timer (timer_period,
                                          std::bind (&RelativePoseTargetServer::update, this),
                                          timer_callback_group_);
    }

   private:
    void goal_pose_callback (const okmr_msgs::msg::GoalPose::SharedPtr msg) {
        current_goal_pose_msg_ = *msg;
        hold_mode_ = false;
        current_goal_eulers_ = euler_from_quaternion (msg->pose.orientation);
    }

    void current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_msg_ = *msg;
    }

    void control_mode_callback (const okmr_msgs::msg::ControlMode::SharedPtr msg) {
        RCLCPP_DEBUG (this->get_logger (),
                      "RelativePoseTargetServer: Control mode callback received: %d",
                      msg->control_mode);

        bool was_enabled = is_enabled_;
        is_enabled_ = (msg->control_mode == okmr_msgs::msg::ControlMode::POSE);

        RCLCPP_DEBUG (this->get_logger (), "RelativePoseTargetServer: enabled: %s -> %s",
                      was_enabled ? "true" : "false", is_enabled_ ? "true" : "false");

        if (!was_enabled && is_enabled_) {
            // Mode changed to pose - start timer and set goal to current pose for safety
            RCLCPP_DEBUG (this->get_logger (), "Pose mode enabled, starting timer");
            current_goal_pose_msg_.pose = current_pose_msg_.pose;

            auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds> (
                std::chrono::duration<double> (1.0 / update_frequency_));

            timer_ = this->create_wall_timer (timer_period,
                                              std::bind (&RelativePoseTargetServer::update, this),
                                              timer_callback_group_);
        }
    }

    void distance_from_goal_callback (
        const std::shared_ptr<okmr_msgs::srv::DistanceFromGoal::Request> request,
        std::shared_ptr<okmr_msgs::srv::DistanceFromGoal::Response> response) {
        (void)request;

        response->translation_differences =
            calculate_relative_translation (current_pose_msg_.pose, current_goal_pose_msg_.pose);

        if (current_goal_pose_msg_.copy_orientation) {
            response->orientation_differences =
                calculate_euler_differences (current_pose_msg_.pose, current_goal_pose_msg_.pose);
        } else {
            response->orientation_differences = geometry_msgs::msg::Vector3 ();
        }
    }

    geometry_msgs::msg::Vector3 euler_from_quaternion (const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion quat;
        tf2::fromMsg (q, quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3 (quat).getRPY (roll, pitch, yaw);

        geometry_msgs::msg::Vector3 eulers;
        eulers.x = roll * (180.0 / M_PI);
        eulers.y = pitch * (180.0 / M_PI);
        eulers.z = yaw * (180.0 / M_PI);
        return eulers;
    }

    geometry_msgs::msg::Vector3 calculate_euler_differences (
        const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Pose& goal_pose) {
        auto current_eulers = euler_from_quaternion (current_pose.orientation);
        auto goal_eulers = euler_from_quaternion (goal_pose.orientation);

        geometry_msgs::msg::Vector3 result;
        result.x = goal_eulers.x - current_eulers.x;
        result.y = goal_eulers.y - current_eulers.y;
        result.z = goal_eulers.z - current_eulers.z;

        return result;
    }

    geometry_msgs::msg::Vector3 calculate_relative_translation (
        const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Pose& goal_pose) {
        tf2::Transform tf_current, tf_goal;
        tf2::fromMsg (current_pose, tf_current);
        tf2::fromMsg (goal_pose, tf_goal);

        tf2::Transform tf_current_inv = tf_current.inverse ();
        tf2::Transform tf_goal_relative = tf_current_inv * tf_goal;
        tf2::Vector3 translation = tf_goal_relative.getOrigin ();

        geometry_msgs::msg::Vector3 relative_translation;
        relative_translation.x = translation.x ();
        relative_translation.y = translation.y ();
        relative_translation.z = translation.z ();

        return relative_translation;
    }

    void update () {
        if (!is_enabled_) {
            // When pose is disabled, set goal pose to current pose
            current_goal_pose_msg_.pose = current_pose_msg_.pose;
            return;
        }

        float yaw = 0.0;
        float pitch = 0.0;
        float roll = 0.0;

        geometry_msgs::msg::Vector3 relative_translation =
            calculate_relative_translation (current_pose_msg_.pose, current_goal_pose_msg_.pose);

        float xy_trig_dist = sqrt (relative_translation.x * relative_translation.x +
                                   relative_translation.y * relative_translation.y);

        if (xy_trig_dist < holding_radius_) {
            if (!hold_mode_) {
                hold_mode_ = true;
                hold_yaw_ = euler_from_quaternion (current_pose_msg_.pose.orientation).z;
            }

            if (current_goal_pose_msg_.copy_orientation) {
                hold_yaw_ = current_goal_eulers_.z;
                pitch = current_goal_eulers_.y;
                roll = current_goal_eulers_.x;
            }
            yaw = hold_yaw_;
        } else {
            yaw =
                atan2 (current_goal_pose_msg_.pose.position.y - current_pose_msg_.pose.position.y,
                       current_goal_pose_msg_.pose.position.x - current_pose_msg_.pose.position.x) *
                (180.0 / M_PI);
        }

        // Create and publish RelativePose target message
        okmr_msgs::msg::RelativePose relative_pose_target;

        // Calculate yaw error
        auto current_eulers = euler_from_quaternion (current_pose_msg_.pose.orientation);
        double yaw_error = (yaw - current_eulers.z);

        // Normalize yaw error to [-180, 180]
        while (yaw_error > 180.0) yaw_error -= 360.0;
        while (yaw_error < -180.0) yaw_error += 360.0;

        //bool yaw_on_target = std::abs (yaw_error) <= yaw_tolerance_;

        // Always correct Z (depth) regardless of yaw -- gating it caused buoyancy-driven rise
        // during yaw correction phases when Z error was zeroed.
        relative_pose_target.translation.z = relative_translation.z;

        // Only command XY translation once yaw is aligned (avoids crabbing sideways to goal)
        /*if (yaw_on_target || xy_trig_dist < holding_radius_) {
            relative_pose_target.translation.x = relative_translation.x;
            relative_pose_target.translation.y = relative_translation.y;
        } Trying something diff */

        relative_pose_target.translation.x = relative_translation.x * std::max(0.0, cos(yaw_error * M_PI / 180.0 )); // surely?
        relative_pose_target.translation.y = relative_translation.y * std::max(0.0, cos(yaw_error * M_PI / 180.0 ));

        relative_pose_target.rotation.x = (roll - current_eulers.x);
        relative_pose_target.rotation.y = (pitch - current_eulers.y);
        //relative_pose_target.rotation.x = -current_eulers.x;  // forced 0 on roll
        //relative_pose_target.rotation.y = -current_eulers.y;  // forced 0 on pitch
        relative_pose_target.rotation.z = yaw_error;

        relative_pose_pub_->publish (relative_pose_target);
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change (
        const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name () == "update_frequency") {
                update_frequency_ = param.as_double ();
                if (update_frequency_ <= 0.0) {
                    result.successful = false;
                    result.reason = "Update frequency must be positive";
                    return result;
                }

                // Update timer period
                auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds> (
                    std::chrono::duration<double> (1.0 / update_frequency_));

                timer_->cancel ();
                timer_ = this->create_wall_timer (
                    timer_period, std::bind (&RelativePoseTargetServer::update, this),
                    timer_callback_group_);
            } else if (param.get_name () == "holding_radius") {
                holding_radius_ = param.as_double ();
                if (holding_radius_ <= 0.0) {
                    result.successful = false;
                    result.reason = "Holding radius must be positive";
                    return result;
                }
            } else if (param.get_name () == "yaw_tolerance") {
                yaw_tolerance_ = param.as_double ();
                if (yaw_tolerance_ <= 0.0) {
                    result.successful = false;
                    result.reason = "Yaw tolerance must be positive";
                    return result;
                }
            }
        }

        return result;
    }

    // Member Variables
    float hold_yaw_ = 0.0;
    bool hold_mode_ = false;
    double update_frequency_;
    double holding_radius_;
    double yaw_tolerance_;
    bool is_enabled_ = false;

    okmr_msgs::msg::GoalPose current_goal_pose_msg_;
    geometry_msgs::msg::PoseStamped current_pose_msg_;
    geometry_msgs::msg::Vector3 current_goal_eulers_;

    rclcpp::Subscription<okmr_msgs::msg::GoalPose>::SharedPtr goal_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
    rclcpp::Subscription<okmr_msgs::msg::ControlMode>::SharedPtr control_mode_subscription_;

    rclcpp::Service<okmr_msgs::srv::DistanceFromGoal>::SharedPtr distance_from_goal_service_;

    rclcpp::Publisher<okmr_msgs::msg::RelativePose>::SharedPtr relative_pose_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::CallbackGroup::SharedPtr control_mode_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main (int argc, char* argv[]) {
    rclcpp::init (argc, argv);
    auto node = std::make_shared<RelativePoseTargetServer> ();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node (node);
    executor.spin ();
    rclcpp::shutdown ();
    return 0;
}
