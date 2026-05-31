#pragma once

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "control_layer_base.hpp"

namespace okmr_controls {

class AccelControlLayer : public ControlLayerBase {
   public:
    AccelControlLayer ();

   protected:
    void update () override;

   private:
    void accel_target_callback (const geometry_msgs::msg::AccelStamped::SharedPtr msg);
    void accel_actual_callback (const geometry_msgs::msg::AccelStamped::SharedPtr msg);
    void gravity_callback (const geometry_msgs::msg::AccelStamped::SharedPtr msg);
    void velocity_actual_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    // Feedforward calculation
    geometry_msgs::msg::Vector3 calculate_feedforward (
        const geometry_msgs::msg::Vector3& velocity,
        const geometry_msgs::msg::Vector3& acceleration);

    geometry_msgs::msg::Vector3 calculate_angular_feedforward (
        const geometry_msgs::msg::Vector3& angular_velocity,
        const geometry_msgs::msg::Vector3& angular_acceleration);

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr accel_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr accel_actual_sub_;
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr gravity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_actual_sub_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_target_pub_;

    // Current state
    geometry_msgs::msg::AccelStamped accel_target_;
    geometry_msgs::msg::AccelStamped accel_actual_;
    geometry_msgs::msg::AccelStamped gravity_;
    geometry_msgs::msg::TwistStamped velocity_actual_;

    // Feedforward parameters (Kmass and Kdrag for each axis)
    double kbuoyancy_;
    double kmass_x_, kmass_y_, kmass_z_;
    double kmass_roll_, kmass_pitch_, kmass_yaw_;
    double kdrag_x_, kdrag_y_, kdrag_z_;
    double kdrag_roll_, kdrag_pitch_, kdrag_yaw_;

    // Parameter callback
    rcl_interfaces::msg::SetParametersResult parameters_callback (
        const std::vector<rclcpp::Parameter>& parameters);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace okmr_controls
