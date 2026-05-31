#include "okmr_controls/accel_control_layer.hpp"

#include <okmr_msgs/msg/control_mode.hpp>

namespace okmr_controls {

AccelControlLayer::AccelControlLayer ()
    : ControlLayerBase ("accel_control_layer", okmr_msgs::msg::ControlMode::ACCELERATION) {
    // Subscribe to acceleration target and velocity target topics
    accel_target_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped> (
        "/accel_target", 10,
        std::bind (&AccelControlLayer::accel_target_callback, this, std::placeholders::_1));

    accel_actual_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped> (
        "/acceleration", 10,
        std::bind (&AccelControlLayer::accel_actual_callback, this, std::placeholders::_1));

    gravity_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped> (
        "/gravity", 10,
        std::bind (&AccelControlLayer::gravity_callback, this, std::placeholders::_1));

    velocity_actual_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "/velocity", 10,
        std::bind (&AccelControlLayer::velocity_actual_callback, this, std::placeholders::_1));

    // Publisher for wrench target
    wrench_target_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped> ("/wrench_target", 10);

    // Initialize messages
    accel_target_.accel.linear.x = accel_target_.accel.linear.y = accel_target_.accel.linear.z =
        0.0;
    accel_target_.accel.angular.x = accel_target_.accel.angular.y = accel_target_.accel.angular.z =
        0.0;

    accel_actual_.accel.linear.x = accel_actual_.accel.linear.y = accel_actual_.accel.linear.z =
        0.0;
    accel_actual_.accel.angular.x = accel_actual_.accel.angular.y = accel_actual_.accel.angular.z =
        0.0;

    velocity_actual_.twist.linear.x = velocity_actual_.twist.linear.y =
        velocity_actual_.twist.linear.z = 0.0;
    velocity_actual_.twist.angular.x = velocity_actual_.twist.angular.y =
        velocity_actual_.twist.angular.z = 0.0;

    // Declare feedforward parameters
    this->declare_parameter ("kmass_x", 0.0);
    this->declare_parameter ("kmass_y", 0.0);
    this->declare_parameter ("kmass_z", 0.0);
    this->declare_parameter ("kbuoyancy", 0.0);
    this->declare_parameter ("kmass_roll", 0.0);
    this->declare_parameter ("kmass_pitch", 0.0);
    this->declare_parameter ("kmass_yaw", 0.0);

    this->declare_parameter ("kdrag_x", 0.0);
    this->declare_parameter ("kdrag_y", 0.0);
    this->declare_parameter ("kdrag_z", 0.0);
    this->declare_parameter ("kdrag_roll", 0.0);
    this->declare_parameter ("kdrag_pitch", 0.0);
    this->declare_parameter ("kdrag_yaw", 0.0);

    // Get initial parameter values
    kmass_x_ = this->get_parameter ("kmass_x").as_double ();
    kmass_y_ = this->get_parameter ("kmass_y").as_double ();
    kmass_z_ = this->get_parameter ("kmass_z").as_double ();
    kbuoyancy_ = this->get_parameter ("kbuoyancy").as_double ();
    kmass_roll_ = this->get_parameter ("kmass_roll").as_double ();
    kmass_pitch_ = this->get_parameter ("kmass_pitch").as_double ();
    kmass_yaw_ = this->get_parameter ("kmass_yaw").as_double ();

    kdrag_x_ = this->get_parameter ("kdrag_x").as_double ();
    kdrag_y_ = this->get_parameter ("kdrag_y").as_double ();
    kdrag_z_ = this->get_parameter ("kdrag_z").as_double ();
    kdrag_roll_ = this->get_parameter ("kdrag_roll").as_double ();
    kdrag_pitch_ = this->get_parameter ("kdrag_pitch").as_double ();
    kdrag_yaw_ = this->get_parameter ("kdrag_yaw").as_double ();

    // Setup parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback (
        std::bind (&AccelControlLayer::parameters_callback, this, std::placeholders::_1));
}

void AccelControlLayer::accel_target_callback (
    const geometry_msgs::msg::AccelStamped::SharedPtr msg) {
    accel_target_ = *msg;
}

void AccelControlLayer::accel_actual_callback (
    const geometry_msgs::msg::AccelStamped::SharedPtr msg) {
    accel_actual_ = *msg;
}

void AccelControlLayer::velocity_actual_callback (
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    velocity_actual_ = *msg;
}

void AccelControlLayer::gravity_callback (const geometry_msgs::msg::AccelStamped::SharedPtr msg) {
    gravity_ = *msg;
}

geometry_msgs::msg::Vector3 AccelControlLayer::calculate_feedforward (
    const geometry_msgs::msg::Vector3& velocity, const geometry_msgs::msg::Vector3& acceleration) {
    geometry_msgs::msg::Vector3 feedforward;

    // Feedforward = Kmass * acceleration + Kdrag * velocity + KBuoyancy * gravity
    feedforward.x =
        kmass_x_ * acceleration.x + kdrag_x_ * velocity.x + kbuoyancy_ * gravity_.accel.linear.x;
    feedforward.y =
        kmass_y_ * acceleration.y + kdrag_y_ * velocity.y + kbuoyancy_ * gravity_.accel.linear.y;
    feedforward.z =
        kmass_z_ * acceleration.z + kdrag_z_ * velocity.z + kbuoyancy_ * gravity_.accel.linear.z;

    return feedforward;
}

geometry_msgs::msg::Vector3 AccelControlLayer::calculate_angular_feedforward (
    const geometry_msgs::msg::Vector3& angular_velocity,
    const geometry_msgs::msg::Vector3& angular_acceleration) {
    geometry_msgs::msg::Vector3 angular_feedforward;

    // Angular feedforward = Kmass * angular_acceleration + Kdrag * angular_velocity
    angular_feedforward.x = kmass_roll_ * angular_acceleration.x + kdrag_roll_ * angular_velocity.x;
    angular_feedforward.y =
        kmass_pitch_ * angular_acceleration.y + kdrag_pitch_ * angular_velocity.y;
    angular_feedforward.z = kmass_yaw_ * angular_acceleration.z + kdrag_yaw_ * angular_velocity.z;

    return angular_feedforward;
}

void AccelControlLayer::update () {
    if (!is_enabled_) return;

    // Calculate acceleration errors
    geometry_msgs::msg::Vector3 linear_error;
    geometry_msgs::msg::Vector3 angular_error;

    linear_error.x = accel_target_.accel.linear.x - accel_actual_.accel.linear.x;
    linear_error.y = accel_target_.accel.linear.y - accel_actual_.accel.linear.y;
    linear_error.z = accel_target_.accel.linear.z - accel_actual_.accel.linear.z;

    angular_error.x = accel_target_.accel.angular.x - accel_actual_.accel.angular.x;
    angular_error.y = accel_target_.accel.angular.y - accel_actual_.accel.angular.y;
    angular_error.z = accel_target_.accel.angular.z - accel_actual_.accel.angular.z;

    // Compute PID control output
    auto pid_output = compute_layer_command (linear_error, angular_error);

    // Calculate feedforward terms
    auto linear_feedforward =
        calculate_feedforward (velocity_actual_.twist.linear, accel_target_.accel.linear);
    auto angular_feedforward =
        calculate_angular_feedforward (velocity_actual_.twist.angular, accel_target_.accel.angular);

    // Combine PID output with feedforward
    geometry_msgs::msg::WrenchStamped wrench_target;
    wrench_target.header.stamp = this->now ();
    wrench_target.header.frame_id = "base_link";

    wrench_target.wrench.force.x = pid_output.first.x + linear_feedforward.x;
    wrench_target.wrench.force.y = pid_output.first.y + linear_feedforward.y;
    wrench_target.wrench.force.z = pid_output.first.z + linear_feedforward.z;

    wrench_target.wrench.torque.x = pid_output.second.x + angular_feedforward.x;
    wrench_target.wrench.torque.y = pid_output.second.y + angular_feedforward.y;
    wrench_target.wrench.torque.z = pid_output.second.z + angular_feedforward.z;

    wrench_target_pub_->publish (wrench_target);
}

rcl_interfaces::msg::SetParametersResult AccelControlLayer::parameters_callback (
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        if (param.get_name () == "kmass_x") {
            kmass_x_ = param.as_double ();
        } else if (param.get_name () == "kmass_y") {
            kmass_y_ = param.as_double ();
        } else if (param.get_name () == "kmass_z") {
            kmass_z_ = param.as_double ();
        } else if (param.get_name () == "kbuoyancy") {
            kbuoyancy_ = param.as_double ();
        } else if (param.get_name () == "kmass_roll") {
            kmass_roll_ = param.as_double ();
        } else if (param.get_name () == "kmass_pitch") {
            kmass_pitch_ = param.as_double ();
        } else if (param.get_name () == "kmass_yaw") {
            kmass_yaw_ = param.as_double ();
        } else if (param.get_name () == "kdrag_x") {
            kdrag_x_ = param.as_double ();
        } else if (param.get_name () == "kdrag_y") {
            kdrag_y_ = param.as_double ();
        } else if (param.get_name () == "kdrag_z") {
            kdrag_z_ = param.as_double ();
        } else if (param.get_name () == "kdrag_roll") {
            kdrag_roll_ = param.as_double ();
        } else if (param.get_name () == "kdrag_pitch") {
            kdrag_pitch_ = param.as_double ();
        } else if (param.get_name () == "kdrag_yaw") {
            kdrag_yaw_ = param.as_double ();
        }
    }

    return result;
}

}  // namespace okmr_controls

int main (int argc, char** argv) {
    rclcpp::init (argc, argv);
    auto node = std::make_shared<okmr_controls::AccelControlLayer> ();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node (node);
    executor.spin ();
    rclcpp::shutdown ();
    return 0;
}
