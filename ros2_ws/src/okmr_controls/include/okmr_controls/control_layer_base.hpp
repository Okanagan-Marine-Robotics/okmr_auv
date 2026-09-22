#ifndef OKMR_CONTROLS_CONTROL_LAYER_BASE_HPP
#define OKMR_CONTROLS_CONTROL_LAYER_BASE_HPP

#include <array>
#include <geometry_msgs/msg/vector3.hpp>
#include <okmr_msgs/msg/control_mode.hpp>
#include <okmr_msgs/msg/pid_debug.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "okmr_controls/pid_controller.hpp"

namespace okmr_controls {

class ControlLayerBase : public rclcpp::Node {
   public:
    ControlLayerBase (const std::string& node_name, int8_t control_mode);

    virtual ~ControlLayerBase ();

   protected:
    bool is_enabled_;
    virtual void update () = 0;

    std::pair<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3> compute_layer_command (
        const geometry_msgs::msg::Vector3& linear_error,
        const geometry_msgs::msg::Vector3& angular_error);

    void reset_controllers ();

    virtual void declare_parameters ();

    rcl_interfaces::msg::SetParametersResult on_parameter_change (
        const std::vector<rclcpp::Parameter>& parameters);

   private:
    enum ControllerIndex {
        X_LINEAR = 0,
        Y_LINEAR = 1,
        Z_LINEAR = 2,
        X_ANGULAR = 3,
        Y_ANGULAR = 4,
        Z_ANGULAR = 5
    };

    std::array<PidController, 6> controllers_;
    std::array<rclcpp::Publisher<okmr_msgs::msg::PidDebug>::SharedPtr, 6> debug_pubs_;

    rclcpp::TimerBase::SharedPtr update_timer_;
    std::chrono::nanoseconds last_update_time_;
    double update_frequency_;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Subscription<okmr_msgs::msg::ControlMode>::SharedPtr control_mode_sub_;
    int8_t required_control_mode_;
    int8_t current_control_mode_;

    rclcpp::CallbackGroup::SharedPtr control_mode_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    void control_mode_callback (const okmr_msgs::msg::ControlMode::SharedPtr msg);
    void load_controller_parameters (ControllerIndex idx, const std::string& prefix);
    void update_controller_gains (ControllerIndex idx, const std::string& prefix);
    void update_controller_gains_from_params (ControllerIndex idx, const std::string& prefix,
                                              const std::vector<rclcpp::Parameter>& parameters);
};

}  // namespace okmr_controls

#endif  // OKMR_CONTROLS_CONTROL_LAYER_BASE_HPP
