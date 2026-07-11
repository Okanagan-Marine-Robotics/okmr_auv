#include "okmr_controls/control_layer_base.hpp"

#include <chrono>

namespace okmr_controls {

ControlLayerBase::ControlLayerBase (const std::string& node_name, int8_t control_mode)
    : Node (node_name),
      is_enabled_ (false),
      last_update_time_ (std::chrono::steady_clock::now ().time_since_epoch ()),
      update_frequency_ (50.0),
      required_control_mode_ (control_mode),
      current_control_mode_ (okmr_msgs::msg::ControlMode::OFF) {
    declare_parameters ();

    const std::vector<std::string> axis_names = {"x_linear",  "y_linear",  "z_linear",
                                                 "x_angular", "y_angular", "z_angular"};
    for (size_t i = 0; i < axis_names.size (); ++i) {
        debug_pubs_[i] = this->create_publisher<okmr_msgs::msg::PidDebug> (
            "~/" + axis_names[i] + "/pid_debug", 10);
    }

    param_callback_handle_ = this->add_on_set_parameters_callback (
        std::bind (&ControlLayerBase::on_parameter_change, this, std::placeholders::_1));

    // Create separate callback groups
    control_mode_callback_group_ =
        this->create_callback_group (rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ =
        this->create_callback_group (rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscribe to control mode with separate callback group
    auto sub_options = rclcpp::SubscriptionOptions ();
    sub_options.callback_group = control_mode_callback_group_;

    control_mode_sub_ = this->create_subscription<okmr_msgs::msg::ControlMode> (
        "/control_mode", 10,
        std::bind (&ControlLayerBase::control_mode_callback, this, std::placeholders::_1),
        sub_options);

    // Initialize update timer with separate callback group
    auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds> (
        std::chrono::duration<double> (1.0 / update_frequency_));

    update_timer_ = this->create_wall_timer (
        timer_period, std::bind (&ControlLayerBase::update, this), timer_callback_group_);
}

ControlLayerBase::~ControlLayerBase () {}

std::pair<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3>
ControlLayerBase::compute_layer_command (const geometry_msgs::msg::Vector3& linear_error,
                                         const geometry_msgs::msg::Vector3& angular_error) {
    auto current_time = std::chrono::steady_clock::now ().time_since_epoch ();
    auto dt = current_time - last_update_time_;
    last_update_time_ = current_time;

    geometry_msgs::msg::Vector3 linear_output;
    geometry_msgs::msg::Vector3 angular_output;

    // Linear controllers
    linear_output.x = controllers_[X_LINEAR].compute_command (linear_error.x, dt);
    linear_output.y = controllers_[Y_LINEAR].compute_command (linear_error.y, dt);
    linear_output.z = controllers_[Z_LINEAR].compute_command (linear_error.z, dt);

    // Angular controllers
    angular_output.x = controllers_[X_ANGULAR].compute_command (angular_error.x, dt);
    angular_output.y = controllers_[Y_ANGULAR].compute_command (angular_error.y, dt);
    angular_output.z = controllers_[Z_ANGULAR].compute_command (angular_error.z, dt);

    for (size_t i = 0; i < controllers_.size (); ++i) {
        auto s = controllers_[i].get_state ();
        okmr_msgs::msg::PidDebug dbg;
        dbg.error = s.error;
        dbg.p_term = s.p_term;
        dbg.i_term = s.i_term;
        dbg.d_term = s.d_term;
        dbg.output = s.output;
        debug_pubs_[i]->publish (dbg);
    }

    return std::make_pair (linear_output, angular_output);
}

void ControlLayerBase::control_mode_callback (const okmr_msgs::msg::ControlMode::SharedPtr msg) {
    int8_t new_mode = msg->control_mode;

    RCLCPP_DEBUG (this->get_logger (),
                  "Control mode callback received: %d (required: %d, current: %d)", new_mode,
                  required_control_mode_, current_control_mode_);

    if (new_mode != current_control_mode_) {
        bool was_enabled = is_enabled_;
        current_control_mode_ = new_mode;
        is_enabled_ = (current_control_mode_ <= required_control_mode_);
        // control mode 0 (pose) is only enabled when its pose mode
        // but control mode 5 (throttle) is always enabled, even if its control mode 1,2,3,etc
        // thats why we use the <= sign, to keep downstream controllers on

        RCLCPP_DEBUG (this->get_logger (), "Control mode changed: %d -> %d, enabled: %s -> %s",
                      current_control_mode_, new_mode, was_enabled ? "true" : "false",
                      is_enabled_ ? "true" : "false");

        if (was_enabled && !is_enabled_) {
            RCLCPP_DEBUG (this->get_logger (), "Control layer disabled, resetting controllers");
            reset_controllers ();
        } else if (!was_enabled && is_enabled_) {
            RCLCPP_DEBUG (this->get_logger (), "Control layer enabled");
        }
    } else {
        RCLCPP_DEBUG (this->get_logger (), "Control mode unchanged: %d", new_mode);
    }
}

void ControlLayerBase::reset_controllers () {
    for (auto& controller : controllers_) {
        controller.reset ();
    }
}

void ControlLayerBase::declare_parameters () {
    // Update frequency parameter
    this->declare_parameter ("update_frequency", 200.0);
    this->declare_parameter ("SAVE_LOCATION", "/tmp");
    update_frequency_ = this->get_parameter ("update_frequency").as_double ();

    // Controller parameters
    const std::vector<std::string> controller_names = {"x_linear",  "y_linear",  "z_linear",
                                                       "x_angular", "y_angular", "z_angular"};

    for (size_t i = 0; i < controller_names.size (); ++i) {
        load_controller_parameters (static_cast<ControllerIndex> (i), controller_names[i]);
    }
}

void ControlLayerBase::load_controller_parameters (ControllerIndex idx, const std::string& prefix) {
    // Default PID gains
    this->declare_parameter (prefix + ".p_gain", 0.0);
    this->declare_parameter (prefix + ".i_gain", 0.0);
    this->declare_parameter (prefix + ".d_gain", 0.0);

    // Integral limits
    this->declare_parameter (prefix + ".i_min", 0.0);
    this->declare_parameter (prefix + ".i_max", 0.0);

    // Output limits
    this->declare_parameter (prefix + ".u_min", 0.0);
    this->declare_parameter (prefix + ".u_max", 0.0);

    // Clamp values
    this->declare_parameter (prefix + ".clamp_values", false);

    // Load initial values
    update_controller_gains (idx, prefix);
}

void ControlLayerBase::update_controller_gains (ControllerIndex idx, const std::string& prefix) {
    double p_gain = this->get_parameter (prefix + ".p_gain").as_double ();
    double i_gain = this->get_parameter (prefix + ".i_gain").as_double ();
    double d_gain = this->get_parameter (prefix + ".d_gain").as_double ();
    double i_min = this->get_parameter (prefix + ".i_min").as_double ();
    double i_max = this->get_parameter (prefix + ".i_max").as_double ();
    double u_min = this->get_parameter (prefix + ".u_min").as_double ();
    double u_max = this->get_parameter (prefix + ".u_max").as_double ();
    bool clamp_values = this->get_parameter (prefix + ".clamp_values").as_bool ();

    controllers_[idx].set_gains (p_gain, i_gain, d_gain, i_min, i_max, u_min, u_max, clamp_values);
}

void ControlLayerBase::update_controller_gains_from_params (
    ControllerIndex idx, const std::string& prefix,
    const std::vector<rclcpp::Parameter>& parameters) {
    // Get current gains
    double p_gain, i_gain, d_gain, i_min, i_max, u_min, u_max;
    bool clamp_values;
    controllers_[idx].get_gains (p_gain, i_gain, d_gain, i_min, i_max, u_min, u_max, clamp_values);

    // Update only the parameters that changed
    for (const auto& param : parameters) {
        const std::string& param_name = param.get_name ();

        if (param_name == prefix + ".p_gain") {
            p_gain = param.as_double ();
        } else if (param_name == prefix + ".i_gain") {
            i_gain = param.as_double ();
        } else if (param_name == prefix + ".d_gain") {
            d_gain = param.as_double ();
        } else if (param_name == prefix + ".i_min") {
            i_min = param.as_double ();
        } else if (param_name == prefix + ".i_max") {
            i_max = param.as_double ();
        } else if (param_name == prefix + ".u_min") {
            u_min = param.as_double ();
        } else if (param_name == prefix + ".u_max") {
            u_max = param.as_double ();
        } else if (param_name == prefix + ".clamp_values") {
            clamp_values = param.as_bool ();
        }
    }

    // Set the updated gains
    controllers_[idx].set_gains (p_gain, i_gain, d_gain, i_min, i_max, u_min, u_max, clamp_values);
}

rcl_interfaces::msg::SetParametersResult ControlLayerBase::on_parameter_change (
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    const std::vector<std::string> controller_names = {"x_linear",  "y_linear",  "z_linear",
                                                       "x_angular", "y_angular", "z_angular"};

    std::map<size_t, bool> controllers_to_update;

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

            update_timer_->cancel ();
            update_timer_ = this->create_wall_timer (
                timer_period, std::bind (&ControlLayerBase::update, this), timer_callback_group_);
        } else {
            // Check if parameter belongs to a controller
            for (size_t i = 0; i < controller_names.size (); ++i) {
                const std::string& prefix = controller_names[i];
                if (param.get_name ().find (prefix) == 0) {
                    controllers_to_update[i] = true;
                    break;
                }
            }
        }
    }

    // Update controller gains for affected controllers
    for (const auto& [controller_idx, _] : controllers_to_update) {
        update_controller_gains_from_params (static_cast<ControllerIndex> (controller_idx),
                                             controller_names[controller_idx], parameters);
        std::cout << "set pid parameter value\n";
    }

    return result;
}

}  // namespace okmr_controls
