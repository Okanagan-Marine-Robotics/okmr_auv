#include "okmr_controls/pid_controller.hpp"

#include <algorithm>
#include <iostream>

namespace okmr_controls {

PidController::PidController ()
    : p_gain_ (0.0),
      i_gain_ (0.0),
      d_gain_ (0.0),
      i_min_ (-1.0),
      i_max_ (1.0),
      u_min_ (-1.0),
      u_max_ (1.0),
      clamp_values_ (false),
      p_error_ (0.0),
      i_term_ (0.0),
      d_error_ (0.0),
      prev_error_ (0.0),
      cmd_ (0.0),
      first_run_ (true) {}

PidController::PidController (double p_gain, double i_gain, double d_gain, double i_min,
                              double i_max, double u_min, double u_max, bool clamp_values)
    : p_gain_ (p_gain),
      i_gain_ (i_gain),
      d_gain_ (d_gain),
      i_min_ (i_min),
      i_max_ (i_max),
      u_min_ (u_min),
      u_max_ (u_max),
      clamp_values_ (clamp_values),
      p_error_ (0.0),
      i_term_ (0.0),
      d_error_ (0.0),
      prev_error_ (0.0),
      cmd_ (0.0),
      first_run_ (true) {}

PidController::~PidController () {}

double PidController::compute_command (double error, std::chrono::nanoseconds dt) {
    double dt_sec = std::chrono::duration<double> (dt).count ();

    if (dt_sec <= 0.0) {
        return cmd_;
    }

    p_error_ = error;

    if (first_run_) {
        prev_error_ = error;
        first_run_ = false;
    }

    // Integral term with clamping
    i_term_ += error * dt_sec;
    i_term_ = std::clamp (i_term_, i_min_ / (i_gain_ + 1e-6), i_max_ / (i_gain_ + 1e-6));

    // Derivative term
    d_error_ = (error - prev_error_) / dt_sec;

    // PID command
    cmd_ = p_gain_ * p_error_ + i_gain_ * i_term_ + d_gain_ * d_error_;
    // std::cout<<"Pgain: "<<p_gain_<<" error: "<<error<<" output: "<<cmd_<<'\n';

    // Output clamping
    if (clamp_values_) {
        cmd_ = std::clamp (cmd_, u_min_, u_max_);
    }

    prev_error_ = error;

    return cmd_;
}

void PidController::reset () {
    p_error_ = 0.0;
    i_term_ = 0.0;
    d_error_ = 0.0;
    prev_error_ = 0.0;
    cmd_ = 0.0;
    first_run_ = true;
}

PidController::PidState PidController::get_state () const {
    return {p_error_, p_gain_ * p_error_, i_gain_ * i_term_, d_gain_ * d_error_, cmd_};
}

void PidController::set_gains (double p_gain, double i_gain, double d_gain, double i_min,
                               double i_max, double u_min, double u_max, bool clamp_values) {
    p_gain_ = p_gain;
    i_gain_ = i_gain;
    d_gain_ = d_gain;
    i_min_ = i_min;
    i_max_ = i_max;
    u_min_ = u_min;
    u_max_ = u_max;
    clamp_values_ = clamp_values;
}

void PidController::get_gains (double& p_gain, double& i_gain, double& d_gain, double& i_min,
                               double& i_max, double& u_min, double& u_max,
                               bool& clamp_values) const {
    p_gain = p_gain_;
    i_gain = i_gain_;
    d_gain = d_gain_;
    i_min = i_min_;
    i_max = i_max_;
    u_min = u_min_;
    u_max = u_max_;
    clamp_values = clamp_values_;
}

}  // namespace okmr_controls
