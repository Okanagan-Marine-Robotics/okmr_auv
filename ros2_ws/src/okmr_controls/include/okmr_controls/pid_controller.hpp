#ifndef OKMR_CONTROLS_PID_CONTROLLER_HPP
#define OKMR_CONTROLS_PID_CONTROLLER_HPP

#include <chrono>

namespace okmr_controls {

class PidController {
   public:
    PidController ();

    PidController (double p_gain, double i_gain, double d_gain, double i_min = -1.0,
                   double i_max = 1.0, double u_min = -1.0, double u_max = 1.0,
                   bool clamp_values = false);

    ~PidController ();

    double compute_command (double error, std::chrono::nanoseconds dt);

    void reset ();

    // When true, the derivative term wraps (error - prev_error) into [-180, 180]
    // before dividing by dt. Required for angle errors in degrees, which can jump
    // by ~360 deg across the wrap boundary (e.g. +179 -> -179) even though the
    // true rate of change is small - without this, that wrap produces a huge
    // spurious derivative spike.
    void set_angular (bool is_angular);

    void set_gains (double p_gain, double i_gain, double d_gain, double i_min = -1.0,
                    double i_max = 1.0, double u_min = -1.0, double u_max = 1.0,
                    bool clamp_values = false);

    void get_gains (double& p_gain, double& i_gain, double& d_gain, double& i_min, double& i_max,
                    double& u_min, double& u_max, bool& clamp_values) const;

    struct PidState {
        double error;
        double p_term;
        double i_term;
        double d_term;
        double output;
    };
    PidState get_state () const;

   private:
    double p_gain_;
    double i_gain_;
    double d_gain_;
    double i_min_;
    double i_max_;
    double u_min_;
    double u_max_;
    bool clamp_values_;

    double p_error_;
    double i_term_;
    double d_error_;
    double prev_error_;

    double cmd_;

    bool first_run_;
    bool is_angular_;
};

}  // namespace okmr_controls

#endif  // OKMR_CONTROLS_PID_CONTROLLER_HPP