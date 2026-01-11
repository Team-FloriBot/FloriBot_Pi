#include "base/pid_controller.h"

namespace base {

PIDController::PIDController(double kp, double ki, double kd,
                             double output_limit,
                             double deadband,
                             double max_accel)
: kp_(kp), ki_(ki), kd_(kd),
  out_lim_(std::abs(output_limit)),
  deadband_(std::abs(deadband)),
  max_accel_(std::abs(max_accel))
{}

void PIDController::reset() {
  integrator_ = 0.0;
  prev_error_ = 0.0;
  last_output_ = 0.0;
  ramped_setpoint_ = 0.0;
  first_run_ = true;
}

double PIDController::compute(double target_setpoint, double measured, double dt,
                             bool freeze_integrator, bool update_state) {
  if (dt <= 0.0) return last_output_;

  auto core = [&](double& integrator,
                  double& prev_error,
                  double& last_output,
                  double& ramped_setpoint,
                  bool& first_run) -> double {
    // --- setpoint ramp limiting ---
    if (first_run) {
      ramped_setpoint = measured; // start from current to avoid initial jump
    }

    const double max_change = max_accel_ * dt;
    const double err_sp = target_setpoint - ramped_setpoint;

    if (err_sp > max_change) ramped_setpoint += max_change;
    else if (err_sp < -max_change) ramped_setpoint -= max_change;
    else ramped_setpoint = target_setpoint;

    // --- PID on ramped setpoint ---
    const double error = ramped_setpoint - measured;

    const double p = kp_ * error;

    if (!freeze_integrator) {
      integrator += ki_ * error * dt;
    }

    double d = 0.0;
    if (!first_run) {
      d = kd_ * ((error - prev_error) / dt);
    }

    double u = p + integrator + d;

    // saturation + conditional anti-windup:
    // allow integration only if it does NOT drive further into saturation.
    if (u > out_lim_) {
      u = out_lim_;
      if (!freeze_integrator && error > 0.0) {
        integrator -= ki_ * error * dt;
      }
    } else if (u < -out_lim_) {
      u = -out_lim_;
      if (!freeze_integrator && error < 0.0) {
        integrator -= ki_ * error * dt;
      }
    }

    // output deadband
    if (std::abs(u) < deadband_) u = 0.0;

    prev_error = error;
    last_output = u;
    first_run = false;
    return u;
  };

  if (!update_state) {
    double integ = integrator_;
    double prev = prev_error_;
    double last = last_output_;
    double ramp = ramped_setpoint_;
    bool first = first_run_;
    return core(integ, prev, last, ramp, first);
  }

  return core(integrator_, prev_error_, last_output_, ramped_setpoint_, first_run_);
}

} // namespace base
