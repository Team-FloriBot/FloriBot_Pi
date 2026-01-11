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

double PIDController::compute(double target_setpoint, double measured, double dt) {
  if (dt <= 0.0) return last_output_;

  // --- setpoint ramp limiting ---
  if (first_run_) {
    ramped_setpoint_ = measured; // start from current to avoid initial jump
  }

  const double max_change = max_accel_ * dt;
  const double err_sp = target_setpoint - ramped_setpoint_;

  if (err_sp > max_change) ramped_setpoint_ += max_change;
  else if (err_sp < -max_change) ramped_setpoint_ -= max_change;
  else ramped_setpoint_ = target_setpoint;

  // --- PID on ramped setpoint ---
  const double error = ramped_setpoint_ - measured;

  const double p = kp_ * error;
  integrator_ += ki_ * error * dt;

  double d = 0.0;
  if (!first_run_) {
    d = kd_ * ((error - prev_error_) / dt);
  }

  double u = p + integrator_ + d;

  // saturation + conditional anti-windup:
  // allow integration only if it does NOT drive further into saturation.
  if (u > out_lim_) {
    u = out_lim_;
    if (error > 0.0) {
      // would increase u further -> undo integration
      integrator_ -= ki_ * error * dt;
    }
  } else if (u < -out_lim_) {
    u = -out_lim_;
    if (error < 0.0) {
      integrator_ -= ki_ * error * dt;
    }
  }

  // output deadband
  if (std::abs(u) < deadband_) u = 0.0;

  prev_error_ = error;
  last_output_ = u;
  first_run_ = false;
  return u;
}

} // namespace base
