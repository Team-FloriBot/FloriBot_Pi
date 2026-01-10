#pragma once
#include <algorithm>
#include <cmath>

namespace base {

/**
 * PID controller with:
 * - output saturation (|u| <= output_limit)
 * - deadband on output
 * - integrator clamping (simple anti-windup)
 * - optional setpoint ramp limiting via max_accel (units: setpoint units per second)
 */
class PIDController {
public:
  PIDController(double kp, double ki, double kd,
                double output_limit = 1.0,
                double deadband = 0.05,
                double max_accel = 1.0);

  void reset();

  /// Compute control output in [-output_limit .. +output_limit]
  double compute(double target_setpoint, double measured, double dt);

private:
  double kp_{0.0}, ki_{0.0}, kd_{0.0};
  double out_lim_{1.0};
  double deadband_{0.05};
  double max_accel_{1.0};

  double integrator_{0.0};
  double prev_error_{0.0};
  double last_output_{0.0};
  double ramped_setpoint_{0.0};
  bool first_run_{true};
};

} // namespace base
