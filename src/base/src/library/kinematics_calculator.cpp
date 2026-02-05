#include "base/kinematics_calculator.h"

namespace base {

KinematicsCalculator::KinematicsCalculator(double track_width_m, double wheel_radius_m)
: track_width_(track_width_m), wheel_radius_(wheel_radius_m)
{
}

WheelSpeedSet KinematicsCalculator::calculateWheelSpeeds(double v_mps, double w_radps) const
{
  WheelSpeedSet out;
  // Differential drive:
  // v_l = v - w * track/2
  // v_r = v + w * track/2
  // omega = v / r
  const double v_l = v_mps - (w_radps * track_width_ * 0.5);
  const double v_r = v_mps + (w_radps * track_width_ * 0.5);

  out.left  = v_l / wheel_radius_;
  out.right = v_r / wheel_radius_;
  return out;
}

} 
