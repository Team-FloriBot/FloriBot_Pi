#include "base/kinematics_calculator.h"

namespace base {

KinematicsCalculator::KinematicsCalculator(double wheel_sep_m, double wheel_radius_m)
: wheel_sep_(wheel_sep_m), wheel_rad_(wheel_radius_m) {}

WheelSpeedSet KinematicsCalculator::calculateWheelSpeeds(double linear_x_mps, double angular_z_rps) const {
  WheelSpeedSet ws;
  // differential drive inverse kinematics:
  // v_l = v - w*L/2 ; v_r = v + w*L/2 ; omega = v / r
  const double v_l = linear_x_mps - angular_z_rps * (wheel_sep_ * 0.5);
  const double v_r = linear_x_mps + angular_z_rps * (wheel_sep_ * 0.5);
  ws.left  = v_l / wheel_rad_;
  ws.right = v_r / wheel_rad_;
  return ws;
}

RobotTwist KinematicsCalculator::calculateRobotTwist(const WheelSpeedSet& s) const {
  RobotTwist t;
  const double v_l = s.left  * wheel_rad_;
  const double v_r = s.right * wheel_rad_;
  t.linear_x = 0.5 * (v_l + v_r);
  t.angular_z = (v_r - v_l) / wheel_sep_;
  return t;
}

} // namespace base
