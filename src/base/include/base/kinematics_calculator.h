#ifndef BASE_KINEMATICS_CALCULATOR_H
#define BASE_KINEMATICS_CALCULATOR_H

namespace base {

struct WheelSpeedSet
{
  double left{0.0};   // rad/s
  double right{0.0};  // rad/s
};

class KinematicsCalculator
{
public:
  KinematicsCalculator(double track_width_m, double wheel_radius_m);

  WheelSpeedSet calculateWheelSpeeds(double v_mps, double w_radps) const;

private:
  double track_width_{0.0};
  double wheel_radius_{0.0};
};

}  // namespace base

#endif  // BASE_KINEMATICS_CALCULATOR_H
