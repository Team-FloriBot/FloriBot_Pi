#pragma once
namespace base {

struct WheelSpeedSet {
  double left{0.0};
  double right{0.0};
};

struct RobotTwist {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

class KinematicsCalculator {
public:
  KinematicsCalculator(double wheel_sep_m, double wheel_radius_m);

  WheelSpeedSet calculateWheelSpeeds(double linear_x_mps, double angular_z_rps) const;
  RobotTwist calculateRobotTwist(const WheelSpeedSet& speeds_radps) const;

private:
  double wheel_sep_{0.0};
  double wheel_rad_{0.0};
};

} // namespace base
