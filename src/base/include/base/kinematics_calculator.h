#ifndef BASE_KINEMATICS_CALCULATOR_H
#define BASE_KINEMATICS_CALCULATOR_H

namespace base {

struct WheelSpeedSet {
    double fl;
    double fr;
    double rl;
    double rr;
};

struct RobotTwist {
    double linear_x;
    double linear_y;
    double angular_z;
};

class KinematicsCalculator {
public:
    // Assumes 4-wheel skid steer/diff drive approximation
    KinematicsCalculator(double wheel_separation, double wheel_radius);

    // Inverse Kinematics: Twist -> Wheel Speeds (rad/s)
    WheelSpeedSet calculateWheelSpeeds(double linear_x, double angular_z);

    // Forward Kinematics: Wheel Speeds (rad/s) -> Twist
    RobotTwist calculateRobotTwist(const WheelSpeedSet& speeds);

private:
    double wheel_sep_;
    double wheel_rad_;
};

} // namespace base

#endif // BASE_KINEMATICS_CALCULATOR_H
