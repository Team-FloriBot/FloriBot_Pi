#include "base/kinematics_calculator.h"

namespace base {

KinematicsCalculator::KinematicsCalculator(double wheel_separation, double wheel_radius)
    : wheel_sep_(wheel_separation), wheel_rad_(wheel_radius) {}

// inverse kinematic: translates movement commands into the required individual wheel speeds.
WheelSpeedSet KinematicsCalculator::calculateWheelSpeeds(double linear_x, double angular_z) {    
    // Differential drive kinematics equation
    // V_left = V - (omega * width / 2)
    // V_right = V + (omega * width / 2)
    // Wheel_RPM = V_wheel / radius

    double vel_left = linear_x - (angular_z * wheel_sep_ / 2.0);
    double vel_right = linear_x + (angular_z * wheel_sep_ / 2.0);

    //Angular velocity
    double rad_s_left = vel_left / wheel_rad_;
    double rad_s_right = vel_right / wheel_rad_;

    // For skid steer, Front and Rear on same side behave identically
    return {rad_s_left, rad_s_right, rad_s_left, rad_s_right};
}

RobotTwist KinematicsCalculator::calculateRobotTwist(const WheelSpeedSet& speeds) {
    // Average left and right side speeds
    double rad_s_left = (speeds.fl + speeds.rl) / 2.0;
    double rad_s_right = (speeds.fr + speeds.rr) / 2.0;

    double vel_left = rad_s_left * wheel_rad_;
    double vel_right = rad_s_right * wheel_rad_;

    RobotTwist twist;
    twist.linear_x = (vel_left + vel_right) / 2.0;
    twist.linear_y = 0.0; // Non-holonomic
    twist.angular_z = (vel_right - vel_left) / wheel_sep_;
    
    return twist;
}

} // namespace base
