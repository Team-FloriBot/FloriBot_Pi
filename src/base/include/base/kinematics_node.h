#ifndef BASE_KINEMATICS_NODE_H
#define BASE_KINEMATICS_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "base/msg/wheel_velocities.hpp"
#include "base/kinematics_calculator.h"

namespace base {

class KinematicsNode : public rclcpp::Node {
public:
    KinematicsNode();

private:
    // Callbacks
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wheelStateCallback(const base::msg::WheelVelocities::SharedPtr msg);

    // Helpers
    void publishOdometry(const RobotTwist& twist, double dt);

    // ROS Handles
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr sub_wheel_states_;
    
    rclcpp::Publisher<base::msg::WheelVelocities>::SharedPtr pub_wheel_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Logic
    std::unique_ptr<KinematicsCalculator> kinematics_;
    
    // Odom State
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    rclcpp::Time last_odom_time_;
};

} // namespace base

#endif // BASE_KINEMATICS_NODE_H
