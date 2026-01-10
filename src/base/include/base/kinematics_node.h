#pragma once
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "base/msg/wheel_velocities.hpp"
#include "base/kinematics_calculator.h"

class KinematicsNode : public rclcpp::Node {
public:
  KinematicsNode();

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<base::msg::WheelVelocities>::SharedPtr pub_wheels_;

  std::unique_ptr<base::KinematicsCalculator> calc_;

  std::string cmd_topic_;
  std::string wheel_cmd_topic_;

  double wheel_sep_m_{0.385};
  double wheel_radius_m_{0.10};

  double v_max_mps_{1.0};
  double w_max_rps_{2.0};
};
