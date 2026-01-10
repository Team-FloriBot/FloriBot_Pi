#include "base/kinematics_node.h"

#include <algorithm>

KinematicsNode::KinematicsNode()
: Node("kinematics_node")
{
  cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  wheel_cmd_topic_ = declare_parameter<std::string>("wheel_cmd_topic", "/wheel_commands");

  wheel_sep_m_ = declare_parameter<double>("track_width_m", 0.385);
  wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.10);

  v_max_mps_ = declare_parameter<double>("v_max_mps", 1.0);
  w_max_rps_ = declare_parameter<double>("w_max_rps", 2.0);

  calc_ = std::make_unique<base::KinematicsCalculator>(wheel_sep_m_, wheel_radius_m_);

  pub_wheels_ = create_publisher<base::msg::WheelVelocities>(wheel_cmd_topic_, 10);
  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic_, 10,
    std::bind(&KinematicsNode::cmdCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "kinematics_node started | cmd=%s -> wheel_cmd=%s | track_width=%.3f m wheel_radius=%.3f m",
    cmd_topic_.c_str(), wheel_cmd_topic_.c_str(), wheel_sep_m_, wheel_radius_m_);
}

void KinematicsNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double v = std::clamp(msg->linear.x,  -v_max_mps_, v_max_mps_);
  const double w = std::clamp(msg->angular.z, -w_max_rps_, w_max_rps_);

  const auto ws = calc_->calculateWheelSpeeds(v, w);

  base::msg::WheelVelocities out;
  out.left = ws.left;
  out.right = ws.right;
  pub_wheels_->publish(out);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
