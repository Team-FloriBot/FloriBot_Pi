#ifndef BASE_KINEMATICS_NODE_H
#define BASE_KINEMATICS_NODE_H

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_ticks4.hpp"
#include "base/kinematics_calculator.h"

namespace base {

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void wheelTicksCallback(const base::msg::WheelTicks4::SharedPtr msg);

  void publishOdom(const rclcpp::Time& stamp, double vx, double wz);

  static int64_t deltaModulo(int64_t cur, int64_t last, int64_t mod);
  static double normalizeAngle(double a);

  // Params
  double wheel_sep_{0.385};         // [m]
  double wheel_rad_{0.100};         // [m]
  double ticks_per_rev_{2048.0};    // effective ticks per wheel revolution
  bool unwrap_modulo_{true};
  int modulo_ticks_{2048};
  bool publish_tf_{true};

  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string wheel_cmd_topic_{"/wheel_commands"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};
  std::string odom_topic_{"/odom"};

  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};

  // Pub/Sub
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<base::msg::WheelTicks4>::SharedPtr sub_ticks_;

  rclcpp::Publisher<base::msg::WheelVelocities>::SharedPtr pub_wheel_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Library
  std::unique_ptr<KinematicsCalculator> kinematics_;

  // Odom state
  bool have_prev_ticks_{false};
  int64_t prev_fl_{0}, prev_fr_{0}, prev_rl_{0}, prev_rr_{0};
  rclcpp::Time last_odom_time_;

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
};

} // namespace base

#endif // BASE_KINEMATICS_NODE_H
