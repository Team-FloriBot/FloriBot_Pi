#include "base/kinematics_node.h"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

namespace base {

KinematicsNode::KinematicsNode()
: Node("kinematics_node")
{
  // Parameters
  declare_parameter("wheel_separation", wheel_sep_);
  declare_parameter("wheel_radius", wheel_rad_);

  declare_parameter("ticks_per_rev", ticks_per_rev_);
  declare_parameter("unwrap_modulo", unwrap_modulo_);
  declare_parameter("modulo_ticks", modulo_ticks_);
  declare_parameter("publish_tf", publish_tf_);

  declare_parameter("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter("wheel_cmd_topic", wheel_cmd_topic_);
  declare_parameter("wheel_ticks_topic", wheel_ticks_topic_);
  declare_parameter("odom_topic", odom_topic_);

  declare_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter("base_frame_id", base_frame_id_);

  wheel_sep_     = get_parameter("wheel_separation").as_double();
  wheel_rad_     = get_parameter("wheel_radius").as_double();

  ticks_per_rev_ = get_parameter("ticks_per_rev").as_double();
  unwrap_modulo_ = get_parameter("unwrap_modulo").as_bool();
  modulo_ticks_  = get_parameter("modulo_ticks").as_int();
  publish_tf_    = get_parameter("publish_tf").as_bool();

  cmd_vel_topic_     = get_parameter("cmd_vel_topic").as_string();
  wheel_cmd_topic_   = get_parameter("wheel_cmd_topic").as_string();
  wheel_ticks_topic_ = get_parameter("wheel_ticks_topic").as_string();
  odom_topic_        = get_parameter("odom_topic").as_string();

  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();

  if (wheel_sep_ <= 0.0) throw std::runtime_error("wheel_separation must be > 0");
  if (wheel_rad_ <= 0.0) throw std::runtime_error("wheel_radius must be > 0");
  if (ticks_per_rev_ <= 0.0) throw std::runtime_error("ticks_per_rev must be > 0");

  if (modulo_ticks_ <= 0) {
    modulo_ticks_ = static_cast<int>(std::llround(ticks_per_rev_));
  }
  if (modulo_ticks_ <= 0) {
    modulo_ticks_ = 1;
  }

  kinematics_ = std::make_unique<KinematicsCalculator>(wheel_sep_, wheel_rad_);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10, std::bind(&KinematicsNode::cmdVelCallback, this, _1));

  sub_ticks_ = create_subscription<base::msg::WheelTicks4>(
    wheel_ticks_topic_, 50, std::bind(&KinematicsNode::wheelTicksCallback, this, _1));

  pub_wheel_cmd_ = create_publisher<base::msg::WheelVelocities>(wheel_cmd_topic_, 10);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  last_odom_time_ = now();

  RCLCPP_INFO(get_logger(),
    "kinematics_node started | cmd=%s -> wheel_cmd=%s | ticks=%s -> odom=%s | track_width=%.3f m wheel_radius=%.3f m",
    cmd_vel_topic_.c_str(), wheel_cmd_topic_.c_str(),
    wheel_ticks_topic_.c_str(), odom_topic_.c_str(),
    wheel_sep_, wheel_rad_);
}

void KinematicsNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  WheelSpeedSet ws = kinematics_->calculateWheelSpeeds(msg->linear.x, msg->angular.z);

  base::msg::WheelVelocities out;
  out.left  = ws.left;
  out.right = ws.right;

  pub_wheel_cmd_->publish(out);
}

void KinematicsNode::wheelTicksCallback(const base::msg::WheelTicks4::SharedPtr msg)
{
  const rclcpp::Time stamp = (msg->header.stamp.nanoseconds() != 0) ? rclcpp::Time(msg->header.stamp) : now();

  const int64_t fl = msg->fl_ticks;
  const int64_t fr = msg->fr_ticks;
  const int64_t rl = msg->rl_ticks;
  const int64_t rr = msg->rr_ticks;

  if (!have_prev_ticks_) {
    have_prev_ticks_ = true;
    prev_fl_ = fl; prev_fr_ = fr; prev_rl_ = rl; prev_rr_ = rr;
    last_odom_time_ = stamp;
    publishOdom(stamp, 0.0, 0.0);
    return;
  }

  const double dt = (stamp - last_odom_time_).seconds();
  if (!(dt > 0.0)) {
    publishOdom(stamp, 0.0, 0.0);
    return;
  }

  int64_t dfl, dfr, drl, drr;
  if (unwrap_modulo_) {
    dfl = deltaModulo(fl, prev_fl_, modulo_ticks_);
    dfr = deltaModulo(fr, prev_fr_, modulo_ticks_);
    drl = deltaModulo(rl, prev_rl_, modulo_ticks_);
    drr = deltaModulo(rr, prev_rr_, modulo_ticks_);
  } else {
    dfl = fl - prev_fl_;
    dfr = fr - prev_fr_;
    drl = rl - prev_rl_;
    drr = rr - prev_rr_;
  }

  prev_fl_ = fl; prev_fr_ = fr; prev_rl_ = rl; prev_rr_ = rr;
  last_odom_time_ = stamp;

  // 4WD diff-drive: mean per side
  const double dleft_ticks  = 0.5 * (static_cast<double>(dfl) + static_cast<double>(drl));
  const double dright_ticks = 0.5 * (static_cast<double>(dfr) + static_cast<double>(drr));

  // ticks -> wheel rotation (rad)
  const double k = (2.0 * M_PI) / ticks_per_rev_;
  const double dphi_left  = dleft_ticks * k;
  const double dphi_right = dright_ticks * k;

  // wheel rotation -> distance (m)
  const double ds_left  = wheel_rad_ * dphi_left;
  const double ds_right = wheel_rad_ * dphi_right;

  // integration
  const double ds = 0.5 * (ds_left + ds_right);
  const double dtheta = (ds_right - ds_left) / wheel_sep_;

  const double theta_mid = theta_ + 0.5 * dtheta;
  x_ += ds * std::cos(theta_mid);
  y_ += ds * std::sin(theta_mid);
  theta_ = normalizeAngle(theta_ + dtheta);

  const double vx = ds / dt;
  const double wz = dtheta / dt;

  publishOdom(stamp, vx, wz);
}

void KinematicsNode::publishOdom(const rclcpp::Time& stamp, double vx, double wz)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = wz;

  pub_odom_->publish(odom);

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id = base_frame_id_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }
}

int64_t KinematicsNode::deltaModulo(int64_t cur, int64_t last, int64_t mod)
{
  int64_t d = cur - last;
  const int64_t half = mod / 2;
  if (d > half)  d -= mod;
  if (d < -half) d += mod;
  return d;
}

double KinematicsNode::normalizeAngle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

} // namespace base

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<base::KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
