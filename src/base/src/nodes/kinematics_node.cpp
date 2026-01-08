#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "base/msg/wheel_ticks4.hpp"

struct PID {
  double kp{0.0}, ki{0.0}, kd{0.0};
  double i{0.0};
  double prev_e{0.0};
  bool first{true};
  double i_min{-50.0}, i_max{50.0};

  void reset() { i = 0.0; prev_e = 0.0; first = true; }

  double step(double e, double dt) {
    if (dt <= 0.0) return 0.0;
    i += e * dt;
    i = std::clamp(i, i_min, i_max);

    double d = 0.0;
    if (!first) d = (e - prev_e) / dt;
    first = false;
    prev_e = e;

    return kp*e + ki*i + kd*d;
  }
};

class KinematicsNode : public rclcpp::Node {
public:
  KinematicsNode() : Node("kinematics_node") {
    // --- Parameters ---
    // Geometry
    wheel_base_m_   = declare_parameter<double>("wheel_base_m", 0.50);
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.10);

    // Encoder scaling: ticks per wheel revolution used in /base/wheel_ticks4
    ticks_per_rev_  = declare_parameter<int64_t>("ticks_per_rev", 131000);

    // cmd / topics
    cmd_topic_   = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    out_topic_   = declare_parameter<std::string>("wheel_cmd_topic", "/base/wheel_cmd");
    ticks_topic_ = declare_parameter<std::string>("wheel_ticks_topic", "/base/wheel_ticks4");

    // Odom
    odom_topic_  = declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_  = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_  = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_  = declare_parameter<bool>("publish_tf", true);
    debug_       = declare_parameter<bool>("debug", true);

    // Control
    control_period_ms_ = declare_parameter<int>("control_period_ms", 20);
    cmd_timeout_ms_    = declare_parameter<int>("cmd_timeout_ms", 300);

    // Feedforward scaling (open-loop baseline)
    v_max_mps_         = declare_parameter<double>("v_max_mps", 0.8);
    use_feedforward_   = declare_parameter<bool>("use_feedforward", true);

    // Deadband
    v_deadband_mps_    = declare_parameter<double>("v_deadband_mps", 0.02);

    // PID gains (velocity error in m/s -> percent correction)
    const double kp = declare_parameter<double>("kp", 30.0);
    const double ki = declare_parameter<double>("ki", 0.0);
    const double kd = declare_parameter<double>("kd", 0.0);
    const double i_min = declare_parameter<double>("i_min", -50.0);
    const double i_max = declare_parameter<double>("i_max",  50.0);

    pid_left_.kp = kp;  pid_left_.ki = ki;  pid_left_.kd = kd;  pid_left_.i_min = i_min; pid_left_.i_max = i_max;
    pid_right_   = pid_left_;

    // Publishers/Subscribers
    pub_wheel_cmd_ = create_publisher<std_msgs::msg::Int16MultiArray>(out_topic_, 10);
    pub_odom_      = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){ onCmd(*msg); }
    );

    sub_ticks_ = create_subscription<base::msg::WheelTicks4>(
      ticks_topic_, 10,
      [this](const base::msg::WheelTicks4::SharedPtr msg){ onTicks(*msg); }
    );

    timer_control_ = create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&KinematicsNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
      "KinematicsNode: wheel_base_m=%.3f wheel_radius_m=%.3f ticks_per_rev=%ld ctrl=%dms",
      wheel_base_m_, wheel_radius_m_, (long)ticks_per_rev_, control_period_ms_);
  }

private:
  static double wrapAngle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void onCmd(const geometry_msgs::msg::Twist& t) {
    v_cmd_ = t.linear.x;
    w_cmd_ = t.angular.z;
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  void onTicks(const base::msg::WheelTicks4& m) {
    rclcpp::Time t = m.header.stamp;
    if (t.nanoseconds() == 0) t = now();

    if (!have_last_) {
      have_last_ = true;
      last_fl_ = m.fl_ticks; last_fr_ = m.fr_ticks;
      last_rl_ = m.rl_ticks; last_rr_ = m.rr_ticks;
      last_t_  = t;
      return;
    }

    if (ticks_per_rev_ <= 0) return;

    const int64_t dfl = m.fl_ticks - last_fl_;
    const int64_t dfr = m.fr_ticks - last_fr_;
    const int64_t drl = m.rl_ticks - last_rl_;
    const int64_t drr = m.rr_ticks - last_rr_;

    double dt = (t - last_t_).seconds();
    if (dt <= 1e-6) dt = 1e-6;

    // meters per tick
    const double meters_per_tick =
      (2.0 * M_PI * wheel_radius_m_) / static_cast<double>(ticks_per_rev_);

    // distances per wheel
    const double ds_fl = static_cast<double>(dfl) * meters_per_tick;
    const double ds_fr = static_cast<double>(dfr) * meters_per_tick;
    const double ds_rl = static_cast<double>(drl) * meters_per_tick;
    const double ds_rr = static_cast<double>(drr) * meters_per_tick;

    // left/right distances (average)
    const double dl = 0.5 * (ds_fl + ds_rl);
    const double dr = 0.5 * (ds_fr + ds_rr);

    // odom integration
    const double ds  = 0.5 * (dr + dl);
    const double dth = (wheel_base_m_ > 1e-6) ? ((dr - dl) / wheel_base_m_) : 0.0;

    const double th_mid = th_ + 0.5 * dth;
    x_  += ds * std::cos(th_mid);
    y_  += ds * std::sin(th_mid);
    th_ = wrapAngle(th_ + dth);

    // measured velocities (m/s)
    v_left_meas_  = dl / dt;
    v_right_meas_ = dr / dt;
    have_meas_ = true;

    // publish odom
    const double v = ds / dt;
    const double w = dth / dt;

    if (debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "dTicks fl=%ld fr=%ld rl=%ld rr=%ld | vL=%.3f vR=%.3f | x=%.3f y=%.3f th=%.3f",
        (long)dfl,(long)dfr,(long)drl,(long)drr, v_left_meas_, v_right_meas_, x_, y_, th_);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;

    pub_odom_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = t;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // update last
    last_fl_ = m.fl_ticks; last_fr_ = m.fr_ticks;
    last_rl_ = m.rl_ticks; last_rr_ = m.rr_ticks;
    last_t_  = t;
  }

  void controlLoop() {
    // Safety: need cmd and measurement
    const auto timeout = rclcpp::Duration(0, static_cast<int64_t>(cmd_timeout_ms_) * 1000000LL);
    if (!have_cmd_ || !have_meas_ || (now() - last_cmd_time_) > timeout) {
      pid_left_.reset();
      pid_right_.reset();
      publishWheelCmd(0, 0);
      return;
    }

    const double b = wheel_base_m_;
    const double vL_set = v_cmd_ - w_cmd_ * (b * 0.5);
    const double vR_set = v_cmd_ + w_cmd_ * (b * 0.5);

    if (std::abs(vL_set) < v_deadband_mps_ && std::abs(vR_set) < v_deadband_mps_) {
      pid_left_.reset();
      pid_right_.reset();
      publishWheelCmd(0, 0);
      return;
    }

    // Use timer period as PID dt
    const double dt_pid = static_cast<double>(control_period_ms_) / 1000.0;

    auto ffPct = [this](double v_set){
      if (!use_feedforward_ || v_max_mps_ <= 1e-6) return 0.0;
      return 100.0 * (v_set / v_max_mps_);
    };

    const double eL = vL_set - v_left_meas_;
    const double eR = vR_set - v_right_meas_;

    double uL = ffPct(vL_set) + pid_left_.step(eL, dt_pid);
    double uR = ffPct(vR_set) + pid_right_.step(eR, dt_pid);

    uL = std::clamp(uL, -100.0, 100.0);
    uR = std::clamp(uR, -100.0, 100.0);

    publishWheelCmd((int)std::lround(uL), (int)std::lround(uR));
  }

  void publishWheelCmd(int left_pct, int right_pct) {
    std_msgs::msg::Int16MultiArray out;
    out.data.resize(2);
    out.data[0] = static_cast<int16_t>(std::clamp(left_pct, -100, 100));
    out.data[1] = static_cast<int16_t>(std::clamp(right_pct, -100, 100));
    pub_wheel_cmd_->publish(out);
  }

  // --- params ---
  double wheel_base_m_{0.50};
  double wheel_radius_m_{0.10};
  int64_t ticks_per_rev_{131000};

  std::string cmd_topic_;
  std::string out_topic_;
  std::string ticks_topic_;

  std::string odom_topic_;
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  bool publish_tf_{true};
  bool debug_{true};

  int control_period_ms_{20};
  int cmd_timeout_ms_{300};
  double v_max_mps_{0.8};
  bool use_feedforward_{true};
  double v_deadband_mps_{0.02};

  // --- ros ---
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_wheel_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<base::msg::WheelTicks4>::SharedPtr sub_ticks_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_control_;

  // --- cmd state ---
  bool have_cmd_{false};
  rclcpp::Time last_cmd_time_;
  double v_cmd_{0.0};
  double w_cmd_{0.0};

  // --- measurement state ---
  bool have_meas_{false};
  bool have_last_{false};
  int64_t last_fl_{0}, last_fr_{0}, last_rl_{0}, last_rr_{0};
  rclcpp::Time last_t_;
  double v_left_meas_{0.0};
  double v_right_meas_{0.0};

  // --- odom state ---
  double x_{0.0}, y_{0.0}, th_{0.0};

  // --- PID ---
  PID pid_left_, pid_right_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
