// src/nodes/kinematics_node.cpp
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

  // Anti-windup: conditional integration based on actuator saturation.
  // Returns saturated command. Integral is only updated if it helps de-saturate.
  double step_aw(double e, double dt, double u_ff, double u_min, double u_max) {
    if (dt <= 0.0) return std::clamp(u_ff, u_min, u_max);

    // Derivative on error (optional)
    double d = 0.0;
    if (!first) d = (e - prev_e) / dt;
    first = false;
    prev_e = e;

    const double p = kp * e;

    // candidate integral update
    double i_candidate = i + e * dt;
    i_candidate = std::clamp(i_candidate, i_min, i_max);

    const double u_unsat = u_ff + p + ki * i_candidate + kd * d;
    const double u_sat   = std::clamp(u_unsat, u_min, u_max);

    // conditional integration:
    // if saturated and error drives further into saturation, reject integral update
    const bool sat_high = (u_unsat > u_max);
    const bool sat_low  = (u_unsat < u_min);

    bool accept_i = true;
    if (sat_high && (e > 0.0)) accept_i = false;
    if (sat_low  && (e < 0.0)) accept_i = false;

    if (accept_i) i = i_candidate;
    // else keep i unchanged

    const double u = u_ff + p + ki * i + kd * d;
    return std::clamp(u, u_min, u_max);
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

    // Topics
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

    // Tick freshness safety (stop if encoder data is stale)
    ticks_timeout_ms_  = declare_parameter<int>("ticks_timeout_ms", 120);

    // Feedforward scaling (open-loop baseline)
    v_max_mps_       = declare_parameter<double>("v_max_mps", 0.8);
    use_feedforward_ = declare_parameter<bool>("use_feedforward", true);

    // Deadbands
    v_deadband_mps_     = declare_parameter<double>("v_deadband_mps", 0.02);
    pid_deadband_mps_   = declare_parameter<double>("pid_deadband_mps", 0.01);

    // Measurement low-pass filter (0..1). Higher = less filtering.
    meas_lpf_alpha_  = declare_parameter<double>("meas_lpf_alpha", 0.15);

    // Minimum dt for tick-based velocity estimation (discard too-small dt)
    min_ticks_dt_s_ = declare_parameter<double>("min_ticks_dt_s", 0.005);

    // Static friction compensation (minimum percent once moving is requested)
    min_pwm_pct_ = declare_parameter<int>("min_pwm_pct", 8);

    // PID gains (velocity error in m/s -> percent correction)
    const double kp = declare_parameter<double>("kp", 18.0);
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
      "KinematicsNode: b=%.3f r=%.3f ticks_per_rev=%ld ctrl=%dms kp=%.3f ki=%.3f kd=%.3f alpha=%.3f min_pwm=%d",
      wheel_base_m_, wheel_radius_m_, (long)ticks_per_rev_, control_period_ms_,
      pid_left_.kp, pid_left_.ki, pid_left_.kd, meas_lpf_alpha_, min_pwm_pct_);
  }

private:
  static double wrapAngle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double applyDeadband(double x, double db) {
    if (std::abs(x) <= db) return 0.0;
    return x;
  }

  int applyStaticFrictionComp(double u_pct, double v_set) const {
    // Only apply when motion is requested (avoid pushing when stopped)
    if (std::abs(v_set) < v_deadband_mps_) return 0;

    // If controller wants exactly 0, keep 0.
    if (std::abs(u_pct) < 1e-9) return 0;

    const double s = (u_pct >= 0.0) ? 1.0 : -1.0;
    const double abs_u = std::abs(u_pct);

    if (abs_u < static_cast<double>(min_pwm_pct_)) {
      return static_cast<int>(std::lround(s * static_cast<double>(min_pwm_pct_)));
    }
    return static_cast<int>(std::lround(u_pct));
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
      last_ticks_time_ = t;
      have_ticks_time_ = true;
      return;
    }

    if (ticks_per_rev_ <= 0) return;

    const int64_t dfl = m.fl_ticks - last_fl_;
    const int64_t dfr = m.fr_ticks - last_fr_;
    const int64_t drl = m.rl_ticks - last_rl_;
    const int64_t drr = m.rr_ticks - last_rr_;

    double dt = (t - last_t_).seconds();
    if (dt < min_ticks_dt_s_) {
      // discard too-fast updates (avoid huge v from quantized ticks)
      last_fl_ = m.fl_ticks; last_fr_ = m.fr_ticks;
      last_rl_ = m.rl_ticks; last_rr_ = m.rr_ticks;
      last_t_  = t;
      last_ticks_time_ = t;
      have_ticks_time_ = true;
      return;
    }

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

    // low-pass filter on measured velocities
    const double a = std::clamp(meas_lpf_alpha_, 0.0, 1.0);
    if (!have_meas_f_) {
      v_left_meas_f_ = v_left_meas_;
      v_right_meas_f_ = v_right_meas_;
      have_meas_f_ = true;
    } else {
      v_left_meas_f_  = a * v_left_meas_  + (1.0 - a) * v_left_meas_f_;
      v_right_meas_f_ = a * v_right_meas_ + (1.0 - a) * v_right_meas_f_;
    }

    // publish odom
    const double v = ds / dt;
    const double w = dth / dt;

    if (debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "dTicks fl=%ld fr=%ld rl=%ld rr=%ld | vL=%.3f vR=%.3f (filt %.3f %.3f) | x=%.3f y=%.3f th=%.3f",
        (long)dfl,(long)dfr,(long)drl,(long)drr,
        v_left_meas_, v_right_meas_, v_left_meas_f_, v_right_meas_f_,
        x_, y_, th_);
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

    last_ticks_time_ = t;
    have_ticks_time_ = true;
  }

  void controlLoop() {
    const rclcpp::Time t_now = now();

    // true dt for controller
    if (!have_control_t_) {
      last_control_t_ = t_now;
      have_control_t_ = true;
      return;
    }
    double dt_pid = (t_now - last_control_t_).seconds();
    last_control_t_ = t_now;
    dt_pid = std::clamp(dt_pid, 0.002, 0.100); // 2ms..100ms

    const auto cmd_timeout = rclcpp::Duration(0, static_cast<int64_t>(cmd_timeout_ms_) * 1000000LL);
    const auto ticks_timeout = rclcpp::Duration(0, static_cast<int64_t>(ticks_timeout_ms_) * 1000000LL);

    const bool cmd_ok =
      have_cmd_ && ((t_now - last_cmd_time_) <= cmd_timeout);

    const bool ticks_ok =
      have_meas_ && have_meas_f_ && have_ticks_time_ && ((t_now - last_ticks_time_) <= ticks_timeout);

    if (!cmd_ok || !ticks_ok) {
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

    auto ffPct = [this](double v_set){
      if (!use_feedforward_ || v_max_mps_ <= 1e-9) return 0.0;
      return 100.0 * (v_set / v_max_mps_);
    };

    const double u_ff_L = ffPct(vL_set);
    const double u_ff_R = ffPct(vR_set);

    double eL = vL_set - v_left_meas_f_;
    double eR = vR_set - v_right_meas_f_;

    // deadband on error to reduce dithering
    eL = applyDeadband(eL, pid_deadband_mps_);
    eR = applyDeadband(eR, pid_deadband_mps_);

    // PI(D)+FF with anti-windup inside PID
    double uL = pid_left_.step_aw(eL, dt_pid, u_ff_L, -100.0, 100.0);
    double uR = pid_right_.step_aw(eR, dt_pid, u_ff_R, -100.0, 100.0);

    // static friction compensation (then clamp again)
    int left_pct  = applyStaticFrictionComp(uL, vL_set);
    int right_pct = applyStaticFrictionComp(uR, vR_set);

    left_pct  = std::clamp(left_pct,  -100, 100);
    right_pct = std::clamp(right_pct, -100, 100);

    publishWheelCmd(left_pct, right_pct);
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
  int ticks_timeout_ms_{120};

  double v_max_mps_{0.8};
  bool use_feedforward_{true};

  double v_deadband_mps_{0.02};
  double pid_deadband_mps_{0.01};

  double meas_lpf_alpha_{0.15};
  double min_ticks_dt_s_{0.005};
  int min_pwm_pct_{8};

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
  bool have_meas_f_{false};
  bool have_last_{false};

  int64_t last_fl_{0}, last_fr_{0}, last_rl_{0}, last_rr_{0};
  rclcpp::Time last_t_;

  double v_left_meas_{0.0};
  double v_right_meas_{0.0};
  double v_left_meas_f_{0.0};
  double v_right_meas_f_{0.0};

  rclcpp::Time last_ticks_time_;
  bool have_ticks_time_{false};

  // --- control timing ---
  rclcpp::Time last_control_t_;
  bool have_control_t_{false};

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
