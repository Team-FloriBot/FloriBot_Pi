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

class KinematicsNode : public rclcpp::Node {
public:
  KinematicsNode() : Node("kinematics_node") {
    // -------------------------
    // Topics
    // -------------------------
    cmd_topic_    = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    out_topic_    = declare_parameter<std::string>("wheel_cmd_topic", "/base/wheel_cmd");
    ticks_topic_  = declare_parameter<std::string>("wheel_ticks_topic", "/base/wheel_ticks4");
    odom_topic_   = declare_parameter<std::string>("odom_topic", "/odom");

    // -------------------------
    // Frames
    // -------------------------
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    // -------------------------
    // Geometry / scaling
    // -------------------------
    wheel_base_m_   = declare_parameter<double>("wheel_base_m", 0.50);
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.10);
    ticks_per_rev_  = declare_parameter<int64_t>("ticks_per_rev", 131000);

    // -------------------------
    // Control loop & safety
    // -------------------------
    control_period_ms_ = declare_parameter<int>("control_period_ms", 20);
    cmd_timeout_ms_    = declare_parameter<int>("cmd_timeout_ms", 300);

    // -------------------------
    // Closed-loop options
    // -------------------------
    use_closed_loop_ = declare_parameter<bool>("use_closed_loop", true);
    use_feedforward_ = declare_parameter<bool>("use_feedforward", true);
    v_max_mps_       = declare_parameter<double>("v_max_mps", 0.8);

    // -------------------------
    // Deadbands / friction
    // -------------------------
    v_deadband_mps_   = declare_parameter<double>("v_deadband_mps", 0.02);
    pid_deadband_mps_ = declare_parameter<double>("pid_deadband_mps", 0.01);
    min_pwm_pct_      = declare_parameter<int>("min_pwm_pct", 8);

    // -------------------------
    // PI gains (side velocity control)
    // units: kp [%/(m/s)], ki [%/(m/s*s)]
    // -------------------------
    vel_kp_ = declare_parameter<double>("vel_kp", 80.0);
    vel_ki_ = declare_parameter<double>("vel_ki", 30.0);
    i_min_  = declare_parameter<double>("i_min", -40.0);
    i_max_  = declare_parameter<double>("i_max",  40.0);

    // -------------------------
    // Measurement filtering
    // -------------------------
    meas_lpf_alpha_ = declare_parameter<double>("meas_lpf_alpha", 0.2);

    // -------------------------
    // Debug
    // -------------------------
    debug_ = declare_parameter<bool>("debug", true);

    // pubs/subs
    pub_wheel_cmd_ = create_publisher<std_msgs::msg::Int16MultiArray>(out_topic_, 10);
    pub_odom_      = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){ onCmd(*msg); });

    sub_ticks_ = create_subscription<base::msg::WheelTicks4>(
      ticks_topic_, 10,
      [this](const base::msg::WheelTicks4::SharedPtr msg){ onTicks(*msg); });

    // control loop timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, control_period_ms_)),
      std::bind(&KinematicsNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
      "KinematicsNode started | closed_loop=%d feedforward=%d period=%dms timeout=%dms | "
      "b=%.3f r=%.3f ticks_per_rev=%ld v_max=%.3f",
      (int)use_closed_loop_, (int)use_feedforward_, control_period_ms_, cmd_timeout_ms_,
      wheel_base_m_, wheel_radius_m_, (long)ticks_per_rev_, v_max_mps_);
  }

private:
  static double wrapAngle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  static int16_t clampPctToInt16(double pct) {
    pct = clampd(pct, -100.0, 100.0);
    return static_cast<int16_t>(std::lround(pct));
  }

  void onCmd(const geometry_msgs::msg::Twist& t) {
    last_cmd_ = t;
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  void onTicks(const base::msg::WheelTicks4& m) {
    rclcpp::Time t = m.header.stamp;
    if (t.nanoseconds() == 0) t = now();

    if (!have_last_ticks_) {
      have_last_ticks_ = true;
      last_fl_ = m.fl_ticks; last_fr_ = m.fr_ticks;
      last_rl_ = m.rl_ticks; last_rr_ = m.rr_ticks;
      last_ticks_time_ = t;
      return;
    }

    if (ticks_per_rev_ <= 0 || wheel_radius_m_ <= 1e-9) return;

    const int64_t dfl = m.fl_ticks - last_fl_;
    const int64_t dfr = m.fr_ticks - last_fr_;
    const int64_t drl = m.rl_ticks - last_rl_;
    const int64_t drr = m.rr_ticks - last_rr_;

    double dt = (t - last_ticks_time_).seconds();
    if (dt <= 1e-6) dt = 1e-6;

    const double meters_per_tick =
      (2.0 * M_PI * wheel_radius_m_) / static_cast<double>(ticks_per_rev_);

    const double ds_fl = static_cast<double>(dfl) * meters_per_tick;
    const double ds_fr = static_cast<double>(dfr) * meters_per_tick;
    const double ds_rl = static_cast<double>(drl) * meters_per_tick;
    const double ds_rr = static_cast<double>(drr) * meters_per_tick;

    // per-wheel linear velocity [m/s]
    const double v_fl = ds_fl / dt;
    const double v_fr = ds_fr / dt;
    const double v_rl = ds_rl / dt;
    const double v_rr = ds_rr / dt;

    // side average velocities [m/s]
    const double vL = 0.5 * (v_fl + v_rl);
    const double vR = 0.5 * (v_fr + v_rr);

    // low-pass filter
    if (!have_meas_) {
      vL_meas_ = vL;
      vR_meas_ = vR;
      have_meas_ = true;
    } else {
      const double a = clampd(meas_lpf_alpha_, 0.0, 1.0);
      vL_meas_ = (1.0 - a) * vL_meas_ + a * vL;
      vR_meas_ = (1.0 - a) * vR_meas_ + a * vR;
    }
    meas_time_ = t;

    // ---- Odometry integration (from ds)
    const double dl = 0.5 * (ds_fl + ds_rl);
    const double dr = 0.5 * (ds_fr + ds_rr);

    const double ds  = 0.5 * (dr + dl);
    const double dth = (wheel_base_m_ > 1e-9) ? ((dr - dl) / wheel_base_m_) : 0.0;

    const double th_mid = th_ + 0.5 * dth;
    x_  += ds * std::cos(th_mid);
    y_  += ds * std::sin(th_mid);
    th_ = wrapAngle(th_ + dth);

    // body velocities
    const double v_body = ds / dt;
    const double w_body = dth / dt;

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

    odom.twist.twist.linear.x = v_body;
    odom.twist.twist.angular.z = w_body;
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

    last_fl_ = m.fl_ticks; last_fr_ = m.fr_ticks;
    last_rl_ = m.rl_ticks; last_rr_ = m.rr_ticks;
    last_ticks_time_ = t;
  }

  void controlLoop() {
    const rclcpp::Time now_t = now();

    // timeout -> stop + reset integrators
    if (!have_cmd_ || (now_t - last_cmd_time_).nanoseconds() > (int64_t)cmd_timeout_ms_ * 1000000LL) {
      publishWheelCmd(0, 0);
      iL_ = 0.0; iR_ = 0.0;
      return;
    }

    // desired body velocities
    double v = last_cmd_.linear.x;   // m/s
    double w = last_cmd_.angular.z;  // rad/s

    // deadband on command
    if (std::fabs(v) < v_deadband_mps_) v = 0.0;
    if (std::fabs(w) < 1e-9) w = 0.0;

    // wheel setpoints [m/s]
    const double b = wheel_base_m_;
    const double vL_sp = v - w * (b * 0.5);
    const double vR_sp = v + w * (b * 0.5);

    // feedforward [%]
    auto ffPct = [this](double v_wheel) -> double {
      if (!use_feedforward_ || v_max_mps_ <= 1e-9) return 0.0;
      return 100.0 * (v_wheel / v_max_mps_);
    };

    double uL = ffPct(vL_sp);
    double uR = ffPct(vR_sp);

    if (!use_closed_loop_) {
      // open-loop only
      applyMinPwm(uL, vL_sp);
      applyMinPwm(uR, vR_sp);
      publishWheelCmd(clampPctToInt16(uL), clampPctToInt16(uR));
      return;
    }

    // Need measurements; if none yet, fallback to open-loop
    const bool meas_fresh = have_meas_ &&
      (now_t - meas_time_).nanoseconds() < (int64_t)500 * 1000000LL; // 500ms
    if (!meas_fresh) {
      applyMinPwm(uL, vL_sp);
      applyMinPwm(uR, vR_sp);
      publishWheelCmd(clampPctToInt16(uL), clampPctToInt16(uR));
      return;
    }

    // PI control in [%]
    const double dt = std::max(1e-3, control_period_ms_ / 1000.0);

    const double eL = vL_sp - vL_meas_;
    const double eR = vR_sp - vR_meas_;

    // small-error deadband for PI
    const double eL_eff = (std::fabs(eL) < pid_deadband_mps_) ? 0.0 : eL;
    const double eR_eff = (std::fabs(eR) < pid_deadband_mps_) ? 0.0 : eR;

    double pL = vel_kp_ * eL_eff;
    double pR = vel_kp_ * eR_eff;

    // tentative integrator update
    double iL_new = clampd(iL_ + vel_ki_ * eL_eff * dt, i_min_, i_max_);
    double iR_new = clampd(iR_ + vel_ki_ * eR_eff * dt, i_min_, i_max_);

    // unsaturated outputs
    double uL_unsat = uL + pL + iL_new;
    double uR_unsat = uR + pR + iR_new;

    // saturate
    double uL_sat = clampd(uL_unsat, -100.0, 100.0);
    double uR_sat = clampd(uR_unsat, -100.0, 100.0);

    // simple anti-windup: accept integrator only if not driving further into saturation
    auto acceptI = [](double u_unsat, double u_sat, double err) {
      if (u_unsat == u_sat) return true;
      // if saturated high and error positive -> would increase further -> reject
      if (u_sat >= 100.0 && err > 0.0) return false;
      // if saturated low and error negative -> would decrease further -> reject
      if (u_sat <= -100.0 && err < 0.0) return false;
      return true;
    };

    if (acceptI(uL_unsat, uL_sat, eL_eff)) iL_ = iL_new;
    if (acceptI(uR_unsat, uR_sat, eR_eff)) iR_ = iR_new;

    // apply min pwm for nonzero setpoint
    uL = uL_sat;
    uR = uR_sat;
    applyMinPwm(uL, vL_sp);
    applyMinPwm(uR, vR_sp);

    publishWheelCmd(clampPctToInt16(uL), clampPctToInt16(uR));

    if (debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "vL_sp=%.3f vL=%.3f uL=%.1f | vR_sp=%.3f vR=%.3f uR=%.1f | eL=%.3f eR=%.3f iL=%.1f iR=%.1f",
        vL_sp, vL_meas_, uL, vR_sp, vR_meas_, uR, eL, eR, iL_, iR_);
    }
  }

  void applyMinPwm(double &u_pct, double v_sp) const {
    if (min_pwm_pct_ <= 0) return;
    if (std::fabs(v_sp) < v_deadband_mps_) return;
    if (std::fabs(u_pct) < 1e-6) return;
    const double s = (u_pct >= 0.0) ? 1.0 : -1.0;
    const double mag = std::fabs(u_pct);
    if (mag < (double)min_pwm_pct_) u_pct = s * (double)min_pwm_pct_;
  }

  void publishWheelCmd(int16_t left_pct, int16_t right_pct) {
    std_msgs::msg::Int16MultiArray out;
    out.data.resize(2);
    out.data[0] = left_pct;
    out.data[1] = right_pct;
    pub_wheel_cmd_->publish(out);
  }

  // -------------------------
  // Params
  // -------------------------
  std::string cmd_topic_;
  std::string out_topic_;
  std::string ticks_topic_;
  std::string odom_topic_;

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  bool publish_tf_{true};

  double wheel_base_m_{0.50};
  double wheel_radius_m_{0.10};
  int64_t ticks_per_rev_{131000};

  int control_period_ms_{20};
  int cmd_timeout_ms_{300};

  bool use_closed_loop_{true};
  bool use_feedforward_{true};
  double v_max_mps_{0.8};

  double v_deadband_mps_{0.02};
  double pid_deadband_mps_{0.01};
  int min_pwm_pct_{8};

  double vel_kp_{80.0};
  double vel_ki_{30.0};
  double i_min_{-40.0};
  double i_max_{ 40.0};

  double meas_lpf_alpha_{0.2};
  bool debug_{true};

  // -------------------------
  // ROS
  // -------------------------
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_wheel_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<base::msg::WheelTicks4>::SharedPtr sub_ticks_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // cmd state
  bool have_cmd_{false};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0,0,RCL_ROS_TIME};

  // ticks / measurement state
  bool have_last_ticks_{false};
  int64_t last_fl_{0}, last_fr_{0}, last_rl_{0}, last_rr_{0};
  rclcpp::Time last_ticks_time_{0,0,RCL_ROS_TIME};

  bool have_meas_{false};
  double vL_meas_{0.0}, vR_meas_{0.0};
  rclcpp::Time meas_time_{0,0,RCL_ROS_TIME};

  // PI integrators
  double iL_{0.0}, iR_{0.0};

  // odom state
  double x_{0.0}, y_{0.0}, th_{0.0};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
