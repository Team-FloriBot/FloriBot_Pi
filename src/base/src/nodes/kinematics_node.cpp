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
    // --- Parameters for cmd_vel -> wheel_cmd ---
    wheel_base_m_ = declare_parameter<double>("wheel_base_m", 0.50);
    v_max_mps_    = declare_parameter<double>("v_max_mps", 0.8);
    cmd_topic_    = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    out_topic_    = declare_parameter<std::string>("wheel_cmd_topic", "/base/wheel_cmd");

    // --- Parameters for odometry ---
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.08);
    ticks_per_rev_  = declare_parameter<int64_t>("ticks_per_rev", 2048);

    ticks_topic_ = declare_parameter<std::string>("wheel_ticks_topic", "/base/wheel_ticks4");
    odom_topic_  = declare_parameter<std::string>("odom_topic", "/odom");

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    // Debug parameter
    debug_ = declare_parameter<bool>("debug", true);

    // Publishers/Subscribers
    pub_wheel_cmd_ = create_publisher<std_msgs::msg::Int16MultiArray>(out_topic_, 10);
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){ onCmd(*msg); }
    );

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ticks_ = create_subscription<base::msg::WheelTicks4>(
      ticks_topic_, 10,
      [this](const base::msg::WheelTicks4::SharedPtr msg){ onTicks(*msg); }
    );

    // Sanity print
    RCLCPP_INFO(get_logger(),
      "KinematicsNode params: wheel_base_m=%.6f wheel_radius_m=%.6f ticks_per_rev=%ld ticks_topic=%s odom_topic=%s",
      wheel_base_m_, wheel_radius_m_, static_cast<long>(ticks_per_rev_),
      ticks_topic_.c_str(), odom_topic_.c_str());

    if (wheel_base_m_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "wheel_base_m is %.6f (<=0). Odometry will be invalid.", wheel_base_m_);
    }
    if (wheel_radius_m_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "wheel_radius_m is %.6f (<=0). Odometry distance will be ~0.", wheel_radius_m_);
    }
    if (ticks_per_rev_ <= 0) {
      RCLCPP_WARN(get_logger(), "ticks_per_rev is %ld (<=0). Odometry disabled.", static_cast<long>(ticks_per_rev_));
    }
  }

private:
  static double wrapAngle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void onCmd(const geometry_msgs::msg::Twist& t) {
    const double v = t.linear.x;      // m/s
    const double w = t.angular.z;     // rad/s
    const double b = wheel_base_m_;

    const double vL = v - w * (b * 0.5);
    const double vR = v + w * (b * 0.5);

    auto toPct = [this](double v_wheel){
      if (v_max_mps_ <= 1e-6) return int16_t{0};
      double pct = 100.0 * (v_wheel / v_max_mps_);
      pct = std::clamp(pct, -100.0, 100.0);
      return static_cast<int16_t>(std::lround(pct));
    };

    std_msgs::msg::Int16MultiArray out;
    out.data.resize(2);
    out.data[0] = toPct(vL);  // left
    out.data[1] = toPct(vR);  // right
    pub_wheel_cmd_->publish(out);
  }

  void onTicks(const base::msg::WheelTicks4& m) {
    rclcpp::Time t = m.header.stamp;
    if (t.nanoseconds() == 0) t = now();

    if (!have_last_) {
      have_last_ = true;
      last_fl_ = m.fl_ticks; last_fr_ = m.fr_ticks;
      last_rl_ = m.rl_ticks; last_rr_ = m.rr_ticks;
      last_t_  = t;

      if (debug_) {
        RCLCPP_INFO(get_logger(),
          "First ticks received: fl=%ld fr=%ld rl=%ld rr=%ld",
          (long)last_fl_, (long)last_fr_, (long)last_rl_, (long)last_rr_);
      }
      return;
    }

    if (ticks_per_rev_ <= 0) return;

    const int64_t dfl = m.fl_ticks - last_fl_;
    const int64_t dfr = m.fr_ticks - last_fr_;
    const int64_t drl = m.rl_ticks - last_rl_;
    const int64_t drr = m.rr_ticks - last_rr_;

    const double meters_per_tick =
      (2.0 * M_PI * wheel_radius_m_) / static_cast<double>(ticks_per_rev_);

    const double ds_fl = static_cast<double>(dfl) * meters_per_tick;
    const double ds_fr = static_cast<double>(dfr) * meters_per_tick;
    const double ds_rl = static_cast<double>(drl) * meters_per_tick;
    const double ds_rr = static_cast<double>(drr) * meters_per_tick;

    const double dl = 0.5 * (ds_fl + ds_rl);
    const double dr = 0.5 * (ds_fr + ds_rr);

    if (wheel_base_m_ <= 1e-6 || wheel_radius_m_ <= 1e-6) {
      // Still publish odom, but it won't move meaningfully
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Invalid geometry params: wheel_base_m=%.6f wheel_radius_m=%.6f",
        wheel_base_m_, wheel_radius_m_);
    }

    const double ds  = 0.5 * (dr + dl);
    const double dth = (wheel_base_m_ > 1e-6) ? ((dr - dl) / wheel_base_m_) : 0.0;

    const double th_mid = th_ + 0.5 * dth;
    x_  += ds * std::cos(th_mid);
    y_  += ds * std::sin(th_mid);
    th_ = wrapAngle(th_ + dth);

    double dt = (t - last_t_).seconds();
    if (dt <= 1e-6) dt = 1e-6;
    const double v = ds / dt;
    const double w = dth / dt;

    if (debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "dTicks fl=%ld fr=%ld rl=%ld rr=%ld | dl=%.6f dr=%.6f ds=%.6f dth=%.6f | x=%.3f y=%.3f th=%.3f",
        (long)dfl,(long)dfr,(long)drl,(long)drr, dl, dr, ds, dth, x_, y_, th_);
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

    last_fl_ = m.fl_ticks; last_fr_ = m.fr_ticks;
    last_rl_ = m.rl_ticks; last_rr_ = m.rr_ticks;
    last_t_  = t;
  }

  // --- cmd_vel -> wheel_cmd ---
  double wheel_base_m_{0.50};
  double v_max_mps_{0.8};
  std::string cmd_topic_;
  std::string out_topic_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_wheel_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;

  // --- ticks -> odom ---
  double wheel_radius_m_{0.08};
  int64_t ticks_per_rev_{2048};
  std::string ticks_topic_;
  std::string odom_topic_;
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  bool publish_tf_{true};
  bool debug_{true};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<base::msg::WheelTicks4>::SharedPtr sub_ticks_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool have_last_{false};
  int64_t last_fl_{0}, last_fr_{0}, last_rl_{0}, last_rr_{0};
  rclcpp::Time last_t_;

  double x_{0.0}, y_{0.0}, th_{0.0};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
