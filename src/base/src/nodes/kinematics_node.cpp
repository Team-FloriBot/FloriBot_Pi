// src/nodes/kinematics_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <algorithm>
#include <cmath>

class KinematicsNode : public rclcpp::Node {
public:
  KinematicsNode() : Node("kinematics_node") {
    wheel_base_m_ = declare_parameter<double>("wheel_base_m", 0.50);
    v_max_mps_    = declare_parameter<double>("v_max_mps", 0.8);
    cmd_topic_    = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    out_topic_    = declare_parameter<std::string>("wheel_cmd_topic", "/base/wheel_cmd");

    pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(out_topic_, 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){ onCmd(*msg); }
    );
  }

private:
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
    out.data[0] = toPct(vL);
    out.data[1] = toPct(vR);
    pub_->publish(out);
  }

  double wheel_base_m_{0.0};
  double v_max_mps_{0.0};
  std::string cmd_topic_;
  std::string out_topic_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
