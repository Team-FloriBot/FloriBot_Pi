#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <vector>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

class HardwareNode : public rclcpp::Node
{
public:
  HardwareNode() : Node("hardware_node")
  {
    // Parameter korrekt (int64!)
    auto left_i64 =
      declare_parameter<std::vector<int64_t>>("left_channels", {0, 2});
    auto right_i64 =
      declare_parameter<std::vector<int64_t>>("right_channels", {1, 3});

    for (auto v : left_i64)  left_channels_.push_back(static_cast<int>(v));
    for (auto v : right_i64) right_channels_.push_back(static_cast<int>(v));

    watchdog_ms_ = declare_parameter<int>("watchdog_timeout_ms", 300);

    sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "/base/wheel_cmd", 10,
      std::bind(&HardwareNode::onCmd, this, std::placeholders::_1));

    timer_ = create_wall_timer(50ms, std::bind(&HardwareNode::watchdog, this));

    last_cmd_time_ = now();

    RCLCPP_INFO(get_logger(), "hardware_node started");
  }

private:
  void onCmd(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) return;

    int left  = std::clamp(msg->data[0], -100, 100);
    int right = std::clamp(msg->data[1], -100, 100);

    RCLCPP_INFO(
      get_logger(),
      "CMD L=%d R=%d | channels L[%d,%d] R[%d,%d]",
      left, right,
      left_channels_[0], left_channels_[1],
      right_channels_[0], right_channels_[1]);

    last_cmd_time_ = now();
  }

  void watchdog()
  {
    if ((now() - last_cmd_time_).seconds() * 1000.0 > watchdog_ms_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Watchdog timeout -> STOP");
    }
  }

  std::vector<int> left_channels_;
  std::vector<int> right_channels_;
  int watchdog_ms_{300};

  rclcpp::Time last_cmd_time_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareNode>());
  rclcpp::shutdown();
  return 0;
}
