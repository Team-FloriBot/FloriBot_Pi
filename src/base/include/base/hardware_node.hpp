#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_ticks4.hpp"
#include "base/pid_controller.h"

class HardwareNode : public rclcpp::Node {
public:
  HardwareNode();
  ~HardwareNode() override;

private:
  enum Wheel : size_t { FL=0, FR=1, RL=2, RR=3 };

  void onWheelCmd(const base::msg::WheelVelocities::SharedPtr msg);

  void watchdog();
  void controlLoop();
  void publishTicks();

  // --- motor driver (I2C) ---
  class NxtServoI2C;
  class NxtServoDriver;

  std::unique_ptr<NxtServoI2C> i2c_;
  std::unique_ptr<NxtServoDriver> driver_;

  // --- encoders (Phidget) ---
  void initEncoders();
  void closeEncoders();

  std::array<void*,4> enc_{ {nullptr,nullptr,nullptr,nullptr} };
  std::array<std::atomic<int64_t>,4> ticks_;
  std::array<int64_t,4> last_ticks_{ {0,0,0,0} };

  // --- PID per wheel ---
  std::array<std::unique_ptr<base::PIDController>,4> pid_;
  std::array<double,4> target_w_radps_{ {0,0,0,0} };

  // params
  std::string wheel_cmd_topic_{"/wheel_commands"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};
  std::string ticks_frame_id_{"base_link"};

  // encoder config
  int phidget_serial_{-1};
  std::array<int,4> encoder_channels_{ {0,1,2,3} };
  std::array<bool,4> invert_{ {false,false,false,false} };
  int64_t ticks_per_rev_{131000};

  // motor controller config
  std::string i2c_dev_{"/dev/i2c-1"};
  int i2c_addr_{0x58};
  std::array<int,4> servo_channels_{ {0,1,2,3} }; // I2C channels for FL/FR/RL/RR

  // per-wheel PWM calibration
  std::array<double,4> k_us_per_pct_{ {5.5,5.5,5.5,5.5} };
  std::array<int,4> neutral_us_{ {1480,1480,1480,1480} };
  std::array<int,4> min_us_{ {900,900,900,900} };
  std::array<int,4> max_us_{ {2100,2100,2100,2100} };
  int pct_min_{-100};
  int pct_max_{100};
  int accel_{100};

  // control
  double max_wheel_radps_{20.0}; // normalization
  double watchdog_timeout_s_{0.3};
  int control_period_ms_{10};
  int publish_ticks_ms_{20};

  // state
  std::mutex mtx_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_control_time_;
  bool stopped_{true};

  // ROS entities
  rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr sub_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr pub_ticks_;
  rclcpp::TimerBase::SharedPtr timer_watchdog_;
  rclcpp::TimerBase::SharedPtr timer_control_;
  rclcpp::TimerBase::SharedPtr timer_ticks_;
};
