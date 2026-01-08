// src/nodes/hardware_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Linux I2C
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Phidget22
#include <phidget22.h>

#include "base/msg/wheel_ticks4.hpp"

using namespace std::chrono_literals;

class NxtServoI2C {
public:
  NxtServoI2C(std::string dev, int addr) : dev_(std::move(dev)), addr_(addr) {}

  void open_bus() {
    fd_ = ::open(dev_.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("open(" + dev_ + ") failed");
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("ioctl(I2C_SLAVE, 0x" + to_hex(addr_) + ") failed");
    }
  }

  ~NxtServoI2C() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void write1(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (::write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
      throw std::runtime_error("I2C write1 failed (reg=0x" + to_hex(reg) + ")");
    }
  }

  void write2(uint8_t reg, uint8_t low, uint8_t high) {
    uint8_t buf[3] = {reg, low, high};
    if (::write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
      throw std::runtime_error("I2C write2 failed (reg=0x" + to_hex(reg) + ")");
    }
  }

private:
  static std::string to_hex(int v) {
    const char* hex = "0123456789abcdef";
    std::string s;
    s.push_back(hex[(v >> 4) & 0xF]);
    s.push_back(hex[v & 0xF]);
    return s;
  }

  std::string dev_;
  int addr_{0};
  int fd_{-1};
};

static void phidget_check(PhidgetReturnCode rc, const char* what) {
  if (rc != EPHIDGET_OK) {
    const char* err = nullptr;
    Phidget_getErrorDescription(rc, &err);
    throw std::runtime_error(std::string(what) + " failed: " + (err ? err : "unknown"));
  }
}

class HardwareNode : public rclcpp::Node {
public:
  HardwareNode() : Node("hardware_node") {
    // -------------------------
    // Parameters (I2C / motor)
    // -------------------------
    i2c_dev_  = declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
    i2c_addr_ = declare_parameter<int>("i2c_addr", 0x58);

    const auto left_i64 =
      declare_parameter<std::vector<int64_t>>("left_channels",  std::vector<int64_t>{2, 0});
    const auto right_i64 =
      declare_parameter<std::vector<int64_t>>("right_channels", std::vector<int64_t>{3, 1});

    left_channels_.clear();
    right_channels_.clear();
    for (auto v : left_i64)  left_channels_.push_back(static_cast<int>(v));
    for (auto v : right_i64) right_channels_.push_back(static_cast<int>(v));

    // Optional: explicit wheel->servo channel mapping for test mode: [fl, fr, rl, rr]
    // If not provided, we derive from left/right: [left[0], right[0], left[1], right[1]]
    const auto wheel_i64 =
      declare_parameter<std::vector<int64_t>>("wheel_channels", std::vector<int64_t>{2, 3, 0, 1});
    if (!wheel_i64.empty()) {
      if (wheel_i64.size() != 4) {
        throw std::runtime_error("wheel_channels must be 4 entries: [fl, fr, rl, rr]");
      }
      wheel_channels_[0] = static_cast<int>(wheel_i64[0]);
      wheel_channels_[1] = static_cast<int>(wheel_i64[1]);
      wheel_channels_[2] = static_cast<int>(wheel_i64[2]);
      wheel_channels_[3] = static_cast<int>(wheel_i64[3]);
      wheel_channels_valid_ = true;
    } else {
      wheel_channels_valid_ = derive_wheel_channels_from_sides();
    }

    // Per-wheel gains [fl, fr, rl, rr]
    // For closed-loop control in kinematics_node, keep these at 1.0 (or ignore them).
    const auto gains =
      declare_parameter<std::vector<double>>(
        "wheel_gains", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    if (gains.size() != 4) {
      throw std::runtime_error("wheel_gains must have 4 entries: [fl, fr, rl, rr]");
    }
    for (size_t i = 0; i < 4; ++i) {
      wheel_gains_[i] = gains[i];
    }

    // NEW: allow ignoring wheel_gains in normal operation (recommended for kinematics closed-loop)
    ignore_wheel_gains_ = declare_parameter<bool>("ignore_wheel_gains", true);

    // PWM mapping: us = pct*k + neutral
    k_us_per_pct_ = declare_parameter<double>("k_us_per_pct", 5.5);
    neutral_us_   = declare_parameter<int>("neutral_us", 1480);
    min_us_       = declare_parameter<int>("min_us", 900);
    max_us_       = declare_parameter<int>("max_us", 2100);

    pct_min_ = declare_parameter<int>("pct_min", -100);
    pct_max_ = declare_parameter<int>("pct_max",  100);

    // For velocity closed-loop, prefer accel=0 (no hidden actuator ramp).
    accel_ = declare_parameter<int>("acceleration", 0);

    watchdog_ms_ = declare_parameter<int>("watchdog_timeout_ms", 300);

    // Test mode: if true, subscribe to /base/wheel_cmd4 and allow per-wheel actuation
    test_mode_ = declare_parameter<bool>("test_mode", false);

    // -------------------------
    // Parameters (Phidget encoders)
    // -------------------------
    phidget_serial_ = declare_parameter<int>("phidget_serial", -1);

    const auto enc_map_i64 =
      declare_parameter<std::vector<int64_t>>("encoder_channels", std::vector<int64_t>{2, 3, 0, 1});
    if (enc_map_i64.size() != 4) {
      throw std::runtime_error("encoder_channels must have 4 entries: [fl, fr, rl, rr]");
    }
    for (size_t i = 0; i < 4; ++i) encoder_channels_[i] = static_cast<int>(enc_map_i64[i]);

    invert_fl_ = declare_parameter<bool>("invert_fl", true);
    invert_fr_ = declare_parameter<bool>("invert_fr", false);
    invert_rl_ = declare_parameter<bool>("invert_rl", true);
    invert_rr_ = declare_parameter<bool>("invert_rr", false);

    publish_ticks_ms_ = declare_parameter<int>("publish_ticks_ms", 20);

    // -------------------------
    // ROS pubs/subs
    // -------------------------
    sub_lr_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "/base/wheel_cmd", 10,
      std::bind(&HardwareNode::on_cmd_lr, this, std::placeholders::_1));

    sub_4_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "/base/wheel_cmd4", 10,
      std::bind(&HardwareNode::on_cmd_4, this, std::placeholders::_1));

    pub_ticks_ = create_publisher<base::msg::WheelTicks4>("/base/wheel_ticks4", 10);

    // -------------------------
    // Init I2C + safety stop
    // -------------------------
    nxt_ = std::make_unique<NxtServoI2C>(i2c_dev_, i2c_addr_);
    nxt_->open_bus();
    set_acceleration_all(accel_);
    stop_all();

    // -------------------------
    // Init encoders
    // -------------------------
    init_encoders();

    // -------------------------
    // Timers
    // -------------------------
    last_cmd_time_ = now();
    timer_watchdog_ = create_wall_timer(50ms, std::bind(&HardwareNode::watchdog, this));
    timer_ticks_ = create_wall_timer(
      std::chrono::milliseconds(publish_ticks_ms_),
      std::bind(&HardwareNode::publish_ticks, this));

    RCLCPP_INFO(get_logger(),
      "hardware_node started | test_mode=%d | I2C %s addr 0x%02x | wheel_map_valid=%d [fl=%d fr=%d rl=%d rr=%d] | gains [%.3f %.3f %.3f %.3f] ignore_wheel_gains=%d accel=%d",
      (int)test_mode_, i2c_dev_.c_str(), i2c_addr_,
      (int)wheel_channels_valid_,
      wheel_channels_[0], wheel_channels_[1], wheel_channels_[2], wheel_channels_[3],
      wheel_gains_[0], wheel_gains_[1], wheel_gains_[2], wheel_gains_[3],
      (int)ignore_wheel_gains_, accel_);
  }

  ~HardwareNode() override {
    try { stop_all(); } catch (...) {}

    for (auto &e : enc_) {
      if (e) {
        Phidget_close((PhidgetHandle)e);
        PhidgetEncoder_delete(&e);
        e = nullptr;
      }
    }
  }

private:
  // -------------------------
  // Helpers (motor)
  // -------------------------
  static int clamp_int(int v, int lo, int hi) {
    return std::max(lo, std::min(v, hi));
  }

  int apply_gain(int pct, double gain) const {
    const double p = static_cast<double>(pct) * gain;
    return clamp_int(static_cast<int>(std::lround(p)), pct_min_, pct_max_);
  }

  int pct_to_pulse_us(int pct) const {
    const double us_f = static_cast<double>(pct) * k_us_per_pct_ + static_cast<double>(neutral_us_);
    int us = static_cast<int>(std::lround(us_f));
    return clamp_int(us, min_us_, max_us_);
  }

  void write_channel_pulse(int channel, int pulse_us) {
    const uint8_t reg  = static_cast<uint8_t>((channel * 2) + 66);
    const uint8_t low  = static_cast<uint8_t>(pulse_us & 0xFF);
    const uint8_t high = static_cast<uint8_t>((pulse_us >> 8) & 0xFF);
    nxt_->write2(reg, low, high);
  }

  void set_acceleration(int channel, int accel) {
    const uint8_t reg = static_cast<uint8_t>(channel + 82);
    nxt_->write1(reg, static_cast<uint8_t>(accel));
  }

  void set_acceleration_all(int accel) {
    accel = clamp_int(accel, 0, 255);
    for (int ch = 0; ch < 4; ++ch) set_acceleration(ch, accel);
  }

  void apply_side(const std::vector<int>& channels, int pct) {
    pct = clamp_int(pct, pct_min_, pct_max_);
    const int pulse = pct_to_pulse_us(pct);
    for (int ch : channels) write_channel_pulse(ch, pulse);
  }

  void apply_wheel_channel(int servo_channel, int pct) {
    pct = clamp_int(pct, pct_min_, pct_max_);
    const int pulse = pct_to_pulse_us(pct);
    write_channel_pulse(servo_channel, pulse);
  }

  void stop_all() {
    apply_side(left_channels_,  0);
    apply_side(right_channels_, 0);
  }

  bool derive_wheel_channels_from_sides() {
    if (left_channels_.size() >= 2 && right_channels_.size() >= 2) {
      wheel_channels_[0] = left_channels_[0];
      wheel_channels_[1] = right_channels_[0];
      wheel_channels_[2] = left_channels_[1];
      wheel_channels_[3] = right_channels_[1];
      return true;
    }
    wheel_channels_ = {0, 1, 2, 3};
    return false;
  }

  // -------------------------
  // ROS callbacks
  // -------------------------
  void on_cmd_lr(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (test_mode_) return;
    if (msg->data.size() < 2) return;

    const int left_pct_in  = clamp_int((int)msg->data[0], pct_min_, pct_max_);
    const int right_pct_in = clamp_int((int)msg->data[1], pct_min_, pct_max_);

    int fl = left_pct_in;
    int fr = right_pct_in;
    int rl = left_pct_in;
    int rr = right_pct_in;

    if (!ignore_wheel_gains_) {
      fl = apply_gain(left_pct_in,  wheel_gains_[0]);
      fr = apply_gain(right_pct_in, wheel_gains_[1]);
      rl = apply_gain(left_pct_in,  wheel_gains_[2]);
      rr = apply_gain(right_pct_in, wheel_gains_[3]);
    }

    try {
      apply_wheel_channel(wheel_channels_[0], fl);
      apply_wheel_channel(wheel_channels_[1], fr);
      apply_wheel_channel(wheel_channels_[2], rl);
      apply_wheel_channel(wheel_channels_[3], rr);
      last_cmd_time_ = now();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "I2C write failed (LR): %s", e.what());
    }
  }

  void on_cmd_4(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (!test_mode_) return;
    if (msg->data.size() < 4) return;

    int fl = clamp_int((int)msg->data[0], pct_min_, pct_max_);
    int fr = clamp_int((int)msg->data[1], pct_min_, pct_max_);
    int rl = clamp_int((int)msg->data[2], pct_min_, pct_max_);
    int rr = clamp_int((int)msg->data[3], pct_min_, pct_max_);

    if (!ignore_wheel_gains_) {
      fl = apply_gain(fl, wheel_gains_[0]);
      fr = apply_gain(fr, wheel_gains_[1]);
      rl = apply_gain(rl, wheel_gains_[2]);
      rr = apply_gain(rr, wheel_gains_[3]);
    }

    try {
      apply_wheel_channel(wheel_channels_[0], fl);
      apply_wheel_channel(wheel_channels_[1], fr);
      apply_wheel_channel(wheel_channels_[2], rl);
      apply_wheel_channel(wheel_channels_[3], rr);
      last_cmd_time_ = now();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "I2C write failed (CMD4): %s", e.what());
    }
  }

  void watchdog() {
    const auto dt = now() - last_cmd_time_;
    const auto timeout = rclcpp::Duration(0, static_cast<int64_t>(watchdog_ms_) * 1000000LL);
    if (dt > timeout) {
      try { stop_all(); } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Watchdog stop failed: %s", e.what());
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Watchdog timeout -> STOP");
    }
  }

  // -------------------------
  // Phidget22 encoder handling
  // -------------------------
  static void CCONV onPositionChangeWithHandle(
      PhidgetEncoderHandle ch,
      void *ctx,
      int positionChange,
      double /*timeChange*/,
      int /*indexTriggered*/)
  {
    auto* self = static_cast<HardwareNode*>(ctx);
    for (size_t i = 0; i < 4; ++i) {
      if (self->enc_[i] == ch) {
        int64_t delta = static_cast<int64_t>(positionChange);
        if (self->invert_[i]) delta = -delta;
        self->ticks_[i].fetch_add(delta, std::memory_order_relaxed);
        return;
      }
    }
  }

  void init_encoders() {
    invert_[0] = invert_fl_;
    invert_[1] = invert_fr_;
    invert_[2] = invert_rl_;
    invert_[3] = invert_rr_;

    for (size_t i = 0; i < 4; ++i) {
      PhidgetEncoderHandle h = nullptr;
      phidget_check(PhidgetEncoder_create(&h), "PhidgetEncoder_create");

      if (phidget_serial_ >= 0) {
        phidget_check(
          Phidget_setDeviceSerialNumber((PhidgetHandle)h, phidget_serial_),
          "Phidget_setDeviceSerialNumber");
      }

      phidget_check(
        Phidget_setChannel((PhidgetHandle)h, encoder_channels_[i]),
        "Phidget_setChannel");

      phidget_check(
        PhidgetEncoder_setOnPositionChangeHandler(h, &HardwareNode::onPositionChangeWithHandle, this),
        "PhidgetEncoder_setOnPositionChangeHandler");

      phidget_check(
        Phidget_openWaitForAttachment((PhidgetHandle)h, 5000),
        "Phidget_openWaitForAttachment");

      phidget_check(PhidgetEncoder_setPosition(h, 0), "PhidgetEncoder_setPosition");

      enc_[i] = h;
      ticks_[i].store(0, std::memory_order_relaxed);

      RCLCPP_INFO(get_logger(), "Encoder[%zu] attached: phidget_channel=%d invert=%d",
                  i, encoder_channels_[i], (int)invert_[i]);
    }
  }

  void publish_ticks() {
    base::msg::WheelTicks4 m;
    m.header.stamp = now();
    m.header.frame_id = "base_link";
    m.fl_ticks = ticks_[0].load(std::memory_order_relaxed);
    m.fr_ticks = ticks_[1].load(std::memory_order_relaxed);
    m.rl_ticks = ticks_[2].load(std::memory_order_relaxed);
    m.rr_ticks = ticks_[3].load(std::memory_order_relaxed);
    pub_ticks_->publish(m);
  }

  // -------------------------
  // Members
  // -------------------------
  // I2C/motor
  std::string i2c_dev_;
  int i2c_addr_{0};

  std::vector<int> left_channels_;
  std::vector<int> right_channels_;

  std::array<int,4> wheel_channels_{ {2, 3, 0, 1} };
  bool wheel_channels_valid_{false};

  std::array<double,4> wheel_gains_{ {1.0, 1.0, 1.0, 1.0} }; // fl, fr, rl, rr
  bool ignore_wheel_gains_{true};

  double k_us_per_pct_{5.5};
  int neutral_us_{1480};
  int min_us_{900};
  int max_us_{2100};
  int pct_min_{-100};
  int pct_max_{100};
  int accel_{0};
  int watchdog_ms_{300};

  bool test_mode_{false};

  // Phidget
  int phidget_serial_{-1};
  std::array<int,4> encoder_channels_{ {2, 3, 0, 1} };
  std::array<bool,4> invert_{ {false,false,false,false} };
  bool invert_fl_{true}, invert_fr_{false}, invert_rl_{true}, invert_rr_{false};
  int publish_ticks_ms_{20};

  std::array<PhidgetEncoderHandle,4> enc_{ {nullptr,nullptr,nullptr,nullptr} };
  std::array<std::atomic<int64_t>,4> ticks_{};

  // ROS
  rclcpp::Time last_cmd_time_;
  rclcpp::TimerBase::SharedPtr timer_watchdog_;
  rclcpp::TimerBase::SharedPtr timer_ticks_;

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_lr_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_4_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr pub_ticks_;

  std::unique_ptr<NxtServoI2C> nxt_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareNode>());
  rclcpp::shutdown();
  return 0;
}
