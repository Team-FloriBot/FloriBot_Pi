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
    // ROS topics (PARAMETRIZED)
    // -------------------------
    cmd_lr_topic_   = declare_parameter<std::string>("wheel_cmd_topic",  "/base/wheel_cmd");
    cmd_4_topic_    = declare_parameter<std::string>("wheel_cmd4_topic", "/base/wheel_cmd4");
    ticks_topic_    = declare_parameter<std::string>("wheel_ticks_topic","/base/wheel_ticks4");
    ticks_frame_id_ = declare_parameter<std::string>("ticks_frame_id",   "base_link");

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

    // Optional explicit wheel->servo channel mapping: [fl, fr, rl, rr]
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
    const auto gains =
      declare_parameter<std::vector<double>>(
        "wheel_gains", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    if (gains.size() != 4) {
      throw std::runtime_error("wheel_gains must have 4 entries: [fl, fr, rl, rr]");
    }
    for (size_t i = 0; i < 4; ++i) wheel_gains_[i] = gains[i];

    // keep true for kinematics-node closed-loop (recommended)
    ignore_wheel_gains_ = declare_parameter<bool>("ignore_wheel_gains", true);

    // -------------------------
    // Per-wheel PWM calibration (fixes different servo drivers)
    // us[ch] = pct * k_us_per_pct[ch] + neutral_us[ch]
    // If you don't set *_4 arrays, scalars below are used for all wheels.
    // NOTE: For ROS2 parameters, integer arrays are int64_t. We store them as int64_t and cast later.
    // -------------------------
    k_us_per_pct_scalar_ = declare_parameter<double>("k_us_per_pct", 5.5);
    neutral_us_scalar_   = declare_parameter<int>("neutral_us", 1480);
    min_us_scalar_       = declare_parameter<int>("min_us", 900);
    max_us_scalar_       = declare_parameter<int>("max_us", 2100);

    // Optional arrays: [fl, fr, rl, rr]
    set_or_default_array("k_us_per_pct_4",
                         std::vector<double>{k_us_per_pct_scalar_, k_us_per_pct_scalar_,
                                             k_us_per_pct_scalar_, k_us_per_pct_scalar_},
                         k_us_per_pct_4_);

    set_or_default_array("neutral_us_4",
                         std::vector<int64_t>{(int64_t)neutral_us_scalar_, (int64_t)neutral_us_scalar_,
                                              (int64_t)neutral_us_scalar_, (int64_t)neutral_us_scalar_},
                         neutral_us_4_);

    set_or_default_array("min_us_4",
                         std::vector<int64_t>{(int64_t)min_us_scalar_, (int64_t)min_us_scalar_,
                                              (int64_t)min_us_scalar_, (int64_t)min_us_scalar_},
                         min_us_4_);

    set_or_default_array("max_us_4",
                         std::vector<int64_t>{(int64_t)max_us_scalar_, (int64_t)max_us_scalar_,
                                              (int64_t)max_us_scalar_, (int64_t)max_us_scalar_},
                         max_us_4_);

    // Optional per-wheel deadband in pct around 0 command: [fl, fr, rl, rr]
    set_or_default_array("deadband_pct_4",
                         std::vector<int64_t>{0, 0, 0, 0},
                         deadband_pct_4_);

    pct_min_ = declare_parameter<int>("pct_min", -100);
    pct_max_ = declare_parameter<int>("pct_max",  100);

    accel_ = declare_parameter<int>("acceleration", 0);
    watchdog_ms_ = declare_parameter<int>("watchdog_timeout_ms", 300);

    // prefer wheel_cmd4 whenever it arrives (recommended with 4-wheel kinematics)
    prefer_cmd4_ = declare_parameter<bool>("prefer_cmd4", true);

    // -------------------------
    // Parameters (Phidget encoders)
    // -------------------------
    phidget_serial_ = declare_parameter<int>("phidget_serial", -1);

    const auto enc_map_i64 =
      declare_parameter<std::vector<int64_t>>("encoder_channels", std::vector<int64_t>{2, 3, 0, 1});
    if (enc_map_i64.size() != 4) {
      throw std::runtime_error("encoder_channels must be 4 entries: [fl, fr, rl, rr]");
    }
    for (size_t i = 0; i < 4; ++i) encoder_channels_[i] = static_cast<int>(enc_map_i64[i]);

    invert_fl_ = declare_parameter<bool>("invert_fl", true);
    invert_fr_ = declare_parameter<bool>("invert_fr", false);
    invert_rl_ = declare_parameter<bool>("invert_rl", true);
    invert_rr_ = declare_parameter<bool>("invert_rr", false);

    publish_ticks_ms_ = declare_parameter<int>("publish_ticks_ms", 20);

    // -------------------------
    // ROS pubs/subs (USING PARAM TOPICS)
    // -------------------------
    sub_lr_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      cmd_lr_topic_, 10,
      std::bind(&HardwareNode::on_cmd_lr, this, std::placeholders::_1));

    sub_4_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      cmd_4_topic_, 10,
      std::bind(&HardwareNode::on_cmd_4, this, std::placeholders::_1));

    pub_ticks_ = create_publisher<base::msg::WheelTicks4>(ticks_topic_, 10);

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
    last_cmd_time_  = now();
    last_cmd4_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);

    timer_watchdog_ = create_wall_timer(50ms, std::bind(&HardwareNode::watchdog, this));
    timer_ticks_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, publish_ticks_ms_)),
      std::bind(&HardwareNode::publish_ticks, this));

    RCLCPP_INFO(get_logger(),
      "hardware_node started | I2C %s addr 0x%02x | wheel_map_valid=%d [fl=%d fr=%d rl=%d rr=%d] | "
      "topics cmd_lr=%s cmd4=%s ticks=%s | gains [%.3f %.3f %.3f %.3f] ignore_wheel_gains=%d accel=%d prefer_cmd4=%d",
      i2c_dev_.c_str(), i2c_addr_,
      (int)wheel_channels_valid_,
      wheel_channels_[0], wheel_channels_[1], wheel_channels_[2], wheel_channels_[3],
      cmd_lr_topic_.c_str(), cmd_4_topic_.c_str(), ticks_topic_.c_str(),
      wheel_gains_[0], wheel_gains_[1], wheel_gains_[2], wheel_gains_[3],
      (int)ignore_wheel_gains_, accel_, (int)prefer_cmd4_);

    RCLCPP_INFO(get_logger(),
      "PWM calib (FL/FR/RL/RR): k=[%.3f %.3f %.3f %.3f] neutral=[%ld %ld %ld %ld] "
      "min=[%ld %ld %ld %ld] max=[%ld %ld %ld %ld] deadband_pct=[%ld %ld %ld %ld]",
      k_us_per_pct_4_[0], k_us_per_pct_4_[1], k_us_per_pct_4_[2], k_us_per_pct_4_[3],
      neutral_us_4_[0], neutral_us_4_[1], neutral_us_4_[2], neutral_us_4_[3],
      min_us_4_[0], min_us_4_[1], min_us_4_[2], min_us_4_[3],
      max_us_4_[0], max_us_4_[1], max_us_4_[2], max_us_4_[3],
      deadband_pct_4_[0], deadband_pct_4_[1], deadband_pct_4_[2], deadband_pct_4_[3]);
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
  enum Wheel : size_t { FL=0, FR=1, RL=2, RR=3 };

  template<typename T>
  void set_or_default_array(const std::string& name,
                            const std::vector<T>& default_vec,
                            std::array<T,4>& out_arr) {
    auto v = declare_parameter<std::vector<T>>(name, default_vec);
    if (v.size() != 4) {
      throw std::runtime_error(name + " must have 4 entries: [fl, fr, rl, rr]");
    }
    for (size_t i = 0; i < 4; ++i) out_arr[i] = v[i];
  }

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

  int pct_to_pulse_us(size_t wheel_idx, int pct) const {
    pct = clamp_int(pct, pct_min_, pct_max_);
    const double k  = k_us_per_pct_4_[wheel_idx];
    const double n  = static_cast<double>(neutral_us_4_[wheel_idx]);
    const double us_f = static_cast<double>(pct) * k + n;
    int us = static_cast<int>(std::lround(us_f));

    const int min_us = (int)min_us_4_[wheel_idx];
    const int max_us = (int)max_us_4_[wheel_idx];
    return clamp_int(us, min_us, max_us);
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

  void apply_wheel(size_t wheel_idx, int servo_channel, int pct) {
    const int db = (int)deadband_pct_4_[wheel_idx];
    if (std::abs(pct) < db) pct = 0;

    pct = clamp_int(pct, pct_min_, pct_max_);
    const int pulse = pct_to_pulse_us(wheel_idx, pct);
    write_channel_pulse(servo_channel, pulse);
  }

  void stop_all() {
    apply_all_wheels(0, 0, 0, 0);
  }

  bool derive_wheel_channels_from_sides() {
    if (left_channels_.size() >= 2 && right_channels_.size() >= 2) {
      wheel_channels_[FL] = left_channels_[0];
      wheel_channels_[FR] = right_channels_[0];
      wheel_channels_[RL] = left_channels_[1];
      wheel_channels_[RR] = right_channels_[1];
      return true;
    }
    wheel_channels_ = {0, 1, 2, 3};
    return false;
  }

  void apply_all_wheels(int fl, int fr, int rl, int rr) {
    fl = clamp_int(fl, pct_min_, pct_max_);
    fr = clamp_int(fr, pct_min_, pct_max_);
    rl = clamp_int(rl, pct_min_, pct_max_);
    rr = clamp_int(rr, pct_min_, pct_max_);

    if (!ignore_wheel_gains_) {
      fl = apply_gain(fl, wheel_gains_[FL]);
      fr = apply_gain(fr, wheel_gains_[FR]);
      rl = apply_gain(rl, wheel_gains_[RL]);
      rr = apply_gain(rr, wheel_gains_[RR]);
    }

    apply_wheel(FL, wheel_channels_[FL], fl);
    apply_wheel(FR, wheel_channels_[FR], fr);
    apply_wheel(RL, wheel_channels_[RL], rl);
    apply_wheel(RR, wheel_channels_[RR], rr);
  }

  // -------------------------
  // ROS callbacks
  // -------------------------
  void on_cmd_lr(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;

    // if cmd4 is preferred and recently received -> ignore LR to avoid fighting
    if (prefer_cmd4_) {
      const auto dt = now() - last_cmd4_time_;
      if (last_cmd4_time_.nanoseconds() != 0 &&
          dt.nanoseconds() < (int64_t)200 * 1000000LL) { // 200 ms
        return;
      }
    }

    const int left_pct_in  = clamp_int((int)msg->data[0], pct_min_, pct_max_);
    const int right_pct_in = clamp_int((int)msg->data[1], pct_min_, pct_max_);

    try {
      apply_all_wheels(left_pct_in, right_pct_in, left_pct_in, right_pct_in);
      last_cmd_time_ = now();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "I2C write failed (LR): %s", e.what());
    }
  }

  void on_cmd_4(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) return;

    int fl = clamp_int((int)msg->data[0], pct_min_, pct_max_);
    int fr = clamp_int((int)msg->data[1], pct_min_, pct_max_);
    int rl = clamp_int((int)msg->data[2], pct_min_, pct_max_);
    int rr = clamp_int((int)msg->data[3], pct_min_, pct_max_);

    try {
      apply_all_wheels(fl, fr, rl, rr);
      last_cmd_time_  = now();
      last_cmd4_time_ = last_cmd_time_;
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
    invert_[FL] = invert_fl_;
    invert_[FR] = invert_fr_;
    invert_[RL] = invert_rl_;
    invert_[RR] = invert_rr_;

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
    m.header.frame_id = ticks_frame_id_;
    m.fl_ticks = ticks_[FL].load(std::memory_order_relaxed);
    m.fr_ticks = ticks_[FR].load(std::memory_order_relaxed);
    m.rl_ticks = ticks_[RL].load(std::memory_order_relaxed);
    m.rr_ticks = ticks_[RR].load(std::memory_order_relaxed);
    pub_ticks_->publish(m);
  }

  // -------------------------
  // Members
  // -------------------------
  // ROS topics
  std::string cmd_lr_topic_;
  std::string cmd_4_topic_;
  std::string ticks_topic_;
  std::string ticks_frame_id_;

  // I2C/motor
  std::string i2c_dev_;
  int i2c_addr_{0};

  std::vector<int> left_channels_;
  std::vector<int> right_channels_;

  std::array<int,4> wheel_channels_{ {2, 3, 0, 1} };
  bool wheel_channels_valid_{false};

  std::array<double,4> wheel_gains_{ {1.0, 1.0, 1.0, 1.0} }; // fl, fr, rl, rr
  bool ignore_wheel_gains_{true};

  // PWM calibration
  double k_us_per_pct_scalar_{5.5};
  int neutral_us_scalar_{1480};
  int min_us_scalar_{900};
  int max_us_scalar_{2100};

  std::array<double,4>  k_us_per_pct_4_{ {5.5, 5.5, 5.5, 5.5} };
  std::array<int64_t,4> neutral_us_4_{ {1480,1480,1480,1480} };
  std::array<int64_t,4> min_us_4_{ {900,900,900,900} };
  std::array<int64_t,4> max_us_4_{ {2100,2100,2100,2100} };
  std::array<int64_t,4> deadband_pct_4_{ {0,0,0,0} };

  int pct_min_{-100};
  int pct_max_{100};
  int accel_{0};
  int watchdog_ms_{300};

  bool prefer_cmd4_{true};

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
  rclcpp::Time last_cmd4_time_;
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
