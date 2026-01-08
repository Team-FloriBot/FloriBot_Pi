#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <algorithm>
#include <array>
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
    if (fd_ < 0) {
      throw std::runtime_error("open(" + dev_ + ") failed");
    }
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
    std::string msg = std::string(what) + " failed: " + (err ? err : "unknown");
    throw std::runtime_error(msg);
  }
}

class HardwareNode : public rclcpp::Node {
public:
  HardwareNode() : Node("hardware_node") {
    // --- I2C params ---
    i2c_dev_  = declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
    i2c_addr_ = declare_parameter<int>("i2c_addr", 0x58);

    // Vector params should be int64_t in ROS parameters
    const auto left_i64  =
        declare_parameter<std::vector<int64_t>>("left_channels",  std::vector<int64_t>{0, 2});
    const auto right_i64 =
        declare_parameter<std::vector<int64_t>>("right_channels", std::vector<int64_t>{1, 3});
    left_channels_.clear();
    right_channels_.clear();
    for (auto v : left_i64)  left_channels_.push_back(static_cast<int>(v));
    for (auto v : right_i64) right_channels_.push_back(static_cast<int>(v));

    // PWM mapping: us = speed*5.5 + 1480
    k_us_per_pct_ = declare_parameter<double>("k_us_per_pct", 5.5);
    neutral_us_   = declare_parameter<int>("neutral_us", 1480);
    min_us_       = declare_parameter<int>("min_us", 900);
    max_us_       = declare_parameter<int>("max_us", 2100);

    // speed clamp
    pct_min_ = declare_parameter<int>("pct_min", -100);
    pct_max_ = declare_parameter<int>("pct_max",  100);

    // acceleration register (0..255)
    accel_ = declare_parameter<int>("acceleration", 30);

    watchdog_ms_ = declare_parameter<int>("watchdog_timeout_ms", 300);

    // --- Encoder (Phidget22) params ---
    // If you have only one device: leave -1 (any)
    phidget_serial_ = declare_parameter<int>("phidget_serial", -1);

    // Which Phidget channels correspond to wheels (fl, fr, rl, rr)
    // default assumes: channel 0=fl, 1=fr, 2=rl, 3=rr
    const auto enc_map_i64 =
        declare_parameter<std::vector<int64_t>>("encoder_channels", std::vector<int64_t>{0,1,2,3});
    if (enc_map_i64.size() != 4) {
      throw std::runtime_error("encoder_channels must have 4 entries (fl,fr,rl,rr)");
    }
    for (size_t i=0;i<4;i++) encoder_channels_[i] = static_cast<int>(enc_map_i64[i]);

    invert_fl_ = declare_parameter<bool>("invert_fl", false);
    invert_fr_ = declare_parameter<bool>("invert_fr", false);
    invert_rl_ = declare_parameter<bool>("invert_rl", false);
    invert_rr_ = declare_parameter<bool>("invert_rr", false);

    publish_ticks_ms_ = declare_parameter<int>("publish_ticks_ms", 20);

    // Subscriber (motor command)
    sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        "/base/wheel_cmd", 10,
        std::bind(&HardwareNode::on_cmd, this, std::placeholders::_1));

    // Publisher (ticks)
    pub_ticks_ = create_publisher<base::msg::WheelTicks4>("/base/wheel_ticks4", 10);

    // Open I2C
    nxt_ = std::make_unique<NxtServoI2C>(i2c_dev_, i2c_addr_);
    nxt_->open_bus();

    // Apply acceleration and force STOP at startup (safety)
    set_acceleration_all(accel_);
    stop_all();

    // Init Phidgets
    init_encoders();

    last_cmd_time_ = now();
    timer_watchdog_ = create_wall_timer(50ms, std::bind(&HardwareNode::watchdog, this));

    timer_ticks_ = create_wall_timer(
      std::chrono::milliseconds(publish_ticks_ms_),
      std::bind(&HardwareNode::publish_ticks, this));

    RCLCPP_INFO(get_logger(),
      "hardware_node started (I2C %s addr 0x%02x, Phidget serial=%d, tick_pub=%dms)",
      i2c_dev_.c_str(), i2c_addr_, phidget_serial_, publish_ticks_ms_);
  }

  ~HardwareNode() override {
    // Stop motors on shutdown
    try { stop_all(); } catch (...) {}

    // Close encoders
    for (auto &e : enc_) {
      if (e) {
        Phidget_close((PhidgetHandle)e);
        PhidgetEncoder_delete(&e);
        e = nullptr;
      }
    }
  }

private:
  static int clamp_int(int v, int lo, int hi) {
    return std::max(lo, std::min(v, hi));
  }

  int pct_to_pulse_us(int pct) const {
    const double us_f = static_cast<double>(pct) * k_us_per_pct_ + static_cast<double>(neutral_us_);
    int us = static_cast<int>(std::lround(us_f));
    us = clamp_int(us, min_us_, max_us_);
    return us;
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

  void stop_all() {
    apply_side(left_channels_,  0);
    apply_side(right_channels_, 0);
  }

  void on_cmd(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;

    const int left_pct  = clamp_int(static_cast<int>(msg->data[0]), pct_min_, pct_max_);
    const int right_pct = clamp_int(static_cast<int>(msg->data[1]), pct_min_, pct_max_);

    try {
      apply_side(left_channels_,  left_pct);
      apply_side(right_channels_, right_pct);
      last_cmd_time_ = now();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "I2C write failed: %s", e.what());
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

  // --------- Phidget22 encoder handling ----------
  static void CCONV onPositionChange(PhidgetEncoderHandle /*ch*/, void *ctx, int64_t positionChange, double /*timeChange*/) {
    auto* self = static_cast<HardwareNode*>(ctx);
    // Determine which encoder fired by scanning handles (only 4, so OK)
    // Accumulate delta ticks
    // NOTE: positionChange is signed
    for (size_t i = 0; i < 4; ++i) {
      // We cannot compare handle here without the handle parameter. If needed, change signature to accept ch and compare.
    }
    (void)positionChange;
  }

  static void CCONV onPositionChangeWithHandle(PhidgetEncoderHandle ch, void *ctx, int64_t positionChange, double /*timeChange*/) {
    auto* self = static_cast<HardwareNode*>(ctx);
    for (size_t i = 0; i < 4; ++i) {
      if (self->enc_[i] == ch) {
        // Apply per-wheel inversion
        const int64_t delta = self->invert_[i] ? -positionChange : positionChange;
        self->ticks_[i] += delta;
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
        phidget_check(Phidget_setDeviceSerialNumber((PhidgetHandle)h, phidget_serial_), "Phidget_setDeviceSerialNumber");
      }

      // Channel on device
      phidget_check(Phidget_setChannel((PhidgetHandle)h, encoder_channels_[i]), "Phidget_setChannel");

      // Optional: set hub port / isHubPortDevice if needed (leave default unless required)

      phidget_check(PhidgetEncoder_setOnPositionChangeHandler(h, &HardwareNode::onPositionChangeWithHandle, this),
                    "PhidgetEncoder_setOnPositionChangeHandler");

      // Open and attach
      phidget_check(Phidget_openWaitForAttachment((PhidgetHandle)h, 5000), "Phidget_openWaitForAttachment");

      // Reset position to 0 so ticks start at 0
      phidget_check(PhidgetEncoder_setPosition(h, 0), "PhidgetEncoder_setPosition");

      enc_[i] = h;
      ticks_[i] = 0;

      RCLCPP_INFO(get_logger(), "Phidget encoder %zu attached: channel=%d invert=%d",
                  i, encoder_channels_[i], (int)invert_[i]);
    }
  }

  void publish_ticks() {
    base::msg::WheelTicks4 m;
    m.header.stamp = now();
    m.header.frame_id = "base_link";
    m.fl_ticks = ticks_[0];
    m.fr_ticks = ticks_[1];
    m.rl_ticks = ticks_[2];
    m.rr_ticks = ticks_[3];
    pub_ticks_->publish(m);
  }

  // --------- Members ----------
  std::string i2c_dev_;
  int i2c_addr_{0};

  std::vector<int> left_channels_;
  std::vector<int> right_channels_;

  double k_us_per_pct_{5.5};
  int neutral_us_{1480};
  int min_us_{900};
  int max_us_{2100};
  int pct_min_{-100};
  int pct_max_{100};
  int accel_{30};
  int watchdog_ms_{300};

  int phidget_serial_{-1};
  std::array<int,4> encoder_channels_{ {0,1,2,3} };
  std::array<bool,4> invert_{ {false,false,false,false} };
  bool invert_fl_{false}, invert_fr_{false}, invert_rl_{false}, invert_rr_{false};
  int publish_ticks_ms_{20};

  // Encoder handles and tick counters
  std::array<PhidgetEncoderHandle,4> enc_{ {nullptr,nullptr,nullptr,nullptr} };
  std::array<int64_t,4> ticks_{ {0,0,0,0} };

  rclcpp::Time last_cmd_time_;
  rclcpp::TimerBase::SharedPtr timer_watchdog_;
  rclcpp::TimerBase::SharedPtr timer_ticks_;

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr pub_ticks_;

  std::unique_ptr<NxtServoI2C> nxt_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareNode>());
  rclcpp::shutdown();
  return 0;
}
