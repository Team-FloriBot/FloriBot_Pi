#include "base/hardware_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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

using namespace std::chrono_literals;

// ---------------------------
// Small helpers
// ---------------------------
static int clamp_int(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }

static void phidget_check(PhidgetReturnCode rc, const char* what) {
  if (rc != EPHIDGET_OK) {
    const char* err = nullptr;
    Phidget_getErrorDescription(rc, &err);
    throw std::runtime_error(std::string(what) + " failed: " + (err ? err : "unknown"));
  }
}

// ---------------------------
// I2C low-level wrapper
// ---------------------------
class HardwareNode::NxtServoI2C {
public:
  NxtServoI2C(std::string dev, int addr) : dev_(std::move(dev)), addr_(addr) {}

  void open_bus() {
    fd_ = ::open(dev_.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("open(" + dev_ + ") failed");
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("ioctl(I2C_SLAVE) failed");
    }
  }

  ~NxtServoI2C() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }

  void write1(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (::write(fd_, buf, 2) != 2) throw std::runtime_error("I2C write1 failed");
  }

  void write2(uint8_t reg, uint8_t lo, uint8_t hi) {
    uint8_t buf[3] = {reg, lo, hi};
    if (::write(fd_, buf, 3) != 3) throw std::runtime_error("I2C write2 failed");
  }

private:
  std::string dev_;
  int addr_{0};
  int fd_{-1};
};

// ---------------------------
// NXT-Servo driver (4 channels)
// ---------------------------
class HardwareNode::NxtServoDriver {
public:
  NxtServoDriver(std::unique_ptr<NxtServoI2C> i2c,
                 std::array<int,4> servo_channels,
                 std::array<double,4> k_us_per_pct,
                 std::array<int,4> neutral_us,
                 std::array<int,4> min_us,
                 std::array<int,4> max_us,
                 int pct_min, int pct_max)
  : i2c_(std::move(i2c)),
    ch_(servo_channels),
    k_(k_us_per_pct),
    neutral_(neutral_us),
    min_(min_us),
    max_(max_us),
    pct_min_(pct_min),
    pct_max_(pct_max)
  {}

  void open() { i2c_->open_bus(); }

  void set_acceleration(int channel, int accel) {
    // same register scheme as existing code: reg = channel + 82
    const uint8_t reg = static_cast<uint8_t>(channel + 82);
    i2c_->write1(reg, static_cast<uint8_t>(clamp_int(accel, 0, 255)));
  }

  void set_acceleration_all(int accel) {
    accel = clamp_int(accel, 0, 255);
    for (int ch = 0; ch < 4; ++ch) set_acceleration(ch, accel);
  }

  void set_pulse_us(int channel, int pulse_us) {
    pulse_us = clamp_int(pulse_us, 500, 2500);
    // register scheme: channel*2
    const uint8_t reg = static_cast<uint8_t>(channel * 2);
    const uint8_t lo = static_cast<uint8_t>(pulse_us & 0xFF);
    const uint8_t hi = static_cast<uint8_t>((pulse_us >> 8) & 0xFF);
    i2c_->write2(reg, lo, hi);
  }

  void stop_all() {
    for (int w = 0; w < 4; ++w) set_pct(static_cast<size_t>(w), 0);
  }

  void set_pct(size_t wheel_idx, int pct) {
    pct = clamp_int(pct, pct_min_, pct_max_);
    const int pulse = clamp_int(static_cast<int>(std::lround(neutral_[wheel_idx] + k_[wheel_idx] * pct)),
                                min_[wheel_idx], max_[wheel_idx]);
    set_pulse_us(ch_[wheel_idx], pulse);
  }

  void set_all(int fl, int fr, int rl, int rr) {
    set_pct(0, fl); set_pct(1, fr); set_pct(2, rl); set_pct(3, rr);
  }

private:
  std::unique_ptr<NxtServoI2C> i2c_;
  std::array<int,4> ch_;
  std::array<double,4> k_;
  std::array<int,4> neutral_, min_, max_;
  int pct_min_{-100}, pct_max_{100};
};

// ---------------------------
// Encoder callback
// ---------------------------
static void onPositionChange(PhidgetEncoderHandle, void* ctx, int, int64_t positionChange, double) {
  auto* p = static_cast<std::pair<std::atomic<int64_t>*, bool>*>(ctx);
  if (!p) return;
  int64_t delta = positionChange;
  if (p->second) delta = -delta;
  p->first->fetch_add(delta, std::memory_order_relaxed);
}

// ---------------------------
// HardwareNode
// ---------------------------
HardwareNode::HardwareNode()
: Node("hardware_node")
{
  // Topics
  wheel_cmd_topic_   = declare_parameter<std::string>("wheel_cmd_topic", "/wheel_commands");
  wheel_ticks_topic_ = declare_parameter<std::string>("wheel_ticks_topic", "/base/wheel_ticks4");
  ticks_frame_id_    = declare_parameter<std::string>("ticks_frame_id", "base_link");

  // Encoder params
  phidget_serial_ = declare_parameter<int>("phidget_serial", -1);
  {
    auto v = declare_parameter<std::vector<int>>("encoder_channels", {2,3,0,1});
    for (size_t i=0;i<4 && i<v.size();++i) encoder_channels_[i]=v[i];
  }
  invert_[FL] = declare_parameter<bool>("invert_fl", true);
  invert_[FR] = declare_parameter<bool>("invert_fr", false);
  invert_[RL] = declare_parameter<bool>("invert_rl", true);
  invert_[RR] = declare_parameter<bool>("invert_rr", false);

  ticks_per_rev_ = declare_parameter<int64_t>("ticks_per_rev", 131000);

  // Motor controller params
  i2c_dev_  = declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
  i2c_addr_ = declare_parameter<int>("i2c_addr", 0x58);

  {
    auto v = declare_parameter<std::vector<int>>("servo_channels", {0,1,2,3});
    for (size_t i=0;i<4 && i<v.size();++i) servo_channels_[i]=v[i];
  }

  {
    auto v = declare_parameter<std::vector<double>>("k_us_per_pct_4", {5.5,5.5,5.5,5.5});
    for (size_t i=0;i<4 && i<v.size();++i) k_us_per_pct_[i]=v[i];
  }
  {
    auto v = declare_parameter<std::vector<int>>("neutral_us_4", {1480,1480,1480,1480});
    for (size_t i=0;i<4 && i<v.size();++i) neutral_us_[i]=v[i];
  }
  {
    auto v = declare_parameter<std::vector<int>>("min_us_4", {900,900,900,900});
    for (size_t i=0;i<4 && i<v.size();++i) min_us_[i]=v[i];
  }
  {
    auto v = declare_parameter<std::vector<int>>("max_us_4", {2100,2100,2100,2100});
    for (size_t i=0;i<4 && i<v.size();++i) max_us_[i]=v[i];
  }

  pct_min_ = declare_parameter<int>("pct_min", -100);
  pct_max_ = declare_parameter<int>("pct_max", 100);
  accel_   = declare_parameter<int>("accel", 100);

  // Control params
  max_wheel_radps_      = declare_parameter<double>("max_wheel_radps", 20.0);
  watchdog_timeout_s_   = declare_parameter<double>("watchdog_timeout_s", 0.3);
  control_period_ms_    = declare_parameter<int>("control_period_ms", 10);
  publish_ticks_ms_     = declare_parameter<int>("publish_ticks_ms", 20);

  // PID params (Floribot1.0 style, per-wheel optional)
  const double kp = declare_parameter<double>("kp", 2.0);
  const double ki = declare_parameter<double>("ki", 0.5);
  const double kd = declare_parameter<double>("kd", 0.0);
  const double out_lim = declare_parameter<double>("output_limit", 1.0);
  const double deadband = declare_parameter<double>("deadband", 0.05);
  const double max_accel = declare_parameter<double>("setpoint_max_accel", 2.0);

  for (size_t i=0;i<4;++i) {
    pid_[i] = std::make_unique<base::PIDController>(kp,ki,kd,out_lim,deadband,max_accel);
    ticks_[i].store(0, std::memory_order_relaxed);
  }

  // ROS
  sub_ = create_subscription<base::msg::WheelVelocities>(
    wheel_cmd_topic_, 10,
    std::bind(&HardwareNode::onWheelCmd, this, std::placeholders::_1));

  pub_ticks_ = create_publisher<base::msg::WheelTicks4>(wheel_ticks_topic_, 10);

  // Init I2C driver
  i2c_ = std::make_unique<NxtServoI2C>(i2c_dev_, i2c_addr_);
  driver_ = std::make_unique<NxtServoDriver>(std::move(i2c_), servo_channels_, k_us_per_pct_, neutral_us_, min_us_, max_us_, pct_min_, pct_max_);
  driver_->open();
  driver_->set_acceleration_all(accel_);
  driver_->stop_all();
  stopped_ = true;

  // Encoders
  initEncoders();

  last_cmd_time_ = now();
  last_control_time_ = now();

  timer_watchdog_ = create_wall_timer(50ms, std::bind(&HardwareNode::watchdog, this));
  timer_control_  = create_wall_timer(std::chrono::milliseconds(std::max(1, control_period_ms_)),
                                      std::bind(&HardwareNode::controlLoop, this));
  timer_ticks_    = create_wall_timer(std::chrono::milliseconds(std::max(1, publish_ticks_ms_)),
                                      std::bind(&HardwareNode::publishTicks, this));

  RCLCPP_INFO(get_logger(),
    "hardware_node started | cmd=%s | ticks=%s | i2c=%s addr=0x%02x | ticks_per_rev=%ld | max_wheel_radps=%.2f",
    wheel_cmd_topic_.c_str(), wheel_ticks_topic_.c_str(), i2c_dev_.c_str(), i2c_addr_,
    static_cast<long>(ticks_per_rev_), max_wheel_radps_);
}

HardwareNode::~HardwareNode() {
  try {
    if (driver_) driver_->stop_all();
  } catch (...) {}
  closeEncoders();
}

void HardwareNode::onWheelCmd(const base::msg::WheelVelocities::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  // 4-wheel diff drive: left command applies to FL & RL; right to FR & RR
  target_w_radps_[FL] = msg->left;
  target_w_radps_[RL] = msg->left;
  target_w_radps_[FR] = msg->right;
  target_w_radps_[RR] = msg->right;
  last_cmd_time_ = now();
}

void HardwareNode::watchdog() {
  const auto t = now();
  if ((t - last_cmd_time_).seconds() > watchdog_timeout_s_) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto& p : pid_) p->reset();
    target_w_radps_ = {0,0,0,0};
  }
}

void HardwareNode::controlLoop() {
  const auto t = now();
  const double dt = (t - last_control_time_).seconds();
  if (dt <= 0.0) return;
  last_control_time_ = t;

  // read ticks and compute wheel angular velocities
  const double ticks_to_rad = 2.0 * M_PI / static_cast<double>(ticks_per_rev_);

  std::array<double,4> w_meas{0,0,0,0};
  std::array<int64_t,4> ticks_now;
  for (size_t i=0;i<4;++i) {
    ticks_now[i] = ticks_[i].load(std::memory_order_relaxed);
    const int64_t d = ticks_now[i] - last_ticks_[i];
    last_ticks_[i] = ticks_now[i];
    w_meas[i] = (static_cast<double>(d) * ticks_to_rad) / dt;
  }

  // copy targets
  std::array<double,4> w_sp;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    w_sp = target_w_radps_;
  }

  // normalize and compute control output u in [-1,1]
  auto clamp01 = [](double v){ return std::max(-1.0, std::min(1.0, v)); };
  std::array<int,4> pct{0,0,0,0};

  for (size_t i=0;i<4;++i) {
    const double sp_n = clamp01(w_sp[i] / max_wheel_radps_);
    const double ms_n = clamp01(w_meas[i] / max_wheel_radps_);
    const double u = pid_[i]->compute(sp_n, ms_n, dt); // already includes deadband + saturation
    pct[i] = static_cast<int>(std::lround(100.0 * clamp01(u)));
  }

  // apply
  driver_->set_all(pct[FL], pct[FR], pct[RL], pct[RR]);
  stopped_ = (pct[FL]==0 && pct[FR]==0 && pct[RL]==0 && pct[RR]==0);
}

void HardwareNode::publishTicks() {
  base::msg::WheelTicks4 m;
  m.header.stamp = now();
  m.header.frame_id = ticks_frame_id_;
  m.fl_ticks = ticks_[FL].load(std::memory_order_relaxed);
  m.fr_ticks = ticks_[FR].load(std::memory_order_relaxed);
  m.rl_ticks = ticks_[RL].load(std::memory_order_relaxed);
  m.rr_ticks = ticks_[RR].load(std::memory_order_relaxed);
  pub_ticks_->publish(m);
}

void HardwareNode::initEncoders() {
  // create a small ctx objects to pass invert + atomic ptr into callback
  static std::array<std::pair<std::atomic<int64_t>*, bool>,4> ctx;

  for (size_t i=0;i<4;++i) {
    PhidgetEncoderHandle h=nullptr;
    phidget_check(PhidgetEncoder_create(&h), "PhidgetEncoder_create");
    phidget_check(Phidget_setDeviceSerialNumber((PhidgetHandle)h, phidget_serial_), "Phidget_setDeviceSerialNumber");
    phidget_check(Phidget_setChannel((PhidgetHandle)h, encoder_channels_[i]), "Phidget_setChannel");
    phidget_check(Phidget_setHubPort((PhidgetHandle)h, 0), "Phidget_setHubPort");

    ctx[i] = { &ticks_[i], invert_[i] };
    phidget_check(PhidgetEncoder_setOnPositionChangeHandler(h, onPositionChange, &ctx[i]),
                  "PhidgetEncoder_setOnPositionChangeHandler");

    phidget_check(Phidget_openWaitForAttachment((PhidgetHandle)h, 5000), "Phidget_openWaitForAttachment");
    enc_[i] = (void*)h;
    ticks_[i].store(0, std::memory_order_relaxed);

    RCLCPP_INFO(get_logger(), "Encoder[%zu] attached | channel=%d invert=%d", i, encoder_channels_[i], (int)invert_[i]);
  }
}

void HardwareNode::closeEncoders() {
  for (size_t i=0;i<4;++i) {
    if (!enc_[i]) continue;
    auto h = (PhidgetEncoderHandle)enc_[i];
    try {
      Phidget_close((PhidgetHandle)h);
      PhidgetEncoder_delete(&h);
    } catch (...) {}
    enc_[i]=nullptr;
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareNode>());
  rclcpp::shutdown();
  return 0;
}
