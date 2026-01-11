#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
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
#include "base/msg/wheel_velocities.hpp"
#include "base/pid_controller.h"

class NxtServoI2C
{
public:
  void openBus(const std::string& dev, int addr_7bit)
  {
    fd_ = ::open(dev.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error("I2C open failed: " + std::string(std::strerror(errno)));
    }
    if (ioctl(fd_, I2C_SLAVE, addr_7bit) < 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("I2C ioctl(I2C_SLAVE) failed: " + std::string(std::strerror(errno)));
    }
  }

  void closeBus()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  ~NxtServoI2C() { closeBus(); }

  void writePulseUs(int channel, int pulse_us)
  {
    const uint8_t reg  = static_cast<uint8_t>(channel * 2 + 66);
    const uint8_t low  = static_cast<uint8_t>(pulse_us & 0xFF);
    const uint8_t high = static_cast<uint8_t>((pulse_us >> 8) & 0xFF);

    uint8_t buf[3]{reg, low, high};
    const ssize_t n = ::write(fd_, buf, 3);
    if (n != 3) {
      throw std::runtime_error("I2C writePulseUs failed: " + std::string(std::strerror(errno)));
    }
  }

private:
  int fd_{-1};
};

static void phidget_check(PhidgetReturnCode rc, const char* what)
{
  if (rc != EPHIDGET_OK) {
    const char* err = nullptr;
    Phidget_getErrorDescription(rc, &err);
    throw std::runtime_error(std::string(what) + " failed: " + (err ? err : "unknown"));
  }
}

static inline int clampInt(int v, int lo, int hi)
{
  return std::max(lo, std::min(v, hi));
}

static inline double clampDouble(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

static inline int signum(double x)
{
  return (x > 0.0) - (x < 0.0);
}

class HardwareNode : public rclcpp::Node
{
public:
  HardwareNode()
  : Node("hardware_node")
  {
    declareParams();
    getParams();

    nxt_.openBus(i2c_dev_, i2c_addr_);

    wheel_cmd_sub_ = create_subscription<base::msg::WheelVelocities>(
      wheel_cmd_topic_, rclcpp::QoS(10),
      std::bind(&HardwareNode::onWheelCmd, this, std::placeholders::_1));

    ticks_pub_ = create_publisher<base::msg::WheelTicks4>(wheel_ticks_topic_, rclcpp::QoS(10));

    initEncoders();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(control_dt_),
      std::bind(&HardwareNode::controlLoop, this));

    ticks_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(5, publish_ticks_ms_)),
      std::bind(&HardwareNode::publishTicks, this));

    last_speed_eval_time_ = now();

    RCLCPP_INFO(get_logger(),
      "hardware_node: dt=%.3f speed_window=%.3f ticks_per_rev=%.1f | u_min=%.2f u_kick=%.2f kick_time=%.2f",
      control_dt_, speed_window_s_, ticks_per_rev_, u_min_, u_kick_, kick_time_s_);
  }

  ~HardwareNode() override
  {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      try { writeMotor(i, 0.0); } catch (...) {}
    }

    for (auto &h : enc_) {
      if (h) {
        Phidget_close((PhidgetHandle)h);
        PhidgetEncoder_delete(&h);
        h = nullptr;
      }
    }

    nxt_.closeBus();
  }

private:
  enum WheelIndex { FL = 0, FR = 1, RL = 2, RR = 3, WHEEL_COUNT = 4 };

  // ---------------- Params ----------------
  std::string wheel_cmd_topic_{"/wheel_commands"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};
  std::string ticks_frame_id_{"base_link"};

  double control_dt_{0.02};

  // ticks per WHEEL revolution
  double ticks_per_rev_{131000.0};

  // PID
  double kp_{0.06};
  double ki_{0.30};
  double kd_{0.0};
  double output_limit_{1.0};
  double deadband_{0.10};
  double setpoint_max_accel_{1.5};
  double stop_eps_radps_{0.05};

  // Measurement filter
  double meas_alpha_{0.25};
  double meas_w_eps_{0.03};

  // speed estimation window (key for low-speed stability)
  double speed_window_s_{0.10}; // 100 ms

  // Output slew
  double u_slew_rate_{1.0}; // 1/s, 0 disables

  // Stiction compensation
  double u_min_{0.18};        // minimum output when moving
  double u_kick_{0.0};        // optional start kick (additional)
  double kick_time_s_{0.0};   // duration after sign change / start

  // I2C
  std::string i2c_dev_{"/dev/i2c-1"};
  int i2c_addr_{0x58};

  std::array<int,4> wheel_channels_{{2, 3, 0, 1}};
  std::array<double,4> wheel_gains_{{1.0, 1.0, 1.0, 1.0}};

  double k_us_per_pct_{5.5};
  int neutral_us_{1480};
  int min_us_{900};
  int max_us_{2100};
  int pct_min_{-100};
  int pct_max_{100};

  // Phidget
  int phidget_serial_{-1};
  std::array<int,4> encoder_channels_{{2, 3, 0, 1}};
  std::array<bool,4> invert_{{true, false, true, false}};

  int publish_ticks_ms_{20};

  // ---------------- ROS ----------------
  rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr wheel_cmd_sub_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr ticks_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr ticks_timer_;

  // ---------------- State ----------------
  NxtServoI2C nxt_;

  double target_w_radps_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  std::array<std::unique_ptr<base::PIDController>, WHEEL_COUNT> pid_{};
  double u_last_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  std::array<std::atomic<int64_t>, WHEEL_COUNT> ticks_{};
  int64_t ticks_last_speed_[WHEEL_COUNT]{0, 0, 0, 0};

  double meas_w_filt_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  rclcpp::Time last_speed_eval_time_;

  // kick state
  rclcpp::Time kick_until_[WHEEL_COUNT];
  int last_dir_[WHEEL_COUNT]{0,0,0,0};

  std::array<PhidgetEncoderHandle, WHEEL_COUNT> enc_{{nullptr, nullptr, nullptr, nullptr}};

  // ---------------- Params ----------------
  void declareParams()
  {
    declare_parameter<std::string>("wheel_cmd_topic", wheel_cmd_topic_);
    declare_parameter<std::string>("wheel_ticks_topic", wheel_ticks_topic_);
    declare_parameter<std::string>("ticks_frame_id", ticks_frame_id_);

    declare_parameter<double>("control_dt", control_dt_);
    declare_parameter<double>("ticks_per_rev", ticks_per_rev_);

    declare_parameter<double>("kp", kp_);
    declare_parameter<double>("ki", ki_);
    declare_parameter<double>("kd", kd_);
    declare_parameter<double>("output_limit", output_limit_);
    declare_parameter<double>("deadband", deadband_);
    declare_parameter<double>("setpoint_max_accel", setpoint_max_accel_);
    declare_parameter<double>("stop_eps_radps", stop_eps_radps_);

    declare_parameter<double>("meas_alpha", meas_alpha_);
    declare_parameter<double>("meas_w_eps", meas_w_eps_);

    declare_parameter<double>("speed_window_s", speed_window_s_);

    declare_parameter<double>("u_slew_rate", u_slew_rate_);

    declare_parameter<double>("u_min", u_min_);
    declare_parameter<double>("u_kick", u_kick_);
    declare_parameter<double>("kick_time_s", kick_time_s_);

    declare_parameter<std::string>("i2c_dev", i2c_dev_);
    declare_parameter<int>("i2c_addr", i2c_addr_);

    declare_parameter<std::vector<int64_t>>("wheel_channels", std::vector<int64_t>{2, 3, 0, 1});
    declare_parameter<std::vector<double>>("wheel_gains", std::vector<double>{1.0, 1.0, 1.0, 1.0});

    declare_parameter<double>("k_us_per_pct", k_us_per_pct_);
    declare_parameter<int>("neutral_us", neutral_us_);
    declare_parameter<int>("min_us", min_us_);
    declare_parameter<int>("max_us", max_us_);
    declare_parameter<int>("pct_min", pct_min_);
    declare_parameter<int>("pct_max", pct_max_);

    declare_parameter<int>("phidget_serial", phidget_serial_);
    declare_parameter<std::vector<int64_t>>("encoder_channels", std::vector<int64_t>{2, 3, 0, 1});

    declare_parameter<bool>("invert_fl", invert_[FL]);
    declare_parameter<bool>("invert_fr", invert_[FR]);
    declare_parameter<bool>("invert_rl", invert_[RL]);
    declare_parameter<bool>("invert_rr", invert_[RR]);

    declare_parameter<int>("publish_ticks_ms", publish_ticks_ms_);
  }

  void getParams()
  {
    wheel_cmd_topic_   = get_parameter("wheel_cmd_topic").as_string();
    wheel_ticks_topic_ = get_parameter("wheel_ticks_topic").as_string();
    ticks_frame_id_    = get_parameter("ticks_frame_id").as_string();

    control_dt_    = get_parameter("control_dt").as_double();
    ticks_per_rev_ = get_parameter("ticks_per_rev").as_double();

    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    output_limit_ = get_parameter("output_limit").as_double();
    deadband_ = get_parameter("deadband").as_double();
    setpoint_max_accel_ = get_parameter("setpoint_max_accel").as_double();
    stop_eps_radps_ = get_parameter("stop_eps_radps").as_double();

    meas_alpha_ = clampDouble(get_parameter("meas_alpha").as_double(), 0.0, 1.0);
    meas_w_eps_ = get_parameter("meas_w_eps").as_double();

    speed_window_s_ = std::max(0.02, get_parameter("speed_window_s").as_double());

    u_slew_rate_ = get_parameter("u_slew_rate").as_double();

    u_min_ = clampDouble(get_parameter("u_min").as_double(), 0.0, 1.0);
    u_kick_ = clampDouble(get_parameter("u_kick").as_double(), 0.0, 1.0);
    kick_time_s_ = std::max(0.0, get_parameter("kick_time_s").as_double());

    i2c_dev_  = get_parameter("i2c_dev").as_string();
    i2c_addr_ = get_parameter("i2c_addr").as_int();

    {
      auto ch = get_parameter("wheel_channels").as_integer_array();
      if (ch.size() != 4) throw std::runtime_error("wheel_channels must have length 4");
      for (int i = 0; i < 4; ++i) wheel_channels_[i] = static_cast<int>(ch[i]);
    }
    {
      auto g = get_parameter("wheel_gains").as_double_array();
      if (g.size() != 4) throw std::runtime_error("wheel_gains must have length 4");
      for (int i = 0; i < 4; ++i) wheel_gains_[i] = g[i];
    }

    k_us_per_pct_ = get_parameter("k_us_per_pct").as_double();
    neutral_us_   = get_parameter("neutral_us").as_int();
    min_us_       = get_parameter("min_us").as_int();
    max_us_       = get_parameter("max_us").as_int();
    pct_min_      = get_parameter("pct_min").as_int();
    pct_max_      = get_parameter("pct_max").as_int();

    phidget_serial_ = get_parameter("phidget_serial").as_int();
    {
      auto em = get_parameter("encoder_channels").as_integer_array();
      if (em.size() != 4) throw std::runtime_error("encoder_channels must have length 4");
      for (int i = 0; i < 4; ++i) encoder_channels_[i] = static_cast<int>(em[i]);
    }

    invert_[FL] = get_parameter("invert_fl").as_bool();
    invert_[FR] = get_parameter("invert_fr").as_bool();
    invert_[RL] = get_parameter("invert_rl").as_bool();
    invert_[RR] = get_parameter("invert_rr").as_bool();

    publish_ticks_ms_ = get_parameter("publish_ticks_ms").as_int();

    if (control_dt_ <= 0.0) throw std::runtime_error("control_dt must be > 0");
    if (ticks_per_rev_ <= 0.0) throw std::runtime_error("ticks_per_rev must be > 0");

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      pid_[i] = std::make_unique<base::PIDController>(
        kp_, ki_, kd_, output_limit_, deadband_, setpoint_max_accel_);
      pid_[i]->reset();
      ticks_[i].store(0, std::memory_order_relaxed);
      ticks_last_speed_[i] = 0;
      meas_w_filt_[i] = 0.0;
      u_last_[i] = 0.0;
      last_dir_[i] = 0;
      kick_until_[i] = now();
    }
  }

  // ---------------- Wheel command ----------------
  void onWheelCmd(const base::msg::WheelVelocities::SharedPtr msg)
  {
    target_w_radps_[FL] = msg->left;
    target_w_radps_[RL] = msg->left;
    target_w_radps_[FR] = msg->right;
    target_w_radps_[RR] = msg->right;
  }

  // ---------------- Phidget callbacks ----------------
  static void CCONV onPositionChange(
    PhidgetEncoderHandle ch,
    void *ctx,
    int positionChange,
    double /*timeChange*/,
    int /*indexTriggered*/)
  {
    auto* self = static_cast<HardwareNode*>(ctx);
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      if (self->enc_[i] == ch) {
        int64_t d = static_cast<int64_t>(positionChange);
        if (self->invert_[i]) d = -d;
        self->ticks_[i].fetch_add(d, std::memory_order_relaxed);
        return;
      }
    }
  }

  void initEncoders()
  {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      PhidgetEncoderHandle h = nullptr;
      phidget_check(PhidgetEncoder_create(&h), "PhidgetEncoder_create");

      if (phidget_serial_ >= 0) {
        phidget_check(
          Phidget_setDeviceSerialNumber((PhidgetHandle)h, phidget_serial_),
          "Phidget_setDeviceSerialNumber");
      }

      phidget_check(Phidget_setChannel((PhidgetHandle)h, encoder_channels_[i]), "Phidget_setChannel");
      phidget_check(
        PhidgetEncoder_setOnPositionChangeHandler(h, &HardwareNode::onPositionChange, this),
        "PhidgetEncoder_setOnPositionChangeHandler");
      phidget_check(Phidget_openWaitForAttachment((PhidgetHandle)h, 5000), "Phidget_openWaitForAttachment");

      phidget_check(PhidgetEncoder_setPosition(h, 0), "PhidgetEncoder_setPosition");

      enc_[i] = h;
      ticks_[i].store(0, std::memory_order_relaxed);
    }
  }

  // ---------------- Speed estimation over window ----------------
  void updateMeasuredSpeeds()
  {
    const rclcpp::Time t = now();
    const double dt = (t - last_speed_eval_time_).seconds();
    if (dt < speed_window_s_) return;

    const double k = (2.0 * M_PI) / ticks_per_rev_;

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const int64_t cur = ticks_[i].load(std::memory_order_relaxed);
      const int64_t d = cur - ticks_last_speed_[i];
      ticks_last_speed_[i] = cur;

      double w = (static_cast<double>(d) * k) / dt; // rad/s

      if (std::abs(w) < meas_w_eps_) w = 0.0;
      meas_w_filt_[i] = meas_alpha_ * w + (1.0 - meas_alpha_) * meas_w_filt_[i];
    }

    last_speed_eval_time_ = t;
  }

  // ---------------- Control loop ----------------
  void controlLoop()
  {
    updateMeasuredSpeeds();

    const rclcpp::Time t = now();

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const double sp = target_w_radps_[i];

      if (std::abs(sp) < stop_eps_radps_) {
        pid_[i]->reset();
        u_last_[i] = 0.0;
        last_dir_[i] = 0;
        writeMotor(i, 0.0);
        continue;
      }

      const int dir = signum(sp);
      if (dir != 0 && dir != last_dir_[i]) {
        // direction change or start -> allow kick window
        kick_until_[i] = t + rclcpp::Duration::from_seconds(kick_time_s_);
        last_dir_[i] = dir;
      }

      double u = pid_[i]->compute(sp, meas_w_filt_[i], control_dt_);

      // stiction compensation: enforce minimum output when moving
      if (dir != 0) {
        if (std::abs(u) < u_min_) {
          u = static_cast<double>(dir) * u_min_;
        }
      }

      // optional start kick (additive)
      if (u_kick_ > 0.0 && kick_time_s_ > 0.0) {
        if (t < kick_until_[i]) {
          u = clampDouble(u + static_cast<double>(dir) * u_kick_, -output_limit_, output_limit_);
        }
      }

      // output slew limiting
      if (u_slew_rate_ > 0.0) {
        const double du_max = u_slew_rate_ * control_dt_;
        u = clampDouble(u, u_last_[i] - du_max, u_last_[i] + du_max);
      }

      u_last_[i] = u;
      writeMotor(i, u);
    }
  }

  // ---------------- Publish ticks ----------------
  void publishTicks()
  {
    base::msg::WheelTicks4 m;
    m.header.stamp = now();
    m.header.frame_id = ticks_frame_id_;
    m.fl_ticks = ticks_[FL].load(std::memory_order_relaxed);
    m.fr_ticks = ticks_[FR].load(std::memory_order_relaxed);
    m.rl_ticks = ticks_[RL].load(std::memory_order_relaxed);
    m.rr_ticks = ticks_[RR].load(std::memory_order_relaxed);
    ticks_pub_->publish(m);
  }

  // ---------------- Motor output ----------------
  int pctToPulseUs(int pct) const
  {
    const double us_f = static_cast<double>(pct) * k_us_per_pct_ + static_cast<double>(neutral_us_);
    int us = static_cast<int>(std::lround(us_f));
    us = clampInt(us, min_us_, max_us_);
    return us;
  }

  void writeMotor(int wheel, double u_norm)
  {
    const double u = clampDouble(u_norm * wheel_gains_[wheel], -output_limit_, output_limit_);

    int pct = static_cast<int>(std::lround((u / output_limit_) * 100.0));
    pct = clampInt(pct, pct_min_, pct_max_);

    const int pulse = pctToPulseUs(pct);
    const int channel = wheel_channels_[wheel];
    nxt_.writePulseUs(channel, pulse);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareNode>());
  rclcpp::shutdown();
  return 0;
}
