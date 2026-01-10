#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_ticks4.hpp"
#include "base/pid_controller.h"

#ifdef HAVE_PHIDGET22
  #include <phidget22.h>
#endif

// ============================================================
// NXT Servo Controller I2C writer (as in FloriBot Pi code)
// - I2C dev: /dev/i2c-1
// - 7-bit addr: 0x58 (decimal 88)
// - Pulse write: reg = channel*2 + 66, then write [reg, low, high]
// ============================================================
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

class HardwareNode : public rclcpp::Node
{
public:
  HardwareNode()
  : Node("hardware_node")
  {
    declareParams();
    getParams();

    // Open I2C motor controller (NXT Servo Controller)
    nxt_.openBus(i2c_dev_, i2c_addr_);

    wheel_cmd_sub_ = create_subscription<base::msg::WheelVelocities>(
      wheel_cmd_topic_, rclcpp::QoS(10),
      std::bind(&HardwareNode::onWheelCmd, this, std::placeholders::_1));

    ticks_pub_ = create_publisher<base::msg::WheelTicks4>(wheel_ticks_topic_, rclcpp::QoS(10));

#ifdef HAVE_PHIDGET22
    initEncoders();
#else
    RCLCPP_WARN(get_logger(),
      "Built without Phidget22 support (HAVE_PHIDGET22=0). Encoders will not update.");
#endif

    timer_ = create_wall_timer(
      std::chrono::duration<double>(control_dt_),
      std::bind(&HardwareNode::controlLoop, this));
  }

  ~HardwareNode() override
  {
#ifdef HAVE_PHIDGET22
    shutdownEncoders();
#endif
    // ensure bus is closed
    nxt_.closeBus();
  }

private:
  enum WheelIndex { FL = 0, FR = 1, RL = 2, RR = 3, WHEEL_COUNT = 4 };

  // ---- ROS params ----
  std::string wheel_cmd_topic_{"/wheel_commands"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};
  std::string ticks_frame_id_{"base_link"};

  // Control timing
  double control_dt_{0.02};        // [s]
  double ticks_per_rev_{2048.0};   // [ticks / wheel rev]

  // PID parameters
  double kp_{0.6};
  double ki_{0.0};
  double kd_{0.0};
  double output_limit_{1.0};       // PID output clamp (normalized)
  double deadband_{0.0};
  double setpoint_max_accel_{std::numeric_limits<double>::infinity()}; // [rad/s^2]

  // Encoder direction invert (per wheel)
  bool invert_fl_{false};
  bool invert_fr_{false};
  bool invert_rl_{false};
  bool invert_rr_{false};

  // ---- Motor mapping params (Pi style) ----
  std::string i2c_dev_{"/dev/i2c-1"};
  int i2c_addr_{0x58}; // 7-bit addr, default 0x58 = 88

  // Order is [FL, FR, RL, RR]
  std::array<int, 4> wheel_channels_{{2, 3, 0, 1}};
  std::array<double, 4> wheel_gains_{{1.0, 1.0, 1.0, 1.0}};

  double k_us_per_pct_{5.5};
  int neutral_us_{1480};
  int min_us_{900};
  int max_us_{2100};
  int pct_min_{-100};
  int pct_max_{100};

  // ---- ROS ----
  rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr wheel_cmd_sub_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr ticks_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Controllers ----
  std::array<std::unique_ptr<base::PIDController>, WHEEL_COUNT> pid_{};

  // Targets (rad/s)
  double target_w_radps_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  // Encoder tick accumulation
  std::atomic<int64_t> ticks_[WHEEL_COUNT]{0, 0, 0, 0};
  int64_t ticks_last_[WHEEL_COUNT]{0, 0, 0, 0};

  // Motor driver
  NxtServoI2C nxt_;

#ifdef HAVE_PHIDGET22
  PhidgetEncoderHandle enc_[WHEEL_COUNT]{nullptr, nullptr, nullptr, nullptr};

  struct EncCbCtx {
    std::atomic<int64_t>* ticks_atomic{nullptr};
    bool invert{false};
  };
  EncCbCtx enc_ctx_[WHEEL_COUNT];

  // phidget22 expects: (EncoderHandle, ctx, positionChange(int), timeChange(double), indexTriggered(int))
  static void onPositionChange(
      PhidgetEncoderHandle,
      void* ctx,
      int positionChange,
      double /*timeChange*/,
      int /*indexTriggered*/)
  {
    auto* c = static_cast<EncCbCtx*>(ctx);
    if (!c || !c->ticks_atomic) return;

    const int64_t delta = static_cast<int64_t>(positionChange);
    if (c->invert) c->ticks_atomic->fetch_add(-delta, std::memory_order_relaxed);
    else          c->ticks_atomic->fetch_add( delta, std::memory_order_relaxed);
  }
#endif

  void declareParams()
  {
    // Topics
    declare_parameter<std::string>("wheel_cmd_topic", wheel_cmd_topic_);
    declare_parameter<std::string>("wheel_ticks_topic", wheel_ticks_topic_);
    declare_parameter<std::string>("ticks_frame_id", ticks_frame_id_);

    // Timing + encoder conversion
    declare_parameter<double>("control_dt", control_dt_);
    declare_parameter<double>("ticks_per_rev", ticks_per_rev_);

    // PID
    declare_parameter<double>("kp", kp_);
    declare_parameter<double>("ki", ki_);
    declare_parameter<double>("kd", kd_);
    declare_parameter<double>("output_limit", output_limit_);
    declare_parameter<double>("deadband", deadband_);
    declare_parameter<double>("setpoint_max_accel", setpoint_max_accel_);

    // Encoder invert
    declare_parameter<bool>("invert_fl", invert_fl_);
    declare_parameter<bool>("invert_fr", invert_fr_);
    declare_parameter<bool>("invert_rl", invert_rl_);
    declare_parameter<bool>("invert_rr", invert_rr_);

    // Motor (I2C)
    declare_parameter<std::string>("i2c_dev", i2c_dev_);
    declare_parameter<int>("i2c_addr", i2c_addr_);

    declare_parameter<std::vector<int>>(
      "wheel_channels",
      std::vector<int>{wheel_channels_.begin(), wheel_channels_.end()}
    );
    declare_parameter<std::vector<double>>(
      "wheel_gains",
      std::vector<double>{wheel_gains_.begin(), wheel_gains_.end()}
    );

    declare_parameter<double>("k_us_per_pct", k_us_per_pct_);
    declare_parameter<int>("neutral_us", neutral_us_);
    declare_parameter<int>("min_us", min_us_);
    declare_parameter<int>("max_us", max_us_);
    declare_parameter<int>("pct_min", pct_min_);
    declare_parameter<int>("pct_max", pct_max_);
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
    deadband_     = get_parameter("deadband").as_double();
    setpoint_max_accel_ = get_parameter("setpoint_max_accel").as_double();

    invert_fl_ = get_parameter("invert_fl").as_bool();
    invert_fr_ = get_parameter("invert_fr").as_bool();
    invert_rl_ = get_parameter("invert_rl").as_bool();
    invert_rr_ = get_parameter("invert_rr").as_bool();

    i2c_dev_  = get_parameter("i2c_dev").as_string();
    i2c_addr_ = get_parameter("i2c_addr").as_int();

    {
      auto ch = get_parameter("wheel_channels").as_integer_array();
      if (ch.size() != 4) {
        throw std::runtime_error("wheel_channels must have length 4 (order: [FL, FR, RL, RR])");
      }
      for (int i = 0; i < 4; ++i) wheel_channels_[i] = static_cast<int>(ch[i]);
    }

    {
      auto g = get_parameter("wheel_gains").as_double_array();
      if (g.size() != 4) {
        throw std::runtime_error("wheel_gains must have length 4 (order: [FL, FR, RL, RR])");
      }
      for (int i = 0; i < 4; ++i) wheel_gains_[i] = g[i];
    }

    k_us_per_pct_ = get_parameter("k_us_per_pct").as_double();
    neutral_us_   = get_parameter("neutral_us").as_int();
    min_us_       = get_parameter("min_us").as_int();
    max_us_       = get_parameter("max_us").as_int();
    pct_min_      = get_parameter("pct_min").as_int();
    pct_max_      = get_parameter("pct_max").as_int();

    // Init controllers (one per wheel)
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      pid_[i] = std::make_unique<base::PIDController>(
        kp_, ki_, kd_,
        output_limit_,
        deadband_,
        setpoint_max_accel_
      );
    }
  }

  void onWheelCmd(const base::msg::WheelVelocities::SharedPtr msg)
  {
    // 4-wheel diff-drive: left -> FL+RL, right -> FR+RR
    target_w_radps_[FL] = msg->left;
    target_w_radps_[RL] = msg->left;
    target_w_radps_[FR] = msg->right;
    target_w_radps_[RR] = msg->right;
  }

#ifdef HAVE_PHIDGET22
  void initEncoders()
  {
    const bool inv[WHEEL_COUNT]{invert_fl_, invert_fr_, invert_rl_, invert_rr_};

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      enc_ctx_[i].ticks_atomic = &ticks_[i];
      enc_ctx_[i].invert = inv[i];

      PhidgetEncoderHandle h = nullptr;
      phidget_check(PhidgetEncoder_create(&h), "PhidgetEncoder_create");
      enc_[i] = h;

      // If you need serial/channel configuration, set here.
      // phidget_check(Phidget_setDeviceSerialNumber((PhidgetHandle)h, serial_), "setDeviceSerialNumber");
      // phidget_check(Phidget_setChannel((PhidgetHandle)h, channel_), "setChannel");

      phidget_check(Phidget_openWaitForAttachment((PhidgetHandle)h, 5000), "openWaitForAttachment");

      phidget_check(
        PhidgetEncoder_setOnPositionChangeHandler(h, onPositionChange, &enc_ctx_[i]),
        "setOnPositionChangeHandler"
      );
    }

    RCLCPP_INFO(get_logger(), "Encoders initialized (Phidget22).");
  }

  void shutdownEncoders()
  {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      if (enc_[i]) {
        Phidget_close((PhidgetHandle)enc_[i]);
        PhidgetEncoder_delete(&enc_[i]);
        enc_[i] = nullptr;
      }
    }
  }

  static void phidget_check(PhidgetReturnCode rc, const char* what)
  {
    if (rc != EPHIDGET_OK) {
      const char* err = nullptr;
      Phidget_getErrorDescription(rc, &err);
      throw std::runtime_error(std::string("Phidget error in ") + what + ": " + (err ? err : "unknown"));
    }
  }
#endif

  void controlLoop()
  {
    // 1) measured wheel speed from tick delta
    double meas_w_radps[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const int64_t t = ticks_[i].load(std::memory_order_relaxed);
      const int64_t dticks = t - ticks_last_[i];
      ticks_last_[i] = t;

      const double rev = static_cast<double>(dticks) / ticks_per_rev_;
      const double rad = rev * 2.0 * M_PI;
      meas_w_radps[i] = rad / control_dt_;
    }

    // 2) PID per wheel -> motor output
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const double u = pid_[i]->compute(target_w_radps_[i], meas_w_radps[i], control_dt_);
      writeMotor(i, u);
    }

    // 3) publish ticks
    base::msg::WheelTicks4 out;
    out.header.stamp = now();
    out.header.frame_id = ticks_frame_id_;
    out.fl_ticks = ticks_[FL].load(std::memory_order_relaxed);
    out.fr_ticks = ticks_[FR].load(std::memory_order_relaxed);
    out.rl_ticks = ticks_[RL].load(std::memory_order_relaxed);
    out.rr_ticks = ticks_[RR].load(std::memory_order_relaxed);
    ticks_pub_->publish(out);
  }

  void writeMotor(int wheel, double u_norm)
  {
    // PID output is normalized and already limited by PIDController to +/- output_limit_
    // Apply optional per-wheel gain (can also be -1.0 for polarity flip).
    const double u = std::clamp(u_norm * wheel_gains_[wheel], -output_limit_, output_limit_);

    // normalize to [-100..100] percent
    int pct = static_cast<int>(std::lround((u / output_limit_) * 100.0));
    pct = std::clamp(pct, pct_min_, pct_max_);

    // percent -> pulse (Pi mapping)
    int pulse = static_cast<int>(std::lround(
      static_cast<double>(neutral_us_) + (static_cast<double>(pct) * k_us_per_pct_)
    ));
    pulse = std::clamp(pulse, min_us_, max_us_);

    // send to NXT servo channel
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
