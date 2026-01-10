#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_ticks4.hpp"

#include "base/pid_controller.h"

#ifdef HAVE_PHIDGET22
  #include <phidget22.h>
#endif

// NOTE: This node is written as a practical hardware-facing controller:
// - subscribes wheel commands (left/right rad/s)
// - reads 4 wheel encoders
// - runs 4 independent PID controllers (FL, FR, RL, RR)
// - drives 4 motor outputs (abstracted here; adapt to your driver)

class HardwareNode : public rclcpp::Node
{
public:
  HardwareNode()
  : Node("hardware_node")
  {
    declareParams();
    getParams();

    wheel_cmd_sub_ = create_subscription<base::msg::WheelVelocities>(
      wheel_cmd_topic_, rclcpp::QoS(10),
      std::bind(&HardwareNode::onWheelCmd, this, std::placeholders::_1));

    ticks_pub_ = create_publisher<base::msg::WheelTicks4>(wheel_ticks_topic_, rclcpp::QoS(10));

#ifdef HAVE_PHIDGET22
    initEncoders();
#else
    RCLCPP_WARN(get_logger(), "Built without Phidget22 support (HAVE_PHIDGET22=0). Encoders will not work.");
#endif

    // Control loop timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(control_dt_),
      std::bind(&HardwareNode::controlLoop, this));
  }

  ~HardwareNode() override
  {
#ifdef HAVE_PHIDGET22
    shutdownEncoders();
#endif
  }

private:
  enum WheelIndex { FL = 0, FR = 1, RL = 2, RR = 3, WHEEL_COUNT = 4 };

  // ---- Parameters ----
  std::string wheel_cmd_topic_{"/wheel_commands"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};

  double control_dt_{0.02};                 // s
  double ticks_per_rev_{2048.0};            // encoder ticks per wheel revolution
  double max_wheel_radps_{20.0};            // used for normalization (if output mapping uses -1..1)

  // PID gains (same for all wheels here; you can extend to per-wheel gains)
  double kp_{0.6};
  double ki_{0.0};
  double kd_{0.0};

  // Output shaping
  double output_limit_{1.0};                // normalized [-limit..limit]
  double deadband_{0.0};                    // normalized
  double setpoint_max_accel_{std::numeric_limits<double>::infinity()}; // rad/s^2

  // Inversion per wheel
  bool invert_fl_{false};
  bool invert_fr_{false};
  bool invert_rl_{false};
  bool invert_rr_{false};

  // ---- ROS ----
  rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr wheel_cmd_sub_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr ticks_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Controllers ----
  base::PidController pid_[WHEEL_COUNT];

  // Target wheel speeds (rad/s)
  double target_w_radps_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};
  double target_w_radps_prev_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  // Measured wheel speeds (rad/s)
  double meas_w_radps_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  // Encoder tick counters (atomic for callback safety)
  std::atomic<int64_t> ticks_[WHEEL_COUNT]{0, 0, 0, 0};
  int64_t ticks_last_[WHEEL_COUNT]{0, 0, 0, 0};

#ifdef HAVE_PHIDGET22
  // Phidget handles and callback context
  PhidgetEncoderHandle enc_[WHEEL_COUNT]{nullptr, nullptr, nullptr, nullptr};

  // We pass (ticks_atomic_ptr, invert_bool) as callback context
  struct EncCbCtx {
    std::atomic<int64_t>* ticks_atomic{nullptr};
    bool invert{false};
  };
  EncCbCtx enc_ctx_[WHEEL_COUNT];

  // === Jazzy / current phidget22 header expects:
  // void (*)(_PhidgetEncoder*, void*, int, double, int)
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
    declare_parameter<std::string>("wheel_cmd_topic", wheel_cmd_topic_);
    declare_parameter<std::string>("wheel_ticks_topic", wheel_ticks_topic_);

    declare_parameter<double>("control_dt", control_dt_);
    declare_parameter<double>("ticks_per_rev", ticks_per_rev_);
    declare_parameter<double>("max_wheel_radps", max_wheel_radps_);

    declare_parameter<double>("kp", kp_);
    declare_parameter<double>("ki", ki_);
    declare_parameter<double>("kd", kd_);

    declare_parameter<double>("output_limit", output_limit_);
    declare_parameter<double>("deadband", deadband_);
    declare_parameter<double>("setpoint_max_accel", setpoint_max_accel_);

    declare_parameter<bool>("invert_fl", invert_fl_);
    declare_parameter<bool>("invert_fr", invert_fr_);
    declare_parameter<bool>("invert_rl", invert_rl_);
    declare_parameter<bool>("invert_rr", invert_rr_);
  }

  void getParams()
  {
    wheel_cmd_topic_   = get_parameter("wheel_cmd_topic").as_string();
    wheel_ticks_topic_ = get_parameter("wheel_ticks_topic").as_string();

    control_dt_    = get_parameter("control_dt").as_double();
    ticks_per_rev_ = get_parameter("ticks_per_rev").as_double();
    max_wheel_radps_ = get_parameter("max_wheel_radps").as_double();

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

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      pid_[i].setGains(kp_, ki_, kd_);
      pid_[i].setDt(control_dt_);
      pid_[i].setOutputLimit(output_limit_);
      pid_[i].setDeadband(deadband_);
    }
  }

  void onWheelCmd(const base::msg::WheelVelocities::SharedPtr msg)
  {
    // Left command applies to FL+RL, right command to FR+RR
    target_w_radps_[FL] = msg->left;
    target_w_radps_[RL] = msg->left;
    target_w_radps_[FR] = msg->right;
    target_w_radps_[RR] = msg->right;
  }

#ifdef HAVE_PHIDGET22
  void initEncoders()
  {
    // Map inversion to wheels
    const bool inv[WHEEL_COUNT]{invert_fl_, invert_fr_, invert_rl_, invert_rr_};

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      enc_ctx_[i].ticks_atomic = &ticks_[i];
      enc_ctx_[i].invert = inv[i];

      PhidgetEncoderHandle h = nullptr;
      phidget_check(PhidgetEncoder_create(&h), "Encoder_create");
      enc_[i] = h;

      // TODO: set serial/channel if you use multiple devices
      // phidget_check(Phidget_setDeviceSerialNumber((PhidgetHandle)h, serial_), "setDeviceSerialNumber");
      // phidget_check(Phidget_setChannel((PhidgetHandle)h, channel_), "setChannel");

      phidget_check(Phidget_openWaitForAttachment((PhidgetHandle)h, 5000), "openWaitForAttachment");

      // Correct callback signature for current phidget22 headers:
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
    // 1) compute measured rad/s from tick deltas
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const int64_t t = ticks_[i].load(std::memory_order_relaxed);
      const int64_t dticks = t - ticks_last_[i];
      ticks_last_[i] = t;

      // ticks -> rev -> rad
      const double rev = static_cast<double>(dticks) / ticks_per_rev_;
      const double rad = rev * 2.0 * M_PI;
      meas_w_radps_[i] = rad / control_dt_;
    }

    // 2) apply setpoint accel limit (per wheel)
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      double sp = target_w_radps_[i];

      if (std::isfinite(setpoint_max_accel_)) {
        const double max_step = setpoint_max_accel_ * control_dt_;
        const double prev = target_w_radps_prev_[i];
        const double delta = sp - prev;
        if (delta >  max_step) sp = prev + max_step;
        if (delta < -max_step) sp = prev - max_step;
      }
      target_w_radps_prev_[i] = sp;

      // 3) PID -> normalized command
      const double u = pid_[i].update(sp, meas_w_radps_[i]);

      // 4) drive motors (replace with your actual driver)
      writeMotor(i, u);
    }

    // 5) publish ticks
    base::msg::WheelTicks4 out;
    out.fl = ticks_[FL].load(std::memory_order_relaxed);
    out.fr = ticks_[FR].load(std::memory_order_relaxed);
    out.rl = ticks_[RL].load(std::memory_order_relaxed);
    out.rr = ticks_[RR].load(std::memory_order_relaxed);
    ticks_pub_->publish(out);
  }

  void writeMotor(int wheel, double u_norm)
  {
    (void)wheel;
    (void)u_norm;

    // Placeholder:
    // - u_norm expected normalized in [-output_limit..output_limit]
    // - Map to your driver (PWM %, PWM µs, CAN command, etc.)
    //
    // Example (pseudo):
    //   driver_.setNormalized(wheel, u_norm);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareNode>());
  rclcpp::shutdown();
  return 0;
}
