#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <array>

#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_ticks4.hpp"

#include "base/pid_controller.h"

#ifdef HAVE_PHIDGET22
  #include <phidget22.h>
#endif

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
  std::string ticks_frame_id_{"base_link"};

  double control_dt_{0.02};                 // s
  double ticks_per_rev_{2048.0};            // ticks per wheel revolution

  // PID gains
  double kp_{0.6};
  double ki_{0.0};
  double kd_{0.0};

  // Controller shaping
  double output_limit_{1.0};                // |u| <= output_limit
  double deadband_{0.0};                    // deadband around 0
  double setpoint_max_accel_{1.0};          // ramp limit inside PIDController (units rad/s^2)

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
  std::array<std::unique_ptr<base::PIDController>, WHEEL_COUNT> pid_{};

  // Targets (rad/s)
  double target_w_radps_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  // Encoder tick counters
  std::atomic<int64_t> ticks_[WHEEL_COUNT]{0, 0, 0, 0};
  int64_t ticks_last_[WHEEL_COUNT]{0, 0, 0, 0};

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

    declare_parameter<bool>("invert_fl", invert_fl_);
    declare_parameter<bool>("invert_fr", invert_fr_);
    declare_parameter<bool>("invert_rl", invert_rl_);
    declare_parameter<bool>("invert_rr", invert_rr_);
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

    // Build 4 identical controllers (one per wheel)
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
    // 4-wheel diff drive: left applies to FL+RL, right to FR+RR
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

      // TODO: set serial/channel if required for your setup
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

    // 2) PID per wheel -> drive
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
    (void)wheel;
    (void)u_norm;

    // Placeholder: map u_norm (typically in [-output_limit .. +output_limit]) to your driver.
    // Example:
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
