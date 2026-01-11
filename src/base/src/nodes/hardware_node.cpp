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

#include "sensor_msgs/msg/joint_state.hpp"

#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_ticks4.hpp"
#include "base/pid_controller.h"

// ---------------- I2C NXT Servo ----------------
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

// ---------------- helpers ----------------
static bool getJointPosition(const sensor_msgs::msg::JointState& js, const std::string& name, double& pos_out)
{
  auto it = std::find(js.name.begin(), js.name.end(), name);
  if (it == js.name.end()) return false;

  const size_t idx = static_cast<size_t>(std::distance(js.name.begin(), it));
  if (idx >= js.position.size()) return false;

  pos_out = js.position[idx];
  return true;
}

static inline bool isZeroStamp(const builtin_interfaces::msg::Time& t)
{
  return (t.sec == 0) && (t.nanosec == 0);
}

static int64_t posToRawTicks(double pos, bool invert)
{
  const int64_t t = static_cast<int64_t>(std::llround(pos));
  return invert ? -t : t;
}

// unwrap delta for modulo encoder (0..N..0)
static int64_t deltaModulo(int64_t cur, int64_t last, int64_t mod)
{
  int64_t d = cur - last;
  const int64_t half = mod / 2;
  if (d >  half) d -= mod;
  if (d < -half) d += mod;
  return d;
}

// clamp helper
static double clampAbs(double v, double lim)
{
  if (v > lim) return lim;
  if (v < -lim) return -lim;
  return v;
}

// ---------------- Node ----------------
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

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(50),
      std::bind(&HardwareNode::jointStatesCallback, this, std::placeholders::_1));

    ticks_pub_ = create_publisher<base::msg::WheelTicks4>(wheel_ticks_topic_, rclcpp::QoS(10));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(control_dt_),
      std::bind(&HardwareNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
      "hardware_node: dt=%.3f tpr=%.1f modulo=%d unwrap=%d js_timeout=%.3f meas_alpha=%.3f max_radps=%.2f",
      control_dt_, ticks_per_rev_, modulo_ticks_, (int)unwrap_enabled_,
      jointstate_timeout_s_, meas_alpha_, max_meas_radps_);
  }

  ~HardwareNode() override
  {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      try { writeMotor(i, 0.0); } catch (...) {}
    }
    nxt_.closeBus();
  }

private:
  enum WheelIndex { FL = 0, FR = 1, RL = 2, RR = 3, WHEEL_COUNT = 4 };

  // Topics
  std::string wheel_cmd_topic_{"/wheel_commands"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};
  std::string ticks_frame_id_{"base_link"};

  // Control timing
  double control_dt_{0.02};

  // Encoder scaling
  double ticks_per_rev_{2048.0};   // effective ticks per wheel revolution (for rad conversion)
  int modulo_ticks_{2048};         // wrap period of raw ticks from encoder (can differ!)
  bool unwrap_enabled_{true};

  double jointstate_timeout_s_{0.20}; // if no JS update -> stop (safety)

  // PID
  double kp_{0.12};
  double ki_{0.25};
  double kd_{0.0};
  double output_limit_{1.0};
  double deadband_{0.10};
  double setpoint_max_accel_{6.0};

  double stop_eps_radps_{0.05};

  // Measurement filtering + sanity
  double meas_alpha_{0.30};        // LPF alpha for measured w
  double max_meas_radps_{30.0};    // reject insane speeds (rad/s)
  int64_t max_tick_jump_{800};     // reject encoder glitches per JS update (ticks)

  // Optional output slew limiting
  double u_slew_rate_{3.0};        // 1/s, 0 disables

  // joint mapping
  std::string joint_states_topic_{"/joint_states"};
  std::string fl_joint_{"joint2"};
  std::string fr_joint_{"joint3"};
  std::string rl_joint_{"joint0"};
  std::string rr_joint_{"joint1"};

  // IMPORTANT: per your info FL+RL must be inverted
  bool invert_fl_{true};
  bool invert_fr_{false};
  bool invert_rl_{true};
  bool invert_rr_{false};

  // Motor I2C
  std::string i2c_dev_{"/dev/i2c-1"};
  int i2c_addr_{0x58};

  std::array<int, 4> wheel_channels_{{2, 3, 0, 1}};         // [FL, FR, RL, RR]
  std::array<double, 4> wheel_gains_{{1.0, 1.0, 1.0, 1.0}}; // polarity per wheel (separately from invert_*)

  double k_us_per_pct_{5.5};
  int neutral_us_{1480};
  int min_us_{900};
  int max_us_{2100};
  int pct_min_{-100};
  int pct_max_{100};

  // ROS
  rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr wheel_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr ticks_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // PID
  std::array<std::unique_ptr<base::PIDController>, WHEEL_COUNT> pid_{};

  // setpoints (rad/s)
  double target_w_radps_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  // raw tick state
  int64_t raw_last_[WHEEL_COUNT]{0, 0, 0, 0};
  bool raw_valid_[WHEEL_COUNT]{false, false, false, false};

  // continuous ticks (published)
  std::atomic<int64_t> ticks_cont_[WHEEL_COUNT]{0, 0, 0, 0};

  // measured speed state (computed ONLY in jointStatesCallback)
  double meas_w_filt_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};
  rclcpp::Time last_js_stamp_;
  bool have_js_{false};

  // output state
  double u_last_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  NxtServoI2C nxt_;

  void declareParams()
  {
    declare_parameter<std::string>("wheel_cmd_topic", wheel_cmd_topic_);
    declare_parameter<std::string>("wheel_ticks_topic", wheel_ticks_topic_);
    declare_parameter<std::string>("ticks_frame_id", ticks_frame_id_);

    declare_parameter<double>("control_dt", control_dt_);

    declare_parameter<double>("ticks_per_rev", ticks_per_rev_);
    declare_parameter<int>("modulo_ticks", modulo_ticks_);
    declare_parameter<bool>("unwrap_enabled", unwrap_enabled_);

    declare_parameter<double>("jointstate_timeout_s", jointstate_timeout_s_);

    declare_parameter<double>("kp", kp_);
    declare_parameter<double>("ki", ki_);
    declare_parameter<double>("kd", kd_);
    declare_parameter<double>("output_limit", output_limit_);
    declare_parameter<double>("deadband", deadband_);
    declare_parameter<double>("setpoint_max_accel", setpoint_max_accel_);
    declare_parameter<double>("stop_eps_radps", stop_eps_radps_);

    declare_parameter<double>("meas_alpha", meas_alpha_);
    declare_parameter<double>("max_meas_radps", max_meas_radps_);
    declare_parameter<int>("max_tick_jump", static_cast<int>(max_tick_jump_));

    declare_parameter<double>("u_slew_rate", u_slew_rate_);

    declare_parameter<std::string>("joint_states_topic", joint_states_topic_);
    declare_parameter<std::string>("fl_joint", fl_joint_);
    declare_parameter<std::string>("fr_joint", fr_joint_);
    declare_parameter<std::string>("rl_joint", rl_joint_);
    declare_parameter<std::string>("rr_joint", rr_joint_);

    declare_parameter<bool>("invert_fl", invert_fl_);
    declare_parameter<bool>("invert_fr", invert_fr_);
    declare_parameter<bool>("invert_rl", invert_rl_);
    declare_parameter<bool>("invert_rr", invert_rr_);

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

    control_dt_ = get_parameter("control_dt").as_double();

    ticks_per_rev_ = get_parameter("ticks_per_rev").as_double();
    modulo_ticks_  = get_parameter("modulo_ticks").as_int();
    unwrap_enabled_ = get_parameter("unwrap_enabled").as_bool();

    jointstate_timeout_s_ = get_parameter("jointstate_timeout_s").as_double();

    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    output_limit_ = get_parameter("output_limit").as_double();
    deadband_ = get_parameter("deadband").as_double();
    setpoint_max_accel_ = get_parameter("setpoint_max_accel").as_double();
    stop_eps_radps_ = get_parameter("stop_eps_radps").as_double();

    meas_alpha_ = std::clamp(get_parameter("meas_alpha").as_double(), 0.0, 1.0);
    max_meas_radps_ = get_parameter("max_meas_radps").as_double();
    max_tick_jump_ = static_cast<int64_t>(get_parameter("max_tick_jump").as_int());

    u_slew_rate_ = get_parameter("u_slew_rate").as_double();

    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    fl_joint_ = get_parameter("fl_joint").as_string();
    fr_joint_ = get_parameter("fr_joint").as_string();
    rl_joint_ = get_parameter("rl_joint").as_string();
    rr_joint_ = get_parameter("rr_joint").as_string();

    invert_fl_ = get_parameter("invert_fl").as_bool();
    invert_fr_ = get_parameter("invert_fr").as_bool();
    invert_rl_ = get_parameter("invert_rl").as_bool();
    invert_rr_ = get_parameter("invert_rr").as_bool();

    i2c_dev_ = get_parameter("i2c_dev").as_string();
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

    if (ticks_per_rev_ <= 0.0) throw std::runtime_error("ticks_per_rev must be > 0");
    if (modulo_ticks_ <= 0) modulo_ticks_ = static_cast<int>(std::llround(ticks_per_rev_));
    if (modulo_ticks_ <= 0) modulo_ticks_ = 1;

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      pid_[i] = std::make_unique<base::PIDController>(
        kp_, ki_, kd_,
        output_limit_,
        deadband_,
        setpoint_max_accel_
      );
      pid_[i]->reset();
      meas_w_filt_[i] = 0.0;
      u_last_[i] = 0.0;
    }
  }

  void onWheelCmd(const base::msg::WheelVelocities::SharedPtr msg)
  {
    target_w_radps_[FL] = msg->left;
    target_w_radps_[RL] = msg->left;
    target_w_radps_[FR] = msg->right;
    target_w_radps_[RR] = msg->right;
  }

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double pfl, pfr, prl, prr;
    const bool ok_fl = getJointPosition(*msg, fl_joint_, pfl);
    const bool ok_fr = getJointPosition(*msg, fr_joint_, pfr);
    const bool ok_rl = getJointPosition(*msg, rl_joint_, prl);
    const bool ok_rr = getJointPosition(*msg, rr_joint_, prr);
    if (!(ok_fl && ok_fr && ok_rl && ok_rr)) return;

    const int64_t raw[WHEEL_COUNT] = {
      posToRawTicks(pfl, invert_fl_),
      posToRawTicks(pfr, invert_fr_),
      posToRawTicks(prl, invert_rl_),
      posToRawTicks(prr, invert_rr_)
    };

    const rclcpp::Time stamp = isZeroStamp(msg->header.stamp) ? now() : rclcpp::Time(msg->header.stamp);

    if (!have_js_) {
      // init baseline
      for (int i = 0; i < WHEEL_COUNT; ++i) {
        raw_last_[i] = raw[i];
        raw_valid_[i] = true;
      }
      last_js_stamp_ = stamp;
      have_js_ = true;
      return;
    }

    const double dt = (stamp - last_js_stamp_).seconds();
    if (!(dt > 0.0) || dt > 1.0) {
      // ignore insane dt
      last_js_stamp_ = stamp;
      for (int i = 0; i < WHEEL_COUNT; ++i) raw_last_[i] = raw[i];
      return;
    }

    // compute dticks from this JS update (and unwrap)
    int64_t dticks[WHEEL_COUNT]{0,0,0,0};
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      if (!raw_valid_[i]) {
        raw_last_[i] = raw[i];
        raw_valid_[i] = true;
        dticks[i] = 0;
        continue;
      }

      int64_t d = raw[i] - raw_last_[i];
      if (unwrap_enabled_) {
        d = deltaModulo(raw[i], raw_last_[i], modulo_ticks_);
      }
      raw_last_[i] = raw[i];

      // glitch rejection (USB spikes / lost packets)
      if (std::llabs(d) > max_tick_jump_) {
        d = 0;
      }

      dticks[i] = d;

      // accumulate continuous ticks for publishing
      ticks_cont_[i].store(ticks_cont_[i].load(std::memory_order_relaxed) + d, std::memory_order_relaxed);
    }

    // ticks -> rad/s using dt from JS packets (NOT control_dt)
    const double k = (2.0 * M_PI) / ticks_per_rev_;
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      double w = (static_cast<double>(dticks[i]) * k) / dt;   // rad/s
      w = clampAbs(w, max_meas_radps_);
      meas_w_filt_[i] = meas_alpha_ * w + (1.0 - meas_alpha_) * meas_w_filt_[i];
    }

    last_js_stamp_ = stamp;
  }

  void controlLoop()
  {
    const rclcpp::Time tnow = now();

    // If no fresh JS -> stop (otherwise PID goes crazy on stale measurement)
    if (!have_js_ || (tnow - last_js_stamp_).seconds() > jointstate_timeout_s_) {
      for (int i = 0; i < WHEEL_COUNT; ++i) {
        pid_[i]->reset();
        u_last_[i] = 0.0;
        writeMotor(i, 0.0);
      }
      publishTicks(tnow);
      return;
    }

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const double sp = target_w_radps_[i];

      if (std::abs(sp) < stop_eps_radps_) {
        pid_[i]->reset();
        u_last_[i] = 0.0;
        writeMotor(i, 0.0);
        continue;
      }

      // measured speed comes from jointStatesCallback (filtered)
      const double meas = meas_w_filt_[i];

      double u = pid_[i]->compute(sp, meas, control_dt_);

      if (u_slew_rate_ > 0.0) {
        const double du_max = u_slew_rate_ * control_dt_;
        u = std::clamp(u, u_last_[i] - du_max, u_last_[i] + du_max);
      }
      u_last_[i] = u;

      writeMotor(i, u);
    }

    publishTicks(tnow);
  }

  void publishTicks(const rclcpp::Time& stamp)
  {
    base::msg::WheelTicks4 out;
    out.header.stamp = stamp;
    out.header.frame_id = ticks_frame_id_;
    out.fl_ticks = ticks_cont_[FL].load(std::memory_order_relaxed);
    out.fr_ticks = ticks_cont_[FR].load(std::memory_order_relaxed);
    out.rl_ticks = ticks_cont_[RL].load(std::memory_order_relaxed);
    out.rr_ticks = ticks_cont_[RR].load(std::memory_order_relaxed);
    ticks_pub_->publish(out);
  }

  void writeMotor(int wheel, double u_norm)
  {
    const double u = std::clamp(u_norm * wheel_gains_[wheel], -output_limit_, output_limit_);

    int pct = static_cast<int>(std::lround((u / output_limit_) * 100.0));
    pct = std::clamp(pct, pct_min_, pct_max_);

    int pulse = static_cast<int>(std::lround(
      static_cast<double>(neutral_us_) + (static_cast<double>(pct) * k_us_per_pct_)
    ));
    pulse = std::clamp(pulse, min_us_, max_us_);

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
