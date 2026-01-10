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

static bool getJointPosition(const sensor_msgs::msg::JointState& js, const std::string& name, double& pos_out)
{
  auto it = std::find(js.name.begin(), js.name.end(), name);
  if (it == js.name.end()) return false;

  const size_t idx = static_cast<size_t>(std::distance(js.name.begin(), it));
  if (idx >= js.position.size()) return false;

  pos_out = js.position[idx];
  return true;
}

static int64_t toTicks(double pos, bool invert)
{
  const int64_t t = static_cast<int64_t>(std::llround(pos));  // encodertest: position already "ticks"
  return invert ? -t : t;
}

class HardwareNode : public rclcpp::Node
{
public:
  HardwareNode()
  : Node("hardware_node")
  {
    declareParams();
    getParams();

    // Open I2C motor controller
    nxt_.openBus(i2c_dev_, i2c_addr_);

    wheel_cmd_sub_ = create_subscription<base::msg::WheelVelocities>(
      wheel_cmd_topic_, rclcpp::QoS(10),
      std::bind(&HardwareNode::onWheelCmd, this, std::placeholders::_1));

    // Encoder source like in encodertest: /joint_states
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10),
      std::bind(&HardwareNode::jointStatesCallback, this, std::placeholders::_1));

    ticks_pub_ = create_publisher<base::msg::WheelTicks4>(wheel_ticks_topic_, rclcpp::QoS(10));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(control_dt_),
      std::bind(&HardwareNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
      "hardware_node: using joint_states topic=%s, joints FL=%s FR=%s RL=%s RR=%s",
      joint_states_topic_.c_str(),
      fl_joint_.c_str(), fr_joint_.c_str(), rl_joint_.c_str(), rr_joint_.c_str());
  }

  ~HardwareNode() override
  {
    nxt_.closeBus();
  }

private:
  enum WheelIndex { FL = 0, FR = 1, RL = 2, RR = 3, WHEEL_COUNT = 4 };

  // ---- ROS params ----
  std::string wheel_cmd_topic_{"/wheel_commands"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};
  std::string ticks_frame_id_{"base_link"};

  // Control timing + conversion
  double control_dt_{0.02};        // [s]
  double ticks_per_rev_{2048.0};   // [ticks / wheel rev]

  // PID parameters
  double kp_{0.6};
  double ki_{0.0};
  double kd_{0.0};
  double output_limit_{1.0};
  double deadband_{0.0};
  double setpoint_max_accel_{std::numeric_limits<double>::infinity()}; // [rad/s^2]

  // ---- Encoder source like encodertest: /joint_states mapping ----
  std::string joint_states_topic_{"/joint_states"};
  std::string fl_joint_{"joint2"};
  std::string fr_joint_{"joint3"};
  std::string rl_joint_{"joint0"};
  std::string rr_joint_{"joint1"};

  bool invert_fl_{false};
  bool invert_fr_{false};
  bool invert_rl_{false};
  bool invert_rr_{false};

  rclcpp::Time last_joint_stamp_;
  bool have_joint_state_{false};

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
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr ticks_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Controllers ----
  std::array<std::unique_ptr<base::PIDController>, WHEEL_COUNT> pid_{};

  // Targets (rad/s)
  double target_w_radps_[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

  // Encoder ticks (absolute), updated from joint_states
  std::atomic<int64_t> ticks_[WHEEL_COUNT]{0, 0, 0, 0};
  int64_t ticks_last_[WHEEL_COUNT]{0, 0, 0, 0};

  // Motor driver
  NxtServoI2C nxt_;

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

    // joint_states mapping (encodertest style)
    declare_parameter<std::string>("joint_states_topic", joint_states_topic_);
    declare_parameter<std::string>("fl_joint", fl_joint_);
    declare_parameter<std::string>("fr_joint", fr_joint_);
    declare_parameter<std::string>("rl_joint", rl_joint_);
    declare_parameter<std::string>("rr_joint", rr_joint_);

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

    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    fl_joint_ = get_parameter("fl_joint").as_string();
    fr_joint_ = get_parameter("fr_joint").as_string();
    rl_joint_ = get_parameter("rl_joint").as_string();
    rr_joint_ = get_parameter("rr_joint").as_string();

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

    if (!(ok_fl && ok_fr && ok_rl && ok_rr)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "joint_states missing mapping: FL(%s)=%d FR(%s)=%d RL(%s)=%d RR(%s)=%d",
        fl_joint_.c_str(), (int)ok_fl,
        fr_joint_.c_str(), (int)ok_fr,
        rl_joint_.c_str(), (int)ok_rl,
        rr_joint_.c_str(), (int)ok_rr
      );
      return;
    }

    ticks_[FL].store(toTicks(pfl, invert_fl_), std::memory_order_relaxed);
    ticks_[FR].store(toTicks(pfr, invert_fr_), std::memory_order_relaxed);
    ticks_[RL].store(toTicks(prl, invert_rl_), std::memory_order_relaxed);
    ticks_[RR].store(toTicks(prr, invert_rr_), std::memory_order_relaxed);

    rclcpp::Time stamp(msg->header.stamp);
    last_joint_stamp_ = (stamp.nanoseconds() != 0) ? stamp : now();
    have_joint_state_ = true;
  }

  void controlLoop()
  {
    // Without joint_states we can't compute speed -> keep motors neutral
    if (!have_joint_state_) {
      for (int i = 0; i < WHEEL_COUNT; ++i) writeMotor(i, 0.0);
      return;
    }

    // measured wheel speed from tick delta
    double meas_w_radps[WHEEL_COUNT]{0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const int64_t t = ticks_[i].load(std::memory_order_relaxed);
      const int64_t dticks = t - ticks_last_[i];
      ticks_last_[i] = t;

      const double rev = static_cast<double>(dticks) / ticks_per_rev_;
      const double rad = rev * 2.0 * M_PI;
      meas_w_radps[i] = rad / control_dt_;
    }

    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const double u = pid_[i]->compute(target_w_radps_[i], meas_w_radps[i], control_dt_);
      writeMotor(i, u);
    }

    base::msg::WheelTicks4 out;
    out.header.stamp = last_joint_stamp_;
    out.header.frame_id = ticks_frame_id_;
    out.fl_ticks = ticks_[FL].load(std::memory_order_relaxed);
    out.fr_ticks = ticks_[FR].load(std::memory_order_relaxed);
    out.rl_ticks = ticks_[RL].load(std::memory_order_relaxed);
    out.rr_ticks = ticks_[RR].load(std::memory_order_relaxed);
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
