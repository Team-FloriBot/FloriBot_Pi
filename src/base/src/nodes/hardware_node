// src/nodes/hardware_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

// Linux I2C
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using namespace std::chrono_literals;

class NxtServoI2C {
public:
  NxtServoI2C(const std::string& dev, int addr) : dev_(dev), addr_(addr) {}

  void openBus() {
    fd_ = ::open(dev_.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("open(" + dev_ + ") failed");
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) throw std::runtime_error("ioctl(I2C_SLAVE) failed");
  }

  void closeBus() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  ~NxtServoI2C(){ closeBus(); }

  // schreibt 2 Bytes (low, high) ab Register (I2C block write)
  void write2(uint8_t reg, uint8_t low, uint8_t high) {
    uint8_t buf[3] = { reg, low, high };
    if (::write(fd_, buf, 3) != 3) throw std::runtime_error("I2C write2 failed");
  }

  // schreibt 1 Byte ab Register
  void write1(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    if (::write(fd_, buf, 2) != 2) throw std::runtime_error("I2C write1 failed");
  }

private:
  std::string dev_;
  int addr_{0};
  int fd_{-1};
};

class HardwareNode : public rclcpp::Node {
public:
  HardwareNode() : Node("hardware_node") {
    // Aus Seminararbeiten: /dev/i2c-1 und Adresse 0x58 :contentReference[oaicite:12]{index=12} :contentReference[oaicite:13]{index=13}
    i2c_dev_   = declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
    i2c_addr_  = declare_parameter<int>("i2c_addr", 0x58);

    // Channel-Mapping: links [0,2], rechts [1,3] :contentReference[oaicite:14]{index=14}
    left_channels_  = declare_parameter<std::vector<int>>("left_channels", {0,2});
    right_channels_ = declare_parameter<std::vector<int>>("right_channels", {1,3});

    // Umrechnung wie Testprogramm: high_time_us = speed*5.5 + 1480 :contentReference[oaicite:15]{index=15}
    k_us_per_pct_   = declare_parameter<double>("k_us_per_pct", 5.5);
    neutral_us_     = declare_parameter<int>("neutral_us", 1480);
    pct_min_        = declare_parameter<int>("pct_min", -100);
    pct_max_        = declare_parameter<int>("pct_max",  100);

    // Beschleunigung: register = channel + 82, 0..255 (µs/24ms) :contentReference[oaicite:16]{index=16}
    accel_          = declare_parameter<int>("acceleration", 30);   // praxisnaher Startwert aus Testprogrammen :contentReference[oaicite:17]{index=17}

    watchdog_ms_    = declare_parameter<int>("watchdog_timeout_ms", 300);

    sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "/base/wheel_cmd", 10,
      [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg){ onWheelCmd(*msg); }
    );

    pub_applied_ = create_publisher<std_msgs::msg::Int16MultiArray>("/base/wheel_cmd_applied", 10);

    // I2C öffnen
    nxt_ = std::make_unique<NxtServoI2C>(i2c_dev_, i2c_addr_);
    nxt_->openBus();

    // Wichtig: sofort stabilisieren -> alle Motoren auf 0 und Beschleunigung setzen
    // Beobachtung: Startbewegung durch Grundspannung/Defekt; durch Setzen von 0 stabil. :contentReference[oaicite:18]{index=18}
    setAllAcceleration(accel_);
    stopAll();

    last_cmd_time_ = now();

    timer_ = create_wall_timer(50ms, [this]{ watchdog(); });
  }

private:
  void onWheelCmd(const std_msgs::msg::Int16MultiArray& msg) {
    if (msg.data.size() < 2) return;

    const int left_pct  = clampPct(msg.data[0]);
    const int right_pct = clampPct(msg.data[1]);

    applySide(left_channels_,  left_pct);
    applySide(right_channels_, right_pct);

    last_cmd_time_ = now();

    std_msgs::msg::Int16MultiArray out;
    out.data = {static_cast<int16_t>(left_pct), static_cast<int16_t>(right_pct)};
    pub_applied_->publish(out);
  }

  int clampPct(int v) const {
    return std::clamp(v, pct_min_, pct_max_);
  }

  void applySide(const std::vector<int>& channels, int pct) {
    // Register-Mapping aus Testprogramm:
    // register = (channel*2) + 66; write [low, high] der Pulsbreite :contentReference[oaicite:19]{index=19}
    const int us = static_cast<int>(std::lround(pct * k_us_per_pct_ + neutral_us_));
    const uint8_t low  = static_cast<uint8_t>(us & 0xFF);
    const uint8_t high = static_cast<uint8_t>((us >> 8) & 0xFF);

    for (int ch : channels) {
      const uint8_t reg = static_cast<uint8_t>((ch * 2) + 66);
      nxt_->write2(reg, low, high);
    }
  }

  void setAllAcceleration(int accel) {
    accel = std::clamp(accel, 0, 255);
    // register = channel + 82 :contentReference[oaicite:20]{index=20}
    for (int ch = 0; ch < 4; ++ch) {
      nxt_->write1(static_cast<uint8_t>(ch + 82), static_cast<uint8_t>(accel));
    }
  }

  void stopAll() {
    // stop = speed 0 -> high_time_us = neutral_us (≈1480 µs im Testprogramm) :contentReference[oaicite:21]{index=21}
    applySide(left_channels_,  0);
    applySide(right_channels_, 0);
  }

  void watchdog() {
    const auto dt = now() - last_cmd_time_;
    if (dt > rclcpp::Duration(0, watchdog_ms_ * 1000000LL)) {
      stopAll();
    }
  }

  std::string i2c_dev_;
  int i2c_addr_{0};
  std::vector<int> left_channels_;
  std::vector<int> right_channels_;
  double k_us_per_pct_{5.5};
  int neutral_us_{1480};
  int pct_min_{-100}, pct_max_{100};
  int accel_{30};
  int watchdog_ms_{300};

  rclcpp::Time last_cmd_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<NxtServoI2C> nxt_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_applied_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareNode>());
  rclcpp::shutdown();
  return 0;
}
