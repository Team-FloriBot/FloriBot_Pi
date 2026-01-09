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
        "wheel_gains", std::v_
