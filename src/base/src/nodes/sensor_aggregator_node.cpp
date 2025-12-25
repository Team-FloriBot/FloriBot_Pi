#include "base/sensor_aggregator_node.h"

#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

using std::placeholders::_1;

namespace base {

SensorAggregatorNode::SensorAggregatorNode() : Node("sensor_aggregator_node") {
    // Publisher: Aggregierte Radgeschwindigkeiten (wie bisher)
    pub_wheel_states_ = this->create_publisher<base::msg::WheelVelocities>("/wheel_states", 10);

    // Publisher: Encoder-Ticks (aus echten Phidget-Encoderpositionen)
    pub_wheel_ticks_ = this->create_publisher<base::msg::WheelTicks4>("/base/wheel_ticks4", 10);

    // Abonnements für die vier individuellen Controller-Status-Topics (wie bisher)
    sub_fl_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_fl/state", 10, std::bind(&SensorAggregatorNode::flStateCallback, this, _1));
    sub_fr_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_fr/state", 10, std::bind(&SensorAggregatorNode::frStateCallback, this, _1));
    sub_rl_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_rl/state", 10, std::bind(&SensorAggregatorNode::rlStateCallback, this, _1));
    sub_rr_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_rr/state", 10, std::bind(&SensorAggregatorNode::rrStateCallback, this, _1));

    // NEW: JointStates von Phidget HighSpeedEncoder
    // Default: phidgets_high_speed_encoder publisht /joint_states
    joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10, std::bind(&SensorAggregatorNode::jointStatesCallback, this, _1));

    // Timer, um die gesammelten Daten regelmäßig (z.B. 50Hz) zu veröffentlichen
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&SensorAggregatorNode::publishAggregatedStates, this));

    RCLCPP_INFO(this->get_logger(), "Sensor Aggregator Node Initialized.");
}

void SensorAggregatorNode::flStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.front_left = msg->measured_velocity;
}

void SensorAggregatorNode::frStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.front_right = msg->measured_velocity;
}

void SensorAggregatorNode::rlStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.rear_left = msg->measured_velocity;
}

void SensorAggregatorNode::rrStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.rear_right = msg->measured_velocity;
}

static bool getJointPosition(const sensor_msgs::msg::JointState &js, const std::string &name, double &pos_out)
{
    // Find joint index by name and read position if available
    auto it = std::find(js.name.begin(), js.name.end(), name);
    if (it == js.name.end()) return false;

    const size_t idx = static_cast<size_t>(std::distance(js.name.begin(), it));
    if (idx >= js.position.size()) return false;

    pos_out = js.position[idx];
    return true;
}

void SensorAggregatorNode::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Default mapping: joint0..joint3 -> fl, fr, rl, rr
    // (später parametrierbar, aber jetzt deterministisch und passend zu deiner Ausgabe)
    double p0, p1, p2, p3;
    const bool ok0 = getJointPosition(*msg, "joint0", p0);
    const bool ok1 = getJointPosition(*msg, "joint1", p1);
    const bool ok2 = getJointPosition(*msg, "joint2", p2);
    const bool ok3 = getJointPosition(*msg, "joint3", p3);

    if (!(ok0 && ok1 && ok2 && ok3)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "joint_states missing one of joint0..joint3 (names/positions not available yet)");
        return;
    }

    // Position ist bei dir bereits ein Zählwert (Integer als double, z.B. 898.0)
    // Wir übernehmen das als kumulative Ticks.
    fl_ticks_ = static_cast<int64_t>(std::llround(p0));
    fr_ticks_ = static_cast<int64_t>(std::llround(p1));
    rl_ticks_ = static_cast<int64_t>(std::llround(p2));
    rr_ticks_ = static_cast<int64_t>(std::llround(p3));

    last_joint_stamp_ = msg->header.stamp;
    have_joint_state_ = true;
}

void SensorAggregatorNode::publishAggregatedStates() {
    // 1) Veröffentliche die gesammelten und aktuellsten Geschwindigkeiten
    pub_wheel_states_->publish(current_velocities_);

    // 2) Veröffentliche echte (kumulative) Encoder-Ticks aus /joint_states
    if (!have_joint_state_) {
        // Encoder noch nicht angekommen
        return;
    }

    base::msg::WheelTicks4 ticks;
    ticks.header.stamp = (last_joint_stamp_.nanoseconds() != 0) ? last_joint_stamp_ : this->now();
    ticks.header.frame_id = "base_link";

    ticks.fl_ticks = fl_ticks_;
    ticks.fr_ticks = fr_ticks_;
    ticks.rl_ticks = rl_ticks_;
    ticks.rr_ticks = rr_ticks_;

    pub_wheel_ticks_->publish(ticks);
}

} // namespace base

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<base::SensorAggregatorNode>());
    rclcpp::shutdown();
    return 0;
}
