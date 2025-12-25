#include "base/sensor_aggregator_node.h"

#include <cmath>

using std::placeholders::_1;

namespace base {

SensorAggregatorNode::SensorAggregatorNode() : Node("sensor_aggregator_node") {
    // Publisher: Aggregierte Radgeschwindigkeiten (wie bisher)
    pub_wheel_states_ = this->create_publisher<base::msg::WheelVelocities>("/wheel_states", 10);

    // Publisher: Aggregierte Encoder-Ticks (neu)
    pub_wheel_ticks_ = this->create_publisher<base::msg::WheelTicks4>("/base/wheel_ticks4", 10);

    // Abonnements für die vier individuellen Controller-Status-Topics
    sub_fl_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_fl/state", 10, std::bind(&SensorAggregatorNode::flStateCallback, this, _1));
    sub_fr_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_fr/state", 10, std::bind(&SensorAggregatorNode::frStateCallback, this, _1));
    sub_rl_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_rl/state", 10, std::bind(&SensorAggregatorNode::rlStateCallback, this, _1));
    sub_rr_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_rr/state", 10, std::bind(&SensorAggregatorNode::rrStateCallback, this, _1));

    // Timer, um die gesammelten Daten regelmäßig (z.B. 50Hz) zu veröffentlichen
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&SensorAggregatorNode::publishAggregatedStates, this));

    RCLCPP_INFO(this->get_logger(), "Sensor Aggregator Node Initialized.");
}

void SensorAggregatorNode::updateTicks(double omega_rad_s, int64_t &tick_counter)
{
    const rclcpp::Time now = this->now();

    if (!time_initialized_) {
        last_update_time_ = now;
        time_initialized_ = true;
        return;
    }

    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    if (dt <= 0.0) {
        return;
    }

    // revolutions = (omega * dt) / (2*pi)
    const double revolutions = (omega_rad_s * dt) / (2.0 * M_PI);

    // ticks = revolutions * TICKS_PER_REV
    const double ticks_f = revolutions * static_cast<double>(TICKS_PER_REV);

    // Rundung auf int64 (kumulativ)
    const int64_t delta_ticks = static_cast<int64_t>(std::llround(ticks_f));

    tick_counter += delta_ticks;
}

void SensorAggregatorNode::flStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.front_left = msg->measured_velocity;
    updateTicks(msg->measured_velocity, fl_ticks_);
}

void SensorAggregatorNode::frStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.front_right = msg->measured_velocity;
    updateTicks(msg->measured_velocity, fr_ticks_);
}

void SensorAggregatorNode::rlStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.rear_left = msg->measured_velocity;
    updateTicks(msg->measured_velocity, rl_ticks_);
}

void SensorAggregatorNode::rrStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.rear_right = msg->measured_velocity;
    updateTicks(msg->measured_velocity, rr_ticks_);
}

void SensorAggregatorNode::publishAggregatedStates() {
    // 1) Veröffentliche die gesammelten und aktuellsten Geschwindigkeiten
    pub_wheel_states_->publish(current_velocities_);

    // 2) Veröffentliche die integrierten (kumulativen) Encoder-Ticks
    base::msg::WheelTicks4 ticks;
    ticks.header.stamp = this->now();
    ticks.header.frame_id = "base_link";  // optional

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
