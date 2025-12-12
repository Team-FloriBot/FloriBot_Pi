#include "base/sensor_aggregator_node.h"

using std::placeholders::_1;

namespace base {

SensorAggregatorNode::SensorAggregatorNode() : Node("sensor_aggregator_node") {
    // Publisher auf dem Topic, auf das KinematicsNode wartet
    pub_wheel_states_ = this->create_publisher<base::msg::WheelVelocities>("/wheel_states", 10);

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

void SensorAggregatorNode::publishAggregatedStates() {
    // Veröffentliche die gesammelten und aktuellsten Daten
    pub_wheel_states_->publish(current_velocities_);
}

} // namespace base

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<base::SensorAggregatorNode>());
    rclcpp::shutdown();
    return 0;
}
