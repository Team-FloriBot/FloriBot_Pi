#ifndef BASE_SENSOR_AGGREGATOR_NODE_H
#define BASE_SENSOR_AGGREGATOR_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_controller_state.hpp"

namespace base {

class SensorAggregatorNode : public rclcpp::Node {
public:
    SensorAggregatorNode();

private:
    // Callbacks für die vier individuellen WheelControllerStates
    void flStateCallback(const base::msg::WheelControllerState::SharedPtr msg);
    void frStateCallback(const base::msg::WheelControllerState::SharedPtr msg);
    void rlStateCallback(const base::msg::WheelControllerState::SharedPtr msg);
    void rrStateCallback(const base::msg::WheelControllerState::SharedPtr msg);

    // Haupt-Timer-Funktion zur Aggregation und Veröffentlichung
    void publishAggregatedStates();

    // Abonnements
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_fl_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_fr_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_rl_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_rr_;

    // Publisher
    rclcpp::Publisher<base::msg::WheelVelocities>::SharedPtr pub_wheel_states_;

    // Gespeicherte Zustände (die letzten gemessenen Geschwindigkeiten)
    base::msg::WheelVelocities current_velocities_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace base

#endif // BASE_SENSOR_AGGREGATOR_NODE_H
