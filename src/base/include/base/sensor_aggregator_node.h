#ifndef BASE_SENSOR_AGGREGATOR_NODE_H
#define BASE_SENSOR_AGGREGATOR_NODE_H

#include <cstdint>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_controller_state.hpp"
#include "base/msg/wheel_ticks4.hpp"

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

    // NEW: JointStates von Phidget HighSpeedEncoder (liefert joint0..joint3)
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Haupt-Timer-Funktion zur Aggregation und Veröffentlichung
    void publishAggregatedStates();

    // Abonnements
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_fl_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_fr_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_rl_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_rr_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

    // Publisher
    rclcpp::Publisher<base::msg::WheelVelocities>::SharedPtr pub_wheel_states_;
    rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr pub_wheel_ticks_;

    // Gespeicherte Zustände (die letzten gemessenen Geschwindigkeiten)
    base::msg::WheelVelocities current_velocities_;

    // Kumulative Encoderstände (aus /joint_states übernommen)
    int64_t fl_ticks_{0};
    int64_t fr_ticks_{0};
    int64_t rl_ticks_{0};
    int64_t rr_ticks_{0};

    // JointState Handling
    std::string joint_states_topic_{"/joint_states"};
    rclcpp::Time last_joint_stamp_;
    bool have_joint_state_{false};

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace base

#endif // BASE_SENSOR_AGGREGATOR_NODE_H
