#ifndef BASE_SENSOR_AGGREGATOR_NODE_H
#define BASE_SENSOR_AGGREGATOR_NODE_H

#include <cstdint>
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
    // Wheel controller states
    void flStateCallback(const base::msg::WheelControllerState::SharedPtr msg);
    void frStateCallback(const base::msg::WheelControllerState::SharedPtr msg);
    void rlStateCallback(const base::msg::WheelControllerState::SharedPtr msg);
    void rrStateCallback(const base::msg::WheelControllerState::SharedPtr msg);

    // Phidget joint states
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Periodic publish
    void publishAggregatedStates();

    // Subscriptions
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_fl_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_fr_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_rl_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_rr_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

    // Publishers
    rclcpp::Publisher<base::msg::WheelVelocities>::SharedPtr pub_wheel_states_;
    rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr pub_wheel_ticks_;

    // Latest velocities
    base::msg::WheelVelocities current_velocities_;

    // Encoder ticks (absolute/cumulative from joint_states.position)
    int64_t fl_ticks_{0};
    int64_t fr_ticks_{0};
    int64_t rl_ticks_{0};
    int64_t rr_ticks_{0};

    // JointState handling
    std::string joint_states_topic_{"/joint_states"};
    rclcpp::Time last_joint_stamp_;
    bool have_joint_state_{false};

    // Mapping parameters
    std::string fl_joint_{"joint2"};
    std::string fr_joint_{"joint3"};  // falls du wirklich joint4 hast, setze per param
    std::string rl_joint_{"joint0"};
    std::string rr_joint_{"joint1"};

    bool invert_fl_{false};
    bool invert_fr_{false};
    bool invert_rl_{false};
    bool invert_rr_{false};

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace base

#endif // BASE_SENSOR_AGGREGATOR_NODE_H
