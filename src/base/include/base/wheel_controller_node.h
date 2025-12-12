#ifndef BASE_WHEEL_CONTROLLER_NODE_H
#define BASE_WHEEL_CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "base/msg/wheel_velocities.hpp"
#include "base/msg/wheel_controller_state.hpp"
#include "base/pid_controller.h"

namespace base {

class WheelControllerNode : public rclcpp::Node {
public:
    WheelControllerNode();

private:
    void commandCallback(const base::msg::WheelVelocities::SharedPtr msg);
    void controlLoop(); // Main timer loop

    // Parameters
    std::string wheel_id_;
    
    // Logic
    std::unique_ptr<PIDController> pid_;
    double setpoint_ = 0.0;
    
    // Plant Simulation State
    double current_velocity_ = 0.0;
    
    // ROS Handles
    rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr sub_cmd_;
    rclcpp::Publisher<base::msg::WheelControllerState>::SharedPtr pub_state_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace base

#endif // BASE_WHEEL_CONTROLLER_NODE_H
