#include "base/wheel_controller_node.h"

namespace base {

WheelControllerNode::WheelControllerNode() : Node("wheel_controller_node") {
    // Declare Params
    this->declare_parameter("wheel_id", "fl"); // fl, fr, rl, rr
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);

    wheel_id_ = this->get_parameter("wheel_id").as_string();
    double kp = this->get_parameter("kp").as_double();
    double ki = this->get_parameter("ki").as_double();
    double kd = this->get_parameter("kd").as_double();

    // Init Logic
    pid_ = std::make_unique<PIDController>(kp, ki, kd);

    // Subscribers
    sub_cmd_ = this->create_subscription<base::msg::WheelVelocities>(
        "/wheel_commands", 10, 
        std::bind(&WheelControllerNode::commandCallback, this, std::placeholders::_1));

    // Publishers (State debug + implicit "simulated" feedback)
    std::string state_topic = "/wheel_controller_" + wheel_id_ + "/state";
    pub_state_ = this->create_publisher<base::msg::WheelControllerState>(state_topic, 10);

    // Control Loop Timer (50Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), 
        std::bind(&WheelControllerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Wheel Controller %s Started. PID: %.2f, %.2f, %.2f", 
        wheel_id_.c_str(), kp, ki, kd);
}

void WheelControllerNode::commandCallback(const base::msg::WheelVelocities::SharedPtr msg) {
    if (wheel_id_ == "fl") setpoint_ = msg->front_left;
    else if (wheel_id_ == "fr") setpoint_ = msg->front_right;
    else if (wheel_id_ == "rl") setpoint_ = msg->rear_left;
    else if (wheel_id_ == "rr") setpoint_ = msg->rear_right;
}

void WheelControllerNode::controlLoop() {
    double dt = 0.02; // 50Hz fixed for simulation

    // 1. Calculate PID
    auto result = pid_->calculate(setpoint_, current_velocity_, dt);

    // 2. Simulate Motor Plant (Simple 1st Order)
    // velocity += (effort - friction*velocity) * dt
    double friction = 0.1;
    double motor_gain = 5.0; // torque to velocity conversion factor
    
    // Apply effort to simulation
    double effort = result.output;
    double acceleration = (effort * motor_gain) - (friction * current_velocity_);
    current_velocity_ += acceleration * dt;

    // 3. Publish State
    base::msg::WheelControllerState state_msg;
    state_msg.header.stamp = this->now();
    state_msg.setpoint_velocity = setpoint_;
    state_msg.measured_velocity = current_velocity_;
    state_msg.velocity_error = result.error;
    state_msg.p_term = result.p_term;
    state_msg.i_term = result.i_term;
    state_msg.d_term = result.d_term;
    state_msg.output_effort = effort;

    pub_state_->publish(state_msg);
}

} // namespace base

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<base::WheelControllerNode>());
    rclcpp::shutdown();
    return 0;
}
