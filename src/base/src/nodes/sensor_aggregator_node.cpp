#include "base/sensor_aggregator_node.h"

#include <algorithm>
#include <cmath>

using std::placeholders::_1;

namespace base {

static bool getJointPosition(const sensor_msgs::msg::JointState &js, const std::string &name, double &pos_out)
{
    auto it = std::find(js.name.begin(), js.name.end(), name);
    if (it == js.name.end()) return false;

    const size_t idx = static_cast<size_t>(std::distance(js.name.begin(), it));
    if (idx >= js.position.size()) return false;

    pos_out = js.position[idx];
    return true;
}

static int64_t toTicks(double pos, bool invert)
{
    const int64_t t = static_cast<int64_t>(std::llround(pos));
    return invert ? -t : t;
}

SensorAggregatorNode::SensorAggregatorNode() : Node("sensor_aggregator_node")
{
    pub_wheel_states_ = this->create_publisher<base::msg::WheelVelocities>("/wheel_states", 10);
    pub_wheel_ticks_  = this->create_publisher<base::msg::WheelTicks4>("/base/wheel_ticks4", 10);

    sub_fl_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_fl/state", 10, std::bind(&SensorAggregatorNode::flStateCallback, this, _1));
    sub_fr_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_fr/state", 10, std::bind(&SensorAggregatorNode::frStateCallback, this, _1));
    sub_rl_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_rl/state", 10, std::bind(&SensorAggregatorNode::rlStateCallback, this, _1));
    sub_rr_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_rr/state", 10, std::bind(&SensorAggregatorNode::rrStateCallback, this, _1));

    // Params: joint_states topic + mapping
    joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");

    fl_joint_ = this->declare_parameter<std::string>("fl_joint", fl_joint_);
    fr_joint_ = this->declare_parameter<std::string>("fr_joint", fr_joint_);
    rl_joint_ = this->declare_parameter<std::string>("rl_joint", rl_joint_);
    rr_joint_ = this->declare_parameter<std::string>("rr_joint", rr_joint_);

    invert_fl_ = this->declare_parameter<bool>("invert_fl", false);
    invert_fr_ = this->declare_parameter<bool>("invert_fr", false);
    invert_rl_ = this->declare_parameter<bool>("invert_rl", false);
    invert_rr_ = this->declare_parameter<bool>("invert_rr", false);

    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10, std::bind(&SensorAggregatorNode::jointStatesCallback, this, _1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&SensorAggregatorNode::publishAggregatedStates, this));

    RCLCPP_INFO(this->get_logger(),
        "SensorAggregator: joints FL=%s FR=%s RL=%s RR=%s (topic=%s)",
        fl_joint_.c_str(), fr_joint_.c_str(), rl_joint_.c_str(), rr_joint_.c_str(),
        joint_states_topic_.c_str());
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

void SensorAggregatorNode::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    double pfl, pfr, prl, prr;
    const bool ok_fl = getJointPosition(*msg, fl_joint_, pfl);
    const bool ok_fr = getJointPosition(*msg, fr_joint_, pfr);
    const bool ok_rl = getJointPosition(*msg, rl_joint_, prl);
    const bool ok_rr = getJointPosition(*msg, rr_joint_, prr);

    if (!(ok_fl && ok_fr && ok_rl && ok_rr)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "joint_states missing mapping: FL(%s)=%d FR(%s)=%d RL(%s)=%d RR(%s)=%d",
            fl_joint_.c_str(), (int)ok_fl,
            fr_joint_.c_str(), (int)ok_fr,
            rl_joint_.c_str(), (int)ok_rl,
            rr_joint_.c_str(), (int)ok_rr);
        return;
    }

    fl_ticks_ = toTicks(pfl, invert_fl_);
    fr_ticks_ = toTicks(pfr, invert_fr_);
    rl_ticks_ = toTicks(prl, invert_rl_);
    rr_ticks_ = toTicks(prr, invert_rr_);

    // Convert builtin_interfaces::msg::Time -> rclcpp::Time and check validity
    rclcpp::Time stamp(msg->header.stamp);
    last_joint_stamp_ = (stamp.nanoseconds() != 0) ? stamp : this->now();

    have_joint_state_ = true;
}

void SensorAggregatorNode::publishAggregatedStates()
{
    pub_wheel_states_->publish(current_velocities_);

    if (!have_joint_state_) return;

    base::msg::WheelTicks4 ticks;
    ticks.header.stamp = last_joint_stamp_;
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
