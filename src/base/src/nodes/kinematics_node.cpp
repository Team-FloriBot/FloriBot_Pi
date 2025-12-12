#include "base/kinematics_node.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

namespace base {

KinematicsNode::KinematicsNode() : Node("kinematics_node") {
    // Parameters
    this->declare_parameter("wheel_separation", 0.5);
    this->declare_parameter("wheel_radius", 0.1);

    double wheel_sep = this->get_parameter("wheel_separation").as_double();
    double wheel_rad = this->get_parameter("wheel_radius").as_double();

    // Initialize Library
    kinematics_ = std::make_unique<KinematicsCalculator>(wheel_sep, wheel_rad);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Setup Pub/Sub
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&KinematicsNode::cmdVelCallback, this, _1));

    sub_wheel_states_ = this->create_subscription<base::msg::WheelVelocities>(
        "/wheel_states", 10, std::bind(&KinematicsNode::wheelStateCallback, this, _1));

    pub_wheel_cmd_ = this->create_publisher<base::msg::WheelVelocities>("/wheel_commands", 10);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    last_odom_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Kinematics Node Initialized. Sep: %.2f, Rad: %.2f", wheel_sep, wheel_rad);
}

void KinematicsNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // 1. Calculate target wheel speeds
    WheelSpeedSet speeds = kinematics_->calculateWheelSpeeds(msg->linear.x, msg->angular.z);

    // 2. Publish
    base::msg::WheelVelocities out_msg;
    out_msg.front_left = speeds.fl;
    out_msg.front_right = speeds.fr;
    out_msg.rear_left = speeds.rl;
    out_msg.rear_right = speeds.rr;
    
    pub_wheel_cmd_->publish(out_msg);
}

void KinematicsNode::wheelStateCallback(const base::msg::WheelVelocities::SharedPtr msg) {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_odom_time_).seconds();
    if (dt <= 0) return;

    // 1. Convert wheel speeds to robot twist
    WheelSpeedSet measured;
    measured.fl = msg->front_left;
    measured.fr = msg->front_right;
    measured.rl = msg->rear_left;
    measured.rr = msg->rear_right;

    RobotTwist twist = kinematics_->calculateRobotTwist(measured);

    // 2. Integrate position
    double delta_x = (twist.linear_x * cos(theta_) - twist.linear_y * sin(theta_)) * dt;
    double delta_y = (twist.linear_x * sin(theta_) + twist.linear_y * cos(theta_)) * dt;
    double delta_th = twist.angular_z * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_th;

    // 3. Publish Odom & TF
    publishOdometry(twist, dt);
    
    last_odom_time_ = current_time;
}

void KinematicsNode::publishOdometry(const RobotTwist& twist, double /*dt*/) {
    // TF Quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    // Publish TF: odom -> base_link
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = this->now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(odom_tf);

    // Publish Odom Message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.orientation = odom_tf.transform.rotation;

    odom_msg.twist.twist.linear.x = twist.linear_x;
    odom_msg.twist.twist.angular.z = twist.angular_z;

    pub_odom_->publish(odom_msg);
}

} // namespace base

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<base::KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
