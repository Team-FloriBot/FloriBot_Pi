#include "base/pid_controller.h"
#include <algorithm> // for clamping if needed

namespace base {

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_sum_(0.0), prev_error_(0.0) {}

PIDController::State PIDController::calculate(double setpoint, double measured, double dt) {
    double error = setpoint - measured;
    
    // P Term
    double p_out = kp_ * error;

    // I Term
    integral_sum_ += error * dt;
    double i_out = ki_ * integral_sum_;

    // D Term
    double derivative = (error - prev_error_) / dt;
    double d_out = kd_ * derivative;

    prev_error_ = error;

    double total_output = p_out + i_out + d_out;

    return {p_out, i_out, d_out, total_output, error};
}

void PIDController::reset() {
    integral_sum_ = 0.0;
    prev_error_ = 0.0;
}

} // namespace base
