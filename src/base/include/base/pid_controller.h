#ifndef BASE_PID_CONTROLLER_H
#define BASE_PID_CONTROLLER_H

namespace base {

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    
    struct State {
        double p_term;
        double i_term;
        double d_term;
        double output;
        double error;
    };

    // Calculate control effort
    State calculate(double setpoint, double measured, double dt);
    
    // Reset integral windup
    void reset();

private:
    double kp_, ki_, kd_;
    double integral_sum_;
    double prev_error_;
};

} // namespace base

#endif // BASE_PID_CONTROLLER_H
