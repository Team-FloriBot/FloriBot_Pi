#ifndef BASE_SENSOR_AGGREGATOR_NODE_H
#define BASE_SENSOR_AGGREGATOR_NODE_H

#include <cstdint>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
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

    // Haupt-Timer-Funktion zur Aggregation und Veröffentlichung
    void publishAggregatedStates();

    // Tick-Integration aus gemessener Winkelgeschwindigkeit (rad/s)
    void updateTicks(double omega_rad_s, int64_t &tick_counter);

    // Abonnements
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_fl_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_fr_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_rl_;
    rclcpp::Subscription<base::msg::WheelControllerState>::SharedPtr sub_rr_;

    // Publisher
    rclcpp::Publisher<base::msg::WheelVelocities>::SharedPtr pub_wheel_states_;
    rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr pub_wheel_ticks_;

    // Gespeicherte Zustände (die letzten gemessenen Geschwindigkeiten)
    base::msg::WheelVelocities current_velocities_;

    // Kumulative Encoderstände (integriert)
    int64_t fl_ticks_{0};
    int64_t fr_ticks_{0};
    int64_t rl_ticks_{0};
    int64_t rr_ticks_{0};

    // Zeitbasis für Integration
    rclcpp::Time last_update_time_;
    bool time_initialized_{false};

    // Encoderauflösung (Quadratur bereits enthalten; ggf. anpassen)
    static constexpr int64_t TICKS_PER_REV = 2048;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace base

#endif // BASE_SENSOR_AGGREGATOR_NODE_H
