#include "../include/outer_loop_controller.h"
#include <cmath>
#include <algorithm>

double OuterLoopPI::update(double ref_rad, double state_rad, double dt) {
    double error = calculate_angular_error(ref_rad, state_rad);
    double p_term = Kp * error;

    // Anti-windup clamping
    double tentative_integral = integral_sum + (error * dt);
    double raw_output = p_term + (Ki * tentative_integral);

    double clamped_output = std::clamp(raw_output, min_output, max_output);

    if (raw_output == clamped_output) {
        integral_sum = tentative_integral; // Unsaturated
    }
    else { // Saturated
        bool error_helps = (raw_output > max_output && error < 0.0) ||
            (raw_output < min_output && error > 0.0);
        if (error_helps) {
            integral_sum = tentative_integral;
        }
    }
    return clamped_output;
}

const double OuterLoopPI::calculate_angular_error(double reference_rad, double state_rad) {
    double error = reference_rad - state_rad;
    while (error > pi)  error -= 2.0 * pi;
    while (error < -pi) error += 2.0 * pi;
    return error;
}