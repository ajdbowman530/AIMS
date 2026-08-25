#pragma once

#include <numbers>

class OuterLoopPI {
private:
	double Kp; // Proportional gain
	double Ki; // Integral gain
	double integral_sum; // Integral term
    double min_output; // Minimum output limit
    double max_output; // Maximum output limit

    double pi = std::numbers::pi;

public:
    OuterLoopPI() = default;
    void set_gains(double p, double i) { Kp = p; Ki = i; }

    double update(double ref_rad, double state_rad, double dt);

    const double calculate_angular_error(double reference_rad, double state_rad);

    void set_limits(double min_limit, double max_limit) {
        // Set using dynamic limiter
		min_output = min_limit;
		max_output = max_limit;
    }

    void reset() { integral_sum = 0.0; }
};

