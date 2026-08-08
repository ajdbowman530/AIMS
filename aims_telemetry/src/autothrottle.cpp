#include <algorithm>
#include <Eigen/Dense>
#include <vector>

#include "../include/autothrottle.h"
#include "../include/scheduler.h"

Autothrottle::Autothrottle(double KP, double KI, double KD, double dt, double max_dry_throttle)
	: KP_(KP), KI_(KI), KD_(KD), dt_(dt), max_dry_throttle_(max_dry_throttle) {
	if (dt_ <= 0.0) {
		throw std::invalid_argument("Autothrottle Error: Timestep dt must be strictly positive.");
	}
}

void Autothrottle::update_gains(double P, double I, double D) {
	KP_ = P;
	KI_ = I;
	KD_ = D;
}

double Autothrottle::compute_control(const double V, const double V_target, const double max_accel, const double trim_throttle, const bool reheat) {
	if (V_last_ < 0) {
		// This would only be true if V_last_ == -1, which would only occur if the autothrottle had just been declared
		V_last_ = V;
	}

	double V_cmd = update_velocity_command(V_last_, V_target, max_accel);
	V_last_ = V_cmd; // Store for next time step
	
	const double error = V_cmd - V;

	// Integral error with anti-windup logic
	const double max_i_auth = 0.2;
	integral_error_ += error * dt_;
	integral_error_ = std::max(-max_i_auth, std::min(max_i_auth, integral_error_));
	const double derivative_error = (error - last_error_) / dt_;

	double throttle_cmd = trim_throttle + (KP_ * error) + (KI_ * integral_error_) + (KD_ * derivative_error);
	last_error_ = error;

	const double upper_limit = reheat ? max_throttle_ : max_dry_throttle_;

	throttle_cmd = std::max(min_throttle_, std::min(upper_limit, throttle_cmd));
	return throttle_cmd;
}

double Autothrottle::update_velocity_command(double V_cmd_last, double V_target, double max_accel) const {
	// Set the target velocity profile subject to the desired maximum acceleration
	double delta_V = V_target - V_cmd_last;
	double max_delta_V = max_accel * dt_;

	if (std::abs(delta_V) > max_delta_V) {
		return V_cmd_last + std::copysign(max_delta_V, delta_V);
	}
	else {
		return V_target;
	}
}

void Autothrottle::reset() {
	integral_error_ = 0;
	V_last_ = -1;
	last_error_ = 0;
}
