#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <vector>

#include "../include/scheduler.h"

class Autothrottle {
public:
	Autothrottle(double KP, double KI, double KD, double dt, double max_dry_throttle = 0.99);

	void update_gains(double KP, double KI, double KD);
	double compute_control(const double V, const double V_target, const double max_accel, const double trim_throttle, const bool reheat = false);
	void reset();

private:

	double update_velocity_command(double V_cmd_current, double V_target, double max_accel) const;
	// V_cmd_current is the current velocity target for the next time step, kts
	// V_target is the terminal airspeed for the autothrottle, kts
	// max_accel is the maximum acceleration allowed (positive or negative), kts/s

	double dt_;

	// PID Gains
	double KP_;
	double KI_;
	double KD_;

	// State memory
	double integral_error_ = 0;
	double last_error_ = 0;
	bool is_first_step_ = true;
	double V_last_ = -1; // Should never be true,

	// Limits and configuration
	double nominal_throttle_;
	double max_dry_throttle_; // See engine XML file for the correct value (usually < 1.00, so approx 0.99 for afterburning engines)
	const double max_throttle_ = 1.0;
	const double min_throttle_ = 0.0;
	const double max_integral_clamping_ = 0.2;
};