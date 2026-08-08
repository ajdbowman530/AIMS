#include <algorithm>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "../include/autothrottle.h"
#include "../include/scheduler.h"
#include "../include/flight_executive.h"

FlightExecutive::FlightExecutive(const ExecutiveConfig& config)
	: aero_dt_(config.aero_dt),
	throttle_dt_(config.throttle_dt),
	lat_controller_(config.aero_dt, config.lat_int_map),
	lon_controller_(config.aero_dt, config.lon_int_map),
	autothrottle_(config.at_kp, config.at_ki, config.at_kd, config.throttle_dt, config.max_dry_throttle)
{
	if (aero_dt_ <= 0 || throttle_dt_ <= 0) {
		throw std::invalid_argument("Flight Executive Error: Timestep dt must be strictly positive.");
	}
}

Eigen::VectorXd FlightExecutive::run_control_cycle(
	const double sim_time, 
	const GainScheduler::LookupCondition current_cond, 
	const Eigen::VectorXd x,
	const Eigen::VectorXd x_cmd, 
	const double max_accel, 
	const bool reheat, 
	const bool output_filter,
	const double output_alpha) {

	// x/x_cmd = [ V; [x_lat]; [x_lon] ]
	const double current_M = current_cond[0];
	const double current_q = current_cond[1];
	const double current_W = current_cond[2];

	if (!is_initalized_) {
		K_ = scheduler_.get_gains(current_cond);

		last_tracked_M_ = current_M;
		last_tracked_q_ = current_q;
		last_tracked_W_ = current_W;

		const size_t n_int_lat = lat_controller_.get_n_int();
		const size_t n_int_lon = lon_controller_.get_n_int();

		n_state_lat_ = K_.K_lat.cols() - n_int_lat;
		n_state_lon_ = K_.K_lon.cols() - n_int_lon;
		n_control_lat_ = K_.K_lat.rows();
		n_control_lon_ = K_.K_lon.rows();

		const Eigen::VectorXd u_trim = scheduler_.get_trim_control();

		// Sizing check
		Eigen::Index expected_total_size = 1 + n_control_lat_ + n_control_lon_;
		if (u_trim.size() != expected_total_size) {
			throw std::runtime_error("Catastrophic Sizing Mismatch: Trim control size does not match executive mapping.");
		}

		last_throttle_time_ = sim_time;
		last_aero_time_ = sim_time;
		last_control_ = u_trim;
		last_u_trim_ = u_trim;
		is_initalized_ = true;

		return last_control_;
	}

	Eigen::VectorXd aero_control = last_control_;

	const bool still_inside_cell = scheduler_.update_cell_weights(current_M, current_q, current_W);
	bool deadband_triggered = (std::abs(current_M - last_tracked_M_) > M_threshold_ * last_tracked_M_ ||
		std::abs(current_q - last_tracked_q_) > q_threshold_ * last_tracked_q_ ||
		std::abs(current_W - last_tracked_W_) > W_threshold_ * last_tracked_W_);
	// Multiplication is computationally cheaper than division and this allows the thresholds to be percentages
	// Not that I need to be that optimized, but still.

	if (!still_inside_cell || deadband_triggered) {
		K_ = scheduler_.get_gains(current_cond); // Isolated expensive path
		last_tracked_M_ = current_M;
		last_tracked_q_ = current_q;
		last_tracked_W_ = current_W;
	}
	else {
		K_ = scheduler_.get_gains_from_current_cell(); // Fast-path O(1) interpolation
	}

	const Eigen::VectorXd u_trim = scheduler_.get_trim_control();
	bool state_modified = false;
	
	// Autothrottle loop
	const double throttle_elapsed = sim_time - last_throttle_time_;
	if (throttle_elapsed > throttle_dt_) {
		const double V = x[0];
		const double V_cmd = x_cmd[0];
		aero_control[0] = autothrottle_.compute_control(V, V_cmd, max_accel, u_trim[0], reheat);
		last_throttle_time_ = sim_time;
		state_modified = true;
	}

	// Aerodynamic controller loop
	const double aero_elapsed = sim_time - last_aero_time_;
	if (aero_elapsed > aero_dt_) {
		// Break up x and x_cmd into lateral and longitudinal blocks
		const Eigen::VectorXd x_lon = x.segment(1, n_state_lon_);
		const Eigen::VectorXd x_lat = x.segment(1 + n_state_lon_, n_state_lat_);

		const Eigen::VectorXd x_lon_cmd = x_cmd.segment(1, n_state_lon_);
		const Eigen::VectorXd x_lat_cmd = x_cmd.segment(1 + n_state_lon_, n_state_lat_);

		//const Eigen::VectorXd u_trim = scheduler_.get_trim_control();
		const Eigen::VectorXd u_lon_trim = u_trim.segment(1, n_control_lon_);
		const Eigen::VectorXd u_lat_trim = u_trim.segment(1 + n_control_lon_, n_control_lat_);

		const Eigen::VectorXd x_lon_trim = scheduler_.get_lon_trim_state();
		const Eigen::VectorXd x_lat_trim = scheduler_.get_lat_trim_state();

		// u_trim filter:
		if (last_u_trim_.size() != u_trim.size()) {
			last_u_trim_ = u_trim;
		}
		const double u_trim_alpha = 0.1;
		last_u_trim_ = u_trim_alpha * u_trim + (1.0 - u_trim_alpha) * last_u_trim_;

		// Call controllers
		lon_controller_.update_gains(K_.K_lon);
		aero_control.segment(1, n_control_lon_) = lon_controller_.compute_control(x_lon, x_lon_cmd, u_lon_trim, x_lon_trim);

		lat_controller_.update_gains(K_.K_lat);
		aero_control.segment(1 + n_control_lon_, n_control_lat_) = lat_controller_.compute_control(x_lat, x_lat_cmd, u_lat_trim, x_lat_trim);

		last_aero_time_ = sim_time;
		state_modified = true;
		// Should return u = [delta_t, delta_e, delta_a, delta_r] or something similar.
	}

	if (output_filter) {
		// Low pass filter
		aero_control = filter_control(output_alpha, aero_control);
	}
	
	if (state_modified) {
		last_control_ = aero_control;
	}
	return last_control_;
}

void FlightExecutive::set_thresholds(const double M_threshold, const double q_threshold, const double W_threshold) {
	M_threshold_ = M_threshold;
	q_threshold_ = q_threshold;
	W_threshold_ = W_threshold;
}

Eigen::VectorXd FlightExecutive::filter_control(const double alpha, const Eigen::VectorXd u_cmd) {
	if (u_cmd.size() != last_control_.size()) {
		throw std::runtime_error("FlightExecutive Error: u_cmd and last_control_ are not of identical size.");
	}
	Eigen::VectorXd u_filtered = alpha * u_cmd + (1.0 - alpha) * last_control_;
	return u_filtered;
}

void FlightExecutive::reset() {
	lat_controller_.reset_integrators();
	lon_controller_.reset_integrators();
	autothrottle_.reset();
}