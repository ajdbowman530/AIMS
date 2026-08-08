#pragma once

#include "scheduler.h"
#include "aero_controller.h"
#include "autothrottle.h"

struct ExecutiveConfig {
	double aero_dt;
	double throttle_dt;
	Eigen::VectorXi lat_int_map;
	Eigen::VectorXi lon_int_map;
	double at_kp, at_ki, at_kd;
	double max_dry_throttle;
};

class FlightExecutive {
public:
    // Pass the config struct and a reference to an already-loaded data deck
    FlightExecutive(const ExecutiveConfig& config);

    Eigen::VectorXd run_control_cycle(const double sim_time,
        const GainScheduler::LookupCondition current_cond,
        const Eigen::VectorXd x,
        const Eigen::VectorXd x_cmd,
        const double max_accel,
        const bool reheat,
        const bool output_filter = true,
        const double output_alpha = 1.0);

    // Provide a clean public gateway so telemetry.cpp can pass the massive JSON data tables
    GainScheduler& get_scheduler() { return scheduler_; }

    void set_thresholds(const double M_threshold, const double q_threshold, const double W_threshold);

    Eigen::VectorXd filter_control(const double alpha, const Eigen::VectorXd u_cmd);

    void reset();

private:
    double aero_dt_;
    double throttle_dt_;
    
    double last_throttle_time_ = -1.0;
    double last_aero_time_ = -1.0;

    bool is_initalized_ = false;
    double last_tracked_M_ = 0.0;
    double last_tracked_q_ = 0.0;
    double last_tracked_W_ = 0.0;

    double M_threshold_ = 0.01;
    double q_threshold_ = 0.01;
    double W_threshold_ = 0.05;

    Eigen::VectorXd last_control_;
    Eigen::VectorXd last_u_trim_;
    GainScheduler::GainSet K_;

    size_t n_state_lat_;
    size_t n_state_lon_;
    size_t n_control_lat_;
    size_t n_control_lon_;

    // Core Member Objects
    GainScheduler scheduler_; // Has a default constructor, handles itself
    AeroController lat_controller_;
    AeroController lon_controller_;
    Autothrottle autothrottle_;
};