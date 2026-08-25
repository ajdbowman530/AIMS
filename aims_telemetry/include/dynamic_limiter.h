#pragma once

#include <numbers>

class PhiQDynamicLimiter {
private:
	// Hard saftey limits incase commands outside of safe aerodynamic and structural ranges are made
	double hard_N_z_max = 9.0;  // Gs
	double hard_N_z_min = -3.0; // Gs
	double hard_alpha_max = 20 * (std::numbers::pi/180); // rads (20 deg)
	double hard_alpha_min = -10 * (std::numbers::pi / 180); // rads (-10 deg)
	double hard_q_limit = 45 * (std::numbers::pi / 180); // rad/s (45 deg/s)

	double k_alpha = 2.0;

public:
	// Commanded limits
	double q_min = -hard_q_limit; // Placeholder for initalization or if limits are not updated
	double q_max = hard_q_limit;

	PhiQDynamicLimiter() = default;
	
	void set_hard_limits(double N_z_min, double N_z_max, double alpha_min, double alpha_max) {
		hard_N_z_min = N_z_min;
		hard_N_z_max = N_z_max;
		hard_alpha_min = alpha_min;
		hard_alpha_max = alpha_max;
	}

	void set_k_alpha(double k) {
		k_alpha = k;
	}

	void update_limits(double V_kts, double alpha_rad, double N_z_cmd, double alpha_max = 0.349, double alpha_min = -0.175); // Sets q_min and q_max
};

class ThetaPDynamicLimiter {
private:
	// I need to figure out which variables are relevant in a roll-rate dynamic limiter
	double hard_p_max  = 4.36;	// Rad
	double hard_p_min = -4.36;	// Rad
	double q_Pa_ref;

public:
	double p_min;
	double p_max;

	ThetaPDynamicLimiter() = default;

	void set_hard_limits(double p_min, double p_max) {
		hard_p_min = p_min;
		hard_p_max = p_max;
	}

	void set_q_ref(double q_Pa) {
		q_Pa_ref = q_Pa;
	}

	void update_limits(double q_Pa, double p_min = -4.36, double p_max = 4.36); // Sets p_min and p_max
};