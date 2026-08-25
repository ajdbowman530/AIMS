#include "../include/dynamic_limiter.h"

#include <algorithm>

void PhiQDynamicLimiter::update_limits(double V_kts, double alpha_rad, double N_z_cmd, double alpha_max, double alpha_min) {
	constexpr double g = 32.174; // ft/s^2
	double V_fps = V_kts * 1.68781; // Convert kts to fps
	double V_safe = std::max(V_fps, 50.0); // Prevent division by zero or very small airspeeds

	// Apply hard limits
	//const double N_z = std::ranges::clamp(N_z_cmd, hard_N_z_min, hard_N_z_max);
	const double N_z = 2.5;
	alpha_min = std::max(alpha_min, hard_alpha_min);
	alpha_max = std::min(alpha_max, hard_alpha_max);

	// q ~= ((N_z - 1) * g) / V
	double N_z_upper = std::clamp(N_z_cmd, hard_N_z_min, hard_N_z_max);

	double q_N_z_max = ((N_z_upper - 1.0) * g) / V_safe;
	double q_N_z_min = ((hard_N_z_min - 1.0) * g) / V_safe;

	double q_alpha_min = k_alpha * (alpha_min - alpha_rad);
	double q_alpha_max = k_alpha * (alpha_max - alpha_rad);

	q_max = std::min({ q_N_z_max, q_alpha_max, hard_q_limit });
	q_min = std::max({ q_N_z_min, q_alpha_min, -hard_q_limit });
}

void ThetaPDynamicLimiter::update_limits(double q_Pa, double p_min_req, double p_max_req) {
	// Apply hard limits
	p_min_req = std::max(p_min_req, hard_p_min);
	p_max_req = std::min(p_max_req, hard_p_max);

	// Scale the roll rate limit based on dynamic pressure (lower at high q)
	double k_roll = 1.0;
	if (q_Pa > q_Pa_ref && q_Pa > 0.0) {
		k_roll = q_Pa_ref / q_Pa;
		k_roll = std::max(k_roll, 0.3); // Maintain at least 30% control authority
	}

	p_max = std::min(p_max_req, k_roll * hard_p_max);
	p_min = std::max(p_min_req, -p_max);
}