# pragma once

#include <Eigen/Dense>
#include <vector>


class AeroController {
public:
	AeroController(double dt, Eigen::VectorXi int_map);

	void update_gains(const Eigen::MatrixXd& K) { K_ = K; }
	
	Eigen::VectorXd compute_control(const Eigen::VectorXd& x, const Eigen::VectorXd& x_cmd, const Eigen::VectorXd& controlled_trim, const Eigen::VectorXd& x_trim);
	// controller_trim is the trim control for the controls trimmed by the controler (i.e. lateral controller would be given elevator trim)

	void reset_integrators();

	size_t get_n_int();

private:
	const double dt_; // Controller time step
	const Eigen::VectorXi int_map_; // vector of length equal to len(x), 0 if state is integrated, 1 is for the first integrator, 2 for second, etc.
	Eigen::MatrixXd K_;
	Eigen::VectorXd integrator_accumulators_;

	size_t n_int_; // Dynamically calculated number of integrators found in the map

	const double max_deflection_ = 1; // Normalized by default

	void validate_mapping(const Eigen::VectorXi& map);
};