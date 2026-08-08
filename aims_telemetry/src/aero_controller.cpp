# include "../include/aero_controller.h"

#include <Eigen/Dense>
#include <vector>
#include <set>

AeroController::AeroController(double dt, Eigen::VectorXi int_map)
    : dt_(dt), int_map_(int_map), n_int_(0) {

    validate_mapping(int_map);

    // Allocate the running integrator vectors based on the verified map count
    integrator_accumulators_ = Eigen::VectorXd::Zero(n_int_);
}

void AeroController::validate_mapping(const Eigen::VectorXi& map) {
    if (map.size() == 0) {
        throw std::invalid_argument("AeroController Error: Integrator map cannot be empty.");
    }

    std::set<int> seen_indices;
    int max_val = 0;

    for (int i = 0; i < map.size(); ++i) {
        int val = map(i);
        if (val < 0) {
            throw std::invalid_argument("AeroController Error: Map contains negative index at position " + std::to_string(i));
        }
        if (val > 0) {
            // Check Safeguard: Ensure no duplicate tracking numbers exist
            if (seen_indices.count(val)) {
                throw std::invalid_argument("AeroController Error: Duplicate integrator assignment detected for index " + std::to_string(val));
            }
            seen_indices.insert(val);
            if (val > max_val) max_val = val;
        }
    }

    // Check Safeguard: Continuous tracking sequence (prevent [0, 1, 3, 0] missing a '2')
    if (max_val != static_cast<int>(seen_indices.size())) {
        throw std::invalid_argument("AeroController Error: Integrator map sequence must be sequential starting from 1 (e.g., 1, 2, 3...).");
    }

    n_int_ = seen_indices.size();
}

void AeroController::reset_integrators() {
    integrator_accumulators_.setZero();
}

Eigen::VectorXd AeroController::compute_control(
    const Eigen::VectorXd& x, 
    const Eigen::VectorXd& x_cmd, 
    const Eigen::VectorXd& control_trim, 
    const Eigen::VectorXd& x_trim) {

    // State vector size check
    if (x.size() != x_cmd.size() || x.size() != x_trim.size()) {
        throw std::runtime_error("AeroController Error: Mismatch between state, command, or trim vector sizes.");
    }

    // int_map_ size check
    if (x.size() != int_map_.size()) {
        throw std::runtime_error("AeroController Error: Incoming state size (" +
            std::to_string(x.size()) + ") does not match configured int_map size (" +
            std::to_string(int_map_.size()) + ").");
    }

    size_t total_augmented_size = x.size() + n_int_;
    if (K_.cols() != total_augmented_size) {
        throw std::runtime_error("AeroController Error: The Gain Matrix K columns (" +
            std::to_string(K_.cols()) + ") do not match augmented state size z (" +
            std::to_string(total_augmented_size) + ").");
    }

    if (control_trim.size() != K_.rows()) {
        throw std::runtime_error("AeroController Error: Trim control size (" +
            std::to_string(control_trim.size()) + ") does not match the control output size (" +
            std::to_string(K_.rows()) + ").");
    }

    const Eigen::VectorXd dx = x - x_trim;

    // Find and integrate error (Convention: Command - Actual)
    for (Eigen::Index i = 0; i < int_map_.size(); ++i) {
        int target_integrator_id = int_map_(i);
        if (target_integrator_id > 0) {
            int tracking_idx = target_integrator_id - 1;

            // Standard control law convention: e = command - plant state
            double error = x_cmd(i) - x(i);
            integrator_accumulators_(tracking_idx) += error * dt_;
        }
    }

    // z = [integrators, plant_states]
    Eigen::VectorXd z(total_augmented_size);
    z.head(n_int_) = integrator_accumulators_;
    //z.tail(x.size()) = dx;
    z.tail(x.size()) = dx;

    // Command law: u = -K * z
    Eigen::VectorXd u = control_trim - K_ * z;

    for (Eigen::Index i = 0; i < u.size(); ++i) {
        if (u(i) > max_deflection_) u(i) = max_deflection_;
        if (u(i) < -max_deflection_) u(i) = -max_deflection_;
    }

    return u;
}

size_t AeroController::get_n_int() {
    return n_int_;
}