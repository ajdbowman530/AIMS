# include "../include/scheduler.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <array>
#include <queue>
#include <string>

namespace py = pybind11;

// Set the data which the gain scheduler uses to get the gains. Should be called before get_gains.
void GainScheduler::set_data(const std::vector<double>& Machs,
	const std::vector<double>& dynamic_pressures,
	const std::vector<double>& weights,
	const std::vector<double>& flap_positions,
	const std::vector<int>& gear_positions,
	const std::vector<Eigen::MatrixXd>& lat_gains,
	const std::vector<Eigen::MatrixXd>& lon_gains,
	const std::vector<Eigen::VectorXd>& trim_controls,
	const std::vector<Eigen::VectorXd>& lat_trim_states,
	const std::vector<Eigen::VectorXd>& lon_trim_states,
	const double max_M,
	const double max_dynamic_pressure,
	const double max_weight) {

	max_Mach_ = max_M;
	max_q_ = max_dynamic_pressure;
	max_W_ = max_weight;

	stored_M_ = Machs;
	stored_q_ = dynamic_pressures;
	stored_W_ = weights;

	// Normalize coordinates
	//std::cout << "[C++] Data copied to local arrays. Normalizing..." << std::endl;
	for (size_t i = 0; i < stored_M_.size(); ++i) {
		stored_M_[i] /= max_Mach_;
		stored_q_[i] /= max_q_;
		stored_W_[i] /= max_W_;
	}

	stored_flap_pos_ = flap_positions;
	stored_gear_pos_ = gear_positions;
	stored_lat_gains_ = lat_gains;
	stored_lon_gains_ = lon_gains;
	stored_trim_controls_ = trim_controls;
	stored_lat_trim_states_ = lat_trim_states;
	stored_lon_trim_states_ = lon_trim_states;

	// Make sure that all vectors are of the same size.
	//std::cout << "[C++] set_data entered." << std::endl;
	//std::cout << "[C++] Machs size: " << Machs.size() << std::endl;
	//std::cout << "[C++] dynamic_pressures size: " << dynamic_pressures.size() << std::endl;
	//std::cout << "[C++] weights size: " << weights.size() << std::endl;
	//std::cout << "[C++] flap_positions size: " << flap_positions.size() << std::endl;
	//std::cout << "[C++] gear_positions size: " << gear_positions.size() << std::endl;
	//std::cout << "[C++] lat_gains size: " << lat_gains.size() << std::endl;
	//std::cout << "[C++] lon_gains size: " << lon_gains.size() << std::endl;
	//std::flush(std::cout);

	if (Machs.size() != dynamic_pressures.size() || Machs.size() != weights.size() ||
		Machs.size() != flap_positions.size() || Machs.size() != gear_positions.size() ||
		Machs.size() != lat_gains.size() || Machs.size() != lon_gains.size()) {
		throw std::runtime_error("GainScheduler Initalization Error: All input vectors must have the same size.");
	}

	// Make sure that the gear position is between 0 and 1
	bool is_gear_valid = std::all_of(stored_gear_pos_.begin(), stored_gear_pos_.end(), [](int pos) {
		return pos >= 0 && pos <= 1;
		});

	if (!is_gear_valid) {
		throw std::runtime_error("GainScheduler Initalization Error: Gear position must be between 0 (retracted) and 1 (deployed).");
	}

	// Clear out old trees if there are any
	trees.flap00_gear0.clear(); trees.flap01_gear0.clear(); trees.flap02_gear0.clear();
	trees.flap03_gear0.clear(); trees.flap05_gear0.clear(); trees.flap075_gear0.clear(); trees.flap10_gear0.clear();
	trees.flap00_gear1.clear(); trees.flap01_gear1.clear(); trees.flap02_gear1.clear();
	trees.flap03_gear1.clear(); trees.flap05_gear1.clear(); trees.flap075_gear1.clear(); trees.flap10_gear1.clear();

	// Grouping vectors
	std::vector<std::vector<AirframePoint>> gear0_buckets(7); // 0: 0.0, 1: 0.1, 2: 0.2, 3: 0.3, 4: 0.5, 5: 0.75, 6: 1.0
	std::vector<std::vector<AirframePoint>> gear1_buckets(7);

	auto get_flap_bucket = [](double flap) -> size_t {
		if (flap >= 0.0 && flap < 0.05) return 0;
		else if (flap >= 0.05 && flap < 0.15) return 1;
		else if (flap >= 0.15 && flap < 0.25) return 2;
		else if (flap >= 0.25 && flap < 0.40) return 3;
		else if (flap >= 0.40 && flap < 0.625) return 4;
		else if (flap >= 0.625 && flap < 0.875) return 5;
		else if (flap >= 0.875 && flap <= 1.0) return 6;
		else throw std::runtime_error("GainScheduler Error: Invalid flap position. Ensure that flap positions are normalized (0 - 1).");
		};

	//std::cout << "[C++] Normalization complete. Building AirframePoints..." << std::endl;
	for (size_t i = 0; i < stored_M_.size(); ++i) {
		AirframePoint pt;
		pt.id = static_cast<int>(i);
			
		pt.coords(0) = stored_M_[i];
		pt.coords(1) = stored_q_[i];
		pt.coords(2) = stored_W_[i];

		int f_idx = get_flap_bucket(stored_flap_pos_[i]);
		if (stored_gear_pos_[i] == 0) {
			gear0_buckets[f_idx].push_back(pt);
		}
		else {
			gear1_buckets[f_idx].push_back(pt);
		}
	}

	//std::cout << "[C++] AirframePoints built. Beginning KD-Tree Generation..." << std::endl;
	build_tree_recursive(trees.flap00_gear0, gear0_buckets[0], 0);
	build_tree_recursive(trees.flap01_gear0, gear0_buckets[1], 0);
	build_tree_recursive(trees.flap02_gear0, gear0_buckets[2], 0);
	build_tree_recursive(trees.flap03_gear0, gear0_buckets[3], 0);
	build_tree_recursive(trees.flap05_gear0, gear0_buckets[4], 0);
	build_tree_recursive(trees.flap075_gear0, gear0_buckets[5], 0);
	build_tree_recursive(trees.flap10_gear0, gear0_buckets[6], 0);

	build_tree_recursive(trees.flap00_gear1, gear1_buckets[0], 0);
	build_tree_recursive(trees.flap01_gear1, gear1_buckets[1], 0);
	build_tree_recursive(trees.flap02_gear1, gear1_buckets[2], 0);
	build_tree_recursive(trees.flap03_gear1, gear1_buckets[3], 0);
	build_tree_recursive(trees.flap05_gear1, gear1_buckets[4], 0);
	build_tree_recursive(trees.flap075_gear1, gear1_buckets[5], 0);
	build_tree_recursive(trees.flap10_gear1, gear1_buckets[6], 0);

	if (stored_lat_gains_.size() != trim_controls.size() ||
		stored_lat_gains_.size() != lat_trim_states.size() ||
		stored_lat_gains_.size() != lon_trim_states.size()) {
		throw std::runtime_error("GainScheduler Initialization Error: Sizing mismatch detected between gain matrices and trim lookup vectors.");
	}

	//std::cout << "[C++] KD-Trees successfully generated!" << std::endl;
}

// Interpolate between linearization points to get the longitudinal and lateral gains for a given flight condition. Should be called after set_data.
GainScheduler::GainSet GainScheduler::get_gains(GainScheduler::LookupCondition flight_cond) {

	if (flight_cond[0] <= 0.0 || flight_cond[1] <= 0.0) {
		std::cout << "  [WARNING] M or qbar is zero/negative! Initialization transient?" << std::endl;
	}

	if (flight_cond.size() != 5) {
		throw::std::runtime_error("GainScheduler Error: Flight condition length != 5. Please ensure the flight condition is a vector containing [Mach, dynamic pressure, weight, flap pos. (normalized), gear position (0/1)].");
	}

	const double Mach = flight_cond[0];
	const double dynamic_pressure = flight_cond[1];
	const double weight = flight_cond[2];
	const double flap_pos = flight_cond[3];
	const double gear_pos = flight_cond[4];

	GainSet set;
	if (stored_M_.empty()) {
		throw std::runtime_error("GainScheduler Error: Data not set. Please call set_data before get_gains.");
	}

	//Check to make sure that there are at least four points to interpolate between
	if (stored_M_.size() < 4) {
		throw::std::runtime_error("GainScheduler Error: Not enough data points to interpolate. At least 4 points are required");
	}

	if (!trees.flap00_gear0.empty()) {
		active_tree_ = &trees.flap00_gear0;
	}
	else {
		// High-priority system error handling
		throw std::runtime_error("GainScheduler Critical Error: Trim tree 'flap00_gear0' is empty or uninitialized!");
	}

	auto get_flap_bucket = [](double flap) -> size_t {
		if (flap >= 0.0 && flap < 0.05) return 0;
		else if (flap >= 0.05 && flap < 0.15) return 1;
		else if (flap >= 0.15 && flap < 0.25) return 2;
		else if (flap >= 0.25 && flap < 0.40) return 3;
		else if (flap >= 0.40 && flap < 0.625) return 4;
		else if (flap >= 0.625 && flap < 0.875) return 5;
		else if (flap >= 0.875 && flap <= 1.0) return 6;
		else throw std::runtime_error("GainScheduler Error: Invalid flap position. Ensure that flap positions are normalized (0 - 1).");
		};
	size_t f_idx = get_flap_bucket(flap_pos);

	// Assign active tree
	if (gear_pos == 0) {
		// If gear == 0 && f_idx == 0, no need to change anything. 
		if (f_idx == 1) {
			if (!trees.flap01_gear0.empty()) {
				active_tree_ = &trees.flap01_gear0;
			}
			else {
				std::cerr << "[WARNING]: Requested flap01_gear0 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 2) {
			if (!trees.flap02_gear0.empty()) {
				active_tree_ = &trees.flap02_gear0;
			}
			else {
				std::cerr << "[WARNING]: Requested flap02_gear0 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 3) {
			if (!trees.flap03_gear0.empty()) {
				active_tree_ = &trees.flap03_gear0;
			}
			else {
				std::cerr << "[WARNING]: Requested flap03_gear0 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 4) {
			if (!trees.flap05_gear0.empty()) {
				active_tree_ = &trees.flap05_gear0;
			}
			else {
				std::cerr << "[WARNING]: Requested flap05_gear0 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 5) {
			if (!trees.flap075_gear0.empty()) {
				active_tree_ = &trees.flap075_gear0;
			}
			else {
				std::cerr << "[WARNING]: Requested flap075_gear0 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 6) {
			if (!trees.flap10_gear0.empty()) {
				active_tree_ = &trees.flap10_gear0;
			}
			else {
				std::cerr << "[WARNING]: Requested flap10_gear0 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
	}
	else {
		if (f_idx == 0) {
			if (!trees.flap00_gear1.empty()) {
				active_tree_ = &trees.flap00_gear1;
			}
			else {
				std::cerr << "[WARNING]: Requested flap00_gear1 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 1) {
			if (!trees.flap01_gear1.empty()) {
				active_tree_ = &trees.flap01_gear1;
			}
			else {
				std::cerr << "[WARNING]: Requested flap01_gear1 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 2) {
			if (!trees.flap02_gear1.empty()) {
				active_tree_ = &trees.flap02_gear1;
			}
			else {
				std::cerr << "[WARNING]: Requested flap02_gear1 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 3) {
			if (!trees.flap03_gear1.empty()) {
				active_tree_ = &trees.flap03_gear1;
			}
			else {
				std::cerr << "[WARNING]: Requested flap03_gear1 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 4) {
			if (!trees.flap05_gear1.empty()) {
				active_tree_ = &trees.flap05_gear1;
			}
			else {
				std::cerr << "[WARNING]: Requested flap05_gear1 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 5) {
			if (!trees.flap075_gear1.empty()) {
				active_tree_ = &trees.flap075_gear1;
			}
			else {
				std::cerr << "[WARNING]: Requested flap075_gear1 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
		else if (f_idx == 6) {
			if (!trees.flap10_gear1.empty()) {
				active_tree_ = &trees.flap10_gear1;
			}
			else {
				std::cerr << "[WARNING]: Requested flap10_gear1 tree is missing. Falling back to cruise envelope baseline." << std::endl;
				if (!trees.flap00_gear0.empty()) {
					active_tree_ = &trees.flap00_gear0;
				}
				else {
					throw std::runtime_error("GainScheduler Catastrophic Failure: Complete loss of all fallback gain trees.");
				}
			}
		}
	}

	// Pick which tree to use
	const FlightCondition current_flight_state(Mach, dynamic_pressure, weight);

	const FlightCondition normalized_condition = normalizeCondition(current_flight_state);

	//std::cout << "  Normalized Search Pt  -> [" << normalized_condition[0] << ", " << normalized_condition[1] << ", " << normalized_condition[2] << "]" << std::endl;

	SearchQueue pq;
	const size_t requested_neighbors = 8;

	// From index 0 find nearest 8 points
	search_knn_iterative(normalized_condition, requested_neighbors, pq);

	std::vector<AirframePoint> candidates;
	while (!pq.empty()) {
		candidates.push_back(pq.top().second);
		pq.pop();
	}

	// Reverse list so that the nearest point is first
	std::reverse(candidates.begin(), candidates.end());

	// Find a tetrahedron which encloses the operating point
	//EnclosingCell cell = find_enclosing_tetrahedron(candidates, normalized_condition, *this);
	const EnclosingCell cell = find_enclosing_tetrahedron(candidates, normalized_condition, *this);

	if (cell.success) {
		// Point is within a cell. Standard interpolaiton
		last_cell_ = cell;
		/*std::cout << "  [SUCCESS] Enclosing cell established successfully with point IDs: "
			<< cell.point_ids[0] << ", " << cell.point_ids[1] << ", "
			<< cell.point_ids[2] << ", " << cell.point_ids[3] << std::endl;*/

		set.K_lon = cell.weights(0) * stored_lon_gains_[cell.point_ids[0]] +
			cell.weights(1) * stored_lon_gains_[cell.point_ids[1]] +
			cell.weights(2) * stored_lon_gains_[cell.point_ids[2]] +
			cell.weights(3) * stored_lon_gains_[cell.point_ids[3]];

		set.K_lat = cell.weights(0) * stored_lat_gains_[cell.point_ids[0]] +
			cell.weights(1) * stored_lat_gains_[cell.point_ids[1]] +
			cell.weights(2) * stored_lat_gains_[cell.point_ids[2]] +
			cell.weights(3) * stored_lat_gains_[cell.point_ids[3]];
	}
	else {
		// Fallback
		const int closest_id = candidates[0].id;
		set.K_lon = stored_lon_gains_[closest_id];
		set.K_lat = stored_lat_gains_[closest_id];

		/*std::cout << "  [CRITICAL FAILURE] KD-Tree or bounding simplex math could not find an enclosing cell!" << std::endl;
		std::cout << "  This means the normalized flight state is outside the convex hull of your envelope database." << std::endl;*/
	}

	return set;
}

GainScheduler::FlightCondition GainScheduler::normalizeCondition(const FlightCondition& raw) const {
	FlightCondition normalized;
	// Normalize flight condition for the k-d search
	normalized(0) = (raw(0) - 0.0) / (max_Mach_ - 0.0);
	normalized(1) = (raw(1) - 0.0) / (max_q_ - 0.0);
	normalized(2) = (raw(2) - 0.0) / (max_W_ - 0.0);
	return normalized;
}

size_t GainScheduler::build_tree_recursive(std::vector<KDNode>& tree, std::vector<AirframePoint>& points, int depth) {
	if (points.empty()) return static_cast<size_t>(-1);

	const int axis = depth % 3;

	std::sort(points.begin(), points.end(), [axis](const AirframePoint& a, const AirframePoint& b) {
		return a.coords(axis) < b.coords(axis);
		});

	const size_t median_idx = points.size() / 2;

	KDNode node;
	node.point = points[median_idx];
	node.axis = axis;

	std::vector<AirframePoint> left_points(points.begin(), points.begin() + median_idx);
	std::vector<AirframePoint> right_points(points.begin() + median_idx + 1, points.end());

	const size_t current_node_idx = tree.size();
	tree.push_back(node);

	const size_t left_child = build_tree_recursive(tree, left_points, depth + 1);
	const size_t right_child = build_tree_recursive(tree, right_points, depth + 1);

	tree[current_node_idx].leftChild = left_child;
	tree[current_node_idx].rightChild = right_child;

	return current_node_idx;
}

void GainScheduler::search_knn_iterative(const FlightCondition& target, const size_t k, SearchQueue& pq) const {
	if (active_tree_ == nullptr || active_tree_->empty()) return;
	const std::vector<KDNode>& tree = *active_tree_;
	const size_t invalid = static_cast<size_t>(-1);

	std::vector<size_t> work_stack;
	work_stack.push_back(0);

	while (!work_stack.empty()) {
		size_t idx = work_stack.back();
		work_stack.pop_back();
		if (idx == invalid) continue;

		const KDNode& node = tree[idx];
		double d_mach = target[0] - node.point.coords[0];
		double d_qbar = target[1] - node.point.coords[1];
		double d_weight = target[2] - node.point.coords[2];
		double dist = std::sqrt(d_mach * d_mach + d_qbar * d_qbar + d_weight * d_weight);

		if (pq.size() < k) {
			pq.emplace(dist, node.point);
		}
		else if (dist < pq.top().first) {
			pq.pop();
			pq.emplace(dist, node.point);
		}

		int axis = node.axis;
		double plane_delta = target[axis] - node.point.coords[axis];
		size_t primary = (plane_delta < 0) ? node.leftChild : node.rightChild;
		size_t secondary = (plane_delta < 0) ? node.rightChild : node.leftChild;

		if (secondary != invalid &&
			(pq.size() < k || std::abs(plane_delta) < pq.top().first)) {
			work_stack.push_back(secondary);
		}
		if (primary != invalid) {
			work_stack.push_back(primary);
		}
	}
}

GainScheduler::EnclosingCell GainScheduler::find_enclosing_tetrahedron(
	const std::vector<AirframePoint>& nearest_neighbors,
	const Eigen::Vector3d& current_condition,
	const GainScheduler& scheduler) const {
		
	EnclosingCell cell;

	// Combination search 
	const int num_candidates = static_cast<int>(nearest_neighbors.size());
	if (num_candidates < 4) return cell;

	for (int i = 0; i < num_candidates - 3; ++i) {
		for (int j = i + 1; j < num_candidates - 2; ++j) {
			for (int k = j + 1; k < num_candidates - 1; ++k) {
				for (int l = k + 1; l < num_candidates; ++l) {

					const int id0 = nearest_neighbors[i].id;
					const int id1 = nearest_neighbors[j].id;
					const int id2 = nearest_neighbors[k].id;
					const int id3 = nearest_neighbors[l].id;

					const Eigen::Vector4d lambdas = scheduler.get_barycentric_coords(id0, id1, id2, id3, current_condition(0), current_condition(1), current_condition(2));

					bool is_inside = (lambdas(0) >= 0.0 && lambdas(1) >= 0.0 &&
						lambdas(2) >= 0.0 && lambdas(3) >= 0.0);

					if (is_inside) {
						// If a combination of points is found to enclose the point, return the cell composed of these points
						cell.point_ids = { id0, id1, id2, id3 };
						cell.weights = lambdas;
						cell.success = true;
						return cell;
					}
				}
			}
		}
	}

	return cell;

}

Eigen::Vector4d GainScheduler::get_barycentric_coords(int id0, int id1, int id2, int id3, double M, double q, double W) const {
	// Perform 3D Barycentric interpolation using four points which enclose the flight condition.
	// Returns the weights for each of the four points, which can then be used to interpolate gains

	// Ensure that id0 is the point nearest to the operation condition for dealing with fallbacks

	// Check to make sure the indices are valid
	const size_t n = stored_M_.size();
	/*if (id0 >= n || id1 >= n || id2 >= n || id3 >= n) {
		throw std::out_of_range("GainScheduler Error: Index out of bounds.");
	}*/

	const double M0 = stored_M_[id0];
	const double M1 = stored_M_[id1];
	const double M2 = stored_M_[id2];
	const double M3 = stored_M_[id3];

	const double q0 = stored_q_[id0];
	const double q1 = stored_q_[id1];
	const double q2 = stored_q_[id2];
	const double q3 = stored_q_[id3];

	const double W0 = stored_W_[id0];
	const double W1 = stored_W_[id1];
	const double W2 = stored_W_[id2];
	const double W3 = stored_W_[id3];

	// The four points in the 3D space of (M, q, W)
	const Eigen::Vector3d r0(M0, q0, W0);
	const Eigen::Vector3d r1(M1, q1, W1);
	const Eigen::Vector3d r2(M2, q2, W2);
	const Eigen::Vector3d r3(M3, q3, W3);

	// Operating point
	const Eigen::Vector3d r_current(M, q, W);

	// Transformation matrix
	Eigen::Matrix3d T;	
	T.col(0) = r0 - r3;
	T.col(1) = r1 - r3;
	T.col(2) = r2 - r3;

	const Eigen::FullPivLU<Eigen::Matrix3d> lu(T);
	if (!lu.isInvertible()) {
		// Fallback for uninvertable transformation matrix

		// Make sure that point 0 is the nearest point to the current condition
		return Eigen::Vector4d(1, 0, 0, 0); // Fallback to first point
	}

	// Solve for lambda 0 through 2
	const Eigen::Vector3d lambda012 = lu.solve(r_current - r3);

	// Find coordinates
	const double lambda0 = lambda012(0);
	const double lambda1 = lambda012(1);
	const double lambda2 = lambda012(2);
	const double lambda3 = 1 - lambda0 - lambda1 - lambda2; // Determine lambda3 from the other coordinates

	//std::cout << "[C++] Calculated lambdas: "
	//	<< lambda0 << ", " << lambda1 << ", " << lambda2 << ", " << lambda3
	//	<< std::endl;

	// Check to make sure that the coordinates are within the tetrahedron
	const double eps = 1e-4;
	//const double eps = std::numeric_limits<double>::epsilon();
	if (lambda0 < -eps || lambda1 < -eps || lambda2 < -eps || lambda3 < -eps) {
		// Inverse distance weighting fallback if outside a valid tetrahedron
		// Calculate distances using the local coordinate vectors already present
		const double d0 = (r0 - r_current).norm();
		const double d1 = (r1 - r_current).norm();
		const double d2 = (r2 - r_current).norm();
		const double d3 = (r3 - r_current).norm();

		// Avoid division by zero
		const double w0 = 1.0 / (d0 + 1e-6);
		const double w1 = 1.0 / (d1 + 1e-6);
		const double w2 = 1.0 / (d2 + 1e-6);
		const double w3 = 1.0 / (d3 + 1e-6);
		const double total_w = w0 + w1 + w2 + w3;

		return Eigen::Vector4d(w0 / total_w, w1 / total_w, w2 / total_w, w3 / total_w);
	}

	return Eigen::Vector4d(lambda0, lambda1, lambda2, lambda3);
}

Eigen::VectorXd GainScheduler::get_trim_control() const {

	/*if (stored_trim_controls_.empty()) {
		throw std::runtime_error("GainScheduler Error: Trim data not set. Please call set_data before get_trim_control().");
	}

	if (last_cell_.success == false) {
		throw std::runtime_error("GainScheduler Error: Enclosing cell not created or not valid. Please call set_data and get_gains before get_trim_control().");
	}*/

	size_t base_size = stored_trim_controls_[0].size();
	for (int i = 0; i < 4; ++i) {
		int id = last_cell_.point_ids[i];
		/*if (id < 0 || id >= static_cast<int>(stored_trim_controls_.size())) {
			throw std::runtime_error("GainScheduler Critical Error: point_id [" + std::to_string(id) + "] is out of bounds for stored_trim_controls (size " + std::to_string(stored_trim_controls_.size()) + ")!");
		}
		if (static_cast<size_t>(stored_trim_controls_[id].size()) != base_size) {
			throw std::runtime_error("GainScheduler Critical Error: Vector size mismatch at point_id " + std::to_string(id) + "!");
		}*/
	}

	Eigen::VectorXd trim_control = last_cell_.weights[0] * stored_trim_controls_[last_cell_.point_ids[0]] +
		last_cell_.weights[1] * stored_trim_controls_[last_cell_.point_ids[1]] +
		last_cell_.weights[2] * stored_trim_controls_[last_cell_.point_ids[2]] +
		last_cell_.weights[3] * stored_trim_controls_[last_cell_.point_ids[3]];

	return trim_control;
}

Eigen::VectorXd GainScheduler::get_lon_trim_state() const {

	/*if (stored_lon_trim_states_.empty()) {
		throw std::runtime_error("GainScheduler Error: Trim data not set. Please call set_data before get_lon_trim_control().");
	}

	if (last_cell_.success == false) {
		throw std::runtime_error("GainScheduler Error: Enclosing cell not created or not valid. Please call set_data and get_gains before get_lon_trim_control().");
	}*/

	Eigen::VectorXd lon_trim_state = last_cell_.weights[0] * stored_lon_trim_states_[last_cell_.point_ids[0]] +
		last_cell_.weights[1] * stored_lon_trim_states_[last_cell_.point_ids[1]] +
		last_cell_.weights[2] * stored_lon_trim_states_[last_cell_.point_ids[2]] +
		last_cell_.weights[3] * stored_lon_trim_states_[last_cell_.point_ids[3]];
	return lon_trim_state;
}

Eigen::VectorXd GainScheduler::get_lat_trim_state() const {

	/*if (stored_lat_trim_states_.empty()) {
		throw std::runtime_error("GainScheduler Error: Trim data not set. Please call set_data before get_lon_trim_control().");
	}

	if (last_cell_.success == false) {
		throw std::runtime_error("GainScheduler Error: Enclosing cell not created or not valid. Please call set_data and get_gains before get_lat_trim_control().");
	}*/

	Eigen::VectorXd lat_trim_state = last_cell_.weights[0] * stored_lat_trim_states_[last_cell_.point_ids[0]] +
		last_cell_.weights[1] * stored_lat_trim_states_[last_cell_.point_ids[1]] +
		last_cell_.weights[2] * stored_lat_trim_states_[last_cell_.point_ids[2]] +
		last_cell_.weights[3] * stored_lat_trim_states_[last_cell_.point_ids[3]];
	return lat_trim_state;
}

bool GainScheduler::update_cell_weights(double M, double q, double W) {
	// If we don't have a valid cell do a full search
	if (!last_cell_.success) {
		return false;
	}

	Eigen::Vector4d new_weights = get_barycentric_coords(
		last_cell_.point_ids[0],
		last_cell_.point_ids[1],
		last_cell_.point_ids[2],
		last_cell_.point_ids[3],
		M, q, W
	);

	// Check if we are still inside the tetrahedron
	const double tolerance = -1e-5;
	if (new_weights[0] >= tolerance && new_weights[1] >= tolerance &&
		new_weights[2] >= tolerance && new_weights[3] >= tolerance) {		
		last_cell_.weights = new_weights;
	}

	// If any weight is less than zero we are outside the tetrahedron
	return false;
}

GainScheduler::GainSet GainScheduler::get_gains_from_current_cell() const {
	/*if (!last_cell_.success) {
		throw std::runtime_error("GainScheduler Error: Cannot extract gains from an invalid cell state.");
	}*/

	GainSet interpolated_gains;

	interpolated_gains.K_lat =
		last_cell_.weights[0] * stored_lat_gains_[last_cell_.point_ids[0]] +
		last_cell_.weights[1] * stored_lat_gains_[last_cell_.point_ids[1]] +
		last_cell_.weights[2] * stored_lat_gains_[last_cell_.point_ids[2]] +
		last_cell_.weights[3] * stored_lat_gains_[last_cell_.point_ids[3]];

	interpolated_gains.K_lon =
		last_cell_.weights[0] * stored_lon_gains_[last_cell_.point_ids[0]] +
		last_cell_.weights[1] * stored_lon_gains_[last_cell_.point_ids[1]] +
		last_cell_.weights[2] * stored_lon_gains_[last_cell_.point_ids[2]] +
		last_cell_.weights[3] * stored_lon_gains_[last_cell_.point_ids[3]];

	return interpolated_gains;
}