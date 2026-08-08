#pragma once

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <utility> 

class GainScheduler {
public:
	using FlightCondition = Eigen::Vector3d; // [Mach, q, W]
	using LookupCondition = Eigen::VectorXd; // [Mach, q, W, flap, gear]

	struct AirframePoint {
		int id;
		FlightCondition coords;
	};

	struct KDNode {
		AirframePoint point;
		int axis; // [0: M, 1: q, 2: W]
		size_t leftChild = static_cast<size_t>(-1);
		size_t rightChild = static_cast<size_t>(-1);
	};

	struct ConfigurationTrees {
		// Separate trees for each flap/gear combination
		std::vector<KDNode> flap00_gear0; // clean
		std::vector<KDNode>	flap01_gear0; // flap 0.1, gear up
		std::vector<KDNode>	flap02_gear0; // flap 0.2, gear up
		std::vector<KDNode>	flap03_gear0; // flap 0.3, gear up
		std::vector<KDNode>	flap05_gear0; // flap 0.5, gear up
		std::vector<KDNode>	flap075_gear0; // flap 0.75, gear up
		std::vector<KDNode>	flap10_gear0; // flap 1, gear up

		std::vector<KDNode> flap00_gear1; // clean gear down
		std::vector<KDNode>	flap01_gear1; // flap 0.1, gear down
		std::vector<KDNode>	flap02_gear1; // flap 0.2, gear down
		std::vector<KDNode>	flap03_gear1; // flap 0.3, gear down
		std::vector<KDNode>	flap05_gear1; // flap 0.5, gear down
		std::vector<KDNode>	flap075_gear1; // flap 0.75, gear down
		std::vector<KDNode>	flap10_gear1; // flap 1, gear down
	};

	struct EnclosingCell {
		Eigen::Vector4i point_ids;
		Eigen::Vector4d weights;
		bool success = false;
	};

	struct GainSet {
		Eigen::MatrixXd K_lon;
		Eigen::MatrixXd K_lat;
	};

	struct NeighborCompare {
		// Collect points sorted by largest distance first so we can easily pop the worst candidate
		bool operator()(const std::pair<double, AirframePoint>& a, const std::pair<double, AirframePoint>& b) {
			return a.first < b.first;
		}
	};
	
	// Fixed pool collector type
	using SearchQueue = std::priority_queue<std::pair<double, AirframePoint>,
		std::vector<std::pair<double, AirframePoint>>,
		NeighborCompare>;

	// Functions:

	double ping() const { return 1.0; } // Test function for debug

	std::string version() const { return "0.1.0"; }

	void set_data(const std::vector<double>& Machs,
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
		const double max_weight);

	GainSet get_gains(LookupCondition flight_cond);

	FlightCondition normalizeCondition(const FlightCondition& raw) const;

	size_t build_tree_recursive(std::vector<KDNode>& tree, std::vector<AirframePoint>& points, int depth);

	void search_knn_iterative(const FlightCondition& target, const size_t k, SearchQueue& pq) const;

	EnclosingCell find_enclosing_tetrahedron(
		const std::vector<AirframePoint>& nearest_neighbors,
		const Eigen::Vector3d& current_condition,
		const GainScheduler& scheduler) const;

	Eigen::Vector4d get_barycentric_coords(int id0, int id1, int id2, int id3, double M, double q, double W) const;

	Eigen::VectorXd get_trim_control() const;
	Eigen::VectorXd get_lon_trim_state() const;
	Eigen::VectorXd get_lat_trim_state() const;

	bool update_cell_weights(double M, double q, double W);
	GainSet get_gains_from_current_cell() const;
private:
	// Private variables

	ConfigurationTrees trees;

	std::vector<double> stored_M_;
	std::vector<double> stored_q_;
	std::vector<double> stored_W_;
	std::vector<double> stored_flap_pos_;
	std::vector<int> stored_gear_pos_;
	std::vector<Eigen::MatrixXd> stored_lat_gains_;
	std::vector<Eigen::MatrixXd> stored_lon_gains_;
	std::vector<Eigen::VectorXd> stored_trim_controls_;
	std::vector<Eigen::VectorXd> stored_lat_trim_states_;
	std::vector<Eigen::VectorXd> stored_lon_trim_states_;
	double max_Mach_;
	double max_q_;
	double max_W_;

	size_t root_node_idx_ = static_cast<size_t>(-1);
	const std::vector<KDNode>* active_tree_ = nullptr;
	EnclosingCell last_cell_;
};