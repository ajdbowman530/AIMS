#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <iostream>

#include "../include/scheduler.h"
#include "../include/aero_controller.h"
#include "../include/flight_executive.h"

namespace py = pybind11;

PYBIND11_MODULE(flight_control, m) {
    m.doc() = "JSBSim Flight Executive and Gain Scheduling Module";

    // 1. Bind the ExecutiveConfig Struct
    py::class_<ExecutiveConfig>(m, "ExecutiveConfig")
        .def(py::init<>())
        .def_readwrite("aero_dt", &ExecutiveConfig::aero_dt)
        .def_readwrite("throttle_dt", &ExecutiveConfig::throttle_dt)
        .def_readwrite("lat_int_map", &ExecutiveConfig::lat_int_map)
        .def_readwrite("lon_int_map", &ExecutiveConfig::lon_int_map)
        .def_readwrite("at_kp", &ExecutiveConfig::at_kp)
        .def_readwrite("at_ki", &ExecutiveConfig::at_ki)
        .def_readwrite("at_kd", &ExecutiveConfig::at_kd)
        .def_readwrite("max_dry_throttle", &ExecutiveConfig::max_dry_throttle);

    py::class_<GainScheduler>(m, "GainScheduler")
        .def("set_data", &GainScheduler::set_data,
            py::arg("Machs"),
            py::arg("dynamic_pressures"),
            py::arg("weights"),
            py::arg("flap_positions"),
            py::arg("gear_positions"),
            py::arg("lat_gains"),
            py::arg("lon_gains"),
            py::arg("trim_controls"),
            py::arg("lat_trim_states"),
            py::arg("lon_trim_states"),
            py::arg("max_M"),
            py::arg("max_dynamic_pressure"),
            py::arg("max_weight"),
            "Populates the multi-dimensional tracking tables and builds the KD-Trees");

    py::class_<FlightExecutive>(m, "FlightExecutive")
        .def(py::init<const ExecutiveConfig&>(), py::arg("config"))
        .def("run_control_cycle", &FlightExecutive::run_control_cycle,
            py::arg("sim_time"),
            py::arg("current_cond"),
            py::arg("x"),
            py::arg("x_cmd"),
            py::arg("max_accel"),
            py::arg("reheat"),
            py::arg("output_filter"),
            py::arg("output_alpha"),
            "Executes the dual-rate control loops and updates surface allocations")

        .def("get_scheduler", &FlightExecutive::get_scheduler, py::return_value_policy::reference)
        .def("set_thresholds", &FlightExecutive::set_thresholds,
            py::arg("M_threshold"),
            py::arg("q_threshold"),
            py::arg("W_threshold"),
            "Updates the macro deadbands used for cache busting")
	    .def("reset", &FlightExecutive::reset, "Resets the internal state of the Flight Executive and its controllers");
}