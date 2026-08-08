import numpy as np
import json
from pathlib import Path
from helper import Helper
from aims_plotter import AIMSPlotter

class LinearizationPoint:
    def __init__(self, pt_id, mach, q_bar, fuel, lon_state, lat_state, lon_input, lat_input, controller_type='LQR', 
                 x_trim_lat=None, x_trim_lon=None, u_trim=None, flap_pos=0, gear_pos=0, lon_act_order=1, lat_act_order=1):
        self.cond = {'M': mach, 'q': q_bar, 'W_fuel': fuel, 'flap': flap_pos, 'gear': gear_pos}
        self.pt_id = pt_id
        self.type = controller_type.lower() # Controller type (LQR, Hinf, etc.)

        self.lon_act_order = lon_act_order
        self.lat_act_order = lat_act_order

        self.tuning_data = {} 
        if self.type == 'lqr':
            self.lon_tuning_data = {'Q': None, 'R': None}
            self.lat_tuning_data = {'Q': None, 'R': None}
        elif self.type == 'h_inf':
            self.lon_tuning_data = {'Wp': None, 'Wu': None, 'Wt': None}
            self.lat_tuning_data = {'Wp': None, 'Wu': None, 'Wt': None}

        # State and control vectors:
        self.lon_state = lon_state # For F-15, ['u', 'w', 'q', 'theta]
        self.lat_state = lat_state # For F-15, ['v', 'p', 'r', 'phi']
        self.lon_input = lon_input # For F-15, ['delta_e']
        self.lat_input = lat_input # For F-15, ['delta_a', 'delta_r']
        # State and control vectors are stored in case someone wants to use a different set of vectors later and also the 
        # F-22 will have more control inputs since it has thrust vectoring.

        self.trim_state_lon = x_trim_lon
        self.trim_state_lat = x_trim_lat
        self.trim_control = u_trim # [throttle, elevator, aileron, rudder] for now
        # To make it more general, probably [throttle, lon_input, lat_input] and remove repeats 

        # Longitudinal Matrices
        self.A_lon = None; self.B_lon = None
        self.K_lon = None
        
        # Lateral Matrices
        self.A_lat = None; self.B_lat = None
        self.K_lat = None

        # Other conditions that are less important for gain scheduling but may be useful for analysis
        self.alt = None
        self.V = None

        # Stability metrics
        self.lon_stability_metrics = {}
        self.lat_stability_metrics = {}

        # Create trim actuator states
        if self.lat_act_order == 1:
            self.lat_act_trim = np.array((self.trim_control[2], self.trim_control[3]))
        elif self.lat_act_order == 2:
            self.lat_act_trim = np.array((self.trim_control[2], 0, self.trim_control[3], 0)) # Assume that in a trim state actuators are static
        else:
            self.lat_act_trim = None

        if self.lon_act_order == 1:
            self.lon_act_trim = np.array((self.trim_control[1]))
        elif self.lon_act_order == 2:
            self.lon_act_trim = np.array((self.trim_control[1], 0)) # Assume that in a trim state actuators are static
        else:
            self.lon_act_trim = None

        self.trim = {
            'control': u_trim, 
            'x_phys_lon': x_trim_lon[:2] if x_trim_lon is not None else None,
            'x_act_lon': self.lon_act_trim,
            'x_phys_lat': x_trim_lat[:3] if x_trim_lat is not None else None,
            'x_act_lat': self.lat_act_trim
        }

    def set_id(self, new_id):
        self.pt_id = new_id

    def set_alt_V(self, alt, V):
        self.alt = alt
        self.V = V
    
    def set_lon_matrices(self, A, B, K):
        self.A_lon = A
        self.B_lon = B
        self.K_lon = K
    
    def set_lon_lqr_data(self, Q, R):
        self.Q_lon = Q
        self.R_lon = R

    def set_lat_matrices(self, A, B, K):
        self.A_lat = A
        self.B_lat = B
        self.K_lat = K

    def set_lat_lqr_data(self, Q, R):
        self.Q_lat = Q
        self.R_lat = R

    def set_q_M(self, q_bar, mach):
        self.cond['q'] = q_bar
        self.cond['M'] = mach

    def set_fuel(self, fuel):
        self.cond['W_fuel'] = fuel

    def set_gear(self, gear_pos):
        self.cond['gear'] = gear_pos

    def set_flap(self, flap_pos):
        self.cond['flap'] = flap_pos

    def set_trim(self, x_trim_lon, x_trim_lat, u_trim):
        self.trim_state_lon = x_trim_lon
        self.trim_state_lat = x_trim_lat
        self.trim_control = u_trim
        
        if self.lat_act_order == 1:
            self.lat_act_trim = np.array((self.trim_control[2], self.trim_control[3]))
        elif self.lat_act_order == 2:
            self.lat_act_trim = np.array((self.trim_control[2], 0, self.trim_control[3], 0))
        else:
            self.lat_act_trim = None

        if self.lon_act_order == 1:
            self.lon_act_trim = np.array((self.trim_control[1]))
        elif self.lon_act_order == 2:
            self.lon_act_trim = np.array((self.trim_control[1], 0))
        else:
            self.lon_act_trim = None

        self.trim = {
            'control': u_trim,
            'x_phys_lon': x_trim_lon[:2] if x_trim_lon is not None else None,
            'x_act_lon': self.lon_act_trim,
            'x_phys_lat': x_trim_lat[:3] if x_trim_lat is not None else None,
            'x_act_lat': self.lat_act_trim
        }

    def get_K(self, axis='lon'):
        if axis == 'lon':
            return self.K_lon
        elif axis == 'lat':
            return self.K_lat
        else:
            raise ValueError("Axis must be 'lon' or 'lat'")
        
    def get_Q(self, axis='lon'):
        if axis == 'lon':
            return self.Q_lon
        elif axis == 'lat':
            return self.Q_lat
        else:
            raise ValueError("Axis must be 'lon' or 'lat'")
    
    def get_R(self, axis='lon'):
        if axis == 'lon':
            return self.R_lon
        elif axis == 'lat':
            return self.R_lat
        else:
            raise ValueError("Axis must be 'lon' or 'lat'")
        
    def get_A(self, axis='lon'):
        if axis == 'lon':
            return self.A_lon
        elif axis == 'lat':
            return self.A_lat
        else:
            raise ValueError("Axis must be 'lon' or 'lat'")
        
    def get_B(self, axis='lon'):
        if axis == 'lon':
            return self.B_lon
        elif axis == 'lat':
            return self.B_lat
        else:
            raise ValueError("Axis must be 'lon' or 'lat'")
    
    def get_q(self):
        return self.cond['q']
    
    def get_M(self):
        return self.cond['M']
    
    def get_fuel(self):
        return self.cond['W_fuel']
    
    def get_alt_V(self):
        return self.alt, self.V
    
    def get_flap(self):
        return self.cond['flap']
    
    def get_gear(self):
        return self.cond['gear']
    
    def get_trim(self):
        return self.trim_state_lon, self.trim_state_lat, self.trim_control
    
    
    def set_stability_metrics(self, axis='lon', gain_margin=None, phase_margin=None, overshoot=None, zeta=None, rise_time=None,
                              settling_time=None, eigs=None, peak_sensitivity=None, peak_cosensitivity=None):
        """Stores stability metrics for later retrieval and display."""
        if axis == 'lon':
            self.lon_stability_metrics = {
                'gain_margin': gain_margin,
                'phase_margin': phase_margin,
                'overshoot': overshoot,
                'damping': zeta,
                'rise_time': rise_time,
                'settling_time': settling_time,
                'eigs': eigs,
                'peak_sensitivity': peak_sensitivity,
                'peak_cosensitivity': peak_cosensitivity
            }
        elif axis == 'lat':
            self.lat_stability_metrics = {
                'gain_margin': gain_margin,
                'phase_margin': phase_margin,
                'overshoot': overshoot,
                'damping': zeta,
                'rise_time': rise_time,
                'settling_time': settling_time,
                'eigs': eigs,
                'peak_sensitivity': peak_sensitivity,
                'peak_cosensitivity': peak_cosensitivity
            }
        else:
            raise ValueError("Axis must be 'lon' or 'lat'")
        
    def get_stability_metrics(self, axis='lon'):
        return self.lon_stability_metrics if axis == 'lon' else self.lat_stability_metrics

    def to_dict(self):
        """Converts the point to a dictionary for JSON export."""
        def clean(val):
            if isinstance(val, np.ndarray):
                if val.ndim == 0:
                    return clean(val.item())
                return [clean(i) for i in val]

            # Dictionaries
            if isinstance(val, dict):
                return {k: clean(v) for k, v in val.items()}
            
            # Lists/Tuples
            if isinstance(val, (list, tuple)):
                return [clean(i) for i in val]
            
            # Complex Numbers
            if isinstance(val, (complex, np.complex128, np.complex64)):
                return {"real": float(val.real), "imag": float(val.imag)}
            
            # NumPy scalars
            if hasattr(val, "item") and callable(getattr(val, "item")):
                return val.item()

            return val
        
        return {
            "id": self.pt_id,
            "type": self.type,
            "conditions": {**self.cond, "alt": self.alt, "V": self.V},
            "lat act model": self.lat_act_order,
            "lon act model": self.lon_act_order,
            "trim": {
                "x_phys_lon": self.trim_state_lon.tolist() if isinstance(self.trim_state_lon, np.ndarray) else self.trim_state_lon,
                "x_act_lon": self.lon_act_trim.tolist() if isinstance(self.lon_act_trim, np.ndarray) else self.lon_act_trim,
                "x_phys_lat": self.trim_state_lat.tolist() if isinstance(self.trim_state_lat, np.ndarray) else self.trim_state_lat,
                "x_act_lat": self.lat_act_trim.tolist() if isinstance(self.lat_act_trim, np.ndarray) else self.lat_act_trim,
                "control": self.trim_control.tolist() if isinstance(self.trim_control, np.ndarray) else self.trim_control,
            },
            "vectors": {
                "lon_state": self.lon_state, "lat_state": self.lat_state,
                "lon_input": self.lon_input, "lat_input": self.lat_input
            },
            "matrices": {
                "A_lon": clean(self.A_lon), "B_lon": clean(self.B_lon), "K_lon": clean(self.K_lon), "Q_lon": clean(self.Q_lon), "R_lon": clean(self.R_lon),
                "A_lat": clean(self.A_lat), "B_lat": clean(self.B_lat), "K_lat": clean(self.K_lat), "Q_lat": clean(self.Q_lat), "R_lat": clean(self.R_lat)
            },
            "stability metrics": {
                "lon": clean(self.lon_stability_metrics), # Now recursive cleaning!
                "lat": clean(self.lat_stability_metrics)
            }
        }
    
    def print_stability_metrics(self, axis='lon'):
        pt_label = "Longitudinal" if axis == 'lon' else "Lateral"
        metrics = self.lon_stability_metrics if axis == 'lon' else self.lat_stability_metrics
        
        if axis not in ['lon', 'lat']:
            raise ValueError("Axis must be 'lon' or 'lat'")

        print(f"\n==================================================")
        print(f"--- {pt_label} Stability Metrics (ID: {self.pt_id}) ---")
        print(f"==================================================")

        for k, v in metrics.items():
            # Clean up key names
            display_key = k.replace('_', ' ').title()
            
            # Raw Eigenvalues list of dictionaries
            if k == 'eigs' and isinstance(v, list):
                print(f"  {display_key}:")
                for idx, eig in enumerate(v):
                    real_part = eig.get('real', 0.0)
                    imag_part = eig.get('imag', 0.0)
                    if abs(imag_part) < 1e-5:
                        print(f"    [{idx}] {real_part:8.4f}")
                    else:
                        sign = "+" if imag_part >= 0 else "-"
                        print(f"    [{idx}] {real_part:8.4f} {sign} {abs(imag_part):.4f}j")
                        
            # Lists of numbers (like damping ratios)
            elif isinstance(v, list):
                formatted_list = ", ".join([f"{item:.3f}" for item in v])
                print(f"  {display_key:<20}: [{formatted_list}]")
                
            # Single float numbers
            elif isinstance(v, (float, int)):
                # Add units depending on the metric type
                unit = " dB" if "gain" in k else " deg" if "phase" in k else " %" if "overshoot" in k else " sec" if "time" in k else ""
                print(f"  {display_key:<20}: {v:.2f}{unit}")
                
            # Fallback for everything else
            else:
                print(f"  {display_key:<20}: {v}")
        print(f"==================================================\n")
        
class Aircraft:
    def __init__(self, name, type, lon_actuator_config, lat_actuator_config, ref_lon_states, ref_lon_controls , 
                 ref_lat_states, ref_lat_controls, 
                 lat_int_map=np.array([1, 2, 0, 0, 0], dtype=np.int32),
                 lon_int_map=np.array([1, 2, 0, 0, 0], dtype=np.int32)):
        self.name = name
        self.type = type
        self.lon_actuator_order = lon_actuator_config['order'] # 0 for no actuator model, 1 for first order, 2 for second order
        self.lat_actuator_order = lat_actuator_config['order']
        self.ref_lon_states = ref_lon_states     # Master list to validate against
        self.ref_lon_controls = ref_lon_controls # Master list to validate against
        self.ref_lat_states = ref_lat_states
        self.ref_lat_controls = ref_lat_controls
        self.points = []

        # Default first order actuator time constants
        self.tau_e = -1
        self.tau_a = -1
        self.tau_r = -1

        # Default second order actuator parameters (natural frequency and damping ratio)
        self.omega_e = -1
        self.zeta_e = -1
        self.omega_a = -1
        self.zeta_a = -1
        self.omega_r = -1
        self.zeta_r = -1

        # Integrator maps
        self.lat_int_map = lat_int_map
        self.lon_int_map = lon_int_map

    def set_name(self, new_name):
        self.name = new_name
    
    def remove_point(self, pt_id):
        try:
            self.points = [p for p in self.points if p.pt_id != pt_id]
            return True
        except Exception as e:
            print(f"Error occurred while removing point: {e}")

    def set_o1_actuator(self, tau_e, tau_a, tau_r):
        self.tau_e = tau_e
        self.tau_a = tau_a
        self.tau_r = tau_r

    def set_o2_actuator(self, omega_e, zeta_e, omega_a, zeta_a, omega_r, zeta_r):
        self.omega_e = omega_e
        self.zeta_e = zeta_e
        self.omega_a = omega_a
        self.zeta_a = zeta_a
        self.omega_r = omega_r
        self.zeta_r = zeta_r

    def add_point(self, lp_node):
        if lp_node.lon_state != self.ref_lon_states:
            raise ValueError(f"State Vector Mismatch! Point {lp_node.pt_id} uses {lp_node.lon_state}, "
                             f"but Aircraft '{self.name}' requires {self.ref_lon_states}")
        
        if lp_node.lon_input != self.ref_lon_controls:
             raise ValueError(f"Control Vector Mismatch! Point {lp_node.pt_id} uses {lp_node.lon_input}, "
                             f"but Aircraft '{self.name}' requires {self.ref_lon_controls}")

        if lp_node.lat_state != self.ref_lat_states:
            raise ValueError(f"State Vector Mismatch! Point {lp_node.pt_id} uses {lp_node.lat_state}, "
                                f"but Aircraft '{self.name}' requires {self.ref_lat_states}")
        
        if lp_node.lat_input != self.ref_lat_controls:
                raise ValueError(f"Control Vector Mismatch! Point {lp_node.pt_id} uses {lp_node.lat_input}, "
                                f"but Aircraft '{self.name}' requires {self.ref_lat_controls}")
        
        self.points.append(lp_node)

    # @classmethod
    def export_to_json(self, filename):
        """Saves the entire aircraft and all points to a JSON file."""
        
        if not filename.endswith('.json'):
            filename += '.json'

        base_dir = Path(__file__).parent
        target_dir = base_dir / "gain schedules"

        target_dir.mkdir(parents=True, exist_ok=True)
        full_path = target_dir / filename

        lon_map_serializable = self.lon_int_map.tolist() if isinstance(self.lon_int_map, np.ndarray) else self.lon_int_map
        lat_map_serializable = self.lat_int_map.tolist() if isinstance(self.lat_int_map, np.ndarray) else self.lat_int_map

        data = {
            "name": filename,
            "type": self.type,
            "lon_actuator_order": self.lon_actuator_order,
            "lat_actuator_order": self.lat_actuator_order,
            "vectors": {
                "ref_lon_states": self.ref_lon_states,
                "ref_lon_controls": self.ref_lon_controls,
                "lon_int_map": lon_map_serializable,
                "ref_lat_states": self.ref_lat_states,
                "ref_lat_controls": self.ref_lat_controls,
                "lat_int_map": lat_map_serializable
            },
            "actuator_params": {
                "tau": [self.tau_e, self.tau_a, self.tau_r],
                "omega": [float(self.omega_e), float(self.omega_a), float(self.omega_r)],
                "zeta": [float(self.zeta_e), float(self.zeta_a), float(self.zeta_r)]
            },
            "linearization_points": [p.to_dict() for p in self.points]
        }
        with open(full_path, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Aircraft '{self.type}' exported to {full_path}")

    @classmethod
    def from_json(cls, filename):
        """Loads an entire Aircraft object and its points from a JSON file."""
        
        if not filename.endswith('.json'):
            filename += '.json'
            
        # Resolve path relative to models.py location
        base_dir = Path(__file__).parent
        full_path = base_dir / "gain schedules" / filename

        if not full_path.exists():
            raise FileNotFoundError(f"Could not find configuration file at: {full_path}")

        with open(full_path, 'r') as f:
            data = json.load(f)

        ac_type = data.get('type', 'Unknown')

        lon_act = {'order': data['lon_actuator_order']}
        lat_act = {'order': data['lat_actuator_order']}

        ref_lon_states = data['vectors']['ref_lon_states']
        ref_lon_controls = data['vectors']['ref_lon_controls']
        lon_int_map = data['vectors']['lon_int_map']
        ref_lat_states = data['vectors']['ref_lat_states']
        ref_lat_controls = data['vectors']['ref_lat_controls']
        lat_int_map = data['vectors']['lat_int_map']

        aircraft = cls(data['name'], ac_type, lon_act, lat_act,
               ref_lon_states, ref_lon_controls,
               ref_lat_states, ref_lat_controls,
               lat_int_map=np.array(lat_int_map, dtype=np.int32),
               lon_int_map=np.array(lon_int_map, dtype=np.int32))

        ap = data['actuator_params']
        aircraft.set_o1_actuator(*ap['tau'])
        aircraft.set_o2_actuator(ap['omega'][0], ap['zeta'][0], 
                                 ap['omega'][1], ap['zeta'][1], 
                                 ap['omega'][2], ap['zeta'][2])

        # Reconstruct linearization points
        for p_data in data['linearization_points']:
            lp = LinearizationPoint(
                pt_id=p_data['id'],
                controller_type=p_data.get('type', 'LQR'), # Default to LQR if not specified
                mach=p_data['conditions']['M'],
                q_bar=p_data['conditions']['q'],
                fuel=p_data['conditions']['W_fuel'],
                x_trim_lon=p_data['trim']['x_phys_lon'],
                x_trim_lat=p_data['trim']['x_phys_lat'],
                u_trim=p_data['trim']['control'],
                lon_state=p_data['vectors']['lon_state'],
                lat_state=p_data['vectors']['lat_state'],
                lon_input=p_data['vectors']['lon_input'],
                lat_input=p_data['vectors']['lat_input']
            )

            # Validation:
            m = p_data['matrices']
            
            def validate_matrix(matrix_list, rows_expected, cols_expected, name):
                if matrix_list is None: return None
                mat = np.array(matrix_list)
                if mat.shape != (rows_expected, cols_expected):
                    raise ValueError(
                        f"Dimension Mismatch in {p_data['id']}: {name} is {mat.shape}, "
                        f"but expected ({rows_expected}, {cols_expected}) based on state/input vectors."
                    )
                return mat

            n_lon_phys = len(lp.lon_state)
            u_lon = len(lp.lon_input)
            n_lat_phys = len(lp.lat_state)
            u_lat = len(lp.lat_input)

            n_lon_int = 1
            n_lat_int = 2

            n_lon_aug = n_lon_phys + n_lon_int
            n_lat_aug = n_lat_phys + n_lat_int

            try:
                lp.A_lon = validate_matrix(m['A_lon'], n_lon_aug, n_lon_aug, "A_lon")
                lp.B_lon = validate_matrix(m['B_lon'], n_lon_aug, u_lon, "B_lon")
                lp.K_lon = validate_matrix(m['K_lon'], u_lon, n_lon_aug, "K_lon")
                
                lp.A_lat = validate_matrix(m['A_lat'], n_lat_aug, n_lat_aug, "A_lat")
                lp.B_lat = validate_matrix(m['B_lat'], n_lat_aug, u_lat, "B_lat")
                lp.K_lat = validate_matrix(m['K_lat'], u_lat, n_lat_aug, "K_lat")
            except ValueError as e:
                print(f"CRITICAL: Failed to load point {p_data['id']}. {e}")
                continue
            
            lp.set_alt_V(p_data['conditions']['alt'], p_data['conditions']['V'])

            # Restore to np arrays
            m = p_data['matrices']
            lp.A_lon = np.array(m['A_lon']) if m['A_lon'] else None
            lp.B_lon = np.array(m['B_lon']) if m['B_lon'] else None
            lp.K_lon = np.array(m['K_lon']) if m['K_lon'] else None
            lp.Q_lon = np.array(m['Q_lon']) if m['Q_lon'] else None
            lp.R_lon = np.array(m['R_lon']) if m['R_lon'] else None
            
            lp.A_lat = np.array(m['A_lat']) if m['A_lat'] else None
            lp.B_lat = np.array(m['B_lat']) if m['B_lat'] else None
            lp.K_lat = np.array(m['K_lat']) if m['K_lat'] else None
            lp.Q_lat = np.array(m['Q_lat']) if m['Q_lat'] else None
            lp.R_lat = np.array(m['R_lat']) if m['R_lat'] else None

            # Restore Stability Metrics
            lp.lon_stability_metrics = p_data['stability metrics']['lon']
            lp.lat_stability_metrics = p_data['stability metrics']['lat']

            aircraft.add_point(lp)

        return aircraft
    
    def summary(self):
        print(f"\n--- Aircraft Summary: {self.name} ---")
        print(f"Longitudinal Actuator Model: Order {self.lon_actuator_order}")
        print(f"Lateral Actuator Model: Order {self.lat_actuator_order}")
        print(f"Total Linearization Points: {len(self.points)}")
        if self.points:
            alts = [p.alt for p in self.points if p.alt is not None]
            machs = [p.cond['M'] for p in self.points]
            print(f"Envelope Covered: Alt [{min(alts)} - {max(alts)} ft], Mach [{min(machs):.2f} - {max(machs):.2f}]")

    def list_points(self):
        print(f"\n=== Linearization Points Database: {self.name} ===")
        if not self.points:
            print('  No linearization points saved.')
            return

        header_fmt = "  {:<4} | {:>5} | {:>8} | {:>5} | {:>5} | {:>12} | {:>9} | {:>8}"
        row_fmt    = "  {:<4} | {:>5.2f} | {:>8.0f} | {:>5} | {:>5} | {:>12.1f} | {:>9.1f} | {:>8.1f}"

        headers = ["ID", "Mach", "q (Pa)", "Flap", "Gear", "Fuel/Pl (lb)", "Alt (ft)", "V (kts)"]
        
        print(header_fmt.format(*headers))
        
        print("  " + "-" * 73)

        # Print data rows
        for p in self.points:
            print(row_fmt.format(
                p.pt_id,
                p.cond['M'],
                p.cond['q'],
                p.cond['flap'],
                p.cond['gear'],
                p.cond['W_fuel'],
                p.alt,
                p.V
            ))
        print("")

    def get_point_by_id(self, pt_id):
        for p in self.points:
            if p.pt_id == pt_id:
                return p
        raise ValueError(f"No point found with ID: {pt_id}")
    
    def get_max_point_id(self):
        if not self.points:
            return 0
        return max(p.pt_id for p in self.points)
    
    def read_pt_by_id(self, pt_id):
        point = self.get_point_by_id(pt_id)
        if point is not None:
            print(f"Point ID: {point.pt_id}")
            print(f"Mach: {point.get_M():.3f}, q_bar: {point.get_q():.1f} Pa, Fuel: {point.get_fuel():.1f} lbs")
            print(f"Altitude: {point.alt:.1f} ft, Velocity: {point.V:.2f} kts")
            # print("Longitudinal K Matrix:")
            Helper.display_matrix(point.get_A(axis='lon'), name="A_lon")
            Helper.display_matrix(point.get_B(axis='lon'), name="B_lon")
            Helper.display_matrix(point.get_Q(axis='lon'), name="Q_lon")
            Helper.display_matrix(point.get_R(axis='lon'), name="R_lon")
            Helper.display_matrix(point.get_K(axis='lon'), name="K_lon")
            point.print_stability_metrics(axis='lon')

            AIMSPlotter.stability_plots(point.get_A(axis='lon'), point.get_B(axis='lon'), point.get_K(axis='lon'), axis="Longitudinal")
            
            # print("Lateral K Matrix:")
            Helper.display_matrix(point.get_A(axis='lat'), name="A_lat")
            Helper.display_matrix(point.get_B(axis='lat'), name="B_lat")
            Helper.display_matrix(point.get_Q(axis='lat'), name="Q_lat")
            Helper.display_matrix(point.get_R(axis='lat'), name="R_lat")
            Helper.display_matrix(point.get_K(axis='lat'), name="K_lat")
            point.print_stability_metrics(axis='lat')

            AIMSPlotter.stability_plots(point.get_A(axis='lat'), point.get_B(axis='lat'), point.get_K(axis='lat'), axis="Lateral")
        else:
            print('Error: Point ID not found.')
    
    def get_scheduling_data(self):
        """
        Extracts database parameters into flat lists and matrix blocks.
        """
        machs = [p.cond['M'] for p in self.points]
        q_bars = [p.cond['q'] for p in self.points]
        weights = [p.cond['W_fuel'] for p in self.points]
        flaps = [p.cond['flap'] for p in self.points]
        gears = [int(p.cond['gear']) for p in self.points]

        lat_gains = [p.get_K(axis='lat') for p in self.points]
        lon_gains = [p.get_K(axis='lon') for p in self.points]

        trim_control = [p.trim['control'] for p in self.points]
        # raw_x_trim_lat = [p.trim['x_lat'] for p in self.points]
        # raw_x_trim_lon = [p.trim['x_lon'] for p in self.points]

        x_trim_lat = []
        x_trim_lon = []

        for p in self.points:
            x_phys_lon = np.atleast_1d(np.asarray(p.trim['x_phys_lon'], dtype=float))
            x_act_lon  = np.atleast_1d(np.asarray(p.trim['x_act_lon'],  dtype=float))
            x_trim_lon.append(np.concatenate([x_phys_lon, x_act_lon]))

            x_phys_lat = np.atleast_1d(np.asarray(p.trim['x_phys_lat'], dtype=float))
            x_act_lat  = np.atleast_1d(np.asarray(p.trim['x_act_lat'],  dtype=float))
            x_trim_lat.append(np.concatenate([x_phys_lat, x_act_lat]))

        # Compute envelope maximum boundaries
        max_m = max(machs) if machs else 2.5
        max_q = max(q_bars) if q_bars else 95800
        max_w = max(weights) if weights else 11000

        return (machs, q_bars, weights, flaps, gears, 
                lat_gains, lon_gains, trim_control, x_trim_lat, x_trim_lon,
                max_m, max_q, max_w)