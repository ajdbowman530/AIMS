import jsbsim
import os
from ambiance import Atmosphere
import numpy as np
from pathlib import Path

from fdm_interface import FDMInterface
from helper import Helper
import flight_control
from models import Aircraft, LinearizationPoint
from flight_planner import FlightPlanner

import matplotlib.pyplot as plt

class FDMEnv:
    def __init__(self, aircraft, flight_plan=None, aero_freq=100, throttle_freq=10, JSBSim_freq=100, max_dry_dT=0.99):
        self.aircraft = aircraft
        self.aircraft_type = aircraft.type
        self.aero_dt = 1.0 / aero_freq
        self.throttle_dt = 1.0 / throttle_freq
        self.JSBSim_dt = 1.0 / JSBSim_freq
        self.is_initialized = False

        self.flight_plan = flight_plan

        self.lat_act_order = aircraft.lat_actuator_order
        self.lon_act_order = aircraft.lon_actuator_order
        
        config = flight_control.ExecutiveConfig()
        config.aero_dt = self.aero_dt
        config.throttle_dt = self.throttle_dt
        config.max_dry_throttle = max_dry_dT
        
        config.lat_int_map = np.array(aircraft.lat_int_map, dtype=np.int32)
        config.lon_int_map = np.array(aircraft.lon_int_map, dtype=np.int32)

        config.at_kp = 0.7
        config.at_ki = 0.02
        config.at_kd = 0.005

        self.executive = flight_control.FlightExecutive(config)
        self.executive.set_thresholds(0.01, 0.01, 0.02) # Mach, q, Weight percentage cache limits

        data = aircraft.get_scheduling_data()
        self.executive.get_scheduler().set_data(*data)

        self.fdm = jsbsim.FGFDMExec(".")
        self.fdm.set_dt(self.JSBSim_dt)
        self.fdm.set_debug_level(0)

        if not self.initialize_fdm():
            raise RuntimeError(f"Could not load physical aero model configuration for: {self.aircraft_type}")
            
        self.interface = FDMInterface(self.fdm)

    def run(self):
        while True: # CLI
            print('\nWelcome to the Flight Dynamics Module Environment!')

            if self.flight_plan is None:
                while True:
                    print('\nNo flight plan read!')
                    print('[1] Read flight plan from JSON')
                    print('[2] Launch flight planner')
                    print('[Q]uit to AIMS menu')
                    choice = input('Enter selection: ')

                    if choice == '1':
                        if self.load_flight_plan():
                            self.fdm_main_menu()

                    elif choice == '2':
                        self.flight_plan = FlightPlanner()
                        self.flight_plan.run()
                        break

                    elif choice == 'q':
                        print('Returning to AIMS menu...')
                        return
                    
                    else:
                        print('Invalid selection')
            else:
                quit = self.fdm_main_menu()
                if quit:
                    break

    def fdm_main_menu(self):
        while True:
            print(f'\nAircraft model {self.aircraft.name} loaded.')
            if self.flight_plan is not None:
                print(f'Flight plan {self.flight_plan.name} loaded.\n')
            else:
                print('No flight plan loaded!\n')
            
            print('[L]oad flight plan from .json')
            print('[P]lot loaded flight plan')
            print('[O]pen flight planner')
            print('[E]xecute flight plan')
            print('[M]odify FDM settings')
            print('[Q]uit')
            choice = input('Enter selection: ').lower()
            match choice:
                case 'e': # execute loaded flight plan
                    self.setup_flight_condition(alt=self.flight_plan.alt_init, 
                                                airspeed=self.flight_plan.vel_init,
                                                fuel_weight=self.flight_plan.w_init,
                                                flap=self.flight_plan.gear_init,
                                                gear=self.flight_plan.gear_init)
                    # self.interface.active_controls = ['throttle-cmd-norm', 'delta_e_cmd_norm', 'delta_a_cmd_norm', 'fcs/rudder-cmd-norm']
                    
                    # self.executive = flight_control.FlightExecutive(config)
                    
                    # Generate x_cmd_matrix
                    x_cmd_matrix = self.flight_plan.generate_x_cmd_matrix()
                    config_matrix = self.flight_plan.get_config_matrix()
                    x_hist = []
                    u_hist = []
                    u_cmd_hist = []

                    alpha = 0.075

                    self.interface.set_FDM_dt(self.flight_plan.sim_dt)

                    self.executive.reset()

                    t_vec = self.flight_plan.t_vec
                    for i in range(t_vec.size):
                        t = t_vec[i]

                        x = self.interface.get_complete_state(lat_act_order=self.lat_act_order, lon_act_order=self.lon_act_order) # [V, alpha, q, [lon act], beta, p, r, [lat act]]
                        u = self.interface.get_control_vals() # [delta_t, delta_e, delta_a, delta_r]

                        x_cmd = x_cmd_matrix[:,i]
                        config = config_matrix[:,i] # flap, gear

                        self.interface.set_config(config)
                        current_cond = self.interface.get_cond()

                        x_hist.append(x)
                        u_hist.append(u)
                        
                        u_cmd = self.executive.run_control_cycle(
                            sim_time=t,
                            current_cond=current_cond,
                            x=x,
                            x_cmd=x_cmd,
                            max_accel=100.0,
                            reheat=True,
                            output_filter=True,
                            output_alpha=alpha
                        )
                        
                        u_cmd_hist.append(u_cmd)
                        self.interface.setControls(u_cmd)

                        self.interface.fdm_step()
                    print('[SUCCESS] Flight plan executed successfully.')

                    choice2 = input('Save results to CSV? (Y/N) ').lower()
                    saved_csv_path = None

                    if choice2 == 'y':
                        filename = input('Enter filename: ')
                        if not filename.endswith('.csv'):
                            filename += '.csv'
                            
                        self.output_to_csv(filename=filename, 
                                        x_hist_matrix=x_hist,
                                        x_cmd_matrix=x_cmd_matrix,
                                        u_cmd_matrix=u_cmd_hist,
                                        u_hist_matrix=u_hist)
                                        
                        BASE_DIR = Path(__file__).resolve().parent
                        saved_csv_path = str(BASE_DIR / "telemetry logs" / filename)

                        choice2 = input('Launch visualizer with current results? (Y/N) ').lower()
                        if choice2 == 'y':
                            from output_visualizer import TelemetryVisualizer
                            from PySide6.QtWidgets import QApplication
                            import sys

                            app = QApplication.instance()
                            if not app:
                                app = QApplication(sys.argv)

                            # Instantiate the visualizer passing the newly exported CSV log directly
                            visualizer = TelemetryVisualizer(initial_file_path=saved_csv_path)
                            visualizer.show()
                            app.exec()

                case 'o':
                    self.flight_plan.run()
                case 'p':
                    self.flight_plan.plot_cmds()
                case 'm': # Modify FDM settings (also stored in the Flight Plan)
                    self.flight_plan.change_settings()
                    self.flight_plan.to_json(self.flight_plan.name) # Save modified settings
                case 'l':
                    # Load flight plan from JSON
                    self.load_flight_plan()
                case 'q':
                    return True

    def load_flight_plan(self):
        BASE_DIR = Path(__file__).resolve().parent
        folder_path = Path(BASE_DIR / "flight plans")
        if not folder_path.exists():
            print("Error: 'flight plans' folder not found. Folder may have been deleted.")
            return

        print('\nDetected .json files:')
        files = [f.name for f in folder_path.iterdir() if f.is_file()]
        
        for f in files:
            if f.endswith('.json') or f.name.endswith('.JSON'):
                print(f)

        print('------------------------------------------')
        filename = input('Enter JSON filename: ')

        if not filename.lower().endswith('.json'):
            filename += '.json'

        full_file_path = folder_path / filename

        try:
            # Clean up console printout tracking statement
            print(f"Reading flight plan from JSON: {filename}...")
            
            self.flight_plan = FlightPlanner.from_json(str(full_file_path))
            
            if self.flight_plan is not None:
                print(f'Flight plan {self.flight_plan.name} loaded successfully!')
                return True
            else:
                print("Error: Flight plan object returned None.")
        except Exception as e:
            print(f"Error loading flight plan: {e}")

    def setup_flight_condition(self, alt, airspeed, fuel_weight, flap=0, gear=0):
        """
        Explicit execution loop used to specify flight environments,
        whether called via file scripts or terminal menus.
        """
        self.alt = alt
        self.airspeed = airspeed
        self.weight = fuel_weight
        self.flap = flap
        self.gear = gear

        print(f"\nTrimming flight model at {alt} ft and {airspeed} kts...")
        self.interface.JSBSimInitalize(alt, airspeed)

        self.interface.set_total_fuel(fuel_weight)
        
        for i in range(2):
            self.fdm.set_property_value(f"propulsion/engine[{i}]/set-running", 1)
            self.fdm.set_property_value(f"propulsion/engine[{i}]/n2", 80.0)

        self.interface.set("fcs/gear-pos-norm", self.gear)
        self.interface.set("fcs/gear-cmd-norm", self.gear)
            
        x_trim = self.interface.get_state()
        u_trim = self.interface.getControlCmd()
        self.interface.set("fcs/gear-pos-norm", self.gear)
        self.interface.set("fcs/gear-cmd-norm", self.gear)
        self.interface.resetToPhysics(x_trim, u_trim)
        
        self.is_initialized = True
        print("Flight Environment initialization completed successfully.")


    def initialize_fdm(self):
        """Attempts to load the aircraft. Returns True if successful."""
        try:
            base_path = os.path.dirname(os.path.abspath(__file__))
            # Construct the path to the jsbsim folder
            jsb_root = os.path.join(base_path, "jsbsim") 

            full_aircraft_path = os.path.abspath(os.path.join(jsb_root, "aircraft", self.aircraft_type))
            # print(f"Checking for existence of folder: {full_aircraft_path}")
            # print(f"Folder exists? {os.path.exists(full_aircraft_path)}")

            self.fdm.set_root_dir(jsb_root)
            self.fdm.set_aircraft_path("aircraft") # JSBSim looks in jsb_root/aircraft
            self.fdm.set_engine_path("engine")     # JSBSim looks in jsb_root/engine
            self.fdm.set_systems_path("systems")

            # print(f"Searching for {self.aircraft_type} in {os.path.join(jsb_root, 'aircraft')}")

            if not self.fdm.load_model(self.aircraft_type):
                print(f"\n[!] Error: JSBSim could not find '{self.aircraft_type}'")
                return False
            
            self.is_loaded = True
            return True
        except Exception as e:
            print(f"Internal JSBSim Error: {e}")
            return False
        

    def output_to_csv(self, filename, x_hist_matrix, x_cmd_matrix, u_hist_matrix, u_cmd_matrix):
        """
        Saves the flight simulation telemetry out to a structured CSV file
        for post-flight analysis in the visualizer.
        """
        import csv

        x_hist_matrix = np.array(x_hist_matrix).T

        BASE_DIR = Path(__file__).resolve().parent
        output_dir = BASE_DIR / "telemetry logs"
        output_dir.mkdir(exist_ok=True)

        if not filename.lower().endswith('.csv'):
            filename += '.csv'
        
        file_path = output_dir / filename

        # List states for output visualization
        headers = [
            'time',
            'V_actual', 'alpha_actual', 'q_actual', 'beta_actual', 'p_actual', 'r_actual',
            'V_cmd', 'alpha_cmd', 'q_cmd', 'beta_cmd', 'p_cmd', 'r_cmd',
            'throttle_cmd', 'elevator_cmd', 'aileron_cmd', 'rudder_cmd',
            'throttle_actual', 'elevator_actual', 'aileron_actual', 'rudder_actual'
        ]

        t_vec = self.flight_plan.t_vec

        try:
            print(f"Writing simulation telemetry to {file_path}...")
            with open(file_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)  # Write header row first
                
                # Iterate through every time step recorded
                for i in range(len(t_vec)):
                    t = t_vec[i]
                    u = u_hist_matrix[i]
                    u_cmd = u_cmd_matrix[i]
                     
                    x = x_hist_matrix[:, i]            
                    x_cmd = x_cmd_matrix[:, i]

                    n_V = 0
                    n_alpha = 1
                    n_q = 2
                    n_beta = len(self.aircraft.lon_int_map) + 1
                    n_p = n_beta + 1
                    n_r = n_beta + 2

                    row = [
                        f"{t:.4f}",
                        f"{x[n_V]:.4f}", f"{x[n_alpha]:.6f}", f"{x[n_q]:.6f}", f"{x[n_beta]:.6f}", f"{x[n_p]:.6f}", f"{x[n_r]:.6f}",
                        f"{x_cmd[n_V]:.4f}", f"{x_cmd[n_alpha]:.6f}", f"{x_cmd[n_q]:.6f}", f"{x_cmd[n_beta]:.6f}", f"{x_cmd[n_p]:.6f}", f"{x_cmd[n_r]:.6f}",
                        f"{u_cmd[0]:.4f}", f"{u_cmd[1]:.4f}", f"{u_cmd[2]:.4f}", f"{u_cmd[3]:.4f}",
                        f"{u[0]:.4f}", f"{u[1]:.4f}", f"{u[2]:.4f}", f"{u[3]:.4f}"
                    ]
                    writer.writerow(row)
                    
            print(f"[SUCCESS] Telemetry successfully saved! ({len(t_vec)} frames)")
        except Exception as e:
            print(f"[ERROR] Failed to write telemetry CSV: {e}")
