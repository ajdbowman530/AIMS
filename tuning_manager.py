from unittest import case

import numpy as np
import control as ct
import jsbsim
from ambiance import Atmosphere
import os
from dataclasses import dataclass, field
import copy

from fdm_interface import FDMInterface
from helper import Helper
from models import Aircraft, LinearizationPoint
from stability_analysis import StabilityAnalysis
from aims_plotter import AIMSPlotter

class TuningManager:
    def __init__(self, aircraft_name):
        self.aircraft = aircraft_name
        self.fdm = jsbsim.FGFDMExec(".")
        self.fdm.set_debug_level(0)
        self.is_loaded = False

        # self.initialize_fdm()

        self.alt = -1
        self.vel = -1
        self.load = -1

        self.interface = FDMInterface(self.fdm)
        self.aircraft_obj = None # This will be set to an Aircraft object after loading

    def initialize_fdm(self):
        """Attempts to load the aircraft. Returns True if successful."""
        try:
            base_path = os.path.dirname(os.path.abspath(__file__))
            jsb_root = os.path.join(base_path, "jsbsim") 

            full_aircraft_path = os.path.abspath(os.path.join(jsb_root, "aircraft", self.aircraft))
            print(f"Checking for existence of folder: {full_aircraft_path}")
            print(f"Folder exists? {os.path.exists(full_aircraft_path)}")

            self.fdm.set_root_dir(jsb_root)
            self.fdm.set_aircraft_path("aircraft") # JSBSim looks in jsb_root/aircraft
            self.fdm.set_engine_path("engine")     # JSBSim looks in jsb_root/engine
            self.fdm.set_systems_path("systems")

            print(f"Searching for {self.aircraft} in {os.path.join(jsb_root, 'aircraft')}")

            if not self.fdm.load_model(self.aircraft):
                print(f"\n[!] Error: JSBSim could not find '{self.aircraft}'")
                return False
            
            self.is_loaded = True
            return True
        except Exception as e:
            print(f"Internal JSBSim Error: {e}")
            return False

    def run(self):
        """This sets up the situation"""

        verified_aircraft = ['f15']

        if not self.is_loaded:
            if not self.initialize_fdm():
                print("Error: Failed to initialize JSBSim environment. Exiting manager.")
                return

        if self.aircraft_obj is None: # If creating a new aircraft, set up actuator modeling
            while True:
                # choice = input('\nLongitudinal actuator model order (0 for none, 1 for first-order, 2 for second-order): ')
                choice = input('\nLongitudinal actuator model order (0 for none, 1 for first-order): ')
                try:
                    lon_act_order = int(choice)
                    # if lon_act_order in [0, 1, 2]:
                    if lon_act_order in [0, 1]:
                        pass
                    else:
                        # print('Error: Please enter 0, 1, or 2.')
                        print('Error: Please enter 0 or 1')
                except ValueError:
                    print('Error: Please enter a numeric value.')
                choice = input('Continue with these actuator settings? (y/n) ').lower()
                if choice == 'y':
                    break
            
            while True:
                # choice = input('\nLateral actuator model order (0 for none, 1 for first-order, 2 for second-order): ')
                choice = input('\nLateral actuator model order (0 for none, 1 for first-order): ')
                try:
                    lat_act_order = int(choice)
                    # if lat_act_order in [0, 1, 2]:
                    if lat_act_order in [0, 1]:
                        pass
                    else:
                        # print('Error: Please enter 0, 1, or 2.')
                        print('Error: Please enter 0 or 1.')
                except ValueError:
                    print('Error: Please enter a numeric value.')
                choice = input('Continue with these actuator settings? (y/n) ').lower()
                if choice == 'y':
                    break
            
            # Lon int map:
            lon_int_map = np.array([0, 0], dtype=np.int32)
            int_alpha = False
            int_q = False
            while True: # Select lon integrators
                print('\nIntegrated variables of the Longitudinal Mode:')
                if int_alpha == True:
                    print('[X] [1] alpha_rad')
                else:
                    print('[ ] [1] alpha_rad')
                if int_q == True:
                    print('[X] [2] q_rad_sec')
                else:
                    print('[ ] [2] q_rad_sec')
                
                choice = input('Toggle variable to be integrated (enter c to continue): ').lower()
                if choice == 'c':
                    if int_alpha or int_q:
                        break
                    else:
                        print('Error: Must integrate one or more states')
                elif choice == '1': # Integrate alpha_rad error
                    int_alpha = not int_alpha
                elif choice == '2': # Integrate q_rad_sec error
                    int_q = not int_q
                else:
                    print('Invalid choice.')
            
            c = 1
            if int_alpha:
                lon_int_map[0] = c
                c += 1
            if int_q:
                lon_int_map[1] = c
                c += 1

            # Lat int map:
            lat_int_map = np.array([0, 0, 0], dtype=np.int32)
            int_beta = False
            int_p = False
            int_r = False
            while True: # Select lon integrators
                print('\nIntegrated variables of the Longitudinal Mode:')
                if int_beta:
                    print('[X] [1] beta_rad')
                else:
                    print('[ ] [1] beta_rad')
                if int_p:
                    print('[X] [2] p_rad_sec')
                else:
                    print('[ ] [2] p_rad_sec')
                if int_r:
                    print('[X] [2] r_rad_sec')
                else:
                    print('[ ] [2] r_rad_sec')
                
                choice = input('Toggle variable to be integrated (enter c to continue): ').lower()
                if choice == 'c':
                    if int_alpha or int_q:
                        break
                    else:
                        print('Error: Must integrate one or more states')
                elif choice == '1': # Integrate alpha_rad error
                    int_beta = not int_beta
                elif choice == '2': # Integrate q_rad_sec error
                    int_p = not int_p
                elif choice == '3': # Integrate q_rad_sec error
                    int_r = not int_r
                else:
                    print('Invalid choice.')
            
            c = 1
            if int_beta:
                lat_int_map[0] = c
                c += 1
            if int_p:
                lat_int_map[1] = c
                c += 1
            if int_r:
                lat_int_map[2] = c
                c += 1

            # n_int_lat = np.count_nonzero(lat_int_map)
            n_lat_act = 2 # Aileron, rudder
            n_lon_act = 1 # elevator

            # Make int map length consistent with the number of states there are in the model including actuator states
            lat_int_map = np.append(lat_int_map, np.zeros(lat_act_order * n_lat_act))
            lon_int_map = np.append(lon_int_map, np.zeros(lon_act_order * n_lon_act))

            self.aircraft_obj = Aircraft(
                type = self.aircraft, 
                name = 'untitled', # Temporary placeholder, name will be filled by filename when saved
                lon_actuator_config={'order': lon_act_order}, 
                lat_actuator_config={'order': lat_act_order},
                ref_lon_states=['alpha_rad', 'q_rad_sec', 'delta_e_norm'],
                ref_lon_controls=['delta_e_cmd_norm'],
                ref_lat_states=['beta_rad', 'p_rad_sec', 'r_rad_sec', 'delta_a_norm', 'delta_r_norm'],
                ref_lat_controls=['delta_a_cmd_norm', 'delta_r_cmd_norm'],
                lat_int_map=lat_int_map, lon_int_map=lon_int_map
            )
            if self.aircraft in verified_aircraft: # list verified aircraft types
                choice = input('\nAutofill first order longitudinal actuator dynamics? (y/n) ').lower()
                if choice == 'y':
                    tau_e = get_act_info(act_mag=lon_act_order,aircraft_type=self.aircraft,axis='lon')
                    self.aircraft_obj.tau_e = tau_e
            else:
                while True: # Longitudinal actuator setup
                    print('\nLongitudinal actuator model setup:')
                    if lon_act_order == 0:
                        print('No actuator model will be used for longitudinal control.')
                    elif lon_act_order == 1:
                        tau_str = input('Enter elevator actuation time constant: ')
                        try:
                            tau_e = float(tau_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for elevator actuation time constant')
                    
                        self.aircraft_obj.tau_e = tau_e
                    elif lon_act_order == 2:
                        print('WARNING: Second order actuator model not yet implemented.')
                        wn_str = input('Enter elevator natural frequency: ')
                        try:
                            self.aircraft_obj.omega_e = float(wn_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for natural frequency.')
                        zeta_str = input('Enter elevator damping: ')
                        try:
                            self.aircraft_obj.zeta_e = float(zeta_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for damping.')
                    choice = input('Continue with these actuator settings? (y/n) ').lower()
                    if choice == 'y':
                        break

            if self.aircraft in verified_aircraft:
                choice = input('Autofill first order lateral actuator dynamics? (y/n) ').lower()
                if choice == 'y':
                    if lat_act_order == 1:
                        tau_a, tau_r = get_act_info(act_mag=lat_act_order,aircraft_type=self.aircraft,axis='lat')
                        self.aircraft_obj.tau_a = tau_a
                        self.aircraft_obj.tau_r = tau_r
                    elif lat_act_order == 2:
                        wn_a, zeta_a, wn_r, zeta_r = get_act_info(act_mag=lat_act_order,aircraft_type=self.aircraft,axis='lat')
                        self.aircraft_obj.omega_a = wn_a
                        self.aircraft_obj.omega_r = wn_r
                        self.aircraft_obj.zeta_a  = zeta_a
                        self.aircraft_obj.zeta_r = zeta_r
            else:
                while True: # Lateral actuator setup
                    print('\nLateral actuator model setup:')
                    if lat_act_order == 0:
                        print('No actuator model will be used for lateral control.')
                    elif lat_act_order == 1:
                        tau_str = input('Enter aileron actuation time constant: ')
                        try:
                            tau_a = float(tau_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for aileron actuation time constant')
                        self.aircraft_obj.tau_a = tau_a
                        tau_str = input('Enter rudder actuation time constant: ')
                        try:
                            tau_r = float(tau_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for rudder actuation time constant')
                        self.aircraft_obj.tau_r = tau_r
                    elif lat_act_order == 2:
                        print('WARNING: Second order actuator model not yet implemented.')
                        wn_str = input('Enter aileron natural frequency: ')
                        try:
                            self.aircraft_obj.omega_a = float(wn_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for natural frequency.')
                        zeta_str = input('Enter aileron damping: ')
                        try:
                            self.aircraft_obj.zeta_a = float(zeta_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for damping.')

                        wn_str = input('Enter rudder natural frequency: ')
                        try:
                            self.aircraft_obj.omega_r = float(wn_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for natural frequency.')
                        zeta_str = input('Enter rudder damping: ')
                        try:
                            self.aircraft_obj.zeta_r = float(zeta_str)
                        except ValueError:
                            print('Error: Please enter a numeric value for damping.')
                    choice = input('\nContinue with these actuator settings? (y/n) ').lower()
                    if choice == 'y':
                        break

        while True: # Create linearization points
            quit = False

            while True:
                self.aircraft_obj.list_points()

                choice = input('\n[A]dd a tuning point\nMake a [C]opy of a tuning point\n[M]odify a tuning point\n[D]elete a tuning point\n[S]ave gain schedule to JSON\n'
                '[R]ead a linearization point\n[Q]uit to AIMS menu\nEnter selection: ').lower()
                if choice == 'q': # Quit to AIMS menu
                    choice = input('Save before quitting? (y/n) ').lower()
                    if choice == 'y':
                        if self.aircraft_obj.name == 'untitled':
                            filename = input('Enter filename: ')
                            self.aircraft_obj.set_name(filename)
                        else:
                            filename = self.aircraft_obj.name
                        self.aircraft_obj.export_to_json(filename)
                        print(f"Saved gain schedule to {filename}.JSON")
                        print('Remember to read gains from JSON before trying to modify or use the aircraft model.')
                    quit = True
                    break

                elif choice == 's': # Save to JSON
                    # Save as JSON
                    if self.aircraft_obj.name == 'untitled':
                        filename = input('Enter filename: ')
                        self.aircraft_obj.set_name(filename)
                    else:
                        filename = self.aircraft_obj.name
                    self.aircraft_obj.export_to_json(filename)
                    print(f'\nSaved gain schedule to {filename}.json')
                    print('Remember to read gains from JSON before trying to modify or use the aircraft model.')
                    pass

                elif choice == 'a': # Add another tuning point
                    break

                elif choice == 'd': # Delete a tuning point
                    self.aircraft_obj.list_points()
                    choice = input('Enter point ID to delete: ')
                    try:
                        point_id = int(choice)
                        success = self.aircraft_obj.remove_point(point_id)
                        if success:
                            print(f'Point {point_id} deleted successfully.')
                        else:
                            print('Error: Point ID not found.')
                    except ValueError:
                        print('Error: Please enter a numeric value for point ID.')

                elif choice == 'r': # Read a linearization point
                    choice = input('Enter point ID to read: ')
                    try:
                        point_id = int(choice)
                        self.aircraft_obj.read_pt_by_id(point_id)
                    except ValueError:
                        print('Error: Please enter a numeric value for point ID.')

                elif choice == 'c': # Make a copy of a tuning point
                    self.aircraft_obj.list_points()
                    choice = input('Enter point ID to copy: ')
                    try:
                        point_id = int(choice)

                        # Create new point, set new point ID
                        original_point = self.aircraft_obj.get_point_by_id(point_id)

                        if original_point is not None:
                            new_point = copy.deepcopy(original_point)
                            new_id = self.aircraft_obj.get_max_point_id() + 1
                            new_point.set_id(new_id)
                            self.aircraft_obj.add_point(new_point)
                            print(f'Point {point_id} copied successfully to Point {new_id}.')
                        else:
                            print('Error: Point ID not found.')
                    except ValueError:
                        print('Error: Please enter a numeric value for point ID.')

                elif choice == 'm': # modify a tuning point
                    self.aircraft_obj.list_points()
                    choice = input('\nEnter point ID to modify: ')

                    # Go through the same process as creating a new point, but pre-fill values with the existing data
                    try:
                        point_id = int(choice)
                        point = self.aircraft_obj.get_point_by_id(point_id)

                        if point is None:
                            print('Error: Point ID not found.')
                            continue

                        print(f'Modifying Point ID {point_id}.')

                        current_alt, current_vel = point.get_alt_V()
                        current_W = point.get_fuel()
                        current_flap = point.get_flap()
                        current_gear = point.get_gear()

                        while True:
                            print(f"\n[1] Current altitude: {current_alt:.1f} ft\n" \
                                f"[2] Currrent airspeed: {current_vel:.1f} kts\n" \
                                f"[3] Current fuel load: {current_W:.1f} lbs\n" \
                                f"[4] Current normalized flap position: {current_flap:.2f} \n" \
                                f"[5] Current gear position: {'Down' if current_gear == 1 else 'Up'}")
                            choice = input('Select a value to modify. Enter "c" to continue with current values. ').lower()

                            match choice:
                                case 'c': # continue with current values
                                    break
                                case '1': # Modify altitude
                                    alt_str = input(f'Desired altitude (ft) [{current_alt}]: ')
                                    try:
                                        alt = float(alt_str) if alt_str else current_alt
                                        current_alt = alt
                                    except ValueError:
                                        print('Error: Please enter a numeric value.')
                                case '2': # Modify airspeed
                                    u_str = input(f'Desired airspeed (kts) [{current_vel}]: ')
                                    try:
                                        u = float(u_str) if u_str else current_vel
                                        current_vel = u
                                    except ValueError:
                                        print('Error: Please enter a numeric value.')
                                case '3': # Modify fuel load
                                    W_str = input(f'Desired fuel load (lbs) [{current_W:.1f}]: ')
                                    try:
                                        W = float(W_str) if W_str else current_W
                                        current_W = W
                                    except ValueError:
                                        print('Error: Please enter a numeric value.')
                                case '4': # Modify flap position
                                    flap_str = input(f'Desired normalized flap setting:\n\t[1] 0.0 (retracted)\n\t[2] 0.1\n\t[3] 0.2\n\t[4] 0.3\n\t[5] 0.5\n\t[6] 0.75\n\t[7] 1.0 (fully deployed)\nCurrent flap position: [{current_flap:.2f}]: ')
                                    match flap_str:
                                        case '1':
                                            current_flap = 0.0
                                        case '2':
                                            current_flap = 0.1
                                        case '3':
                                            current_flap = 0.2
                                        case '4':
                                            current_flap = 0.3
                                        case '5':
                                            current_flap = 0.5
                                        case '6':
                                            current_flap = 0.75
                                        case '7':
                                            current_flap = 1.0
                                        case _:
                                            print('Error: Invalid selection.')
                                case '5': # Modify gear position
                                    gear_str = input(f'Gear (0 for up, 1 for down) [{current_gear:d}]: ')
                                    try:
                                        gear = int(gear_str) if gear_str else current_gear
                                        if gear not in [0, 1]:
                                            print('Error: Enter either 0 or 1.')
                                        else:
                                            current_gear = gear
                                    except ValueError:
                                        print('Error: Please enter a numeric value (0 or 1).')

                        # Update the point with new values
                        q, M = Helper.calc_q_M(current_alt, current_vel)
                        print(f'\nCalculated conditions: \nq = {q:.0f} Pa\nM = {M:.2f}\nW = {current_W:.1f} lbs\nFlap position = {current_flap:.2f}" \
                              "\nGear position = {"Down" if current_gear == 1 else "Up"}')
                        
                        choice = input('Proceede with these conditions? (y/n) ').lower()
                        if choice == 'y':
                            self.alt = current_alt
                            self.vel = current_vel
                            self.load = current_W

                            self.flap = current_flap
                            self.gear = current_gear

                            # linearization:
                            print('\nStarting linearization...')
                            A_lon, A_lat, B_lon, B_lat, x_trim_lon, x_act_trim_lon, x_trim_lat, x_act_trim_lat, u_trim = self.linearize_model()
                            if A_lon is None: # Linearization failed, return to point menu
                                print('Linearization failed. Returning to point menu.')
                                continue
                            choice = input('\nProceed with these state-space matrices? (y/n) ').lower()

                            if choice == 'y':
                                while True: # Tune longitudinal controller

                                    Q_lon_init = copy.deepcopy(point.get_Q(axis='lon'))
                                    R_lon_init = copy.deepcopy(point.get_R(axis='lon'))

                                    K_lon, Q_lon, R_lon = self.lon_tuning(A_lon, B_lon, Q_lon=Q_lon_init, R_lon=R_lon_init)

                                    if K_lon is not None:
                                        print('Longitudinal mode successfully tuned!')
                                        Helper.display_matrix(K_lon, name="Longitudinal LQR Gains")

                                        # Do the stability metrics and whatnot.
                                        print('\nStability metrics:')
                                        stability_lon = StabilityAnalysis(A_lon, B_lon, K=K_lon)
                                        stability_lon.print_metrics()

                                        AIMSPlotter.stability_plots(A_lon, B_lon, K_lon, axis="Longitudinal")

                                        Q_lon_init = Q_lon
                                        R_lon_init = R_lon

                                        choice = input('\nUse these gains and continue to lateral tuning? (y/n) ').lower()
                                        if choice == 'y':
                                            break

                                while True: # Tune lateral controller
                                    Q_lat_init = copy.deepcopy(point.get_Q(axis='lat'))
                                    R_lat_init = copy.deepcopy(point.get_R(axis='lat'))

                                    K_lat, Q_lat, R_lat = self.lat_tuning(A_lat, B_lat, Q_lat=Q_lat_init, R_lat=R_lat_init)

                                    if K_lat is not None:
                                        print('Lateral mode successfully tuned!')
                                        Helper.display_matrix(K_lat, name="Lateral LQR Gains")

                                        # Do the stability metrics and whatnot.
                                        print('\nStability metrics:')
                                        stability_lat = StabilityAnalysis(A_lat, B_lat, K=K_lat)
                                        stability_lat.print_metrics()

                                        AIMSPlotter.stability_plots(A_lat, B_lat, K_lat, axis="Lateral")

                                        choice = input('\nUse these gains and update point? (y/n) ').lower()
                                        if choice == 'y':
                                            break

                                # Update point with new values
                                point.set_alt_V(current_alt, current_vel)
                                point.set_fuel(current_W)
                                point.set_gear(current_gear)
                                point.set_flap(current_flap)
                                point.set_q_M(q, M) 
                                point.set_trim(x_trim_lon, x_trim_lat, u_trim)
                                point.set_lon_matrices(A_lon, B_lon, K_lon)
                                point.set_lat_matrices(A_lat, B_lat, K_lat)
                                point.set_lon_lqr_data(Q_lon, R_lon)
                                point.set_lat_lqr_data(Q_lat, R_lat)
                                point.set_stability_metrics(axis='lon', **stability_lon.get_metrics_dict())
                                point.set_stability_metrics(axis='lat', **stability_lat.get_metrics_dict())

                                # self.aircraft_obj.add_point(point)

                    except ValueError: # Bad point id or non-numeric input
                        print('Error: Please enter a numeric value.')

            if quit:
                break
            
            # Make new linearization point
            while True: # set altitude
                alt_str = input('\nDesired altitude (ft): ')
                try:
                    alt = float(alt_str)
                    break
                except ValueError:
                    print('Error: Please enter a numeric value for altitude.')

            while True: # set airspeed
                u_str = input('Desired airspeed (kts): ')
                try:
                    u = float(u_str)
                    break
                except ValueError:
                    print('Error: Please enter a numeric value for airspeed.')

            while True: # Set fuel/payload weight
                W_str = input('Desired fuel/payload load (lbs): ')
                try:
                    W = float(W_str)
                except ValueError:
                    print('Error: Please enter a numeric value for fuel load.')

                if W >= 0:
                    self.interface.set_total_fuel(W)
                    break
                else:
                    print('Error: Please enter a positive number.')

            while True: # Set flap position
                flap_str = input('Desired normalized flap position: \n[1] 0.00 (retracted)\n' \
                '[2]0.10\n[3]0.20\n[4]0.30\n[5]0.50\n[6]0.75\n1.00 (fully extended)\nEnter selection: ')
                try:
                    flap = float(flap_str)
                    if flap in [0, 1, 2, 3, 4, 5, 6, 7]:
                        match flap_str: # set flap position based on selection
                                case '1':
                                    flap = 0.0
                                case '2':
                                    flap = 0.1
                                case '3':
                                    flap = 0.2
                                case '4':
                                    flap = 0.3
                                case '5':
                                    flap = 0.5
                                case '6':
                                    flap = 0.75
                                case '7':
                                    flap = 1.0
                                case _:
                                    print('Error: Invalid selection.') 
                        break
                    else:
                        print('Error: Please enter a valid choice.')
                except ValueError:
                    print('Error: Please enter a numeric value.')

            while True: # Set gear position
                gear_str = input('Gear (0 for up, 1 for down): ')
                try:
                    gear = int(gear_str)
                    if gear in [0, 1]:
                        break
                    else:
                        print('Error: Please enter 0 (up) or 1 (down).')
                except ValueError:
                    print('Error: Please enter a numeric value (0 or 1).')

            q, M = Helper.calc_q_M(alt, u)
            print(f'\nCalculated conditions: q = {q:.0f} Pa, M = {M:.2f}, W = {W:.1f} lbs')
            
            choice = input('Proceede with these conditions? (y/n) ').lower()
            if choice == 'y':
                self.alt = alt
                self.vel = u
                self.load = W

                self.flap = flap
                self.gear = gear

                print('\nStarting linearization...')
                # linearize the stuff
                A_lon, A_lat, B_lon, B_lat, x_trim_lon, x_act_trim_lon, x_trim_lat, x_act_trim_lat, u_trim = self.linearize_model()
                if A_lon is None: # Linearization failed, return to point menu
                    print('Linearization failed. Returning to point menu.')
                    continue
                choice = input('\nProceed with these state-space matrices? (y/n) ').lower()
                if choice == 'y':
                    print('\nLongitudinal tuning: ')
                    n_x = np.size(A_lon,axis=0)
                    n_u = np.size(B_lon,axis=1)

                    Q = np.eye(n_x)
                    R = np.eye(n_u)

                    while True: # set LQR weights
                        K_lon, Q_lon, R_lon = self.lon_tuning(A_lon, B_lon, Q_lon=Q, R_lon=R)

                        if K_lon is not None:
                            print('Longitudinal mode successfully tuned!')
                            Helper.display_matrix(K_lon, name="Longitudinal LQR Gains")

                            # Do the stability metrics and whatnot.
                            print('\nStability metrics:')
                            stability_lon = StabilityAnalysis(A_lon, B_lon, K=K_lon)
                            stability_lon.print_metrics()

                            AIMSPlotter.stability_plots(A_lon, B_lon, K_lon, axis="Longitudinal")

                            choice = input('\nUse these gains and continue to lateral tuning? (y/n) ').lower()
                            if choice == 'y':
                                break


                    print('\nLateral tuning: ')
                    n_x = np.size(A_lat,axis=0)
                    n_u = np.size(B_lat,axis=1)

                    Q = np.eye(n_x)
                    R = np.eye(n_u)

                    while True: # Tune LQR
                        K_lat, Q_lat, R_lat = self.lat_tuning(A_lat, B_lat, Q_lat=Q, R_lat=R)

                        if K_lat is not None:
                            print('Lateral mode successfully tuned!')
                            Helper.display_matrix(K_lat, name="Lateral LQR Gains")

                            # Do stability metrics
                            print('\nStability metrics:')
                            stability_lat = StabilityAnalysis(A_lat, B_lat, K=K_lat)
                            stability_lat.print_metrics()

                            AIMSPlotter.stability_plots(A_lat, B_lat, K_lat, axis="Lateral")

                            choice = input('\nUse these gains? (y/n) ').lower()
                            if choice == 'y':
                                break

                    choice = input('\nSave point to aircraft? (y/n) ').lower()
                    if choice == 'n':
                        print('Point not saved.')
                    else:
                        point = LinearizationPoint(pt_id=len(self.aircraft_obj.points),
                                                mach=M, q_bar=q, fuel=W, 
                                                x_trim_lon = x_trim_lon, x_trim_lat = x_trim_lat, u_trim = u_trim,
                                                lon_state=['alpha_rad', 'q_rad_sec', 'delta_e_norm'], 
                                                lat_state=['beta_rad', 'p_rad_sec', 'r_rad_sec', 'delta_a_norm', 'delta_r_norm'],
                                                lon_input=['delta_e_cmd_norm'],
                                                lat_input=['delta_a_cmd_norm', 'delta_r_cmd_norm'], 
                                                controller_type='LQR',
                                                gear_pos=self.gear, flap_pos=self.flap)
                        
                        point.set_lon_matrices(A=A_lon, B=B_lon, K=K_lon)
                        point.set_lon_lqr_data(Q=Q_lon, R=R_lon)
                        point.set_lat_matrices(A=A_lat, B=B_lat, K=K_lat)
                        point.set_lat_lqr_data(Q=Q_lat, R=R_lat)
                        point.set_alt_V(alt=self.alt, V=self.vel)

                        stability_metrics = stability_lat.get_metrics_dict()
                        point.set_stability_metrics(axis='lat', **stability_metrics)
                        stability_metrics = stability_lon.get_metrics_dict()
                        point.set_stability_metrics(axis='lon', **stability_metrics)

                        self.aircraft_obj.points.append(point)
                        print('Point saved to aircraft.')
                        
                        print(f'Current number of tuning points for {self.aircraft_obj.name}: {len(self.aircraft_obj.points)}')

            else:
                choice = input('Modify conditions or return to AIMS menu (m/q) ').lower()
                if choice.lower() == 'q':
                    return
                
    def lon_tuning(self, A_lon, B_lon, Q_lon, R_lon):
        # This only works for first order actuator models now

        while True: # Modify longitudinal Q and R, first order only for now
            print('\nQ values: ')
            print(f'[1]     Q_int_q = {Q_lon[0,0]}')
            print(f'[2]     Q_alpha = {Q_lon[1,1]}')
            print(f'[3]     Q_q = {Q_lon[2,2]}')
            print(f'[4]     Q_delta_e = {Q_lon[3,3]}')

            print('\nR values: ')
            print(f'[5]     R_delta_e_cmd = {R_lon[0,0]}')

            choice = input('\nSelect a value to modify. Enter "c" to continue. ')
            if choice.lower() == 'c': # Continue with current Q and R
                break
            elif choice == '1': # Change Q_int_alpha
                choice = input('Q_int_q = ')
                try:
                    choice = float(choice)
                    Q_lon[0,0] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')

            elif choice == '2': # Change Q_alpha
                choice = input('Q_alpha = ')
                try:
                    choice = float(choice)
                    Q_lon[1,1] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')
            
            elif choice == '3': # Change Q_q
                choice = input('Q_q = ')
                try:
                    choice = float(choice)
                    Q_lon[2,2] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')

            elif choice == '4': # Change Q_delta_e
                choice = input('Q_delta_e = ')
                try:
                    choice = float(choice)
                    Q_lon[3,3] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')
            
            elif choice == '5': # Change R
                choice = input('R_delta_e_cmd = ')
                try:
                    choice = float(choice)
                    R_lon[0,0] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')
            else: # invalid choice
                print('Error: Invalid selection.')
        
        print('Running LQR...')

        n_x = np.size(A_lon,axis=0)
        sys_cont = ct.ss(A_lon, B_lon, np.eye(n_x), 0)
        sys_disc = sys_cont.sample(self.fdm.get_delta_t())

        try:
            K_lon, S, E = ct.dlqr(sys_disc.A, sys_disc.B, Q_lon, R_lon)
        except Exception as error:
            print('Error: LQR Failed')
            return None, None, None

        print('Longitudinal mode successfully tuned!')

        return K_lon, Q_lon, R_lon

    def lat_tuning(self, A_lat, B_lat, Q_lat, R_lat):
        # For first order actuator models only
        while True: # Modify longitudinal Q and R, first order only for now
            print('\nQ values: ')
            print(f'[1]     Q_int_beta = {Q_lat[0,0]}')
            print(f'[2]     Q_int_p = {Q_lat[1,1]}')
            print(f'[3]     Q_beta = {Q_lat[2,2]}')
            print(f'[4]     Q_p = {Q_lat[3,3]}')
            print(f'[5]     Q_r = {Q_lat[4,4]}')
            print(f'[6]     Q_delta_a = {Q_lat[5,5]}')
            print(f'[7]     Q_delta_r = {Q_lat[6,6]}')

            print('\nR values: ')
            print(f'[8]     R_delta_a_cmd = {R_lat[0,0]}')
            print(f'[9]     R_delta_r_cmd = {R_lat[1,1]}')

            choice = input('\nSelect a value to modify. Enter "c" to continue. ')
            if choice.lower() == 'c': # Continue with current Q and R
                break
            elif choice == '1': # Change Q_int_beta
                choice = input('Q_int_beta = ')
                try:
                    choice = float(choice)
                    Q_lat[0,0] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')

            elif choice == '2': # Change Q_int_p
                choice = input('Q_int_p = ')
                try:
                    choice = float(choice)
                    Q_lat[1,1] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')
            
            elif choice == '3': # Change Q_beta
                choice = input('Q_beta = ')
                try:
                    choice = float(choice)
                    Q_lat[2,2] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')

            elif choice == '4': # Change Q_p
                choice = input('Q_p = ')
                try:
                    choice = float(choice)
                    Q_lat[3,3] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')

            elif choice == '5': # Change Q_r
                choice = input('Q_r = ')
                try:
                    choice = float(choice)
                    Q_lat[4,4] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')
            
            elif choice == '6': # Change Q_delta_a
                choice = input('Q_delta_a = ')
                try:
                    choice = float(choice)
                    Q_lat[5,5] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')
            
            elif choice == '7': # Change Q_delta_r
                choice = input('Q_delta_r = ')
                try:
                    choice = float(choice)
                    Q_lat[6,6] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')
            
            elif choice == '8': # Change R_delta_r_cmd
                choice = input('R_delta_r_cmd = ')
                try:
                    choice = float(choice)
                    R_lat[0,0] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')

            elif choice == '9': # Change R_delta_r_cmd
                choice = input('R_delta_r_cmd = ')
                try:
                    choice = float(choice)
                    R_lat[1,1] = choice
                except ValueError:
                    print('Error: Please enter a numeric value.')

            else: # invalid choice
                print('Error: Invalid selection.')
        
        print('Running LQR...')

        n_x = np.size(A_lat,axis=0)
        sys_cont = ct.ss(A_lat, B_lat, np.eye(n_x), 0)
        sys_disc = sys_cont.sample(self.fdm.get_delta_t())

        try:
            K_lat, S, E = ct.dlqr(sys_disc.A, sys_disc.B, Q_lat, R_lat)
        except Exception as error:
            print('Error: LQR failed.')
            return None, None, None

        return K_lat, Q_lat, R_lat

    def linearize_model(self):
        """Get A and B matrices from JSBSim by perturbing trimmed states and controls."""
        print(f"Linearizing {self.aircraft} at {self.alt:.1f}ft, {self.vel:.1f}kts...\n")

        self.fdm.set_property_value('fcs/flap-cmd-norm', self.flap)
        self.fdm.set_property_value('fcs/flap-pos-norm', self.flap)
        self.fdm.set_property_value('fcs/gear-cmd-norm', self.gear)
        self.fdm.set_property_value('fcs/gear-pos-norm', self.gear)
        
        try:
            self.interface.JSBSimInitalize(self.alt,self.vel)

            try:
                x_phys_lon = self.interface.get_lon_phys_state()
                x_phys_lat = self.interface.get_lat_phys_state()
            except Exception as e:
                print("="*60)
                print("Error with get_lon/lat_state()...")
                print(f"Details: {e}")
                print("="*60)
                
            x_act_lon = self.interface.get_lon_act_state(self.aircraft_obj.lon_actuator_order)
            x_act_lat = self.interface.get_lat_act_state(self.aircraft_obj.lat_actuator_order)
            
            u_trim = self.interface.getControlCmd()

            # x_lon_clean = self.interface.get_lon_state()[:2]
            u_lon_clean = [u_trim[1]]

            try:
                A_lon, B_lon = self.linearize_axis(
                    x_phys_lon, u_lon_clean, 
                    eps_x=[np.deg2rad(0.5), np.deg2rad(2.0)], 
                    eps_u=[0.01], 
                    is_lon=True
                )

                u_lat_clean = [u_trim[2], u_trim[3]]  # [aileron, rudder]
                A_lat, B_lat = self.linearize_axis(
                    x_phys_lat, u_lat_clean, 
                    eps_x=[np.deg2rad(0.5), np.deg2rad(2.0), np.deg2rad(2.0)], 
                    eps_u=[0.01, 0.01], 
                    is_lon=False
                )
            except Exception as e:
                print("="*60)
                print("Error with linearize axis.")
                print(f"Details: {e}")
                print("="*60)

            print('Linearization successful!')

            print('\nLongitudinal linearization:')
            lon_order = self.aircraft_obj.lon_actuator_order
            if lon_order == 0: # No actuator model 
                pass
            elif lon_order == 1: # First order model
                tau = [self.aircraft_obj.tau_e]
                A_p, B_p = first_order_packing(A=A_lon, B=B_lon, tau=tau)
                A_lon, B_lon = add_integrators(A=A_p, B=B_p, int_map=self.aircraft_obj.lon_int_map)
            elif lon_order == 2: # Second order model 
                print('Second order actuator model not yet implemented.')        
            else: # Invalid choice
                print('Invalid choice.\n')
            
            print('\nLateral linearization:')
            lat_order = self.aircraft_obj.lat_actuator_order
            if lat_order == 0: # No actuator model
                pass
            elif lat_order == 1: # First order model
                tau = [self.aircraft_obj.tau_a, self.aircraft_obj.tau_r]
                A_p, B_p = first_order_packing(A=A_lat, B=B_lat, tau=tau)
                A_lat, B_lat = add_integrators(A=A_p, B=B_p, int_map=self.aircraft_obj.lat_int_map)
            elif lat_order == 2:
                print('Second order actuator model not yet implemented.')
            else: # Invalid choice
                print('Invalid actuator model order.\n')

            Helper.display_matrices(A_lon, B_lon, name="Longitudinal System")
            Helper.display_matrices(A_lat, B_lat, name="Lateral System")
            return A_lon, A_lat, B_lon, B_lat, x_phys_lon, x_act_lon, x_phys_lat, x_act_lat, u_trim
        except Exception as e:
            print('\n' + '='*60)
            print('Error linearizing model. Please check aircraft setup and trimming conditions.')
            print(f'Details: {e}')
            print('='*60 + '\n')
            # Return None to indicate a safe, handled failure to the caller
            return None, None, None, None, None, None, None, None, None

    def get_state_derivative(self, target_states, target_controls, is_lon=True):
        """
        Unified helper to get x_dot by perturbing properties without a full IC reset.
        """
        try:
            if is_lon:
                self.fdm.set_property_value("ic/alpha-rad", target_states[0])
                self.fdm.set_property_value("ic/q-rad_sec", target_states[1])
                self.fdm.set_property_value("fcs/elevator-cmd-norm", target_controls[0])
            else:
                self.fdm.set_property_value("ic/beta-rad", target_states[0])
                self.fdm.set_property_value("ic/p-rad_sec", target_states[1])
                self.fdm.set_property_value("ic/r-rad_sec", target_states[2])
                self.fdm.set_property_value("fcs/aileron-cmd-norm", target_controls[0])
                self.fdm.set_property_value("fcs/rudder-cmd-norm", target_controls[1])

            self.fdm.set_property_value("ic/reset-ics", 1)
            self.fdm.run_ic()
            
            start_state = self.interface.get_lon_state() if is_lon else self.interface.get_lat_state()

            dt_window = 0.15 
            steps = int(dt_window / self.fdm.get_delta_t())
            for _ in range(steps):
                self.fdm.run()

            end_state = self.interface.get_lon_state() if is_lon else self.interface.get_lat_state()
            
            if np.any(np.isnan(end_state)) or np.any(np.isinf(end_state)):
                return None

            return (end_state - start_state) / dt_window
        except Exception:
            return None
    
    def secondOrderLatPacking(self, A, B, omega_a, zeta_a, omega_r, zeta_r):
        A_p = np.zeros((13,13))
        B_p = np.zeros((13,2))

        A_p[0,2] = -1.0 # beta integrator state
        A_p[1,3] = -1.0 # p integrator state

        A_p[2:5, 2:5] = A
        A_p[2:5, 5:7] = B

        # Aileron actuator modeling:
        A_a = np.zeros((2,2))
        A_a[0,0] = 0
        A_a[0,1] = 1
        A_a[1,0] = -omega_a**2
        A_a[1,1] = -2*zeta_a*omega_a

        B_a = np.zeros((2,1))
        B_a[0,0] = 0
        B_a[1,0] = omega_a**2

        # Rudder actuator modeling:
        A_r = np.zeros((2,2))
        A_r[0,0] = 0
        A_r[0,1] = 1
        A_r[1,0] = -omega_r**2
        A_r[1,1] = -2*zeta_r*omega_r

        B_r = np.zeros((2,1))
        B_r[0,0] = 0
        B_r[1,0] = omega_r**2

        # These are likely not right
        A_p[7:9, 7:9] = A_a
        A_p[7:9, 9:11] = B_a
        A_p[9:11, 9:11] = A_r
        A_p[9:11, 11:13] = B_r

        # Same for these
        B_p[7:9, 0:1] = B_a
        B_p[9:11, 1:2] = B_r

        return A_p, B_p

    def linearize_axis(self, x_trim_clean, u_trim_axis, eps_x, eps_u, is_lon):
        """
        Generalized numerical linearization for a clean aerodynamic plant.
        x_trim_clean: list or array of pure aero states (e.g., [alpha, q] or [beta, p, r])
        u_trim_axis: list of current trim inputs for this axis (e.g., [elevator] or [aileron, rudder])
        eps_x: list of perturbation steps for states
        eps_u: list of perturbation steps for inputs
        is_lon: bool indicating longitudinal or lateral axis
        """
        nx = len(x_trim_clean)
        nu = len(u_trim_axis)
        
        A = np.zeros((nx, nx))
        B = np.zeros((nx, nu))

        # A Matrix
        for i in range(nx):
            x_plus = list(x_trim_clean)
            x_minus = list(x_trim_clean)
            
            # Perturb state element i
            x_plus[i] += eps_x[i]
            x_minus[i] -= eps_x[i]
            
            dp = self.get_state_derivative(x_plus, u_trim_axis, is_lon=is_lon)
            dm = self.get_state_derivative(x_minus, u_trim_axis, is_lon=is_lon)
            
            if dp is not None and dm is not None:
                A[:, i] = (dp[:nx] - dm[:nx]) / (2 * eps_x[i])
            else:
                print(f"Warning: A matrix calculation failed at state index {i}")
                return None, None

        # B Matrix
        for j in range(nu):
            u_plus = list(u_trim_axis)
            u_minus = list(u_trim_axis)
            
            u_plus[j] += eps_u[j]
            u_minus[j] -= eps_u[j]
            
            dp = self.get_state_derivative(x_trim_clean, u_plus, is_lon=is_lon)
            dm = self.get_state_derivative(x_trim_clean, u_minus, is_lon=is_lon)
            
            if dp is not None and dm is not None:
                B[:, j] = (dp[:nx] - dm[:nx]) / (2 * eps_u[j])
            else:
                print(f"Warning: B matrix calculation failed at input index {j}")
                return None, None

        return A, B
    
def add_integrators(A, B, int_map):
    """
    Augments state space matrices A and B with tracking integrators.
    Matches the layout: z = [integrators, plant_states]
    """
    int_map = np.asarray(int_map, dtype=np.int32)
    active_integrator_mask = int_map > 0
    n_int = np.sum(active_integrator_mask)
    
    n_plant = A.shape[0]
    n_inputs = B.shape[1]
    
    A_int = np.zeros((n_int + n_plant, n_int + n_plant))
    B_int = np.zeros((n_int + n_plant, n_inputs))
    
    A_int[n_int:, n_int:] = A
    B_int[n_int:, :] = B
    
    current_int_row = 0
    
    for plant_idx, integrator_id in enumerate(int_map):
        if integrator_id > 0:
            A_int[current_int_row, n_int + plant_idx] = -1.0
            current_int_row += 1
            
    return A_int, B_int

def first_order_packing(A, B, tau):
    """
    Generic method to add first-order actuators to an LTI system.
    Layout: z = [x_plant; x_actuators]^T
    
    A: numpy array (n_plant x n_plant)
    B: numpy array (n_plant x n_inputs)
    tau: list or numpy array of actuator time constants (length must equal n_inputs)
    """
    l = len(tau)
    n_plant = A.shape[0]
    n_inputs = B.shape[1]
    
    if l != n_inputs:
        raise ValueError(f"Length of tau ({l}) must match number of B matrix inputs ({n_inputs})")

    A_packed = np.zeros((n_plant + l, n_plant + l))
    B_packed = np.zeros((n_plant + l, n_inputs))

    A_packed[0:n_plant, 0:n_plant] = A
    
    A_packed[0:n_plant, n_plant:] = B

    for i in range(l):
        act = 1.0 / tau[i]
        
        A_packed[n_plant + i, n_plant + i] = -act
        
        B_packed[n_plant + i, i] = act

    return A_packed, B_packed

def get_act_info(act_mag, aircraft_type, axis='lat'):
    """
    Returns actuator information based on the type of aircraft, the model order, and the axis
    """
    if aircraft_type == 'f15':
        if act_mag == 1:
            if axis == 'lat':
                return 0.3, 0.4
            elif axis == 'lon':
                return 0.6
        if act_mag == 2:
            if axis == 'lat':
                return 20, 0.70
            if axis == 'lon':
                return 30, 0.70, 25, 0.70
    if aircraft_type == 'f16':
        if act_mag == 1:
            if axis == 'lat':
                return 0.0495
            elif axis == 'lon':
                return 0.0495, 0.0495
        if act_mag == 2:
            if axis == 'lat':
                return 20.2, 0.707
            if axis == 'lon':
                return 20.2, 0.707, 26, 0.72
    else:
        return None