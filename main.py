import sys
from pathlib import Path
from tuning_manager import TuningManager
from fdm_env import FDMEnv
from flight_planner import FlightPlanner
import numpy as np
import models
import subprocess

# Use PyQt6 to make a GUI eventually

def main():    
    print("=== AIMS: Aircraft Integrated Management Sandbox ===")
    print('Block 0, August 2026\n')
    # choice = input("Launch (G)UI or (H)eadless? [G/h]: ").lower() or 'g'    

    choice = 'h' # force launch headless since there is no GUI

    if choice == 'g':
        print('No GUI yet! Launching headless...')
        run_headless()
        # Eventually use PyQt6 to make GUI
    else:
        run_headless()

    sys.exit()

def run_headless():
    print('Launching headless...')

    aircraft = None
    flight_plan = None
    BASE_DIR = Path(__file__).resolve().parent

    while True:
        print('\n=======AIMS=======')

        if aircraft is None:
            print('No aircraft model read')
        else:
            print(f"Aircraft model '{aircraft.name}' loaded")
        
        if flight_plan is not None:
            print(f"Flight plan {flight_plan.name} loaded")

        print('[1]  Read gains from JSON')
        print('[2]  Launch controller tuning manager')
        print('[3]  Flight Planner')
        print('[4]  Launch FDM Environment')
        print('[5]  Launch Output Visualization')
        print('[Q]  Quit')
        choice = input('Make a selection: ')

        if choice == '1': # load aircraft from JSON file
            folder_path = Path(BASE_DIR/"gain schedules")
            if not folder_path.exists():
                print("Error: 'gain schedules' folder not found. Folder may have been deleted.")
                return

            print('\nDetected .json files:')
            files = [f.name for f in folder_path.iterdir() if f.is_file()]
            
            for f in files:
                if f.endswith('.json') or f.name.endswith('.JSON'):
                    print(f)

            print('------------------------------------------')
            filename = input('Enter JSON filename: ')

            try:
                print('Reading gains from JSON...')
                aircraft = models.Aircraft.from_json(filename)
                print(f"Aircraft '{aircraft.name}' loaded successfully!\n")

                print('Reference states and controls:')
                print('Longitudinal states:', aircraft.ref_lon_states)
                print('Longitudinal controls:', aircraft.ref_lon_controls)
                print('Longitudinal actuator model order:', aircraft.lon_actuator_order)
                print('Longitudinal integrator map: ', aircraft.lon_int_map)
                print('------------------------------------------')
                print('Lateral states:', aircraft.ref_lat_states)
                print('Lateral controls:', aircraft.ref_lat_controls)
                print('Lateral actuator model order:', aircraft.lat_actuator_order)
                print('Lateral integrator map: ', aircraft.lat_int_map)
            except Exception as e:
                print(f"Error loading aircraft model: {e}")

        elif choice == '2': # launch tuning manager
            def select_aircraft():
                print('\nValidated aircraft aerodynamic models: ')
                print('[1] F-15')
                print('[2] F-16A Block-32')
                print('[Q] Quit to main menu')

                while True:
                    choice = input('Select aircraft: ').lower()

                    if choice == '1':
                        return 'f15'
                    elif choice == '2':
                        return 'f16'
                    elif choice == 'q':
                        break
                    else:
                        print('Invalid choice.')

            while True:
                if aircraft is not None:
                    print(f"\nCurrent aircraft: {aircraft.name}")
                    choice = input('[1] Modify current aircraft\n[2] Create new aircraft\n[Q] Return to main menu\nEnter selection: ').lower()
                    if choice == '1':
                        manager = TuningManager(aircraft.type)
                        manager.aircraft_obj = aircraft
                        manager.run()
                        break
                    elif choice == '2':
                        aircraft_str = select_aircraft()
                    else:
                        break
                else:
                    aircraft_str = select_aircraft()
                if aircraft_str is None:
                    break
                manager = TuningManager(aircraft_str.strip())
                if manager.initialize_fdm():
                    print("Model loaded successfully.")
                    manager.run()
                    break
                else:
                    print("Invalid aircraft name. Try again.")

        elif choice == '3': # Flight planner
            while True:
                print('\n[1] Read flight plan from JSON')
                print('[2] Create new flight plan')
                print('[Q] Return to AIMS menu')
                choice2 = input('Enter selection: ').lower()
                
                if choice2 == 'q':
                    break

                elif choice2 == '2': # Create new flight plan
                    if aircraft is not None: # If aircraft is loaded, use length of state vectors in flight planner
                        planner = FlightPlanner(n_lon_states=len(aircraft.ref_lon_states),
                                                n_lat_states=len(aircraft.ref_lat_states))
                    else: # Otherwise default to 0th order actuator model
                        planner = FlightPlanner()
                    planner.run()

                elif choice2 == '1':
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
                        print(f"Reading flight plan from JSON: {filename}...")
                        
                        flight_plan = FlightPlanner.from_json(str(full_file_path))
                        
                        if flight_plan is not None:
                            flight_plan.run()
                        else:
                            print("Error: Flight plan object returned None.")
                    except Exception as e:
                        print(f"Error loading flight plan: {e}")

        elif choice == '4':
            if aircraft is not None:
                flight_env = FDMEnv(aircraft=aircraft, flight_plan=flight_plan)
                flight_env.run()
            else:
                print('No aircraft loaded, returning to main menu...')
        
        elif choice == '5': # Output visualization
            print("\nLaunching Output Visualizer...")
            
            visualizer_script = Path(__file__).resolve().parent / "output_visualizer.py"
            
            subprocess.Popen([sys.executable, str(visualizer_script)])

        elif choice.lower() == 'q': # quit program
            print("Exiting AIMS...")
            sys.exit()

        else: # invalid choice
            print("Invalid choice.")

if __name__ == "__main__":
    main()

