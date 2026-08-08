import numpy as np
import matplotlib.pyplot as plt
import json
from pathlib import Path

from helper import Helper

class FlightPlanner:
    def __init__(self, n_lon_states=2, n_lat_states=3, name=None, sim_dt=1/250, aero_dt=1/100, throttle_dt=1/10, tf=10):
        if n_lon_states < 2 or n_lat_states < 3:
            print('[WARNING] state vectors smaller than minimum size, crash likely imminent.')

        self.name = name
        self.throttle_dt = throttle_dt
        self.aero_dt = aero_dt
        self.sim_dt = sim_dt
        self.tf = tf

        self.alt_init = -1
        self.vel_init = -1
        self.w_init = -1
        
        self.gear_init = 0
        self.flap_init = 0
        self.config_commands = []

        self.n_lon_states = n_lon_states
        self.n_lat_states = n_lat_states

        self.t_vec = np.arange(0, self.tf, self.sim_dt)
        self.commands = []
        self.generate_x_cmd_matrix()
        self.get_config_matrix()
    
    def run(self):
        
        if self.alt_init == -1 and self.vel_init == -1 and self.w_init == -1:
            print('\nWelcome to the Flight Planner!')
            print('Set initial conditions:')
            while True:
                alt_init = input('Initial altitude (ft): ')
                try:
                    alt_init = float(alt_init)
                    break
                except Exception:
                    print('Error: Invalid entry')
            while True:
                vel_init = input('Initial airspeed (kts): ')
                try: 
                    vel_init = float(vel_init)
                    break
                except Exception:
                    print('Error: Invalid entry')
            while True: 
                w_init = input('Initial fuel weight (lbs): ')
                try:
                    w_init = float(w_init)
                    break
                except Exception:
                    print('Error: Invalid entry')
            
            self.alt_init = alt_init
            self.vel_init = vel_init
            self.w_init = w_init

            self.add_step(state='V', t_start=0, t_end=self.tf, magnitude=vel_init)
            
            print('Defaulting to gear up/flaps up.')

        while True:
            print('\nWelcome to the Flight Planner!')
            if self.name is None:
                print('Loaded flight plan: Untitled')
            else:
                print('Loaded flight plan: ' + self.name)
            print('[A]dd command')
            print('[M]odify aircraft configuration')
            print('[P]lot flight plan')
            print('[S]ave flight plan')
            print('[C]hange settings')
            print('[Q]uit Flight Planner')
            choice = input('Enter selection: ').lower()

            if choice == 'q': # Quit to AIMS menu
                choice2 = input('Save before quiting? (y/n) ').lower()
                if choice2 == 'n':
                    return
                else:
                    if self.name == None:
                        filename = input('Enter file name: ')
                    else:
                        filename = self.name
                    self.to_json(filename=filename)
                    return

            elif choice == 's': # Save to JSON
                if self.name == None:
                    filename = input('Enter file name: ')
                else:
                    filename = self.name
                self.to_json(filename=filename)
            
            elif choice == 'p': # Plot flight plan
                print('Close plot to continue...')
                self.plot_cmds()
            
            elif choice == 'c':
                self.change_settings()

            elif choice == 'm':
                while True:
                    print('\nModify aircraft configuration:')
                    print('[1] Gear state')
                    print('[2] Flap state')
                    print('[Q]uit to Flight Planner menu')
                    choice2 = input('Enter selection: ')

                    if choice2 == 'q':
                        break
                    elif choice2 in ['1', '2']:
                        while True:
                            t0 = input('\nInitial time: ')
                            try:
                                t0 = float(t0)
                                break
                            except Exception:
                                print('Error: Invalid selection')
                        while True:
                            tf = input('Terminal time (leave blank for command to continue for the flight duration): ')
                            if tf == '':
                                tf = self.tf
                                break
                            else:
                                try:
                                    tf = float(tf)
                                    break
                                except Exception:
                                    print('Error: Invalid selection')
                        if choice2 == '1': # Gear state
                            while True:
                                gear = input('\nEnter gear state (0 up, 1 down): ')
                                if gear in ['0', '1']:
                                    gear = int(gear)
                                    if t0 == 0:
                                        self.gear_init = gear
                                    self.config_commands.append(ConfigCommand('gear', t0, tf, gear))
                                else:
                                    print('Error: Invalid selection')

                        if choice2 == '2': # Flap state:
                            while True:
                                print('\nFlap positions (normalized):')
                                print('[1] 0.00')
                                print('[2] 0.10')
                                print('[3] 0.20')
                                print('[4] 0.30')
                                print('[5] 0.50')
                                print('[6] 0.75')
                                print('[7] 1.00')
                                choice2 = input('Enter selection: ')

                                match choice2:
                                    case '1':
                                        flap = 0.00
                                        break
                                    case '2':
                                        flap = 0.10
                                        break
                                    case '3':
                                        flap = 0.20
                                        break
                                    case '4':
                                        flap = 0.30
                                        break
                                    case '5':
                                        flap = 0.50
                                        break
                                    case '6':
                                        flap = 0.75
                                        break
                                    case '7':
                                        flap = 1.00
                                        break
                                    case _:
                                        print('Error: Invalid selection')
                            if t0 == 0:
                                self.flap_init = flap
                            self.config_commands.append(ConfigCommand('flap', t0, tf, flap))
                            
                    else:
                        print('Error: Invalid selection')

            elif choice == 'a': # Add command
                while True:
                    print('\nAdding command:')
                    print('[S]tep command')
                    print('[R]amp command')
                    print('S[I]ne command')
                    print('[P]lot flight plan')
                    print('[C]hange settings')
                    print('[Q]uit to Flight Planner menu')
                    choice2 = input('Enter selection: ').lower()

                    if choice2 in ['s', 'r', 'i']: # Add step command
                        while True: # Select variable
                            print('\nState variables: ')
                            print('[1] V (kts)')
                            print('[2] alpha (deg)')
                            print('[3] q (deg/s)')
                            print('[4] beta (deg)')
                            print('[5] p (deg/s)')
                            print('[6] r (deg/s)')
                            state_sel = input('Enter selection: ')
                            match state_sel:
                                case '1':
                                    state = 'V'
                                    break
                                case '2':
                                    state = 'alpha'
                                    break
                                case '3':
                                    state = 'q'
                                    break
                                case '4':
                                    state = 'beta'
                                    break
                                case '5':
                                    state = 'p'
                                    break
                                case '6':
                                    state = 'r'
                                    break
                                case _:
                                    print('Error: Invalid selection')

                        if state in ['alpha', 'beta']: # Used for printouts later
                            units = 'deg'
                        elif state == 'V':
                            units = 'kts'
                        else:
                            units = 'deg/s'
                            
                        while True: # Select t0
                            t0 = input('Command t0: ')
                            try: 
                                t0 = float(t0)
                                break
                            except Exception:
                                print('Error: Invalid entry')
                        while True: # Select tf:
                            tf = input('Command terminal time (leave blank for command to continue for the flight duration): ')
                            if tf == '':
                                tf = self.tf
                            try: 
                                tf = float(tf)
                                break
                            except Exception:
                                print('Error: Invalid entry')

                        if choice2 == 's': # Step command
                            while True: # Select magnitude:                                    
                                mag = input('Step magnitude (' + units + '): ')
                                try: 
                                    mag = float(mag)
                                    break
                                except Exception:
                                    print('Error: Invalid entry')
                            self.add_step(state=state, t_start=t0, t_end=tf, magnitude=mag)

                        elif choice2 == 'r': # Ramp command
                            while True: # Select initial magnitude:
                                init_mag = input('Initial magnitude (' + units + ') (leave blank for current magnitude): ')
                                if init_mag == '': # Find magnitude at t0
                                    init_mag = self.get_mag(state=state,t=t0)
                                try: 
                                    init_mag = float(init_mag)
                                    break
                                except Exception:
                                    print('Error: Invalid entry')
                            while True: # Select terminal magnitude:
                                final_mag = input('Terminal magnitude (' + units + ') (leave blank for current magnitude): ')
                                if final_mag == '': # Find magnitude at tf
                                    final_mag = self.get_mag(state=state,t=tf)
                                try: 
                                    final_mag = float(final_mag)
                                    break
                                except Exception:
                                    print('Error: Invalid entry')
                            self.add_ramp(state=state, t_start=t0, t_end=tf, start_mag=init_mag, end_mag=final_mag)
                        
                        elif choice2 == 'i': # Sine command
                            while True: # Select amplitude
                                amp = input('Command amplitude (' + units + '): ')
                                try:
                                    amp = float(amp)
                                    break
                                except Exception:
                                    print('Error: Invalid entry')
                            while True: # Select frequency
                                freq = input('Command frequency (Hz): ')
                                try:
                                    freq = float(freq)
                                    break
                                except Exception:
                                    print('Error: Invalid entry')
                            self.add_sine(state=state, t_start=t0, t_end=tf, amplitude=amp, freq_hz=freq)
                    elif choice2 == 'q':
                        break
                    elif choice2 == 'p': # Plot flight plan
                        print('Close plot to continue...')
                        self.plot_cmds()
                    
                    elif choice2 == 'c':
                        self.change_settings()
                    else:
                        print('Error: Invalid selection')
                    

    def to_json(self, filename):
        """
        Serializes the flight planner settings and the list of command recipes to a JSON file.
        """
        
        # Ensure the filename ends with the correct extension
        if not filename.lower().endswith('.json'):
            filename += '.json'

        base_dir = Path(__file__).parent
        target_dir = base_dir / "flight plans"

        target_dir.mkdir(parents=True, exist_ok=True)
        full_path = target_dir / filename
            
        # Package up top-level configuration metadata
        export_data = {
            "name": filename.replace('.json', ''),
            "IC": {
                "alt": self.alt_init,
                "vel": self.vel_init,
                "w": self.w_init
            },
            "n_lon_states": self.n_lon_states,
            "n_lat_states": self.n_lat_states,
            "sim_dt": self.sim_dt,
            "aero_dt": self.aero_dt,
            "throttle_dt": self.throttle_dt,
            "tf": self.tf,
            "commands": [],
            "config commands": []
        }
        
        for cmd in self.commands:
            cmd_dict = {
                "type": cmd.__class__.__name__,  # Stores "StepCommand", "RampCommand", etc.
                "target_state": cmd.target_state,
                "t_start": cmd.t_start,
                "t_end": cmd.t_end
            }
            
            if isinstance(cmd, StepCommand):
                cmd_dict["magnitude"] = cmd.magnitude
            elif isinstance(cmd, RampCommand):
                cmd_dict["start_mag"] = cmd.start_mag
                cmd_dict["end_mag"] = cmd.end_mag
            elif isinstance(cmd, SineCommand):
                cmd_dict["amplitude"] = cmd.amplitude
                cmd_dict["freq_hz"] = cmd.freq_rad_sec / (2 * np.pi) # Convert rad/s back to Hz
                cmd_dict["phase_rad"] = cmd.phase_rad
                
            export_data["commands"].append(cmd_dict)

        for cmd in self.config_commands:
            config_cmd_dict = {
                "type": cmd.config_type,
                "t_start": cmd.t_start,
                "t_end": cmd.t_end,
                "value": cmd.value
            }
            export_data["config commands"].append(config_cmd_dict)
            
        # Write data
        try:
            with open(full_path, 'w') as f:
                json.dump(export_data, f, indent=4)
            print(f"[SUCCESS] Flight plan successfully exported to {full_path}")
            self.name = filename.replace('.json', '') # Keep internal track of name
        except Exception as e:
            print(f"Error: Failed to save file due to: {e}")
    
    @classmethod
    def from_json(cls, filename):
        """
        Loads a JSON flight profile file and returns a fully populated, 
        pre-rendered FlightPlanner instance.
        """
        
        if not filename.lower().endswith('.json'):
            filename += '.json'
            
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            planner = cls(
                n_lon_states=data["n_lon_states"],
                n_lat_states=data["n_lat_states"],
                name=data["name"],
                sim_dt=data["sim_dt"],
                aero_dt=data["aero_dt"],
                throttle_dt=data["throttle_dt"],
                tf=data["tf"]
            )

            planner.alt_init = data['IC']['alt']
            planner.vel_init = data['IC']['vel']
            planner.w_init = data['IC']['w']
            
            planner.commands.clear()
            
            for cmd_data in data["commands"]:
                cmd_type = cmd_data["type"]
                target = cmd_data["target_state"]
                t0 = cmd_data["t_start"]
                tf = cmd_data["t_end"]
                
                if cmd_type == "StepCommand":
                    cmd_obj = StepCommand(target, t0, tf, cmd_data["magnitude"])
                elif cmd_type == "RampCommand":
                    cmd_obj = RampCommand(target, t0, tf, cmd_data["start_mag"], cmd_data["end_mag"])
                elif cmd_type == "SineCommand":
                    cmd_obj = SineCommand(target, t0, tf, cmd_data["amplitude"], cmd_data["freq_hz"], cmd_data["phase_rad"])
                else:
                    print(f"[WARNING] Unknown command recipe type skipped: {cmd_type}")
                    continue
                    
                planner.commands.append(cmd_obj)
            
            for config_cmd_data in data.get("config commands", []):
                type = config_cmd_data["type"]
                t0 = config_cmd_data["t_start"]
                tf = config_cmd_data["t_end"]
                mag = config_cmd_data["value"]

                try:
                    config_cmd_obj = ConfigCommand(config_type=type, t_start=t0, t_end=tf, value=mag)
                    planner.config_commands.append(config_cmd_obj)
                except Exception:
                    print(f'[WARNING] Unknown configuration command: {type}')
                
            planner.generate_x_cmd_matrix()
            print(f"[SUCCESS] Loaded flight plan '{planner.name}' from {filename}")
            return planner
            
        except Exception as e:
            print(f"Error: Failed to load file due to: {e}")
            return None

    def generate_x_cmd_matrix(self):
        """Generates/regenerates the entire x_cmd_matrix from the command recipe list."""

        old_matrix = getattr(self, 'x_cmd_matrix', None)
        old_len = len(old_matrix[0]) if old_matrix is not None else 0

        # Re-build continuous time vector safely
        self.t_vec = np.arange(0, self.tf, self.sim_dt)
        new_len = len(self.t_vec)
        self.x_cmd_matrix = np.zeros((1 + self.n_lon_states + self.n_lat_states, new_len))
        
        for step_idx, t in enumerate(self.t_vec):
            for cmd in self.commands:
                if cmd.t_start <= t < cmd.t_end:
                    matrix_row = self.get_state_idx(cmd.target_state)
                    self.x_cmd_matrix[matrix_row, step_idx] = cmd.evaluate(t)

        if old_len > 0 and new_len > old_len:
            last_valid_slice = old_matrix[:, old_len - 1]
            
            for step_idx in range(old_len, new_len):
                t = self.t_vec[step_idx]
                for row_idx in range(self.x_cmd_matrix.shape[0]):
                    has_active_cmd = any(
                        self.get_state_idx(cmd.target_state) == row_idx and cmd.t_start <= t < cmd.t_end
                        for cmd in self.commands
                    )
                    if not has_active_cmd:
                        self.x_cmd_matrix[row_idx, step_idx] = last_valid_slice[row_idx]
                
        print(f"[SUCCESS] Regenerated command matrix tracking grid ({self.x_cmd_matrix.shape})")
        return self.x_cmd_matrix

    def get_config_matrix(self):
        self.t_vec = np.arange(0, self.tf, self.sim_dt)
        config_matrix = np.zeros((2, len(self.t_vec)))
        
        config_matrix[0, :] = self.flap_init
        config_matrix[1, :] = self.gear_init
        
        for step_idx, t in enumerate(self.t_vec):
            for cmd in self.config_commands:
                if cmd.t_start <= t < cmd.t_end:
                    if cmd.config_type == 'gear':
                        config_matrix[0, step_idx] = cmd.value
                    elif cmd.config_type == 'flap':
                        config_matrix[1, step_idx] = cmd.value
                        
        return config_matrix

    def get_mag(self, state, t):
        """
        Returns the commanded magnitude of a given state string (e.g., 'alpha') 
        at a specific continuous timestamp 't' based on the current command recipes.
        Newer commands take priority and overwrite older ones.
        """
        current_val = 0.0
        
        for cmd in self.commands:
            if cmd.target_state == state:
                if cmd.t_start <= t < cmd.t_end:
                    current_val = cmd.evaluate(t)
                    
        return current_val

    def change_settings(self):
        """
        Change settings of flight plan
        """
        while True:
            print('\nCurrent flight plan settings: ')
            print(f'[1] Simulation frequency: {1/self.sim_dt:.3f} Hz')
            print(f'[2] Autothrottle frequency: {1/self.throttle_dt:.3f} Hz')
            print(f'[3] Aerodynamic controller frequency: {1/self.aero_dt:.3f} Hz')
            print(f'[4] tf: {self.tf:.3f} s')
            print('[Q] Quit')
            choice = input('Enter selection: ').lower()

            if choice == 'q':
                break
            elif choice == '1':
                choice2 = input('Enter new simulation frequency: ')
                try:
                    sim_freq = float(choice2)
                    self.sim_dt = 1/sim_freq
                    self.generate_x_cmd_matrix() 
                    print(f'Simulation freq set to {1/self.sim_dt:.3f}')
                except Exception:
                    print('Error: Invalid entry')
            elif choice == '2':
                choice2 = input('Enter new autothrottle frequency: ')
                try:
                    throttle_freq = float(choice2)
                    self.throttle_dt = 1/throttle_freq
                    print(f'Simulation frequency set to {1/self.throttle_dt:.3f}')
                except Exception:
                    print('Error: Invalid entry')
            elif choice == '3':
                choice2 = input('Enter new aerodynamic controller dt: ')
                try:
                    throttle_freq = float(choice2)
                    self.throttle_dt = 1/throttle_freq
                    print(f'Simulation frequency set to {1/self.throttle_dt:.3f}')
                except Exception:
                    print('Error: Invalid entry')
            elif choice == '4':
                choice2 = input('Enter new tf: ')
                try:
                    self.tf = float(choice2)
                    self.generate_x_cmd_matrix()
                    print(f'tf successfully set to {self.tf:.3f}')
                except Exception:
                    print('Error: Invalid entry')
            else:
                print('Error: Invalid entry')


    def plot_cmds(self):
        """
        Plot state commands
        """
        cfg_matrix = self.get_config_matrix()

        fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

        # Subplot 1: Longitudinal states
        axs[0].plot(self.t_vec, Helper.rad2deg(self.x_cmd_matrix[1, :]), label='alpha', color='blue', linewidth=1.5)
        axs[0].plot(self.t_vec, Helper.rad2deg(self.x_cmd_matrix[2, :]), label='q', color='red', linewidth=1.5)
        axs[0].legend(loc='upper right')
        axs[0].grid(True, linestyle=':', alpha=0.6)
        axs[0].set_ylabel('Longitudinal States\n(deg or deg/s)')

        axs[0].set_title('Flight State Commands', fontsize=12, fontweight='bold')        

        # Subplot 2: Lateral states
        axs[1].plot(self.t_vec, Helper.rad2deg(self.x_cmd_matrix[1+self.n_lon_states,:]), label='beta', color='darkorange', linewidth=1.5)
        axs[1].plot(self.t_vec, Helper.rad2deg(self.x_cmd_matrix[2+self.n_lon_states,:]), label='p', color='purple', linewidth=1.5)
        axs[1].plot(self.t_vec, Helper.rad2deg(self.x_cmd_matrix[3+self.n_lon_states,:]), label='r', color='olive', linewidth=1.5)
        axs[1].legend(loc='upper right')
        axs[1].grid(True, linestyle=':', alpha=0.6)
        axs[1].set_ylabel('Lateral States\n(deg or deg/s)')

        # Subplot 3: Velocity
        axs[2].plot(self.t_vec, self.x_cmd_matrix[0,:], label='V', color='steelblue')
        axs[2].set_ylabel('Velocity (kts)')

        # Subplot 4: Configuration states
        axs[3].plot(self.t_vec, cfg_matrix[0, :], label='gear command', color='red', drawstyle='steps-post')
        axs[3].plot(self.t_vec, cfg_matrix[1, :], label='flap command', color='blue', drawstyle='steps-post')
        axs[3].grid(True, linestyle=':', alpha=0.6)
        axs[3].legend(loc='upper right')
        axs[3].set_ylabel('Position / State')
        axs[3].set_xlabel('Time (s)')

        plt.tight_layout()
        plt.show()

    def get_state_idx(self, state):
        match state:
            case 'V':
                return 0
            case 'alpha':
                return 1
            case 'q':
                return 2
            case 'beta':
                return self.n_lon_states + 1
            case 'p':
                return self.n_lon_states + 2
            case 'r':
                return self.n_lon_states + 3
            case _:
                print('Error: Invalid state passed to get_state_idx')
    
    def update_state_dimensions(self, new_n_lon, new_n_lat):
        """
        Migrates the flight plan layout to a brand-new state-space sizing configuration
        (e.g., moving from a 0th-order actuator model to a 2nd-order model).
        """
        print(f"\n[MIGRATION] Restructuring plan grid from Long/Lat ({self.n_lon_states}/{self.n_lat_states}) "
              f"to ({new_n_lon}/{new_n_lat})...")
        
        # Update structural limits
        self.n_lon_states = new_n_lon
        self.n_lat_states = new_n_lat
        
        self.generate_x_cmd_matrix()
    
    def add_step(self, state, t_start, t_end, magnitude):
        if state != 'V': # Convert to radians for non-velocity components
            magnitude =  Helper.deg2rad(magnitude)
        self.commands.append(StepCommand(state, t_start, t_end, magnitude))
        self.generate_x_cmd_matrix() # Re-render instantly

    def add_ramp(self, state, t_start, t_end, start_mag, end_mag):
        if state != 'V':
            start_mag = Helper.deg2rad(start_mag)
            end_mag = Helper.deg2rad(end_mag)
        self.commands.append(RampCommand(state, t_start, t_end, start_mag, end_mag))
        self.generate_x_cmd_matrix()

    def add_sine(self, state, t_start, t_end, amplitude, freq_hz):
        if state != 'V':
            amplitude = Helper.deg2rad(amplitude)
        self.commands.append(SineCommand(state, t_start, t_end, amplitude, freq_hz))
        self.generate_x_cmd_matrix()

    def clear_cmds(self):
        self.commands.clear()
        self.generate_x_cmd_matrix()

class StepCommand:
    def __init__(self, target_state, t_start, t_end, magnitude):
        self.target_state = target_state # Can be string name or state index
        self.t_start = t_start
        self.t_end = t_end
        self.magnitude = magnitude

    def evaluate(self, t):
        """Returns the profile value at any continuous timestamp t."""
        if self.t_start <= t < self.t_end:
            return self.magnitude
        return 0.0
    
class RampCommand:
    def __init__(self, target_state, t_start, t_end, start_mag, end_mag):
        self.target_state = target_state
        self.t_start = t_start
        self.t_end = t_end
        self.start_mag = start_mag
        self.end_mag = end_mag

    def evaluate(self, t):
        if self.t_start <= t < self.t_end:
            # Linear interpolation fraction across the window duration
            fraction = (t - self.t_start) / (self.t_end - self.t_start)
            return self.start_mag + fraction * (self.end_mag - self.start_mag)
        return 0.0

class SineCommand:
    def __init__(self, target_state, t_start, t_end, amplitude, freq_hz, phase_rad=0.0):
        self.target_state = target_state
        self.t_start = t_start
        self.t_end = t_end
        self.amplitude = amplitude
        self.freq_rad_sec = 2 * np.pi * freq_hz
        self.phase_rad = phase_rad

    def evaluate(self, t):
        if self.t_start <= t < self.t_end:
            return self.amplitude * np.sin(self.freq_rad_sec * (t - self.t_start) + self.phase_rad)
        return 0.0

class ConfigCommand:
    def __init__(self, config_type, t_start, t_end, value):
        self.config_type = config_type  # 'gear' or 'flap'
        self.t_start = t_start
        self.t_end = t_end
        self.value = value