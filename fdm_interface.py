import jsbsim
import os
import numpy as np

r2d = 180/np.pi
d2r = np.pi/180
fps_to_kts = 0.592484
kts_to_fps = 1/fps_to_kts
kts_to_mps = 0.514444
mps_to_kts = 1/kts_to_mps
ft_to_m = 0.3048
m_to_ft = 1/ft_to_m

class FDMInterface:
    def __init__(self, fdm_exec, control_naming_list = None):
        self.fdm = fdm_exec
        self.property_map = {
            "throttle-cmd-norm": "fcs/throttle-cmd-norm",
            "delta_e_cmd_norm": "fcs/elevator-cmd-norm",
            "delta_a_cmd_norm": "fcs/aileron-cmd-norm",
            "delta_r_cmd_norm": "fcs/rudder-cmd-norm",
            "flap_cmd_norm": "fcs/flap-pos-norm",
            "gear_cmd_norm": "gear/gear-cmd-norm"
        }
        self.active_controls = control_naming_list

        # Initialize engine
        # self.fdm.set_property_value("simulation/do_simple_trim", 0)
        self.fdm.set_property_value("simulation/hold-fc-frozen", 0)
        self.fdm.set_property_value("simulation/freeze-position", 0)
        self.fdm.set_property_value("simulation/terminate", 0)

        for i in range(2): # F-15 has two engines
            self.fdm.set_property_value(f"propulsion/engine[{i}]/set-running", 1)
            self.fdm.set_property_value(f"propulsion/engine[{i}]/filler-rolling", 1) # Some models use this
            self.fdm.set_property_value(f"propulsion/engine[{i}]/n2", 80.0) # Spin up the turbine to 80%
            self.fdm.set_property_value(f"propulsion/engine[{i}]/thrust-lbs", 20000) # Maybe change this number or comment out this line later

    
    def get(self, prop):
        return self.fdm.get_property_value(prop)

    def set(self, prop, value):
        self.fdm.set_property_value(prop, value)

    def set_total_fuel(self, total_lbs):
        per_tank = total_lbs / 2.0
        self.fdm.set_property_value("propulsion/tank[0]/contents-lbs", per_tank)
        self.fdm.set_property_value("propulsion/tank[1]/contents-lbs", per_tank)
        # Force mass property recalculation
        self.fdm.run_ic()
        print(f'Fuel weight: {self.get("propulsion/total-fuel-lbs")} lbs')

    def resetToPhysics(self, x, u):
        # x = np.concatenate(([vt_kts], np.asarray(x_lon).ravel(), np.asarray(x_lat).ravel()))
        # x_lon = [alpha, q]
        # x_lat = [beta, p, r]
        
        # Convert to body fps
        vt_fps = x[0] * kts_to_fps
        alpha_rad = x[1]
        beta_rad = x[3]

        u_vel = vt_fps * np.cos(alpha_rad) * np.cos(beta_rad)
        v_vel = vt_fps * np.sin(beta_rad)
        w_vel = vt_fps * np.sin(alpha_rad) * np.cos(beta_rad)

        p = x[4]
        q = x[2]
        r = x[5]

        self.fdm.set_property_value("velocities/u-fps", u_vel)
        self.fdm.set_property_value("velocities/v-fps", v_vel)
        self.fdm.set_property_value("velocities/w-fps", w_vel)
        self.fdm.set_property_value("velocities/p-rad_sec", p)
        self.fdm.set_property_value("velocities/q-rad_sec", q)
        self.fdm.set_property_value("velocities/r-rad_sec", r)
        self.fdm.set_property_value("attitude/phi-rad", 0)
        self.fdm.set_property_value("attitude/theta-rad", 0)
        
        self.fdm.set_property_value("fcs/throttle-cmd-norm", u[0])
        self.fdm.set_property_value("fcs/elevator-cmd-norm", u[1])
        self.fdm.set_property_value("fcs/elevator-pos-norm", u[1])
        self.fdm.set_property_value("fcs/left-aileron-cmd-norm", u[2])
        self.fdm.set_property_value("fcs/left-aileron-pos-norm", u[2])
        self.fdm.set_property_value("fcs/rudder-cmd-norm", u[3])
        self.fdm.set_property_value("fcs/rudder-pos-norm", u[3])

    def get_lon_state(self, act_order=1):
        x_phys = self.get_lon_phys_state()
        x_act = self.get_lon_act_state(act_order)

        return np.append(x_phys, x_act)
    
    def get_lon_phys_state(self):
        alpha = self.get("aero/alpha-rad")
        q = self.get("velocities/q-rad_sec")

        return np.array([alpha, q])
    
    def get_lon_act_state(self, act_order=1):
        delta_e = self.get('fcs/elevator-pos-norm')
        if act_order == 0:
            return []
        elif act_order == 1:
            return [delta_e]
        elif act_order == 2:
            # I do not know how I will calculate the actuator rate here
            return [delta_e, 0]
        else:
            print('Error: Invalid actuator order in get_lon_act_state')
    
    def get_lat_state(self, act_order=1):
        x_phys = self.get_lat_phys_state()
        x_act = self.get_lat_act_state(act_order)
        return np.append(x_phys, x_act)
    
    def get_lat_phys_state(self):
        beta = self.get("aero/beta-rad")
        p = self.get("velocities/p-rad_sec")
        r = self.get("velocities/r-rad_sec")

        return np.array([beta, p, r])

    def get_lat_act_state(self, act_order=1):
        delta_a = self.fdm.get_property_value('fcs/aileron-pos-norm')
        delta_r = self.fdm.get_property_value('fcs/rudder-pos-norm')
        if act_order == 0:
            return []
        elif act_order == 1:
            return [delta_a, delta_r]
        elif act_order == 2:
            # I do not know how I will calculate the actuator rates here.
            return [delta_a, 0, delta_r, 0]
        else:
            print('Error: Invalid actuator order in get_lat_act_state')

    def get_state(self):
        """
        Returns JSBSim aerodynamic states of aircraft, excluding actuator states.
        x = [v, alpha, q, beta, p, r]
        """
        # Velocities
        u_vel = self.fdm.get_property_value("velocities/u-fps") * fps_to_kts
        v_vel = self.fdm.get_property_value("velocities/v-fps") * fps_to_kts
        w_vel = self.fdm.get_property_value("velocities/w-fps") * fps_to_kts

        vt_kts = np.sqrt(u_vel**2 + v_vel**2 + w_vel**2)

        x_lon = self.get_lon_phys_state()
        x_lat = self.get_lat_phys_state()

        x = np.concatenate(([vt_kts], np.asarray(x_lon).ravel(), np.asarray(x_lat).ravel()))
        return x
    
    def get_complete_state(self, lat_act_order=1, lon_act_order=1):
        """
        Returns JSBSim aircraft states including actuator states.
        x = [v, alpha, q, [lon act], beta, p, r, [lat act]]
        """
        # Velocities
        u_vel = self.get("velocities/u-fps") * fps_to_kts
        v_vel = self.get("velocities/v-fps") * fps_to_kts
        w_vel = self.get("velocities/w-fps") * fps_to_kts

        vt_kts = np.sqrt(u_vel**2 + v_vel**2 + w_vel**2)

        x_lon = self.get_lon_state(act_order=lon_act_order)
        x_lat = self.get_lat_state(act_order=lat_act_order)

        x = np.concatenate(([vt_kts], np.asarray(x_lon).ravel(), np.asarray(x_lat).ravel()))
        return x
    
    def setControls(self, u):
        throttle_cmd = u[0]

        self.set("fcs/throttle-cmd-norm[0]", throttle_cmd)
        self.set("fcs/throttle-cmd-norm[1]", throttle_cmd)

        self.fdm.set_property_value("fcs/elevator-cmd-norm", u[1])
        self.fdm.set_property_value("fcs/aileron-cmd-norm", u[2])
        self.fdm.set_property_value("fcs/rudder-cmd-norm", u[3])

    def set_controls_vec(self, u_vec):
        # Dynamically loop through whatever controls this specific JSON file declared
        for i, control_name in enumerate(self.active_controls):
            jsbsim_property = self.property_map[control_name]
            self.fdm.set_property_value(jsbsim_property, u_vec[i])

    def getControlCmd(self):
        u_trim = np.array([
        self.fdm.get_property_value("fcs/throttle-cmd-norm"),
        self.fdm.get_property_value("fcs/elevator-pos-norm"),
        self.fdm.get_property_value("fcs/aileron-pos-norm"),
        self.fdm.get_property_value("fcs/rudder-pos-norm")
        ])
        return u_trim
    
    def get_control_vals(self):
        u_act = np.array([
            self.get('fcs/throttle-cmd-norm'),
            self.get('fcs/elevator-pos-norm'),
            self.get('fcs/left-aileron-pos-norm'),
            self.get('fcs/rudder-pos-norm')
        ])
        return u_act
    
    def set_FDM_dt(self, dt):
        self.fdm.set_dt(dt)

    def JSBSimInitalize(self, alt, V, flap=0, gear=0):
        dt = self.fdm.get_delta_t()

        self.fdm.reset_to_initial_conditions(0)
        self.fdm.set_dt(dt)
        
        # Set initial altitude and velocity
        self.fdm.set_property_value("ic/h-sl-ft", alt)
        self.fdm.set_property_value("ic/vt-kts", V)

        self.fdm.set_property_value("ic/psi-true-rad", 0.0)
        self.fdm.set_property_value("ic/phi-rad", 0.0)

        self.fdm.set_property_value("ic/p-rad_sec", 0.0)
        self.fdm.set_property_value("ic/q-rad_sec", 0.0)
        self.fdm.set_property_value("ic/r-rad_sec", 0.0)

        self.fdm.run_ic()

        self.fdm.set_property_value("simulation/do_simple_trim", 0)
        self.fdm.set_property_value("simulation/hold-fc-frozen", 0)
        self.fdm.set_property_value("simulation/freeze-position", 0)
        self.fdm.set_property_value("simulation/terminate", 0)

        self.fdm.set_property_value("fcs/throttle-cmd-norm[0]", 0.5)
        self.fdm.set_property_value("fcs/throttle-cmd-norm[1]", 0.5)

        self.set("gear/gear-pos-norm", gear)
        self.set("gear/gear-cmd-norm", gear)

        self.set("fcs/flap-pos-norm", flap)
        self.set("fcs/flap-cmd-norm", flap)

        try:
            self.fdm.do_trim(1) 
        except Exception as e:
            print(f"Full trim failed: {e}. Trying longitudinal trim...")
            try:
                self.fdm.do_trim(0)
            except Exception as e_sub:
                raise RuntimeError(f"JSBSim Trimmer completely failed to converge: {e_sub}")
        
        for i in range(2):
            self.fdm.set_property_value(f"propulsion/engine[{i}]/set-running", 1)
            self.fdm.set_property_value(f"propulsion/engine[{i}]/n2", 80.0)

    def fdm_step(self):
        self.fdm.run()

    def get_cond(self):
        # return np.array of [Mach, q, W]
        Mach = self.get('velocities/mach')
        q = self.get('aero/qbar-psf') * 47.880259
        W = self.get('propulsion/total-fuel-lbs')
        flap = self.get('fcs/flap-pos-norm')
        
        gear_raw = self.get('gear/gear-pos-norm')
        if gear_raw < 0.5:
            gear = 0
        else:
            gear = 1

        return np.array([Mach, q, W, flap, gear])
    
    def set_config(self, config):
        """
        Set flap and gear position given 1D config array
        config = [flap, gear]
        """
        self.set("fcs/flap-pos-norm", config[0])
        self.set("gear/gear-cmd-norm", config[1])

    # Make a replay log for position, angles, angular rates, and control inputs
    # Make a replay log for engineering data (accelerations, KE, PE, etc.))