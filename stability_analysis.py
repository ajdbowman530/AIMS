import sys

import numpy as np
import control as ct

class StabilityAnalysis:

    # Gain margin
    # Phase margin
    # Overshoot
    # Damping
    # Rise time
    # Settling time
    # Eigenvalues
    # Peak sensitivity
    # Peak cosensitivity

    def __init__ (self, A, B, K=None, L=None):
        self.A = A
        self.B = B
        if K is None:
            K = np.zeros((B.shape[1], A.shape[0]))  # No state feedback
        else:
            self.K = K
        if L is None:
            L = np.zeros((A.shape[0], A.shape[0]))  # No observer
        else:
            self.L = L

        self.A_cl = A - B @ K - L @ A

        # L(s) = K * (sI - A)^-1 * B
        self.sys_loop = ct.ss(self.A, self.B, self.K, 0)

        self.analyze()

    def eigen_analysis(self):
        eigvals, eigvecs = np.linalg.eig(self.A_cl)
        if np.all(np.real(eigvals) > 0):
            print("Warning: System has one or more unstable modes.")
        self.eig = eigvals
        return eigvals, eigvecs

    def get_damping(self):
        if not hasattr(self, 'eig'):
            self.eigen_analysis()
        
        self.zeta = np.zeros(len(self.eig))
        for e in range(0,len(self.eig)):
            if np.imag(self.eig[e]) != 0:
                self.zeta[e] = -np.real(self.eig[e]) / np.abs(self.eig[e])
            else:
                self.zeta[e] = 1.0 if np.real(self.eig[e]) < 0 else 0.0
        return self.zeta
    
    def get_mimo_margins(self):
        omega = np.logspace(-2, 3, 1000)
        
        margins_tuple = ct.disk_margins(self.sys_loop, omega)

        # print(margins_tuple)

        upper_gm = margins_tuple[1]
        self.pm_deg = margins_tuple[2]
        
        self.gm_db = ct.mag2db(upper_gm)
        
        return self.gm_db, self.pm_deg
    
    def step_response_metrics(self, T_end=5.0):
        n_inputs = self.sys_loop.ninputs
        n_outputs = self.sys_loop.noutputs
        identity_fb = np.eye(n_inputs)

        self.sys_cl = ct.feedback(self.sys_loop, identity_fb)
        T, yout = ct.step_response(self.sys_cl, T=T_end)

        # For MIMO (Lateral), yout shape is (n_outputs, n_inputs, len(T))
        # We want to look at the primary diagonal (e.g., Aileron to Roll Rate)
        if n_inputs > 1:
            # Look at Aileron (Input 0) and find which output has the largest move
            # This usually finds Roll Rate (p) automatically
            main_output_idx = np.argmax(np.max(np.abs(yout[:, 0, :]), axis=1))
            resp = yout[main_output_idx, 0, :]
            # print(f"DEBUG: Analyzing Output {main_output_idx} for Input 0")
        else:
            # Longitudinal SISO case
            resp = yout

        # Calculate metrics on the specific response 'resp'
        peak = np.max(resp)
        overshoot = (peak - 1) * 100 if peak > 1 else 0
        
        # Use try/except because 'where' can return empty if the system is too slow or unstable
        try:
            idx_90 = np.where(resp >= 0.9)[0][0]
            idx_10 = np.where(resp >= 0.1)[0][0]
            rise_time = T[idx_90] - T[idx_10]
            
            # Settling time (2% band)
            settling_idx = np.where(np.abs(resp - 1) > 0.02)[0][-1]
            settling_time = T[settling_idx]
        except IndexError:
            rise_time = np.nan
            settling_time = np.nan

        self.overshoot, self.rise_time, self.settling_time = overshoot, rise_time, settling_time
        return overshoot, rise_time, settling_time
    
    def get_sensitivity_metrics(self):
        n_inputs = self.sys_loop.ninputs
        I = np.eye(n_inputs)

        # S = (I + L)^-1
        self.S = ct.feedback(I, self.sys_loop)
        # T = L(I + L)^-1
        self.T = ct.feedback(self.sys_loop, I)

        # Compute frequency response to find the peak (Infinity Norm)
        # This works for both SISO (Lon) and MIMO (Lat)
        mag_s, _, _ = ct.frequency_response(self.S)
        mag_t, _, _ = ct.frequency_response(self.T)

        # Ms is the maximum peak of the sensitivity function
        # For MIMO, we take the max across all input/output pairings
        self.Ms = np.max(mag_s)
        self.Mt = np.max(mag_t)
    
    def analyze(self):
        self.eigen_analysis()
        self.get_damping()
        self.get_mimo_margins()
        self.step_response_metrics()
        self.get_sensitivity_metrics()
    
    def print_metrics(self):
        print(f"Gain Margin: {self.gm_db:.2f} dB")
        print(f"Phase Margin: {self.pm_deg:.2f} degrees")
        print(f"Overshoot: {self.overshoot:.2f} %")
        print(f"Rise Time: {self.rise_time:.2f} s")
        print(f"Settling Time: {self.settling_time:.2f} s")
        print(f"Peak Sensitivity (Ms): {self.Ms:.2f} dB")
        print(f"Peak Cosensitivity (Mt): {self.Mt:.2f} dB")

        self.check_stability()

    def check_stability(self):
        print("\n--- Stability Analysis ---")
        is_good = True

        # Check eigenvalues
        if np.any(np.real(self.eig) >= 0):
            print("Warning: System has one or more unstable modes")
            is_good = False
        
        # Check phase margin
        if self.pm_deg <= 30:
            print("Warning: Phase margin is low (<= 30 deg), system may be fragile.")
            is_good = False
        elif self.pm_deg <= 45:
            print("Caution: Phase margin is moderate (30 < pm <= 45), consider improving robustness.")
            is_good = False
        
        # Check gain margin
        if self.gm_db <= 6:
            print("Warning: Gain margin is low (<= 6 dB), system may be fragile.")
            is_good = False
        elif self.gm_db <= 10:
            print("Caution: Gain margin is moderate (6 < gm <= 10 dB), consider improving robustness.")
            is_good = False

        # Check Peak Sensitivity
        if self.Ms >= 4:
            print("Warning: Peak sensitivity (Ms) is high (>= 4), system may be fragile.")
            is_good = False
        elif self.Ms >= 2:
            print("Caution: Peak sensitivity (Ms) is moderate (2 <= Ms < 4), consider improving robustness.")
            is_good = False
        
        # Check Peak Cosensitivity
        if self.Mt >= 4:
            print("Warning: Peak cosensitivity (Mt) is high (>= 4), system may be fragile.")
            is_good = False
        elif self.Mt >= 2:
            print("Caution: Peak cosensitivity (Mt) is moderate (2 <= Mt < 4), consider improving robustness.")
            is_good = False
        
        if is_good:
            print('No stability issues detected.')

    def get_metrics_dict(self):
        return {
            'gain_margin': self.gm_db,
            'phase_margin': self.pm_deg,
            'overshoot': self.overshoot,
            'rise_time': self.rise_time,
            'settling_time': self.settling_time,
            'peak_sensitivity': self.Ms,
            'peak_cosensitivity': self.Mt,
            'eigs': self.eig,
            'zeta': self.zeta
        }