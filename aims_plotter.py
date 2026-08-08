import matplotlib.pyplot as plt
import control as ct
import numpy as np

class AIMSPlotter:

    @classmethod
    def stability_plots(cls, A, B, K=None, axis=""):
        while True:
            choice = input('\nDisplay system plots:\n[1] Eigenvalue plot\n[2] Step response\n[3] Bode plot\n[4] Nyquist plot\n[5] Singular values plot\nEnter any other value to continue: ')
            match choice:
                case '1': # Eigvenalue plot
                    print('Close plot window to continue...')
                    A_cl = A - B @ K
                    cls.plot_eigs(A_cl, A, title=f"{axis} Eigenvalues")
                case '2': # Step response
                    print('Close plot window to continue...')
                    A_cl = A - B @ K
                    C_cl = np.eye(A.shape[0]) # Full state output
                    D_cl = np.zeros((A.shape[0], B.shape[1]))
                    cls.plot_step_response(A_cl, B, C_cl, D_cl, title=f"{axis} Closed-Loop Step Response")
                case '3': # Bode plot
                    print('Close plot window to continue...')
                    cls.plot_bode(A, B, K, title=f"{axis} Bode Plot")
                    print('Note that due to how actuator dynamics are modeled, gain and phase margins noted on the Bode plot may not be meaningful. Refer to printed stability metrics for gain and phase margins.')
                case '4': # Nyquist plot
                    print('Close plot window to continue...')
                    cls.plot_mimo_nyquist(A, B, K, title=f"{axis} MIMO Nyquist Plot")
                case '5': # Singular values plot
                    print('Close plot window to continue...')
                    cls.plot_mimo_singular_values(A, B, K, title=f"{axis} Singular Values")
                case _: # Continue
                    break
                
    def plot_eigs(A_cl, A_ol=None, title="Eigenvalues"):
        """Plots the eigenvalues of the closed-loop system, optionally overlaying open-loop poles."""
        eigs_cl = np.linalg.eigvals(A_cl)
        plt.figure(figsize=(6,6))
        plt.axhline(0, color='black', linewidth=1, alpha=0.5)
        plt.axvline(0, color='black', linewidth=1, alpha=0.5)
        plt.scatter(eigs_cl.real, eigs_cl.imag, color='red', marker='x', s=100, label='Closed-loop Poles')

        if A_ol is not None:
            eigs_ol = np.linalg.eigvals(A_ol)
            plt.scatter(eigs_ol.real, eigs_ol.imag, color='blue', marker='o', s=100, label='Open-loop Poles')
        
        plt.title(title)
        plt.xlabel("Real")
        plt.ylabel("Imaginary")
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.show()

    @staticmethod
    def plot_step_response(A_cl, B_cl, C_cl, D_cl, title="Closed-Loop Step Response"):
        """Generates a transient time-domain step response plot supporting both Lon and Lat."""
        sys_cl = ct.ss(A_cl, B_cl, C_cl, D_cl)
        
        plt.figure(figsize=(7, 4))
        
        t, y = ct.step_response(sys_cl, T=np.linspace(0, 5, 500))
        
        if y.ndim == 3:
            y_matrix = y[:, 0, :]
        else:
            y_matrix = y
            
        num_states = y_matrix.shape[0]
        
        lon_labels_4state = [
            "Integrator (q_err)",
            "alpha_rad",
            "q_rad_sec",
            "delta_e_norm"
        ]
        
        lat_labels_7state = [
            "Integrator (beta_err)",
            "Integrator (p_err)",
            "beta_rad",
            "p_rad_sec",
            "r_rad_sec",
            "delta_a_norm",
            "delta_r_norm"
        ]
        
        for i in range(num_states):
            if num_states == 4:
                label_text = lon_labels_4state[i]
            elif num_states == 7:
                label_text = lat_labels_7state[i]
            else:
                label_text = f"State {i}"
                
            plt.plot(t, y_matrix[i, :], label=label_text, lw=2)

        plt.title(title)
        plt.xlabel("Time (seconds)")
        plt.ylabel("Response Amplitude")
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend(loc="upper right")

        plt.show()

    @staticmethod
    def plot_mimo_nyquist(A, B, K, title=""):
        num_inputs = B.shape[1]
        D_loop = np.zeros((num_inputs, num_inputs))
        sys_loop = ct.ss(A, B, K, D_loop)
        
        plt.figure(figsize=(6, 6)) 
        
        if num_inputs == 1: # SISO plot
            ct.nyquist_plot(sys_loop, indent_radius=1e-4, indent_direction='right')
            
            # Focus near critical point
            plt.xlim([-2.5, 1.5])
            plt.ylim([-2.0, 2.0])
            plt.plot(-1, 0, marker='o', color='crimson', markersize=8, label='Critical Point (-1,0)')
            plt.grid(True, which='both', linestyle=':', alpha=0.6)
            plt.legend(loc="upper right")
            plt.suptitle(title)
            plt.show()
        else: # MIMO plot
            omega = np.logspace(-2, 3, 1000)
            response = ct.frequency_response(sys_loop, omega)
            
            H_jw = response.magnitude * np.exp(1j * response.phase)
            
            loci_1 = []
            loci_2 = []
            for i in range(len(omega)):
                eigs = np.linalg.eigvals(H_jw[:, :, i])
                loci_1.append(eigs[0])
                if len(eigs) > 1:
                    loci_2.append(eigs[1])
            
            loci_1 = np.array(loci_1)
            loci_2 = np.array(loci_2)
            
            plt.plot(loci_1.real, loci_1.imag, label='Characteristic Locus $\lambda_1(L)$', color='blue', lw=1.5)
            if len(loci_2) > 0:
                plt.plot(loci_2.real, loci_2.imag, label='Characteristic Locus $\lambda_2(L)$', color='cyan', lw=1.5)
                
            plt.plot(-1, 0, marker='o', color='crimson', markersize=8, label='Critical Point (-1,0)')
            plt.xlim([-3.0, 2.0])
            plt.ylim([-2.5, 2.5])
            plt.axhline(0, color='black', lw=0.7)
            plt.axvline(0, color='black', lw=0.7)
            plt.grid(True, which='both', linestyle=':', alpha=0.6)
            plt.legend(loc="upper right")
            plt.suptitle(f"{title} (Characteristic Loci)")
            plt.show()

    @staticmethod
    def plot_bode(A, B, K, title=""):
        num_inputs = B.shape[1]
        D_loop = np.zeros((num_inputs, num_inputs))
        sys_loop = ct.ss(A, B, K, D_loop)
        
        plt.figure()
        ct.bode_plot(sys_loop, dB=True, display_margins=False)
            
        plt.suptitle(title)
        plt.show()

    @staticmethod
    def plot_mimo_singular_values(A, B, K, omega_vector=None, title="MIMO Sensitivity & Complementary Sensitivity"):
        """
        Generates a robust control Sigma Plot showing the maximum singular values of
        the Input Sensitivity S_u(jw) and Co-sensitivity T_u(jw) matrices.
        Aligns analysis with Lavretsky & Wise robust aerospace design standards.
        """
        if omega_vector is None:
            omega_vector = np.logspace(-2, 3, 1000)
            
        num_states = A.shape[0]
        num_inputs = B.shape[1]
        
        max_sigma_S = []
        max_sigma_T = []
        
        I_state = np.eye(num_states)
        I_input = np.eye(num_inputs)

        for omega in omega_vector:
            # Open loop response L_u(jw) = K * (jw*I - A)^-1 * B
            sI_minus_A = (1j * omega * I_state) - A
            state_resp = np.linalg.solve(sI_minus_A, B)
            L_jw = K @ state_resp
            
            # Sensitivity S_u = (I + L_u)^-1
            S_jw = np.linalg.inv(I_input + L_jw)
            
            # Co-sensitivity T_u = L_u * (I + L_u)^-1
            T_jw = L_jw @ S_jw
            
            sv_S = np.linalg.svd(S_jw, compute_uv=False)
            sv_T = np.linalg.svd(T_jw, compute_uv=False)
            
            max_sigma_S.append(sv_S[0])   # \bar{\sigma}(S_u)
            max_sigma_T.append(sv_T[0])  # \bar{\sigma}(T_u)
            
        max_sigma_S = np.array(max_sigma_S)
        max_sigma_T = np.array(max_sigma_T)
        
        # Convert to Db
        max_S_db = 20 * np.log10(max_sigma_S)
        max_T_db = 20 * np.log10(max_sigma_T)
        
        plt.figure(figsize=(10, 6))
        
        plt.semilogx(omega_vector, max_S_db, label=r'Sensitivity $\bar{\sigma}(S_u)$', color='blue', lw=2)
        plt.semilogx(omega_vector, max_T_db, label=r'Co-snsitivity $\bar{\sigma}(T_u)$', color='magenta', lw=2)
        
        plt.axhline(0, color='black', linestyle='-', lw=0.8)
        plt.axhline(6, color='red', linestyle='--', lw=1, label='Typical Peak Threshold (+6 dB)')
        
        plt.title(title)
        plt.xlabel("Frequency (rad/sec)")
        plt.ylabel("Gain (dB)")
        plt.grid(True, which="both", linestyle=":", alpha=0.6)
        plt.legend(loc="upper right")
        plt.show()