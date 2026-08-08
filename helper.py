import numpy as np
from ambiance import Atmosphere

class Helper:

    @staticmethod
    def display_matrices(A, B, name="System"):
        """Display a pair of A and B matrices"""
        print(f"\n--- {name} Matrices ---")
        # Suppress small values and set precision for readability
        format_cfg = {
            'precision': 4,
            'suppress_small': True,
            'separator': '  '
        }
        
        print(f"A Matrix ({A.shape[0]}x{A.shape[1]}):")
        print(np.array2string(A, **format_cfg))
        
        print(f"\nB Matrix ({B.shape[0]}x{B.shape[1]}):")
        print(np.array2string(B, **format_cfg))
        print("-" * (len(name) + 20))

    @staticmethod
    def display_matrix(A, name = "Matrix"):
        """Display a matrix"""
        print(f"\n--- {name} ---")
        # Suppress small values and set precision for readability
        format_cfg = {
            'precision': 4,
            'suppress_small': True,
            'separator': '  '
        }
        
        print(f"{name} ({A.shape[0]}x{A.shape[1]}):")
        print(np.array2string(A, **format_cfg))

    @staticmethod
    def deg2rad(degrees):
        """Convert degrees to radians"""
        return degrees * np.pi / 180.0
    
    @staticmethod
    def rad2deg(radians):
        """Convert radians to degrees"""
        return radians * 180.0 / np.pi
    
    @staticmethod
    def calc_q_M(alt, u, T=None): # input alt in ft, u in kts, output q in Pa, M dimensionless
        gamma = 1.4
        R = 287.05
        alt_m = alt * 0.3048
        u_mps = u * 0.514444
        atm = Atmosphere(alt_m)
        
        if T==None:
            T = atm.temperature  # Temperature in Kelvin
        
        a = np.sqrt(gamma * R * T)
        M = float(u_mps / a)

        rho = atm.density  # Air density [kg/m^3]
        q = float(0.5 * rho * u_mps**2)  # Dynamic pressure [Pa] 

        return q, M

