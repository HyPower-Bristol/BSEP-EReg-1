import CoolProp.CoolProp as CP
import numpy as np
import pandas as pd
import os

def generate_n2o_lut():
    # Define range for Density (Low Gas to High Liquid)
    # N2O Critical Density is approx 452 kg/m3.
    # Liquid density at 298K is approx 700-800 kg/m3.
    # Gas density is much lower.
    # We want a comprehensive range.
    
    # Create a range of densities. 
    # High density (Liquid) down to Low density (Gas)
    rho_min = 1.0     # kg/m^3 (Gas)
    rho_max = 1000.0  # kg/m^3 (Liquid)
    num_points = 1000
    
    densities = np.linspace(rho_min, rho_max, num_points)
    
    # Storage for results
    data = {
        'Density': [],
        'Pressure': [],
        'Temperature': [],
        'Quality': [],
        'InternalEnergy': [],
        'Entropy': []
    }
    
    # Fixed Internal Energy or Entropy? 
    # Actually, for a blowdown, if it's adiabatic, Entropy is constant (Isentropic).
    # If it's slow, maybe isothermal?
    # BUT, the lookup table usually defines the saturation curve.
    # If we are in the two-phase region, T and P are coupled by the saturation curve.
    # The density determines the quality 'x'.
    # rho = 1 / ( (1-x)/rho_L + x/rho_G )
    
    # Let's assume we are tracing the SATURATION curve for the tank.
    # Meaning, for a given average density in the tank, what is the equilibrium P and T?
    # This implies we are strictly ON the saturation dome (Two-Phase).
    # If density > rho_Liquid_Sat(T_amb), we are Compressed Liquid (Single Phase).
    # If density < rho_Gas_Sat(T_amb), we are Superheated Gas (Single Phase).
    # HOWEVER, blowdown usually follows a path.
    
    # Approach:
    # We assume the fluid tracks the saturation curve as long as it's two-phase.
    # The temperature drops as pressure drops.
    # But wait, Density alone doesn't define the state if we don't know the Entropy or Internal Energy.
    # Simulation logic usually tracks Mass (m) and Internal Energy (U).
    # Then rho = m/V, u = U/m. State = f(rho, u).
    
    # SIMPLIFICATION for this script:
    # 1. Create a general table of Property(rho, u) -> P, T.  (Two variable lookup).
    # OR 
    # 2. Assume Isentropic expansion? No, heat transfer matters.
    # OR
    # 3. Assume Saturation Equilibrium at all times?
    # The "Preliminary_N2O_characterisation.py" script used conservation of U.
    # Let's verify what the Matlab script will need. 
    # For a high fidelity sim, we need:
    #   State = (Density, Internal Energy).
    #   Get P, T from (D, U).
    
    # So, we should generate a 2D table? Or just use CoolProp directly in Python?
    # Using CoolProp in Matlab is slow or requires setup.
    # A 1D Lookup is fastest if we assume a path (e.g. Isentropic).
    # But let's try to map the Saturation Curve specifically, as that's the "boiling" pressure.
    
    # Let's generate the Two-Phase Saturation properties vs Temperature.
    # T_min (Triple point) to T_crit.
    T_trip = CP.PropsSI('Tmin', 'N2O')
    T_crit = CP.PropsSI('Tcrit', 'N2O')
    
    T_vals = np.linspace(T_trip + 1, T_crit - 1, 500)
    
    sat_data = {
        'T': [],
        'P': [],
        'rho_L': [],
        'rho_V': [],
        'u_L': [],
        'u_V': [],
        'h_L': [],
        'h_V': [],
        's_L': [],
        's_V': []
    }
    
    for T in T_vals:
        sat_data['T'].append(T)
        sat_data['P'].append(CP.PropsSI('P', 'T', T, 'Q', 0, 'N2O'))
        sat_data['rho_L'].append(CP.PropsSI('D', 'T', T, 'Q', 0, 'N2O'))
        sat_data['rho_V'].append(CP.PropsSI('D', 'T', T, 'Q', 1, 'N2O'))
        sat_data['u_L'].append(CP.PropsSI('U', 'T', T, 'Q', 0, 'N2O'))
        sat_data['u_V'].append(CP.PropsSI('U', 'T', T, 'Q', 1, 'N2O'))
        sat_data['h_L'].append(CP.PropsSI('H', 'T', T, 'Q', 0, 'N2O'))
        sat_data['h_V'].append(CP.PropsSI('H', 'T', T, 'Q', 1, 'N2O'))
        sat_data['s_L'].append(CP.PropsSI('S', 'T', T, 'Q', 0, 'N2O'))
        sat_data['s_V'].append(CP.PropsSI('S', 'T', T, 'Q', 1, 'N2O'))
        
    df = pd.DataFrame(sat_data)
    
    # Save to CSV
    output_path = 'n2o_saturation_properties.csv'
    df.to_csv(output_path, index=False)
    print(f"Generated {output_path} with {len(df)} rows.")

    # Also generate a Superheated Gas table? 
    # Not strictly needed if we assume we stop when liquid runs out, or just use Ideal Gas for pure vapor phase (sanity validation).
    
    # Let's also generate a Compressed Liquid table?
    # Liquid N2O is slightly compressible.
    # But usually we assume incompressible liquid density at T_initial for the 'piston' phase until P_ullage < P_sat.
    
if __name__ == "__main__":
    generate_n2o_lut()
