"""
Original author by: Oscar Kail
Currently edited by: Umei Nambio
Last edit: 10/01/2025
Status: In progress
"""

from CoolProp.CoolProp import PropsSI as p
from scipy.optimize import least_squares
import numpy as np
import math
import matplotlib.pyplot as plt

"""
Inputs:
- Tank volume
- Temperature of contents
- Pressure (nitrogen)

Outputs:
- Vapour quality (0 for saturated liquid to 1 for saturated vapour)
- Tank mass (for the input volume)

Notes:
- Relatively simple to solve if total N2O mass is known

Principal equations:

x = ((V_tank/m_tot) - 1/rho_liq) / (1/rho_vap - 1/rho_liq)
x = ((U_tot/m_tot) - u_liq) / (u_vap - u_liq)
V_tank = m_tot*((1 - x)/rho_liq + x/rho_vap)
U_tot = p('U', 'T', T_ambient, 'Q', x, 'N2O') * m_tot
"""

T_ambient = 298		# Assumed ambient temperature (K)
V_tank = 30 * 1e-3	# Assumed N2O tank volume (m3)
P_tank = 58 * 1e5	# Set tank pressure
m_tot = 12			# Assumed total N2O mass

T_vapour = p('T', 'P', P_tank, 'Q', 1, 'N2O')
P_vapour = p('P', 'T', T_ambient, 'Q', 1, 'N2O')

if P_vapour >= P_tank:
	print("Self pressurised system.")


# Static N2O properties

rho_vap = p('D', 'T', T_ambient, 'Q', 1, 'N2O')
rho_liq = p('D', 'T', T_ambient, 'Q', 0, 'N2O')
u_vap = p('U', 'T', T_ambient, 'Q', 1, 'N2O')
u_liq = p('U', 'T', T_ambient, 'Q', 0, 'N2O')

x_vals = np.linspace(0, 1, 101)
x_solutions = []
m_tot_solutions = []

def fraction_equations(variables):
    """
    Defines the equations required to be solved
    """

    x, m_tot = variables


    eqn_1 = (m_tot * ((1 - x)/rho_liq + x/rho_vap)) - V_tank
    U_tot = p('U', 'T', T_ambient, 'Q', x, 'N2O') * m_tot
    eqn_2 = U_tot - (m_tot * (x * (u_vap - u_liq) + u_liq))
    
    return [eqn_1, eqn_2]


initial_guess = [0.3, 10]

##x_solution, m_tot_solution = fsolve(fraction_equations, initial_guess)
##print(x_solution)
##print(m_tot_solution)

for x in x_vals:
	guess = [x, 10] 
	
	try:
		res = least_squares(fraction_equations, guess, bounds = ((0, 0), (1, 12))) 
		x_solution, m_tot_solution = res.x
	except ValueError:
		x_solution = -0.5
		m_tot_solution = 5
		

	x_solutions.append(x_solution)
	m_tot_solutions.append(m_tot_solution)
	
print(x_vals)
print(m_tot_solutions)

plt.plot(x_vals, x_solutions)
plt.xlabel("Starting vapour quality")
plt.ylabel("Final vapour quality (after calculations)")
plt.show()
plt.plot(x_vals, m_tot_solutions)
plt.xlabel("Vapour quality")
plt.ylabel("Total tank mass (kg)")
plt.show()
