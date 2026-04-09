"""
Written by: Oscar Kail
Last edit: 21/12/2024
Status: In progress
"""

from CoolProp.CoolProp import PropsSI as p
from scipy.optimize import fsolve
import numpy as np
import matplotlib.pyplot as plt

"""
Inputs:
- Tank volume
- Temperature of contents
- Pressure (nitrogen)

Outputs:
- Volume / mass fraction

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

x_vals = np.linspace(0, 1, 100)
x_solutions = []
m_tot_solutions = []

def fraction_equations(variables):
	"""
	Defines the equations required to be solved
	"""

	x, m_tot = variables

	eqn_1 = m_tot * ((1 - x)/rho_liq + x/rho_vap) - V_tank
	U_tot = p('U', 'T', T_ambient, 'Q', x, 'N2O') * m_tot
	eqn_2 = U_tot - m_tot * (x * (u_vap - u_liq) + u_liq)

	return [eqn_1, eqn_2]

initial_guess = [0.3, 10]

x_solution, m_tot_solution = fsolve(fraction_equations, initial_guess)
print(x_solution)
print(m_tot_solution)

for x in x_vals:
	guess = [x, 10]
	try:
		x_solution, m_tot_solution = fsolve(fraction_equations, guess)
		x_solutions.append(x_solution)
		m_tot_solutions.append(m_tot_solution)
	except ValueError:
		x_solution = 0
		m_tot_solution = 5


plt.plot(x_vals, x_solutions)
plt.show()
plt.plot(x_vals, m_tot_solutions)
plt.show()



