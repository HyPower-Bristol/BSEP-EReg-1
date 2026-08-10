import numpy as np
import matplotlib.pyplot as plt

rho = 785
theta = np.linspace(0.01, 1, 100)
CdA_fuel = 0.62 / np.sqrt(2*rho*10*1e5)
#Cd = CdA / (5*2*np.pi*0.25*(0.0013**2))
CdA_ox = 1.94 / np.sqrt(2*rho*10*1e5)
P_inj = 30*1e5
Isp = 180
CF = 1.39
At = np.pi * 0.02**2

Pc = [0]
m_dot_ox = []
m_dot_fuel = []

for n in range(0, len(theta)):
	Pc_iter = []
	tolerance = True
	iteration = 0

	m_dot_ox_init = theta[n] * CdA_ox * np.sqrt(2*rho*(P_inj - Pc[n]))
	m_dot_fuel_init = theta[n] * CdA_fuel * np.sqrt(2*rho*(P_inj - Pc[n]))
	Pc_init = (Isp * (m_dot_ox_init + m_dot_fuel_init) * 9.81) / (CF * At)
	Pc_iter.append(Pc_init)

	while tolerance:
		m_dot_ox_n = theta[n] * CdA_ox * np.sqrt(2*rho*(P_inj - Pc_iter[iteration]))
		m_dot_fuel_n = theta[n] * CdA_fuel * np.sqrt(2*rho*(P_inj - Pc_iter[iteration]))
		Pc_new = (Isp * (m_dot_ox_n + m_dot_fuel_n) * 9.81) / (CF * At)
		Pc_iter.append(Pc_new)

		iteration += 1

		if abs((Pc_iter[iteration] - Pc_iter[iteration - 1]) / Pc_iter[iteration - 1]) < 0.02:
			Pc.append(Pc_iter[-1])
			m_dot_ox.append(m_dot_ox_n)
			m_dot_fuel.append(m_dot_fuel_n)
			tolerance = False

	print(Pc[-1])



"""
for n in range(0, len(theta)):
	Pc_iter = []
	while abs((Pc[n] - Pc[n-1])/Pc[n-1]) > 0.02:
		m_dot_ox_n = theta[n] * CdA_ox * np.sqrt(2*rho*(P_inj - Pc[n-1]))
		m_dot_fuel_n = theta[n] * CdA_fuel * np.sqrt(2*rho*(P_inj - Pc[n-1]))
		Pc_new = (Isp * (m_dot_ox_n + m_dot_fuel_n) * 9.81) / (CF * At)
		Pc.append(Pc_new)

"""