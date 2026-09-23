function [P_1, P_2, rho_1, rho_2, T]= fcn(m_1, m_2, m_3, V_1, V_2, T_0_N2, rho_1_0, gamma_N2, R_N2, rho_L)

rho_1 = m_1/V_1; %Find the HP tank gas density
rho_2 = m_2/(V_2 - m_3/rho_L); %Find the ullage gas density
T = T_0_N2*(rho_1/rho_1_0)^(gamma_N2 - 1); %Find the ullage gas temperature (assumiong adiabatic expansion)
P_1 = rho_1*R_N2*T; %Find the HP gas pressure
P_2 = rho_2*R_N2*T; %Find the ullage gas pressure (Tank pressure)