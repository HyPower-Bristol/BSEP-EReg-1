function [P_1, T] = hp_tank_properties(m_1, V_1, T_0_N2, rho_1_0, gamma_N2, R_N2)
%HP_TANK_PROPERTIES HP nitrogen bottle state (adiabatic blowdown, ideal gas).
% Same law as v1's Gas Volume Properties, HP side only.
%#codegen
rho_1 = m_1 / V_1;
T = T_0_N2 * (rho_1 / rho_1_0)^(gamma_N2 - 1);
P_1 = rho_1 * R_N2 * T;
end
