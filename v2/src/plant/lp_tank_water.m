function P_2 = lp_tank_water(m_2, m_3, T, V_2, rho_L, R_N2)
%LP_TANK_WATER Incompressible-liquid LP tank ullage pressure (v1 law, LP side
% only - the HP side lives in hp_tank_properties). T is the shared lumped
% gas temperature from the HP bottle, exactly as v1 assumed.
%#codegen
rho_2 = m_2 / (V_2 - m_3 / rho_L);
P_2 = rho_2 * R_N2 * T;
end
