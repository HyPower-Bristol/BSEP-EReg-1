function [P_tank_pa, T_tank, m_L, m_V, P_N2_pa, P_sat_pa, rho_L_now, h_L_now, s_L_now, flag] = ...
    n2o_tank_properties(m_N2O, m_N2, U_total, V_2, n2o_lut, R_N2, cv_N2)
%N2O_TANK_PROPERTIES Equilibrium two-phase N2O tank with N2 supercharge.
%
% State: total N2O mass, N2 pad mass, total internal energy (adiabatic walls).
% Assumes full thermodynamic equilibrium at one temperature T: saturated N2O
% liquid + vapor sharing the ullage with ideal-gas N2 (Dalton). Solves T by
% bisection on the energy closure:
%   m_V(T)  = rho_V*(V - m_N2O/rho_L)/(1 - rho_V/rho_L)   (volume closure)
%   U_calc(T) = m_L*u_L + m_V*u_V + m_N2*cv_N2*T           (energy closure)
% U_calc is monotone increasing in T over the saturation dome.
% flag: 0 ok, 1 phase-boundary clamp hit, 2 T pinned at LUT range edge.
%#codegen

Tlo = n2o_lut.T(1) + 0.25;
Thi = n2o_lut.T(end) - 0.25;
flag = 0;

for i = 1:55
    Tm = 0.5 * (Tlo + Thi);
    if u_closure(Tm, m_N2O, m_N2, V_2, n2o_lut, cv_N2) > U_total
        Thi = Tm;
    else
        Tlo = Tm;
    end
end
T_tank = 0.5 * (Tlo + Thi);
if T_tank < n2o_lut.T(1) + 0.5 || T_tank > n2o_lut.T(end) - 0.5
    flag = 2;
end

[~, m_V, m_L, clamped] = u_closure(T_tank, m_N2O, m_N2, V_2, n2o_lut, cv_N2);
if clamped
    flag = max(flag, 1);
end

rho_L_now = interp1(n2o_lut.T, n2o_lut.rho_L, T_tank);
P_sat_pa  = interp1(n2o_lut.T, n2o_lut.P, T_tank);
h_L_now   = interp1(n2o_lut.T, n2o_lut.h_L, T_tank);
s_L_now   = interp1(n2o_lut.T, n2o_lut.s_L, T_tank);

V_ull = V_2 - m_L / rho_L_now;
V_ull = max(V_ull, 1e-9);
P_N2_pa = m_N2 * R_N2 * T_tank / V_ull;
P_tank_pa = P_sat_pa + P_N2_pa;
end

function [Ucalc, m_V, m_L, clamped] = u_closure(T, m_N2O, m_N2, V_2, n2o_lut, cv_N2)
rhoL = interp1(n2o_lut.T, n2o_lut.rho_L, T);
rhoV = interp1(n2o_lut.T, n2o_lut.rho_V, T);
m_V_raw = rhoV * (V_2 - m_N2O / rhoL) / (1 - rhoV / rhoL);
m_V = min(max(m_V_raw, 0), m_N2O);
clamped = (m_V ~= m_V_raw);
m_L = m_N2O - m_V;
Ucalc = m_L * interp1(n2o_lut.T, n2o_lut.u_L, T) + m_V * interp1(n2o_lut.T, n2o_lut.u_V, T) ...
    + m_N2 * cv_N2 * T;
end
