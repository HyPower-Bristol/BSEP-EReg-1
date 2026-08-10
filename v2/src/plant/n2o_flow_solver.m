function [P_3_pa, P_4_pa, m_dot_L, m_dot_N2] = n2o_flow_solver( ...
    P_1_pa, P_2_pa, T_hp, K_v_1, K_v_2, rho_L_now, h_L_now, s_L_now, P_sat_pa, ...
    n2o_lut, K_v_4, Cd_3, A_3, R_N2, Tst, rhost, Pst)
%N2O_FLOW_SOLVER Component flows for the N2O variant.
%
% Gas side: verbatim v1 physics - solve P_4 between the regulator valve
% (choked/non-choked ISA Kv gas flow from the HP bottle) and the check valve
% into the tank ullage.
% Liquid side: solve P_3 between the run valve (Kv liquid flow, live N2O
% density) and a Dyer/NHNE injector orifice to atmosphere:
%   m_SPI = Cd*A*sqrt(2*rho_L*dP)
%   m_HEM = Cd*A*rho_2*sqrt(2*(h_1 - h_2)),  state 2 = isentropic to P_atm
%   m_dot = (k*m_SPI + m_HEM)/(1+k),  k = sqrt((P_3-P_atm)/(P_sat-P_atm))
% Supercharge (P_3 >> P_sat) drives k up -> SPI-dominated, as physics demands.
%#codegen

MAX_ITER = 60;
TOLERANCE = 1e-7;
h = 1e-6;
P_atm_pa = 101325;

P_1 = P_1_pa * 1e-5;
P_2 = P_2_pa * 1e-5;
SG = rho_L_now / 1000.0;

% --- Liquid side: P_3 between run valve and Dyer orifice ---
liquid_error_fun = @(p3) (calc_m_dot_valve(p3, P_2, K_v_2, rho_L_now, SG) - ...
    calc_m_dot_dyer(p3, P_atm_pa, Cd_3, A_3, rho_L_now, h_L_now, s_L_now, P_sat_pa, n2o_lut));
x0_P3 = max(P_2 - 0.5, 1.0);
P_3_bar = newton_solver(liquid_error_fun, x0_P3, MAX_ITER, TOLERANCE, h);
% robustness: bisection fallback if Newton wandered (kinks at the flash boundary)
if ~isfinite(P_3_bar) || abs(liquid_error_fun(P_3_bar)) > 1e-4
    P_3_bar = bisect(liquid_error_fun, P_atm_pa * 1e-5, max(P_2, 1.0) + 1.0, 80);
end

% --- Gas side: P_4 between regulator valve and check valve (v1 verbatim) ---
gas_error_fun = @(p4) (calc_m_dot_c(p4, P_1, K_v_1, rhost, T_hp, Pst, R_N2, Tst) - ...
    calc_m_dot_d(p4, P_2, K_v_4, rhost, T_hp, Pst, R_N2, Tst));
x0_P4 = 1.0;
P_4_bar = newton_solver(gas_error_fun, x0_P4, MAX_ITER, TOLERANCE, h);

P_3_pa = P_3_bar * 1e5;
P_4_pa = P_4_bar * 1e5;
m_dot_L  = calc_m_dot_valve(P_3_bar, P_2, K_v_2, rho_L_now, SG);
m_dot_N2 = calc_m_dot_c(P_4_bar, P_1, K_v_1, rhost, T_hp, Pst, R_N2, Tst);
end

% =========================================================================

function x_root = newton_solver(fun, x0, max_iter, tol, h)
x_root = x0;
for i = 1:max_iter
    f_val = fun(x_root);
    if abs(f_val) < tol
        return;
    end
    f_deriv = (fun(x_root + h) - f_val) / h;
    if abs(f_deriv) < 1e-10
        return;
    end
    x_new = x_root - f_val / f_deriv;
    if abs(x_new - x_root) < tol
        x_root = x_new;
        return;
    end
    x_root = x_new;
end
end

function x = bisect(fun, lo, hi, iters)
flo = fun(lo);
for i = 1:iters
    x = 0.5 * (lo + hi);
    fx = fun(x);
    if sign(fx) == sign(flo)
        lo = x; flo = fx;
    else
        hi = x;
    end
end
x = 0.5 * (lo + hi);
end

function m_dot = calc_m_dot_valve(P_3, P_2, K_v_2, rho_L, SG)
% Run valve, Kv liquid flow (v1 form, live density). Pressures in bar.
delta_P = P_2 - P_3;
Q = K_v_2 * (delta_P / SG) / sqrt(abs(delta_P / SG) + 1e-9);
m_dot = Q * rho_L * (1 / 3600);
end

function m_dot = calc_m_dot_dyer(P_3_bar, P_atm_pa, Cd, A, rho_L, h_1, s_1, P_sat_pa, n2o_lut)
% Dyer/NHNE orifice to atmosphere. Signed-SPI form keeps the solver smooth.
P_3_pa = P_3_bar * 1e5;
dP = P_3_pa - P_atm_pa;
m_spi = Cd * A * 2 * rho_L * dP / sqrt(abs(2 * rho_L * dP) + 1e-9);
if dP <= 0 || P_sat_pa <= P_atm_pa * 1.0001
    m_dot = m_spi;      % backflow or non-flashing regime: SPI only
    return;
end
% HEM branch: isentropic expansion to atmospheric saturation state
T_2 = interp1(n2o_lut.P, n2o_lut.T, P_atm_pa);
s_L2 = interp1(n2o_lut.T, n2o_lut.s_L, T_2);
s_V2 = interp1(n2o_lut.T, n2o_lut.s_V, T_2);
x_2 = (s_1 - s_L2) / (s_V2 - s_L2);
x_2 = min(max(x_2, 0), 1);
h_L2 = interp1(n2o_lut.T, n2o_lut.h_L, T_2);
h_V2 = interp1(n2o_lut.T, n2o_lut.h_V, T_2);
h_2 = h_L2 + x_2 * (h_V2 - h_L2);
rhoL2 = interp1(n2o_lut.T, n2o_lut.rho_L, T_2);
rhoV2 = interp1(n2o_lut.T, n2o_lut.rho_V, T_2);
rho_2 = 1 / ((1 - x_2) / rhoL2 + x_2 / rhoV2);
m_hem = Cd * A * rho_2 * sqrt(2 * max(h_1 - h_2, 0));
k = sqrt(dP / (P_sat_pa - P_atm_pa));
m_dot = (k * m_spi + m_hem) / (1 + k);
end

function m_dot_c = calc_m_dot_c(P_4, P_1, K_v_1, rhost, T, Pst, R_N2, Tst)
% Regulator valve (v1 verbatim). Pressures in bar.
Q_N = 0.0;
if (P_4 >= 0.528 * P_1)
    arg = (P_1 - P_4) * P_4 / (rhost * T);
    Q_N = K_v_1 * 514 * arg / sqrt(abs(arg) + 1e-9);
else
    Q_N = K_v_1 * 257 * P_1 / sqrt(abs(rhost * T) + 1e-9);
end
m_dot_c = Q_N * (Pst / (R_N2 * Tst)) * (1 / 3600);
end

function m_dot_d = calc_m_dot_d(P_4, P_2, K_v_4, rhost, T, Pst, R_N2, Tst)
% Check valve (v1 verbatim). Pressures in bar.
Q_N = 0.0;
if (P_2 >= 0.528 * P_4)
    arg = (P_4 - P_2) * P_2 / (rhost * T);
    Q_N = K_v_4 * 514 * arg / sqrt(abs(arg) + 1e-9);
else
    Q_N = K_v_4 * 257 * P_4 / sqrt(abs(rhost * T) + 1e-9);
end
m_dot_d = Q_N * (Pst / (R_N2 * Tst)) * (1 / 3600);
end
