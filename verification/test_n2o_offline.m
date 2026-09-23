function test_n2o_offline()
%TEST_N2O_OFFLINE Unit tests for the N2O tank equilibrium solve and the
% Dyer/NHNE flow solver, pure MATLAB (no Simulink).
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'scripts'), fullfile(here, '..', 'src', 'plant'));
P = ereg_params('N2O');
lut = P.n2o_lut;
g = @(col, T) interp1(lut.T, lut.(col), T);

% --- 1. IC round trip: solver must recover the construction temperature ---
ic = P.n2o_ic;
[Pt, T, m_L, m_V, P_N2, P_sat, rhoL, hL, sL, flag] = ...
    n2o_tank_properties(ic.m_N2O, ic.m_N2, ic.U, P.V_2, lut, P.R_N2, P.cv_N2);
fprintf('IC: T=%.6f K (want %.1f), P_tank=%.3f bar, P_sat=%.3f bar, m_L=%.3f kg, m_V=%.4f kg, flag=%d\n', ...
    T, P.T_amb, Pt / 1e5, P_sat / 1e5, m_L, m_V, flag);
assert(abs(T - P.T_amb) < 1e-6, 'IC temperature not recovered: %.6f', T);
assert(flag == 0, 'IC hit a clamp (flag %d)', flag);
assert(abs(P_N2) < 1, 'P_N2 should be ~0 with no pad');
assert(abs(Pt - ic.P_sat) / ic.P_sat < 1e-9, 'P_tank != P_sat at zero pad');
assert(abs(m_L - g('rho_L', P.T_amb) * P.V_fill) < 1e-6, 'liquid mass wrong');

% --- 2. N2 pad raises pressure by the partial-pressure amount at same T ---
m_pad = 0.05;
U2 = ic.U + m_pad * P.cv_N2 * P.T_amb;   % add N2 carrying its own energy
[Pt2, T2, m_L2, ~, P_N2_2] = ...
    n2o_tank_properties(ic.m_N2O, m_pad, U2, P.V_2, lut, P.R_N2, P.cv_N2);
V_ull = P.V_2 - m_L2 / g('rho_L', T2);
fprintf('pad: T=%.4f K, P_tank=%.3f bar, P_N2=%.3f bar\n', T2, Pt2 / 1e5, P_N2_2 / 1e5);
assert(abs(T2 - P.T_amb) < 1e-3, 'pad at same energy/T shifted T: %.4f', T2);
assert(abs(P_N2_2 - m_pad * P.R_N2 * T2 / V_ull) / P_N2_2 < 1e-6, 'Dalton partial pressure wrong');

% --- 3. Draining liquid with its enthalpy must self-cool the tank ---
dm = 1.0;
U3 = ic.U - dm * g('h_L', P.T_amb);
[Pt3, T3] = n2o_tank_properties(ic.m_N2O - dm, 0, U3, P.V_2, lut, P.R_N2, P.cv_N2);
fprintf('drain 1 kg: T %.2f -> %.4f K, P_sat %.2f -> %.3f bar\n', ...
    P.T_amb, T3, ic.P_sat / 1e5, Pt3 / 1e5);
assert(T3 < P.T_amb, 'no self-cooling after liquid drain');
assert(Pt3 < ic.P_sat, 'pressure did not collapse with cooling');
assert(T3 > P.T_amb - 5, 'implausibly strong cooling: %.2f K', T3);

% --- 4. Dyer flow: supercharged upstream leans SPI; ordering HEM < Dyer < SPI ---
P_up_pa = 55e5;                      % supercharged tank pressure
P3_bar = P_up_pa * 1e-5;             % fully-open run valve limit
[~, ~, m_dyer, ~] = n2o_flow_solver(300e5, P_up_pa, 300, 0, 1e9, ...
    rhoL, hL, sL, P_sat, lut, P.K_v_4, P.Cd_3, P.A_3, P.R_N2, P.Tst, P.rhost, P.Pst);
% independent SPI/HEM references at P3 ~= tank pressure
dP = P_up_pa - 101325;
m_spi = P.Cd_3 * P.A_3 * sqrt(2 * rhoL * dP);
T2s = interp1(lut.P, lut.T, 101325);
x2 = (sL - g('s_L', T2s)) / (g('s_V', T2s) - g('s_L', T2s));
h2 = g('h_L', T2s) + x2 * (g('h_V', T2s) - g('h_L', T2s));
rho2 = 1 / ((1 - x2) / g('rho_L', T2s) + x2 / g('rho_V', T2s));
m_hem = P.Cd_3 * P.A_3 * rho2 * sqrt(2 * max(hL - h2, 0));
fprintf('flow: SPI=%.3f HEM=%.3f Dyer(solved)=%.3f kg/s, x2=%.3f\n', m_spi, m_hem, m_dyer, x2);
assert(x2 > 0 && x2 < 1, 'downstream quality out of range');
assert(m_hem < m_spi, 'HEM should be below SPI here');
assert(m_dyer > m_hem * 0.95 && m_dyer < m_spi * 1.001, 'Dyer outside [HEM, SPI]');
k = sqrt(dP / (P_sat - 101325));
assert(k > 1, 'supercharge should give k>1 (got %.2f)', k);

% --- 5. Gas side unchanged: P_4 between tank and bottle, positive makeup flow ---
[~, P4, ~, m_n2] = n2o_flow_solver(300e5, P_up_pa, 300, 0.05, 1, ...
    rhoL, hL, sL, P_sat, lut, P.K_v_4, P.Cd_3, P.A_3, P.R_N2, P.Tst, P.rhost, P.Pst);
fprintf('gas: P_4=%.2f bar, m_dot_N2=%.4f kg/s\n', P4 / 1e5, m_n2);
assert(m_n2 > 0, 'no N2 makeup flow');
assert(P4 > P_up_pa && P4 < 300e5, 'P_4 not between tank and bottle');

% --- 6. Closed run valve: zero liquid flow ---
[~, ~, m0] = n2o_flow_solver(300e5, P_up_pa, 300, 0, 0, ...
    rhoL, hL, sL, P_sat, lut, P.K_v_4, P.Cd_3, P.A_3, P.R_N2, P.Tst, P.rhost, P.Pst);
assert(abs(m0) < 1e-9, 'liquid flow with closed run valve: %.3g', m0);

fprintf('N2O offline physics test PASSED\n');
end
