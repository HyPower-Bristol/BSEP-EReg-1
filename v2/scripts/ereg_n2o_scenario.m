function P = ereg_n2o_scenario(P)
%EREG_N2O_SCENARIO Flight-scale N2O overrides + saturation LUT + tank ICs.
% Self-pressurized nitrous (P_sat ~ 41 bar at 288 K) supercharged by the EReg
% to a 55 bar setpoint from a 300 bar HP bottle - the report's flight regime.
% The water-rig numbers are meaningless for N2O, hence the overrides.
here = fileparts(mfilename('fullpath'));
csvfile = fullfile(here, '..', '..', 'n2o_archive', 'n2o_saturation_properties.csv');
M = readmatrix(csvfile);
assert(size(M, 2) == 10, 'unexpected LUT format in %s', csvfile);
P.n2o_lut = struct( ...
    'T', M(:, 1), 'P', M(:, 2), 'rho_L', M(:, 3), 'rho_V', M(:, 4), ...
    'u_L', M(:, 5), 'u_V', M(:, 6), 'h_L', M(:, 7), 'h_V', M(:, 8), ...
    's_L', M(:, 9), 's_V', M(:, 10));

% scenario (report: inlet 120-300 bar, outlet 30-55 bar)
P.P_1_0 = 300 * 1e5;      % HP bottle [Pa]
P.Target_Pressure = 55;   % [bar]
P.T_amb = 288;            % initial tank temperature [K]
P.A_3 = 20e-6;            % injector area resized for ~1.3 kg/s N2O (fits the fill)

% N2 caloric constants (ideal, consistent u=cv*T / h=cp*T reference)
P.cv_N2 = P.R_N2 / (P.gamma_N2 - 1);
P.cp_N2 = P.gamma_N2 * P.R_N2 / (P.gamma_N2 - 1);

% controller: report's flight gains as starting point (error in bar);
% flight gain schedule slope 2.5 (report Fig. 7), not the water rig's 5
P.ctrl.run     = struct('K_P', 16, 'K_I', 17, 'K_D', 8, 'N', 100);
% dead-head pad: 300 bar bottle into 3 L ullage needs tiny valve angles
% (valve deadband ends ~5 deg; Kv(6 deg) already flows ~0.04 kg/s) -> small K_P
P.ctrl.press   = struct('K_P', 0.5, 'K_I', 0.3, 'K_D', 2, 'N', 100);  % provisional
P.ctrl.gs_gain = 2.5;
P.ctrl.P_HP_0_bar = P.P_1_0 * 1e-5;
P.ctrl.inv_P_HP_0_bar = 1 / (P.P_1_0 * 1e-5);

P.compat_v1 = false;      % replication is water-only
P.sequence = { ...
    0.0, 'OFF',        0; ...
    0.5, 'ARMED',      0; ...
    1.0, 'PRESSURIZE', P.Target_Pressure; ...
    6.0, 'RUN',        P.Target_Pressure};

% ICs: saturation equilibrium at T_amb, liquid fills V_fill, no N2 pad yet
g = @(col) interp1(P.n2o_lut.T, P.n2o_lut.(col), P.T_amb);
rho_L0 = g('rho_L');
rho_V0 = g('rho_V');
P.fluid.rho_L = rho_L0;   % nominal density for plots/volume conversions
V_ull0 = P.V_2 - P.V_fill;
m_L0 = rho_L0 * P.V_fill;
m_V0 = rho_V0 * V_ull0;
P.n2o_ic = struct( ...
    'm_N2O', m_L0 + m_V0, ...
    'm_N2',  0, ...
    'U',     m_L0 * g('u_L') + m_V0 * g('u_V'), ...
    'T',     P.T_amb, ...
    'P_sat', g('P'));
end
