function P = ereg_params_dual()
%EREG_PARAMS_DUAL Dual-branch scenario: one HP N2 bottle feeding two EReg
% controller instances - fuel branch (IPA, 50 bar) + oxidizer branch
% (two-phase N2O, 55 bar) - simultaneously. Flight-like regime.
% Simplification (documented): both LP tanks share V_2; identical valve/servo
% hardware on both branches.

% --- Timing ---
P.Ts_sim = 1/500;
P.Ts_ctrl = 1/500;
P.Sim_Duration = 10;

% --- Shared HP bottle (bigger than single-branch: feeds two tanks) ---
P.P_1_0 = 300 * 1e5;
P.V_1 = 6e-3;
P.T_0_N2 = 300;

% --- Shared LP tank geometry & valve hardware ---
P.V_2 = 15e-3;
P.V_fill = 12e-3;
P.Kv_1_max = 1;
P.Servo_Speed = 180;
P.gear_ratio = 1.7;
P.backlash_width = 0.5;
P.backlash_init = 0.5;

% --- Gas constants ---
P.R_N2 = 296; P.gamma_N2 = 1.4; P.Tst = 273.15; P.Pst = 101325;
P.cv_N2 = P.R_N2 / (P.gamma_N2 - 1);
P.cp_N2 = P.gamma_N2 * P.R_N2 / (P.gamma_N2 - 1);

% --- Fuel branch (IPA) ---
P.fu.rho_L = 786;
P.fu.Cd_3 = 0.7;
P.fu.A_3 = 15e-6;         % sized ~1 kg/s at 50 bar
P.fu.K_v_4 = 0.5;
P.fu.Target = 50;         % [bar]
P.fu.m_3_0 = P.fu.rho_L * P.V_fill;
% Prelaunch pad: fuel ullage pre-pressurized to 45 bar before the sequence
% (report Fig. 12 shows the fuel tank already at pressure at t=0). Both
% branches then do a gentle ~5-10 bar PRESSURIZE top-up; a 50 bar dead-head
% from vacuum in 5 s overshoots on any no-anti-windup gain set.
P.fu.P_2_0 = 45e5;
P.fu.m_2_0 = P.fu.P_2_0 * (P.V_2 - P.V_fill) / (P.R_N2 * P.T_0_N2);

% --- Oxidizer branch (N2O, two-phase) ---
P.ox.Cd_3 = 0.7;
P.ox.A_3 = 20e-6;         % ~1.3 kg/s at 55 bar
P.ox.K_v_4 = 0.5;
P.ox.Target = 55;         % [bar]
P.ox.T_amb = 288;
here = fileparts(mfilename('fullpath'));
M = readmatrix(fullfile(here, '..', 'data', 'n2o_saturation_properties.csv'));
P.n2o_lut = struct('T', M(:,1), 'P', M(:,2), 'rho_L', M(:,3), 'rho_V', M(:,4), ...
    'u_L', M(:,5), 'u_V', M(:,6), 'h_L', M(:,7), 'h_V', M(:,8), 's_L', M(:,9), 's_V', M(:,10));
g = @(col) interp1(P.n2o_lut.T, P.n2o_lut.(col), P.ox.T_amb);
m_L0 = g('rho_L') * P.V_fill;
m_V0 = g('rho_V') * (P.V_2 - P.V_fill);
P.ox.ic = struct('m_N2O', m_L0 + m_V0, 'm_N2', 0, ...
    'U', m_L0 * g('u_L') + m_V0 * g('u_V'));
P.ox.rho_L_nom = g('rho_L');

% --- Controller (shared gain set for both instances; N2O flight values) ---
P.ctrl.Ts = P.Ts_ctrl;
P.ctrl.int_limit = 20;
P.ctrl.feedforward = 0;
P.ctrl.gs_gain = 2.5;
P.ctrl.P_HP_0_bar = P.P_1_0 * 1e-5;
P.ctrl.inv_P_HP_0_bar = 1 / (P.P_1_0 * 1e-5);
P.ctrl.servo_speed_max = P.Servo_Speed;
P.ctrl.run   = struct('K_P', 16, 'K_I', 17, 'K_D', 8, 'N', 100);
P.ctrl.press = struct('K_P', 0.5, 'K_I', 0.3, 'K_D', 2, 'N', 100);
P.ctrl.inner = struct('K_P', 3, 'K_I', 2, 'K_D', 0, 'N', 100);

% --- Per-branch mission sequences (same timing, different setpoints) ---
P.compat_v1 = false;
P.fu.sequence = {0,'OFF',0; 0.5,'ARMED',0; 1.0,'PRESSURIZE',P.fu.Target; 6.0,'RUN',P.fu.Target};
P.ox.sequence = {0,'OFF',0; 0.5,'ARMED',0; 1.0,'PRESSURIZE',P.ox.Target; 6.0,'RUN',P.ox.Target};

% --- Derived ---
P.rho_1_0 = P.P_1_0 / (P.R_N2 * P.T_0_N2);
P.m_1_0 = P.V_1 * P.rho_1_0;
P.rhost = P.Pst / (P.R_N2 * P.Tst);
assert(P.V_fill <= P.V_2);
end
