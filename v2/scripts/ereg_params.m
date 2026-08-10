function P = ereg_params()
%EREG_PARAMS Single source of truth for every tunable in the EReg v2 sim.
% Edit values here; nothing is defined anywhere else. Derived quantities at the
% bottom reuse v1's exact expressions so the water-graph replication stays
% bit-comparable - don't simplify them.

% --- Timing ---
P.Ts_sim       = 1/500;   % plant/solver step [s] (v1 solver rate)
P.Ts_ctrl      = 1/500;   % controller tick [s] (parameterized; = Ts_sim for replication)
P.Sim_Duration = 10;      % [s]

% --- HP nitrogen tank ---
P.T_0_N2 = 300;           % initial N2 temperature [K]
P.P_1_0  = 15 * 1e5;      % initial HP pressure [Pa] (v1: 15 bar)
P.V_1    = 3e-3;          % HP tank volume [m^3]

% --- LP propellant tank ---
P.V_2   = 15e-3;          % LP tank volume [m^3]
P.m_3_0 = 12;             % initial liquid mass [kg]
P.P_2_0 = 0;              % initial ullage pressure [Pa]

% --- Flow path ---
P.A_3      = 60e-6;       % injector orifice area [m^2]
P.Cd_3     = 0.7;         % injector discharge coefficient
P.K_v_4    = 0.5;         % check valve flow coefficient
P.Kv_1_max = 1;           % regulator valve max Kv

% --- Fluid (swap for IPA/N2O upgrades) ---
P.fluid.name  = 'water';
P.fluid.rho_L = 1000;     % liquid density [kg/m^3]

% --- Gas constants ---
P.R_N2     = 296;
P.gamma_N2 = 1.4;
P.Tst      = 273.15;
P.Pst      = 101325;

% --- Servo / valve hardware ---
P.Servo_Speed    = 180;   % max servo speed [deg/s] (both directions)
P.gear_ratio     = 1.7;   % servo angle -> valve angle divisor
P.backlash_width = 0.5;   % [deg]
P.backlash_init  = 0.5;   % Backlash block InitialOutput [deg]

% --- Pressure transducer placeholders (passthrough until PT model lands) ---
P.pt.Ts            = P.Ts_ctrl;
P.pt.noise_std_bar = 0;
P.pt.quant_lsb_bar = 0;
P.pt.seed          = 0;

% --- Controller (fed to ereg_controller_step as p) ---
P.ctrl.Ts              = P.Ts_ctrl;
P.ctrl.int_limit       = 20;      % outer PID integrator state clamp
P.ctrl.feedforward     = 0;       % [deg]
P.ctrl.gs_gain         = 5;       % gain-schedule slope (1x full -> 6x empty)
P.ctrl.P_HP_0_bar      = P.P_1_0 * 1e-5;      % v1 expression - keep verbatim
P.ctrl.inv_P_HP_0_bar  = 1 / (P.P_1_0 * 1e-5);
P.ctrl.servo_speed_max = P.Servo_Speed;
P.ctrl.run   = struct('K_P', 8, 'K_I', 2, 'K_D', 8, 'N', 100);  % v1 gains
P.ctrl.press = struct('K_P', 2, 'K_I', 0.7, 'K_D', 6, 'N', 100);
% ^ overdamped dead-head set, tuned by sweep (smoke_test_clean): reaches
%   2.98 bar in ~4.6 s with zero overshoot (dead-head overshoot is permanent -
%   the check valve cannot vent). Sensitive to K_I: 0.5 stalls at ~2.45 bar,
%   1.0 overshoots to ~3.23 bar.
P.ctrl.inner = struct('K_P', 3, 'K_I', 2, 'K_D', 0, 'N', 100);  % position loop (v1)

% --- Test sequence: {t_start [s], mode, P_set [bar]} rows ---
P.Target_Pressure = 3;    % [bar]
P.sequence = { ...
    0.0, 'OFF', 0; ...
    1.0, 'RUN', P.Target_Pressure };
P.compat_v1 = true;       % reproduce v1 grid/interpolation artifacts exactly

% --- Derived (v1 expressions verbatim - do not refactor) ---
P.rho_L   = P.fluid.rho_L;
P.rho_1_0 = P.P_1_0 / (P.R_N2 * P.T_0_N2);
P.rho_2_0 = P.P_2_0 / (P.R_N2 * P.T_0_N2);
P.m_2_0   = P.rho_2_0 * (P.V_2 - P.m_3_0 / P.rho_L);
P.m_1_0   = P.V_1 * P.rho_1_0;
P.rhost   = P.Pst / (P.R_N2 * P.Tst);
end
