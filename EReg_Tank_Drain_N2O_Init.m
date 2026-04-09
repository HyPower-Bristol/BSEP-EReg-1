% EReg_Tank_Drain_N2O_Init.m
% Initialization Script for N2O E-Reg Simulation
% Replaces EReg_Tank_Drain_Init.m for N2O propellant context.

clear; clc; close all;

%-------------------------------------------------------
%------------- User Configurables ----------------------
%-------------------------------------------------------

% --- Controller Parameters ---
params.K_P = 16;           % Proportional Constant
params.K_I = 17;           % Integral Constant
params.K_D = 8;            % Derivative Constant
params.N = 100;            % Filter Coefficient (Derivative) - Unused in script sim for now
params.Feedforward = 0;
params.Servo_Speed = 180;  % [deg/s] - Faster servo for updated model?
params.Kv_1_max = 1;       % Reg Valve Max Kv (Approx scaling in sim)

% --- Hardware Constants ---
params.V_1 = 10 * 1e-3;    % HP Tank volume [m^3] (10L)
params.V_2 = 40 * 1e-3;    % Run Tank volume [m^3] (13L)
params.A_3 = 80*1e-6;      % Injector Orifice Area [m^2]
params.Cd_3 = 0.7;         % Injector Orifice Cd
params.K_v_4 = 0.5;        % Regulator/Check Valve Coeff

% --- Initial Conditions ---
params.T_0_N2 = 293.15;    % Initial temperature [K] (20 deg C)
params.P_1_0 = 300 * 1e5;  % Initial N2 HP Pressure [Pa] (300 bar)
params.P_2_0 = 50 * 1e5;   % Initial Run Tank Pressure [Pa] (50 bar)

% Calculate N2 Mass from Pressure (Ideal Gas Law for Init)
params.m_1_0 = (params.P_1_0 * params.V_1) / (296 * params.T_0_N2);
params.m_3_0 = 20;         % Initial N2O Mass [kg]

% --- Physics Constants ---
params.R_N2 = 296;         % Gas Constant for Nitrogen [J/kgK]
params.P_atm = 101325;     % Atmospheric Pressure [Pa]

% --- Simulation Timing ---
params.Sim_Duration = 6.0; % [s]
params.Time_Step = 0.001;  % [s]

% --- Setpoint Profile Generation ---
Num_Steps = floor(params.Sim_Duration / params.Time_Step) + 1;
params.setpoint = zeros(1, Num_Steps);
params.run_valve_setpoint = zeros(1, Num_Steps);
params.Target_Pressure = 55; % [bar] Target for N2O per requirements (example)

% Define Profile (Time-based indices)
% 0-1s: Pre-pressurization (Regulate to Target)
% 1-5s: Run (Open Valve, Maintain Target)
% 5s+: Close

idx_1s = round(1.0 / params.Time_Step);
idx_5s = round(5.0 / params.Time_Step);

% Phase 1: Ramp/Hold Target
for i = 1:Num_Steps
    if i < idx_1s
        params.setpoint(i) = params.Target_Pressure;
        params.run_valve_setpoint(i) = 0; % Closed
    elseif i < idx_5s
        params.setpoint(i) = params.Target_Pressure;
        params.run_valve_setpoint(i) = 1; % Open (Main burn)
    else
        params.setpoint(i) = params.Target_Pressure; % Maintain P
        params.run_valve_setpoint(i) = 0; % Closed
    end
end

% Check for LUT
if ~exist('n2o_saturation_properties.csv', 'file')
    disp('Generating N2O LUT...');
    system('python3 generate_n2o_lut.py');
end

%-------------------------------------------------------
%------------- Run Simulation --------------------------
%-------------------------------------------------------
disp('Running N2O Simulation...');
sim_results = EReg_Tank_Drain_N2O_Sim(params);
disp('Simulation Complete.');

%-------------------------------------------------------
%------------- Plotting --------------------------------
%-------------------------------------------------------
Ereg_N2O_Plot(sim_results);
