% EReg_Tank_Drain_N2O_Init.m
% Initialisation Script for N2O E-Reg Simulation
% Configures all parameters and runs the MATLAB-based sim + plotter.
%
% Usage: Run this script directly.

clear; clc; close all;

%% =============================================================
%  USER CONFIGURABLES
%  =============================================================

% --- Controller Parameters ---
% TUNING NOTES (N2O plant):
%   - The N2O plant is "stiff" because vapour pressure provides a ~50 bar
%     floor. The controller only manages the ~5 bar supercharge margin.
%   - With Kv=1.5 the valve operates in a more useful range (5-30 deg)
%     giving better control authority than Kv=4 (which only needed 1-2 deg).
%   - K_P = 20: Aggressive proportional to get fast response to the 5 bar
%     error during ramp-up and relight recovery.
%   - K_I = 15: Strong integral to eliminate steady-state offset. The
%     anti-windup clamp in the sim prevents saturation during relight.
%   - K_D = 3:  Light derivative to dampen overshoot without amplifying
%     noise. Kept low because the plant is already well-damped by thermal mass.
params.K_P = 20;            % Proportional gain [deg/bar]
params.K_I = 15;            % Integral gain [deg/(bar*s)]
params.K_D = 3;             % Derivative gain [deg*s/bar]
params.N   = 100;           % Derivative filter coefficient
params.Feedforward = 0;     % Feedforward step [deg]
params.Servo_Speed = 180;   % Servo actuation speed [deg/s]
params.Kv_1_max = 1.5;      % Reg valve maximum Kv
                             % Kv=1.5 gives effective area ~3.6e-5 m^2.
                             % At 300 bar upstream, max N2 flow ~ 0.15 kg/s
                             % which is enough to maintain supercharge but
                             % requires the valve to open to 15-30 deg,
                             % giving the controller useful dynamic range.

% --- Hardware Constants ---
params.V_1 = 6.8 * 1e-3;   % HP N2 tank volume [m^3] (6.8 L)
params.V_2 = 30 * 1e-3;    % N2O run tank volume [m^3] (30 L)
params.A_3 = 13.27e-6;     % Injector orifice area [m^2] (13.27 mm^2)
params.Cd_3 = 0.6;          % Injector discharge coefficient
params.K_v_4 = 0.5;         % Check valve flow coefficient

% --- Initial Conditions ---
params.T_0_N2 = 293.15;     % Ambient / initial temperature [K] (20 deg C)
params.P_1_0  = 300 * 1e5;  % HP N2 tank initial pressure [Pa] (300 bar)
params.P_2_0  = 51 * 1e5;   % Run tank initial pressure [Pa]
                             % Just above vapour pressure (~50 bar at 293K)

% N2 mass
params.R_N2 = 296;           % Gas constant for N2 [J/kgK]
params.m_1_0 = (params.P_1_0 * params.V_1) / (params.R_N2 * params.T_0_N2);
fprintf('HP N2 mass: %.2f kg\n', params.m_1_0);

% N2O propellant mass
params.m_3_0 = 12;           % Initial N2O liquid mass [kg]

% --- Downstream / Chamber Pressure ---
params.P_atm = 25 * 1e5;    % Chamber pressure [Pa] (25 bar hot fire)

% --- Simulation Timing ---
params.Sim_Duration = 12.0;  % Extended to see post-relight recovery
params.Time_Step    = 0.001;  % Time step [s] (1 ms)

% --- Target Pressure ---
params.Target_Pressure = 55;  % [bar]

fprintf('Supercharge margin: %.1f bar above P_vap(293K)\n', ...
    params.Target_Pressure - 50);
fprintf('Injector dP at target: %.1f bar\n', ...
    params.Target_Pressure - params.P_atm/1e5);

%% =============================================================
%  SETPOINT PROFILE GENERATION
%  =============================================================

Num_Steps = floor(params.Sim_Duration / params.Time_Step) + 1;
params.setpoint           = zeros(1, Num_Steps);
params.run_valve_setpoint = zeros(1, Num_Steps);

% ---------------------------------------------------------------
% PROFILE WITH RELIGHT:
%   0.0 -  1.0 s : Pre-pressurisation (ramp to target, run valve closed)
%   1.0 -  4.0 s : Burn phase 1 (run valve open)
%   4.0 -  5.0 s : RELIGHT pause (run valve CLOSED for 1 second)
%                   - Simulates engine restart sequence
%                   - E-Reg must hold/reduce pressure (no outflow)
%   5.0 -  9.0 s : Burn phase 2 (run valve re-opens)
%   9.0 - 12.0 s : Shutdown (run valve closed)
%
% OPTIONAL THROTTLE: Reduce setpoint mid-burn to test throttling
%   6.0 -  7.0 s : Throttle down to 45 bar (if enabled)
% ---------------------------------------------------------------

enable_throttle = false;  % Set true to test throttle capability
throttle_pressure = 45;   % [bar] reduced pressure during throttle

for i = 1:Num_Steps
    t = (i-1) * params.Time_Step;

    % --- SETPOINT ---
    if t < 0.2
        % Hold at initial
        params.setpoint(i) = params.P_2_0 / 1e5;
    elseif t < 1.0
        % Linear ramp to target
        frac = (t - 0.2) / 0.8;
        params.setpoint(i) = params.P_2_0/1e5 + frac * (params.Target_Pressure - params.P_2_0/1e5);
    elseif enable_throttle && t >= 6.0 && t < 7.0
        % Throttle down
        params.setpoint(i) = throttle_pressure;
    else
        params.setpoint(i) = params.Target_Pressure;
    end

    % --- RUN VALVE ---
    if t >= 1.0 && t < 4.0
        % Burn phase 1
        params.run_valve_setpoint(i) = 1;
    elseif t >= 4.0 && t < 5.0
        % RELIGHT: run valve CLOSED
        params.run_valve_setpoint(i) = 0;
    elseif t >= 5.0 && t < 9.0
        % Burn phase 2
        params.run_valve_setpoint(i) = 1;
    else
        params.run_valve_setpoint(i) = 0;
    end
end

%% =============================================================
%  CHECK N2O LOOK-UP TABLE
%  =============================================================

if ~exist('n2o_saturation_properties.csv', 'file')
    fprintf('N2O saturation LUT not found. Generating...\n');
    system('python3 generate_n2o_lut.py');
    if ~exist('n2o_saturation_properties.csv', 'file')
        error('Failed to generate n2o_saturation_properties.csv. Check Python/CoolProp.');
    end
end

%% =============================================================
%  RUN SIMULATION
%  =============================================================

fprintf('\n--- Running N2O E-Reg Simulation ---\n');
sim_results = EReg_Tank_Drain_N2O_Sim(params);
fprintf('--- Simulation Complete ---\n\n');

%% =============================================================
%  PLOTTING
%  =============================================================

Ereg_N2O_Plot(sim_results);
