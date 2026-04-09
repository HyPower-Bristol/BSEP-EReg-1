% EReg_N2O_Simulink_Init.m
% Workspace initialisation for N2O E-Reg Simulink model.
%
% Run this BEFORE opening/running the Simulink model.
% It loads the N2O saturation LUTs into the workspace and sets all
% parameters that the MATLAB Function block needs.
%
% INTEGRATION GUIDE:
% ==================
% Your existing EReg_Tank_Drain.slx has this signal flow:
%
%   [Setpoint] -> [Error] -> [PID] -> [Servo Actuated Valve] -> [Kv_reg]
%                                                                   |
%                                                                   v
%   [Run Valve Profile] ---------> [Kv_water] -----> [Tank Plant Model]
%                                                         |
%                                          [P_tank, P_HP, m_1, m_3] -> feedback
%
% WHAT TO CHANGE IN SIMULINK:
% ---------------------------
% 1. DELETE the contents of the "Tank Plant Model" subsystem.
%    (Keep the subsystem block itself and its input/output ports.)
%
% 2. Inside the now-empty "Tank Plant Model" subsystem, add a single
%    MATLAB Function block.
%
% 3. Rename it "N2O Two-Phase Plant".
%
% 4. Double-click it and paste the ENTIRE contents of N2O_Tank_Plant.m
%    as the function code. (Copy from "function [P_2_bar, ..." to the end.)
%
% 5. Configure the block's INPUT ports. The function signature has two
%    Simulink inputs (the rest are parameters):
%
%    Input 1: Kv_reg   -> Connect from "Regulator Valve Kv" (normalised 0-1)
%                         This is the output of your existing Servo Actuated
%                         Valve subsystem, labelled "Normalized Valve Area"
%                         in your model.
%
%    Input 2: Kv_water -> Connect from "Water Valve Step Response Profile"
%                         (your run valve setpoint, 0 or 1).
%
% 6. Configure the block's OUTPUT ports:
%
%    Output 1: P_2_bar    -> Connect to "P_tank [bar]" logging/feedback
%    Output 2: P_HP_bar   -> Connect to "P_HP [bar]" logging/feedback
%    Output 3: m_dot_L    -> Connect to mass flow logging (optional)
%    Output 4: m_3        -> Connect to "m_3" logging (propellant mass)
%    Output 5: m_1        -> Connect to "m_1" logging (N2 mass)
%    Output 6: T_N2O_out  -> Connect to temperature logging (new signal)
%
% 7. In the MATLAB Function block editor, click "Edit Data" and set ALL
%    parameters (everything except Kv_reg and Kv_water) to Scope = "Parameter"
%    with Source = "From workspace". This tells Simulink to pull them from
%    the workspace variables defined below.
%
%    The parameters to set as workspace parameters are:
%      V_1, V_2, A_3, Cd_3, R_N2, P_atm,
%      T_sat_LUT, P_sat_LUT, rhoL_LUT, rhoV_LUT, uL_LUT, uV_LUT, hL_LUT,
%      m_1_0, m_N2O_liq_0, m_N2_ull_0, T_N2O_0, U_N2O_0, P_1_0, rho_1_0, T_1_0
%
% 8. Set the Simulink solver to Fixed-Step, ode1 (Euler), step size 0.001.
%    This matches the dt=0.001 inside the MATLAB Function block.
%
% 9. Make sure the signal names for logging match what EregPlot.m expects:
%      'P_HP [bar]', 'P_tank [bar]', 'm_1', 'm_3', 'setpoint'
%    Add To Workspace or Scope blocks as needed.
%
% 10. Run this init script, then run the Simulink model.

clear; clc; close all;

%% =============================================================
%  CONTROLLER PARAMETERS (same as tuned script sim)
%  =============================================================
K_P = 20;
K_I = 15;
K_D = 3;
N   = 100;
Feedforward = 0;
Servo_Speed = 180;
Kv_1_max = 1.5;

%% =============================================================
%  HARDWARE CONSTANTS
%  =============================================================
V_1  = 6.8e-3;      % HP N2 tank volume [m^3]
V_2  = 30e-3;       % N2O run tank volume [m^3]
A_3  = 13.27e-6;    % Injector orifice area [m^2]
Cd_3 = 0.6;         % Injector discharge coefficient
R_N2 = 296;         % N2 gas constant [J/kgK]
P_atm = 25e5;       % Chamber/downstream pressure [Pa]

%% =============================================================
%  N2O SATURATION LOOK-UP TABLES
%  =============================================================
fprintf('Loading N2O saturation properties...\n');
opts = detectImportOptions('n2o_saturation_properties.csv');
opts.VariableNamingRule = 'preserve';
n2o_tab = readtable('n2o_saturation_properties.csv', opts);
n2o_tab = sortrows(n2o_tab, 'T');

% Extract as column vectors for workspace (Simulink needs these)
T_sat_LUT = n2o_tab.T;
P_sat_LUT = n2o_tab.P;
rhoL_LUT  = n2o_tab.rho_L;
rhoV_LUT  = n2o_tab.rho_V;
uL_LUT    = n2o_tab.u_L;
uV_LUT    = n2o_tab.u_V;
hL_LUT    = n2o_tab.h_L;

fprintf('  LUT loaded: %d points, T range [%.1f, %.1f] K\n', ...
    length(T_sat_LUT), T_sat_LUT(1), T_sat_LUT(end));

%% =============================================================
%  INITIAL CONDITIONS
%  =============================================================
T_0_N2    = 293.15;            % Ambient temperature [K]
P_1_0     = 300e5;             % HP N2 initial pressure [Pa]
P_2_0     = 51e5;              % Run tank initial pressure [Pa]

% HP tank
rho_1_0   = P_1_0 / (R_N2 * T_0_N2);
m_1_0     = rho_1_0 * V_1;
T_1_0     = T_0_N2;

% N2O
m_N2O_liq_0 = 12;              % Initial N2O liquid mass [kg]
T_N2O_0     = T_0_N2;

% Look up properties at initial temperature
rhoL_init = interp1(T_sat_LUT, rhoL_LUT, T_N2O_0, 'linear', 'extrap');
rhoV_init = interp1(T_sat_LUT, rhoV_LUT, T_N2O_0, 'linear', 'extrap');
P_vap_init = interp1(T_sat_LUT, P_sat_LUT, T_N2O_0, 'linear', 'extrap');
uL_init   = interp1(T_sat_LUT, uL_LUT, T_N2O_0, 'linear', 'extrap');
uV_init   = interp1(T_sat_LUT, uV_LUT, T_N2O_0, 'linear', 'extrap');

V_liq_init = m_N2O_liq_0 / rhoL_init;
V_ull_init = V_2 - V_liq_init;

% N2 ullage mass (from partial pressure)
P_N2_init  = max(0, P_2_0 - P_vap_init);
rho_N2_init = P_N2_init / (R_N2 * T_N2O_0);
m_N2_ull_0 = rho_N2_init * V_ull_init;

% N2O vapour mass
m_N2O_vap_init = rhoV_init * V_ull_init;

% Total internal energy of N2O system
U_N2O_0 = m_N2O_liq_0 * uL_init + m_N2O_vap_init * uV_init;

fprintf('\nInitial Conditions:\n');
fprintf('  HP N2:  P=%.0f bar, m=%.2f kg, T=%.1f K\n', P_1_0/1e5, m_1_0, T_1_0);
fprintf('  Run Tank: P=%.1f bar (P_vap=%.1f, P_N2=%.1f)\n', P_2_0/1e5, P_vap_init/1e5, P_N2_init/1e5);
fprintf('  N2O:    m_liq=%.1f kg, m_vap=%.2f kg, T=%.1f K\n', m_N2O_liq_0, m_N2O_vap_init, T_N2O_0);
fprintf('  Ullage: %.1f%% of tank\n', V_ull_init/V_2*100);

%% =============================================================
%  SIMULATION TIMING & SETPOINTS
%  =============================================================
Sim_Duration = 12;
Time_Step = 0.001;
Num_Steps = round(Sim_Duration / Time_Step);
time = linspace(0, Sim_Duration, Num_Steps);

Target_Pressure = 55; % [bar]

% Setpoint profile
setpoint = zeros(1, Num_Steps);
run_valve_setpoint = zeros(1, Num_Steps);

for i = 1:Num_Steps
    t = time(i);

    % Setpoint ramp
    if t < 0.2
        setpoint(i) = P_2_0 / 1e5;
    elseif t < 1.0
        frac = (t - 0.2) / 0.8;
        setpoint(i) = P_2_0/1e5 + frac * (Target_Pressure - P_2_0/1e5);
    else
        setpoint(i) = Target_Pressure;
    end

    % Run valve with relight at 4-5s
    if t >= 1.0 && t < 4.0
        run_valve_setpoint(i) = 1;
    elseif t >= 4.0 && t < 5.0
        run_valve_setpoint(i) = 0;  % Relight pause
    elseif t >= 5.0 && t < 9.0
        run_valve_setpoint(i) = 1;
    else
        run_valve_setpoint(i) = 0;
    end
end

% For existing Simulink blocks that expect timeseries or arrays
% (These names must match what the Simulink model references)
servo_off_setpoint = ones(1, Num_Steps); % Servo always active
throttle_setpoint = servo_off_setpoint;

%% =============================================================
%  LEGACY COMPATIBILITY (for existing Simulink blocks)
%  =============================================================
% These are needed if the existing PID, Gain Scheduling, or other
% blocks reference workspace variables from the ethanol init.
P_1_0_Pa = P_1_0;
P_2_0_Pa = P_2_0;
rho_1_0_val = rho_1_0;  % Avoid conflict with workspace var name
rho_L = rhoL_init;       % Density used in some existing blocks
gamma_N2 = 1.4;
Tst = 273.15;
Pst = 101325;
rhost = Pst / (R_N2 * Tst);
m_2_0 = m_N2_ull_0;
m_3_0 = m_N2O_liq_0;
T_0_N2_val = T_0_N2;
P_4_0 = P_atm;

% Original init stored some things in bar then converted
% Make sure workspace has both forms
P_1_0_bar = P_1_0 / 1e5;
P_2_0_bar = P_2_0 / 1e5;

fprintf('\nWorkspace ready. Open Simulink model and run.\n');
fprintf('Solver: Fixed-step, ode1 (Euler), step=0.001\n');
fprintf('Duration: %g s\n', Sim_Duration);
