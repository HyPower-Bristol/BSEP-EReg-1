function results = EReg_Tank_Drain_N2O_Sim(params)
% EReg_Tank_Drain_N2O_Sim
% Simulates the time evolution of a Supercharged Nitrous Oxide Run Tank
% pressurised by a Nitrogen High Pressure Tank.
%
% Physics Model:
%   - N2O two-phase equilibrium with energy balance (temperature-coupled)
%   - Dalton's Law ullage (N2 partial + N2O vapour pressure)
%   - N2O vapour mass tracked (evaporation/condensation)
%   - Polytropic N2 HP tank blowdown
%   - Compressible gas flow through regulator valve
%   - Incompressible liquid flow through injector
%
% Inputs:
%   params - Struct from EReg_Tank_Drain_N2O_Init.m
%
% Outputs:
%   results - Struct containing time-series data

%% ================================================================
%  1. UNPACK PARAMETERS
%  ================================================================

% Hardware
V_1       = params.V_1;         % HP Tank volume [m^3]
V_2       = params.V_2;         % Run Tank volume [m^3]
A_inj     = params.A_3;         % Injector orifice area [m^2]
Cd_inj    = params.Cd_3;        % Injector discharge coefficient
Kv_1_max  = params.Kv_1_max;    % Reg valve max Kv

% Physics
R_N2      = params.R_N2;        % Gas constant for N2 [J/kgK]
gamma_N2  = 1.4;                % Specific heat ratio for N2
n_poly    = 1.3;                % Polytropic index for N2 blowdown
Cp_N2     = R_N2 * gamma_N2 / (gamma_N2 - 1); % ~1040 J/kgK
T_ambient = params.T_0_N2;      % Initial / ambient temperature [K]
P_atm     = params.P_atm;       % Atmospheric pressure [Pa]

% Controller
K_P       = params.K_P;
K_I       = params.K_I;
K_D       = params.K_D;
Servo_Spd = params.Servo_Speed;  % [deg/s]

% Timing
dt        = params.Time_Step;
t_end     = params.Sim_Duration;

%% ================================================================
%  2. LOAD N2O SATURATION LOOK-UP TABLES
%  ================================================================

opts = detectImportOptions('n2o_saturation_properties.csv');
opts.VariableNamingRule = 'preserve';
n2o_tab = readtable('n2o_saturation_properties.csv', opts);

% Sort by temperature (should already be, but ensure)
n2o_tab = sortrows(n2o_tab, 'T');

T_vec     = n2o_tab.T;
P_vec     = n2o_tab.P;
rhoL_vec  = n2o_tab.rho_L;
rhoV_vec  = n2o_tab.rho_V;
uL_vec    = n2o_tab.u_L;
uV_vec    = n2o_tab.u_V;
hL_vec    = n2o_tab.h_L;
hV_vec    = n2o_tab.h_V;
sL_vec    = n2o_tab.s_L;
sV_vec    = n2o_tab.s_V;

% Griddedinterpolant (T -> property), linear with nearest extrapolation
T_to_Psat  = griddedInterpolant(T_vec, P_vec,    'linear', 'nearest');
T_to_rhoL  = griddedInterpolant(T_vec, rhoL_vec, 'linear', 'nearest');
T_to_rhoV  = griddedInterpolant(T_vec, rhoV_vec, 'linear', 'nearest');
T_to_uL    = griddedInterpolant(T_vec, uL_vec,   'linear', 'nearest');
T_to_uV    = griddedInterpolant(T_vec, uV_vec,   'linear', 'nearest');
T_to_hL    = griddedInterpolant(T_vec, hL_vec,   'linear', 'nearest');
T_to_hV    = griddedInterpolant(T_vec, hV_vec,   'linear', 'nearest');

% Reverse lookup: P -> T (for initialisation if needed)
[P_sorted, idx_sort] = sort(P_vec);
T_sorted = T_vec(idx_sort);
P_to_Tsat = griddedInterpolant(P_sorted, T_sorted, 'linear', 'nearest');

% Temperature bounds for clamping
T_min = T_vec(1)   + 0.5;   % Stay away from triple point
T_max = T_vec(end) - 2.0;   % Stay away from critical point

%% ================================================================
%  3. INITIAL STATE
%  ================================================================

time = (0 : dt : t_end)';
len  = length(time);

% --- HP Tank (N2) ---
P_1_0   = params.P_1_0;                     % [Pa]
m_1_0   = params.m_1_0;                     % [kg]
rho_1_0 = m_1_0 / V_1;                      % Initial N2 density
T_1_0   = P_1_0 / (rho_1_0 * R_N2);        % Consistent temperature

% State
m_1   = m_1_0;
rho_1 = rho_1_0;
T_1   = T_1_0;
P_1   = P_1_0;

% --- Run Tank (N2O + N2 ullage) ---
T_N2O = T_ambient;                           % N2O liquid temperature [K]
T_N2O = max(T_min, min(T_max, T_N2O));       % Clamp to LUT range

% Liquid N2O
m_N2O_liq = params.m_3_0;                    % [kg]
rho_L_0   = T_to_rhoL(T_N2O);               % Liquid density at T_ambient
V_liq_0   = m_N2O_liq / rho_L_0;            % Liquid volume
V_ull_0   = V_2 - V_liq_0;                   % Ullage volume

if V_ull_0 < 0
    error('Initial N2O mass exceeds tank volume! Reduce m_3_0 or increase V_2.');
end

% N2O vapour in ullage (saturated)
P_vap_0     = T_to_Psat(T_N2O);             % Vapour pressure at T_ambient
rho_V_0     = T_to_rhoV(T_N2O);             % Vapour density at T_ambient
m_N2O_vap   = rho_V_0 * V_ull_0;            % N2O vapour mass in ullage

% N2 partial pressure in ullage
P_N2_ull_0  = max(0, params.P_2_0 - P_vap_0);
rho_N2_ull  = P_N2_ull_0 / (R_N2 * T_N2O);
m_N2_ull    = rho_N2_ull * V_ull_0;          % N2 mass in ullage

% Total run tank pressure
P_2 = P_N2_ull_0 + P_vap_0;

% Internal energy of the N2O system (liquid + vapour)
u_L_0 = T_to_uL(T_N2O);
u_V_0 = T_to_uV(T_N2O);
U_N2O = m_N2O_liq * u_L_0 + m_N2O_vap * u_V_0;  % Total internal energy [J]

% Total N2O mass (conserved when no outflow, split between phases)
m_N2O_total = m_N2O_liq + m_N2O_vap;

% Specific heat of liquid N2O (approximate, for supplementary calcs)
% Cp_N2O_liq ~ 2000 J/kgK near 293K (from CoolProp data)
Cp_N2O_liq = 2000;

%% ================================================================
%  4. PRE-ALLOCATE HISTORY ARRAYS
%  ================================================================

P_1_hist        = zeros(len, 1);
P_2_hist        = zeros(len, 1);
T_N2O_hist      = zeros(len, 1);
T_1_hist        = zeros(len, 1);
m_1_hist        = zeros(len, 1);
m_N2O_liq_hist  = zeros(len, 1);
m_N2O_vap_hist  = zeros(len, 1);
m_N2_ull_hist   = zeros(len, 1);
valve_pos_hist  = zeros(len, 1);
m_dot_reg_hist  = zeros(len, 1);
m_dot_inj_hist  = zeros(len, 1);
P_vap_hist      = zeros(len, 1);
quality_hist    = zeros(len, 1);

%% ================================================================
%  5. CONTROL STATE INITIALISATION
%  ================================================================

pid_int_err = 0;
last_err    = 0;
valve_pos   = 0;    % Valve angle [deg], 0 = closed, 90 = fully open

% Effective valve area from Kv
% Kv -> Cv = 1.156*Kv, Area ~ 2.4e-5 * Kv [m^2]
A_valve_max = 2.4e-5 * Kv_1_max;

fprintf('=== N2O E-Reg Simulation ===\n');
fprintf('T_ambient = %.1f K | P_target = %.1f bar\n', T_ambient, params.Target_Pressure);
fprintf('m_N2O = %.1f kg | V_tank = %.1f L | Ullage = %.1f%%\n', ...
    m_N2O_total, V_2*1e3, V_ull_0/V_2*100);
fprintf('P_vap(T0) = %.1f bar | P_2_init = %.1f bar\n', P_vap_0/1e5, P_2/1e5);
fprintf('N2 HP: %.1f bar, %.2f kg\n\n', P_1/1e5, m_1);

%% ================================================================
%  6. TIME INTEGRATION LOOP
%  ================================================================

for i = 1:len

    %% A. STORE CURRENT STATE
    P_1_hist(i)       = P_1;
    P_2_hist(i)       = P_2;
    T_N2O_hist(i)     = T_N2O;
    T_1_hist(i)       = T_1;
    m_1_hist(i)       = m_1;
    m_N2O_liq_hist(i) = m_N2O_liq;
    m_N2O_vap_hist(i) = m_N2O_vap;
    m_N2_ull_hist(i)  = m_N2_ull;
    valve_pos_hist(i) = valve_pos;
    P_vap_hist(i)     = T_to_Psat(T_N2O);
    quality_hist(i)   = m_N2O_vap / max(m_N2O_total, 1e-9);

    %% B. SKIP INTEGRATION ON LAST STEP (just store)
    if i == len
        m_dot_reg_hist(i) = m_dot_reg_hist(max(i-1,1));
        m_dot_inj_hist(i) = m_dot_inj_hist(max(i-1,1));
        break;
    end

    %% C. HP TANK STATE (Polytropic N2 blowdown)
    rho_1 = m_1 / V_1;
    % Polytropic: T = T_0 * (rho/rho_0)^(n-1)
    T_1 = T_1_0 * (rho_1 / rho_1_0)^(n_poly - 1);
    T_1 = max(T_1, 200);  % Floor to prevent unrealistic cooling
    P_1 = rho_1 * R_N2 * T_1;

    %% D. RUN TANK THERMODYNAMICS (Two-phase N2O + N2)

    % Clamp T_N2O to LUT range
    T_N2O = max(T_min, min(T_max, T_N2O));

    % Look up saturation properties at current temperature
    P_vap    = T_to_Psat(T_N2O);
    rho_L    = T_to_rhoL(T_N2O);
    rho_V    = T_to_rhoV(T_N2O);
    u_L      = T_to_uL(T_N2O);
    u_V      = T_to_uV(T_N2O);
    h_L      = T_to_hL(T_N2O);
    h_V      = T_to_hV(T_N2O);

    % Volumes
    if m_N2O_liq > 0.001
        V_liq   = m_N2O_liq / rho_L;
        V_ull   = V_2 - V_liq;
        V_ull   = max(V_ull, 1e-6);  % Protect against zero
    else
        % Liquid depleted: pure gas blowdown
        V_liq   = 0;
        V_ull   = V_2;
        m_N2O_liq = 0;
    end

    % N2 partial pressure in ullage
    P_N2_partial = (m_N2_ull * R_N2 * T_N2O) / V_ull;

    % Total tank pressure (Dalton's Law)
    P_2 = P_N2_partial + P_vap;

    %% E. PID CONTROLLER

    % Setpoint (bar -> Pa)
    idx_sp = min(i, length(params.setpoint));
    P_target = params.setpoint(idx_sp) * 1e5;

    % Error in bar (controller was tuned in bar units)
    err_bar = (P_target - P_2) / 1e5;

    % PID terms
    P_term = K_P * err_bar;
    pid_int_err = pid_int_err + err_bar * dt;
    % Anti-windup: clamp integral
    pid_int_err = max(-20, min(20, pid_int_err));
    I_term = K_I * pid_int_err;
    D_term = K_D * (err_bar - last_err) / dt;

    u_cmd = P_term + I_term + D_term;

    % Map to valve angle [0, 90] deg
    target_angle = max(0, min(90, u_cmd));

    % Slew rate limit (servo speed)
    max_delta = Servo_Spd * dt;
    delta = target_angle - valve_pos;
    delta = max(-max_delta, min(max_delta, delta));
    valve_pos = valve_pos + delta;
    valve_pos = max(0, min(90, valve_pos));

    valve_frac = valve_pos / 90;   % Normalised 0-1
    last_err = err_bar;

    %% F. FLOW CALCULATIONS

    % --- Regulator: N2 from HP tank -> Run tank ullage ---
    A_reg_eff = A_valve_max * valve_frac;

    if P_1 > P_2 && A_reg_eff > 0
        m_dot_reg = calc_gas_flow(P_1, P_2, T_1, R_N2, gamma_N2, A_reg_eff);
    else
        m_dot_reg = 0;
    end

    % --- Injector: Liquid N2O from run tank -> chamber ---
    run_open = params.run_valve_setpoint(idx_sp);

    if run_open > 0.5 && m_N2O_liq > 0.001
        % Chamber pressure (simple model: atmospheric + combustion back-pressure)
        % For cold flow: P_chamber = P_atm
        % For hot fire: P_chamber ~ 20-30 bar (from combustion model)
        P_chamber = P_atm;

        dP_inj = P_2 - P_chamber;
        if dP_inj > 0
            m_dot_inj = Cd_inj * A_inj * sqrt(2 * rho_L * dP_inj);
        else
            m_dot_inj = 0;
        end
    else
        m_dot_inj = 0;
    end

    m_dot_reg_hist(i) = m_dot_reg;
    m_dot_inj_hist(i) = m_dot_inj;

    %% G. STATE UPDATE (Forward Euler)

    % --- Mass updates ---
    % HP tank: N2 leaves
    dm_1 = -m_dot_reg * dt;
    m_1  = max(0, m_1 + dm_1);

    % Run tank ullage: N2 enters
    m_N2_ull = m_N2_ull + m_dot_reg * dt;

    % Liquid N2O leaves through injector
    dm_inj = m_dot_inj * dt;
    m_N2O_liq = max(0, m_N2O_liq - dm_inj);

    % Total N2O mass (liquid + vapour) — outflow only removes liquid
    m_N2O_total = m_N2O_liq + m_N2O_vap;

    % --- Energy balance on N2O system ---
    % Energy removed by liquid outflow
    dU_out = m_dot_inj * h_L * dt;

    % Energy added by N2 gas entering ullage (heats the gas space slightly)
    % This N2 arrives at T_1 (HP tank temperature)
    % Its energy goes into the gas, not directly into the N2O energy budget.
    % However, heat transfer between gas and liquid surface matters.
    % Simplified: assume N2 quickly equilibrates to T_N2O (liquid thermal mass dominates)
    % Net energy input to N2O from warm N2 gas:
    dU_N2_heat = m_dot_reg * Cp_N2 * (T_1 - T_N2O) * dt * 0.1;
    % Factor 0.1: only ~10% of N2 thermal energy transfers to liquid in short time

    % Update total internal energy
    U_N2O = U_N2O - dU_out + dU_N2_heat;

    % --- Solve for new T_N2O from energy balance ---
    % U_N2O = m_liq * u_L(T) + m_vap * u_V(T)
    % m_liq + m_vap = m_N2O_total  (but m_vap depends on T through rho_V)
    % m_vap = rho_V(T) * V_ullage(T)
    % V_ullage = V_2 - m_liq/rho_L(T)
    %
    % This is implicit in T. Solve iteratively.

    if m_N2O_liq > 0.001
        T_new = solve_N2O_temperature(T_N2O, U_N2O, m_N2O_total, m_N2_ull, ...
                                       V_2, R_N2, T_to_rhoL, T_to_rhoV, ...
                                       T_to_uL, T_to_uV, T_min, T_max);
        T_N2O = T_new;

        % Recompute phase split at new temperature
        rho_L_new = T_to_rhoL(T_N2O);
        rho_V_new = T_to_rhoV(T_N2O);

        % Solve: m_liq + m_vap = m_N2O_total
        %        m_liq/rho_L + m_vap/rho_V + V_N2 = V_2
        % where V_N2 = m_N2_ull * R_N2 * T_N2O / P_N2_partial ... but P_N2 depends on V_ull
        % Simplification: N2 volume is small compared to N2O, and we know V_ull from liquid
        % m_vap = rho_V * (V_2 - m_liq/rho_L)
        % m_liq + rho_V*(V_2 - m_liq/rho_L) = m_N2O_total
        % m_liq * (1 - rho_V/rho_L) = m_N2O_total - rho_V * V_2
        denom = 1 - rho_V_new / rho_L_new;
        if abs(denom) > 1e-9
            m_N2O_liq = (m_N2O_total - rho_V_new * V_2) / denom;
            m_N2O_liq = max(0, m_N2O_liq);
        end
        m_N2O_vap = m_N2O_total - m_N2O_liq;
        m_N2O_vap = max(0, m_N2O_vap);
    else
        % All liquid gone — gas-only blowdown
        m_N2O_liq = 0;
        m_N2O_vap = m_N2O_total;
        % T_N2O stays at last value (no more evaporative cooling)
    end

end

%% ================================================================
%  7. PACK RESULTS
%  ================================================================

% Create structs mimicking timeseries for the plotter
results.P_HP_bar.Time   = time;  results.P_HP_bar.Data   = P_1_hist / 1e5;
results.P_tank_bar.Time = time;  results.P_tank_bar.Data = P_2_hist / 1e5;
results.m_1.Time        = time;  results.m_1.Data        = m_1_hist;
results.m_3.Time        = time;  results.m_3.Data        = m_N2O_liq_hist;
results.setpoint.Time   = time;  results.setpoint.Data   = params.setpoint(1:len);
results.m_dot_reg.Time  = time;  results.m_dot_reg.Data  = m_dot_reg_hist;
results.m_dot_inj.Time  = time;  results.m_dot_inj.Data  = m_dot_inj_hist;
results.valve_pos.Time  = time;  results.valve_pos.Data  = valve_pos_hist;

% Additional N2O-specific outputs
results.T_N2O.Time      = time;  results.T_N2O.Data      = T_N2O_hist;
results.T_HP.Time       = time;  results.T_HP.Data       = T_1_hist;
results.P_vap.Time      = time;  results.P_vap.Data      = P_vap_hist / 1e5;
results.m_N2O_vap.Time  = time;  results.m_N2O_vap.Data  = m_N2O_vap_hist;
results.m_N2_ull.Time   = time;  results.m_N2_ull.Data   = m_N2_ull_hist;
results.quality.Time    = time;  results.quality.Data     = quality_hist;

fprintf('\n=== Simulation Complete ===\n');
fprintf('Final T_N2O = %.1f K (%.1f C)\n', T_N2O, T_N2O - 273.15);
fprintf('Final P_tank = %.1f bar\n', P_2/1e5);
fprintf('Final m_N2O_liq = %.2f kg\n', m_N2O_liq);
fprintf('N2 used = %.2f kg\n', m_1_0 - m_1);

end

%% ================================================================
%  HELPER FUNCTIONS
%  ================================================================

function T_new = solve_N2O_temperature(T_guess, U_target, m_total, m_N2_ull, ...
                                        V_tank, R_N2, T_to_rhoL, T_to_rhoV, ...
                                        T_to_uL, T_to_uV, T_min, T_max)
% Iteratively solve for T such that:
%   U_target = m_liq * u_L(T) + m_vap * u_V(T)
% subject to:
%   m_liq + m_vap = m_total
%   V_liq + V_ull = V_tank
%   m_liq = (m_total - rho_V(T) * V_tank) / (1 - rho_V(T)/rho_L(T))

    T = T_guess;
    max_iter = 15;
    tol = 1.0;  % 1 Joule tolerance on energy

    for k = 1:max_iter
        T = max(T_min, min(T_max, T));

        rho_L = T_to_rhoL(T);
        rho_V = T_to_rhoV(T);
        u_L   = T_to_uL(T);
        u_V   = T_to_uV(T);

        % Phase split
        denom = 1 - rho_V / rho_L;
        if abs(denom) < 1e-12
            % Near critical point — can't split phases
            break;
        end
        m_liq = (m_total - rho_V * V_tank) / denom;
        m_liq = max(0, min(m_total, m_liq));
        m_vap = m_total - m_liq;

        % Computed energy
        U_calc = m_liq * u_L + m_vap * u_V;

        % Residual
        residual = U_target - U_calc;

        if abs(residual) < tol
            break;
        end

        % Numerical derivative dU/dT (finite difference)
        dT_fd = 0.05;  % 50 mK step
        T_plus = min(T_max, T + dT_fd);

        rho_L_p = T_to_rhoL(T_plus);
        rho_V_p = T_to_rhoV(T_plus);
        u_L_p   = T_to_uL(T_plus);
        u_V_p   = T_to_uV(T_plus);

        denom_p = 1 - rho_V_p / rho_L_p;
        if abs(denom_p) < 1e-12, break; end
        m_liq_p = (m_total - rho_V_p * V_tank) / denom_p;
        m_liq_p = max(0, min(m_total, m_liq_p));
        m_vap_p = m_total - m_liq_p;

        U_plus = m_liq_p * u_L_p + m_vap_p * u_V_p;

        dUdT = (U_plus - U_calc) / dT_fd;

        if abs(dUdT) < 1e-3
            break;  % Flat — can't improve
        end

        % Newton step
        delta_T = residual / dUdT;
        % Dampen large steps
        delta_T = max(-2, min(2, delta_T));
        T = T + delta_T;
    end

    T_new = max(T_min, min(T_max, T));
end


function m_dot = calc_gas_flow(P_up, P_down, T_up, R, gamma, Area)
% Compressible isentropic gas flow (choked / unchoked)
    PR = P_down / P_up;
    PR_crit = (2 / (gamma + 1))^(gamma / (gamma - 1));

    if PR <= PR_crit
        % Choked flow
        C = sqrt(gamma * (2/(gamma+1))^((gamma+1)/(gamma-1)));
        m_dot = Area * P_up / sqrt(R * T_up) * C;
    else
        % Subsonic flow
        m_dot = Area * P_up / sqrt(R * T_up) * ...
                sqrt(2*gamma/(gamma-1) * (PR^(2/gamma) - PR^((gamma+1)/gamma)));
    end

    % Sanity: flow must be positive (check valve)
    m_dot = max(0, m_dot);
end
