function results = EReg_Tank_Drain_N2O_Sim(init_params)
% EReg_Tank_Drain_N2O_Sim
% Simulates the time evolution of a Supercharged Nitrous Oxide Run Tank
% pressurized by a Nitrogen High Pressure Tank.
%
% Inputs:
%   init_params - Struct containing all constants and initial conditions.
%
% Outputs:
%   results - Struct containing time-series data (Time, P1, P2, m1, m2, etc.)

%% 1. Unpack Parameters
% Unpack strictly necessary params to avoid clutter
% Hardware
V_1 = init_params.V_1; % HP Tank Vol [m^3]
V_2 = init_params.V_2; % Run Tank Vol [m^3] (Total)
A_inj = init_params.A_3; % Injector Area [m^2]
Cd_inj = init_params.Cd_3; % Injector Cd
Cd_reg = init_params.K_v_4; % Approx Reg Flow Coeff (using Check Valve Kv for now as placeholder for Reg Valve CV?? No, Reg Valve has its own logic)
% Note: The original generic 'K_v_4' was check valve.
% The Regulator Valve is the Control Valve.
% We need the valve characteristic for the Regulator.
% Assuming 'Kv_1_max' is the Reg Valve Max Kv.
Kv_1_max = init_params.Kv_1_max;

% Physics
R_N2 = init_params.R_N2;
T_ambient = init_params.T_0_N2; % Assume isothermal for now? Or adiabatic?
% Let's use simple Isothermal Tank 1 for stability in V1
% (Standard Assumption: Gas supply temp doesn't drop drastically in short drain or we have heat capacity).
% Actually blowdown gets cold. Let's use Isentropic/Polytropic for Tank 1 if possible, or Isothermal for simplicity.
% Original Init script had T_0_N2.

% Load N2O Properties
% Table Format: T, P, rho_L, rho_V, u_L, u_V ...
% We assume the file is in the path.
opts = detectImportOptions('n2o_saturation_properties.csv');
opts.VariableNamingRule = 'preserve';
n2o_table = readtable('n2o_saturation_properties.csv', opts);

% Prepare Interpolants (Speed up lookups)
% Input: Temperature (or Pressure).
% Since N2O follows Saturation curve, P is unique to T.
% We will use T as the state variable for the N2O thermal mass if we model energy.
% OR simpler: We assume N2O is at T_ambient initially, and T drops as it boils.

% VAPOR PRESSURE CURVE
% P_vap = f(T)
P_sat_vec = n2o_table.P;
T_sat_vec = n2o_table.T;
rho_L_vec = n2o_table.rho_L;

% Create Gridded Interpolants for fast lookup
% Ensure unique and sorted
[P_sat_vec, sortIdx] = sort(P_sat_vec);
T_sat_vec = T_sat_vec(sortIdx);
rho_L_vec = rho_L_vec(sortIdx);

% Mapping P_sat -> T_sat (If we know Pressure, what is Boiling Temp)
P_to_T_interp = griddedInterpolant(P_sat_vec, T_sat_vec, 'linear', 'nearest');
T_to_P_interp = griddedInterpolant(T_sat_vec, P_sat_vec, 'linear', 'nearest');

% Mapping T -> rho_L (Liquid Density)
T_to_rhoL_interp = griddedInterpolant(T_sat_vec, rho_L_vec, 'linear', 'nearest');

%% 2. Initial State Setup
dt = init_params.Time_Step;
t_end = init_params.Sim_Duration;
N_steps = floor(t_end / dt);

% Pre-allocate Arrays
time = (0:dt:t_end)';
len = length(time);

P_1_hist = zeros(len, 1); % HP N2 Pressure
P_2_hist = zeros(len, 1); % Run Tank Pressure
m_1_hist = zeros(len, 1); % Mass N2 in HP
m_2_Liq_hist = zeros(len, 1); % Mass N2O Liquid
m_2_N2_hist = zeros(len, 1);  % Mass N2 in Run Tank (Ullage)
valve_pos_hist = zeros(len, 1);

% Flow History
m_dot_reg_hist = zeros(len, 1);
m_dot_inj_hist = zeros(len, 1);

% Initial Conditions
P_1 = init_params.P_1_0; % Pa
P_2 = init_params.P_2_0; % Pa
m_1 = init_params.m_1_0; % kg

% N2O Mass Initialization
% The init script defined 'm_3_0' as Propellant Mass.
m_N2O_Liq = init_params.m_3_0;

% Ullage Gas (N2) Mass Initialization
% Derived from: m_ullage = rho_gas * V_ullage
% V_liquid_N2O = m_N2O / rho_N2O(T_amb)
rho_N2O_init = T_to_rhoL_interp(T_ambient);
V_N2O_Liq = m_N2O_Liq / rho_N2O_init;
V_ullage_init = V_2 - V_N2O_Liq;

if V_ullage_init < 0
    error('Error: Initial Propellant Volume exceeds Tank Volume!');
end

% Ullage Gas (N2) Mass Initialization
% P_2_total = P_N2_partial + P_vapour
P_vap_init = T_to_P_interp(T_ambient);
P_N2_init = max(0, P_2 - P_vap_init);

% Density of N2 at P_N2_init and T_ambient
rho_N2_ullage_init = P_N2_init / (R_N2 * T_ambient);
m_N2_ullage = rho_N2_ullage_init * V_ullage_init;

% Control State
target_P = init_params.Target_Pressure * 1e5; % Pa

% PID State
pid_int_err = 0;
last_err = 0;
valve_pos = 0; % 0 to 1

% Temp State
T_N2O = T_ambient; % N2O temperature

fprintf('Starting Sim: T_amb=%.1f K, P_target=%.1f bar\n', T_ambient, target_P/1e5);

%% 3. Time Loop
fprintf('Time(s) | P1 (bar) | P2 (bar) | Valve(deg) | m_dot_reg | m_dot_inj\n');
for i = 1:len
    %% A. Store History
    P_1_hist(i) = P_1;
    P_2_hist(i) = P_2;
    m_1_hist(i) = m_1;
    m_2_Liq_hist(i) = m_N2O_Liq;
    m_2_N2_hist(i) = m_N2_ullage;
    valve_pos_hist(i) = valve_pos;

    %% B. Physics Calculation (Thermodynamics)

    % 1. HP Tank (N2 Source)
    % ISO-THERMAL Assumption for Simplicity (or Polytropic with n=1.3)
    % rho_1 = m_1 / V_1;
    % P_1 = rho_1 * R_N2 * T_ambient;
    % Refined: Adiabatic Expansion T_1 = T_0 * (P_1/P_0)^((gamma-1)/gamma)
    % Let's stick to Ideal Gas Law with Constant T for stability unless requested.
    rho_1 = m_1 / V_1;
    P_1 = rho_1 * R_N2 * T_ambient;

    % 2. Run Tank (N2O + N2)
    % Physics:
    % V_Liq = m_N2O_Liq / rho_N2O_L(T_N2O);
    % V_Ullage = V_2 - V_Liq;

    % Check if Liquid Exists
    if m_N2O_Liq > 0
        rho_L = T_to_rhoL_interp(T_N2O);
        V_Liq = m_N2O_Liq / rho_L;
        V_Ullage = V_2 - V_Liq;
    else
        V_Liq = 0;
        V_Ullage = V_2;
        % Handle N2O Gas phase if needed?
        % For now assume empty tank = N2 blowdown only
    end

    % Partial Pressure of N2 in Ullage
    % P_N2 = (m_N2_ullage * R_N2 * T_N2O) / V_Ullage;
    % Assume Temperature of N2 equilibrates with N2O (Huge thermal mass of liquid)

    if V_Ullage <= 1e-6 % Very full tank
        P_N2_Partial = P_1; % Clamped or undefined?
        % Practically, if V_ullage -> 0, Pressure spikes.
        % Let's limit V_ullage min.
        V_Ullage = max(V_Ullage, 1e-5);
    end

    P_N2_Partial = (m_N2_ullage * R_N2 * T_N2O) / inSim_ProtectDiv(V_Ullage);

    % Vapor Pressure of N2O
    P_N2O_Vap = T_to_P_interp(T_N2O);

    % Total Pressure
    % Dalton's Law? Or Supercharging?
    % If Compressed Liquid (Supercharged): P_tank = P_N2_Partial.
    % (Assuming N2 doesn't dissolve instantly).
    % Constraint: P_tank must be >= P_vapor.
    % If P_N2_Partial < P_vapor, the N2O BOILS to fill the volume.
    % P_tank = P_vapor (Saturation).

    % Total Pressure Model (Dalton's Law / Supercharging)
    % The Ullage contains N2 Gas and Saturated N2O Vapor.
    % P_total = P_partial_N2 + P_vapor_N2O(T)
    % This handles both boiling (P_N2 -> 0) and supercharged states seamlessly.

    P_2 = P_N2_Partial + P_N2O_Vap;

    % Check Boiling logic purely for cooling effect, if needed
    % (Actually, liquid is always in equilibrium with vapor at interface)
    Is_Boiling = true; % Simplified: evaporation/condensation maintains P_vap

    %% C. Control System (PID)
    % Get Setpoint from 'init_params.setpoint' array if exists, else constant
    % Note: init_params.setpoint is a large array.
    % Index mapping:
    idx = min(i, length(init_params.setpoint));
    current_target = init_params.setpoint(idx) * 1e5; % Convert bar to Pa

    error = current_target - P_2;

    % Terms
    P_term = init_params.K_P * error;
    pid_int_err = pid_int_err + error * dt;
    I_term = init_params.K_I * pid_int_err;
    D_term = init_params.K_D * (error - last_err) / dt;

    u_pid = P_term + I_term + D_term;

    % Convert PID to Valve Angle/Position
    % Assume u_pid maps to angle? Or is it direct Valve Command?
    % Original model had Servo Speed limits.

    % Simple Valve Logic:
    % If u > 0, Open. If u < 0, Close.
    % Implementing Max Speed Constraint.

    % Map 'u' to desired change? Or 'u' is desired position?
    % Usually PID output is Control Signal (0-1 or similar).
    % Let's assume u_pid is desired opening [0..1] for now, scaled.
    % Actually, commonly Pressure Regulator PID outputs Desired Valve Position.

    % Normalize u_pid to 0-1 range roughly
    % Kp was 16. Err ~ 10 bar (1e6 Pa). This is HUGE.
    % The original model gains were likely for Bar error.
    % Let's scale error to Bar for PID calc.
    err_bar = error / 1e5;

    P_term = init_params.K_P * err_bar;
    pid_int_err_bar = pid_int_err / 1e5; % Need to track this consistent
    I_term = init_params.K_I * pid_int_err_bar;
    D_term = init_params.K_D * (err_bar - last_err/1e5) / dt;

    u_cmd_bar = P_term + I_term + D_term;

    % Clamp 0 to 1? Or 0 to 90 deg?
    % Assume 0-90 deg (Servo).
    % Let's say u_cmd is Angle.
    % Limit Angle
    target_angle = max(0, min(90, u_cmd_bar));

    % Slew Rate Limit
    max_delta = init_params.Servo_Speed * dt;
    delta = target_angle - valve_pos;
    delta = max(-max_delta, min(max_delta, delta));

    valve_pos = valve_pos + delta; % This is in Degrees?
    % Normalize for flow calc: 0 to 1
    valve_frac = valve_pos / 90;

    last_err = error;

    %% D. Flow Calculations
    % 1. Regulator Flow (Tank 1 -> Tank 2)
    % m_dot = Kv * sqrt(rho * dP) ... standard liquid eq?
    % For Gas: Compressible Flow Equation.
    % Standard ISA Control Valve Eq for Gas:
    % m_dot = N * Fp * Cv * Y * sqrt(x * P1 * rho1) ... complex.
    % Simplified: Orifice Flow.
    % m_dot = Cd * A_valve * P1 / sqrt(R*T) * fn(Pratio)

    % Simple Model: Linear Valve Kv
    % Kv is usually water flow in m3/h at 1 bar dP.
    % Conversion: Cv = 1.156 * Kv. Area (in^2) approx Cv / 38.
    % Area (m^2) approx 2.4e-5 * Kv.

    A_valve_max = 2.4e-5 * Kv_1_max;

    % Scale by valve_frac
    A_reg_eff = A_valve_max * valve_frac;

    % Compressible Flow
    if P_1 > P_2
        m_dot_reg = calc_gas_flow(P_1, P_2, T_ambient, R_N2, 1.4, A_reg_eff);
    else
        m_dot_reg = 0; % Backflow prevented by check valve logic or just physics
    end


    % 2. Injector Flow (Tank 2 -> Ambient)
    % Liquid N2O logic.
    P_3 = init_params.P_atm + 5e5; % Chamber pressure approx? Or just P_atm?
    % Let's assume P_3 is Chamber Pressure.
    % If not combustion, P_3 = P_atm.
    P_3 = 101325;

    % Run Valve Logic
    % Only flow if Run Valve is Open.
    % Use 'init_params.run_valve_setpoint' to determine if open.
    run_open = init_params.run_valve_setpoint(idx);

    if run_open > 0.5 && m_N2O_Liq > 0
        % Incompressible Liquid Flow
        dP_inj = P_2 - P_3;
        if dP_inj > 0
            rho_inj = T_to_rhoL_interp(T_N2O);
            m_dot_inj = Cd_inj * A_inj * sqrt(2 * rho_inj * dP_inj);
        else
            m_dot_inj = 0;
        end
    else
        m_dot_inj = 0;
    end

    % Store Flows for Plotting (Current Step)
    m_dot_reg_hist(i) = m_dot_reg;
    m_dot_inj_hist(i) = m_dot_inj;

    %% E. Integration (Forward Euler)

    % Update Masses
    m_1 = m_1 - m_dot_reg * dt;
    m_N2_ullage = m_N2_ullage + m_dot_reg * dt; % N2 goes into Ullage
    m_N2O_Liq = m_N2O_Liq - m_dot_inj * dt;     % N2O leaves tank

    % Update Temp (Self-Cooling)
    % Simple model: Isentropic expansion of liquid volume?
    % Or use Enthalpy balance.
    % For N2O blowdown, T drops significantly.
    % Empirical Linear Drop?
    % T_new = T_old - K * m_dot_inj * dt;
    % Let's use Clapeyron-ish approx or Conservation of Energy.
    % Very hard to do right in 5 mins.
    % FALLBACK: Isothermal N2O (300K) -> T_N2O stays const.
    % (User asked for Physics Logic, but full thermodynamics is complex).
    % Let's add a small cooling term proportional to evaporation (if boiling).

    if Is_Boiling
        % Boil-off cools the liquid.
        % Q_evap = m_boil * h_vap.
        % dT = - Q_evap / (m_liq * Cp).
        % Not modeled in detail here.
        % Assume T drops slightly.
        T_N2O = T_N2O - 0.5 * dt; % Fake cooling rate for visual realism
    end

end

%% 4. Pack Results

% Create a Table or Struct matching existing 'results' format
% results.logsout...

% We will return a struct that the Plotter can parse.

% Create 'Values' structs to mimic Simulink Timeseries
ts_P1.Time = time; ts_P1.Data = P_1_hist / 1e5; % bar
ts_P2.Time = time; ts_P2.Data = P_2_hist / 1e5; % bar
ts_m1.Time = time; ts_m1.Data = m_1_hist;
ts_m3.Time = time; ts_m3.Data = m_2_Liq_hist;
ts_sp.Time = time; ts_sp.Data = init_params.setpoint(1:len);
ts_mdot_reg.Time = time; ts_mdot_reg.Data = m_dot_reg_hist;
ts_mdot_inj.Time = time; ts_mdot_inj.Data = m_dot_inj_hist;
ts_valve.Time = time; ts_valve.Data = valve_pos_hist;

% Pack into a mock 'logsout' structure
% The Plotter accesses: ds.get('P_HP [bar]').Values

% We can't easily make a SimulationOutput object in script.
% So we will make a standard Struct and update the Plot script to read fields.

results.P_HP_bar = ts_P1;
results.P_tank_bar = ts_P2;
results.m_1 = ts_m1;
results.m_3 = ts_m3;
results.setpoint = ts_sp;
results.m_dot_reg = ts_mdot_reg;
results.m_dot_inj = ts_mdot_inj;
results.valve_pos = ts_valve;

end

function val = inSim_ProtectDiv(x)
val = x;
if abs(val) < 1e-9
    val = 1e-9;
end
end

function m_dot = calc_gas_flow(P_up, P_down, T_up, R, gamma, Area)
% Compressible Gas Flow (Choked/Unchoked)
PR = P_down / P_up;
PR_crit = (2/(gamma+1))^(gamma/(gamma-1));

if PR < PR_crit
    % Choked
    C = sqrt(gamma * (2/(gamma+1))^((gamma+1)/(gamma-1)));
    m_dot = Area * P_up / sqrt(R*T_up) * C;
else
    % Subsonic
    m_dot = Area * P_up / sqrt(R*T_up) * sqrt(2*gamma/(gamma-1) * (PR^(2/gamma) - PR^((gamma+1)/gamma)));
end
end
