function [P_2_bar, P_HP_bar, m_dot_L, m_3, m_1, T_N2O_out] = N2O_Tank_Plant(Kv_reg, Kv_water, ...
    V_1, V_2, A_3, Cd_3, R_N2, P_atm, ...
    T_sat_LUT, P_sat_LUT, rhoL_LUT, rhoV_LUT, uL_LUT, uV_LUT, hL_LUT, ...
    m_1_0, m_N2O_liq_0, m_N2_ull_0, T_N2O_0, U_N2O_0, P_1_0, rho_1_0, T_1_0)
%#codegen
% N2O_Tank_Plant - Two-phase N2O tank plant for Simulink MATLAB Function block
%
% This function computes one time step of the N2O tank thermodynamics.
% It is designed to be called from a MATLAB Function block in Simulink.
%
% INPUTS (from Simulink signals):
%   Kv_reg      - Regulator valve Kv (from controller via servo model) [normalised 0-1]
%   Kv_water    - Water/run valve Kv (0 = closed, 1 = open)
%
% INPUTS (from workspace/parameters):
%   V_1, V_2    - Tank volumes [m^3]
%   A_3, Cd_3   - Injector area and discharge coefficient
%   R_N2        - N2 gas constant [J/kgK]
%   P_atm       - Downstream/chamber pressure [Pa]
%   T_sat_LUT, P_sat_LUT, rhoL_LUT, rhoV_LUT, uL_LUT, uV_LUT, hL_LUT
%               - N2O saturation LUT vectors (indexed by row, same length)
%
% INPUTS (persistent state, initialised from workspace):
%   m_1_0, m_N2O_liq_0, m_N2_ull_0, T_N2O_0, U_N2O_0, P_1_0, rho_1_0, T_1_0
%               - Initial conditions
%
% OUTPUTS (to Simulink):
%   P_2_bar     - Run tank total pressure [bar]
%   P_HP_bar    - HP N2 tank pressure [bar]
%   m_dot_L     - Liquid N2O mass flow through injector [kg/s]
%   m_3         - N2O liquid mass remaining [kg]
%   m_1         - N2 mass remaining in HP tank [kg]
%   T_N2O_out   - N2O liquid temperature [K]

% ---- Persistent State Variables ----
persistent m1 m_N2O_liq m_N2O_vap m_N2_ull T_N2O U_N2O rho1 T1 m_N2O_total initialised

if isempty(initialised) || initialised == 0
    m1          = m_1_0;
    m_N2O_liq   = m_N2O_liq_0;
    T_N2O       = T_N2O_0;
    U_N2O       = U_N2O_0;
    rho1        = rho_1_0;
    T1          = T_1_0;
    m_N2_ull    = m_N2_ull_0;

    % Compute initial vapour mass
    rhoV_init   = interp1_lut(T_sat_LUT, rhoV_LUT, T_N2O);
    rhoL_init   = interp1_lut(T_sat_LUT, rhoL_LUT, T_N2O);
    V_liq_init  = m_N2O_liq / rhoL_init;
    V_ull_init  = V_2 - V_liq_init;
    m_N2O_vap   = rhoV_init * max(V_ull_init, 1e-6);
    m_N2O_total = m_N2O_liq + m_N2O_vap;

    initialised = 1;
end

% ---- Sample Time ----
dt = 0.001;  % Must match Simulink fixed-step solver (1 ms)

% ---- Polytropic N2 index ----
n_poly = 1.3;
gamma_N2 = 1.4;

% ---- Clamp T_N2O to LUT range ----
T_min = T_sat_LUT(1)   + 0.5;
T_max = T_sat_LUT(end) - 2.0;
T_N2O = max(T_min, min(T_max, T_N2O));

% ======== HP TANK (N2 source) ========
rho1 = m1 / V_1;
T1   = T_1_0 * (rho1 / rho_1_0)^(n_poly - 1);
T1   = max(T1, 200);
P_1  = rho1 * R_N2 * T1;

% ======== RUN TANK (N2O + N2 ullage) ========
P_vap = interp1_lut(T_sat_LUT, P_sat_LUT, T_N2O);
rho_L = interp1_lut(T_sat_LUT, rhoL_LUT, T_N2O);
rho_V = interp1_lut(T_sat_LUT, rhoV_LUT, T_N2O);
u_L   = interp1_lut(T_sat_LUT, uL_LUT,   T_N2O);
u_V   = interp1_lut(T_sat_LUT, uV_LUT,   T_N2O);
h_L   = interp1_lut(T_sat_LUT, hL_LUT,   T_N2O);

% Volumes
if m_N2O_liq > 0.001
    V_liq = m_N2O_liq / rho_L;
    V_ull = max(V_2 - V_liq, 1e-6);
else
    V_liq = 0;
    V_ull = V_2;
    m_N2O_liq = 0;
end

% Partial pressures (Dalton's Law)
P_N2_partial = (m_N2_ull * R_N2 * T_N2O) / V_ull;
P_2 = P_N2_partial + P_vap;

% ======== FLOW CALCULATIONS ========

% --- Regulator: N2 from HP -> Run tank ullage ---
Kv_max_area = 1.5;  % Must match init file Kv_1_max
A_valve_max = 2.4e-5 * Kv_max_area;
A_reg_eff   = A_valve_max * Kv_reg;  % Kv_reg is 0-1 normalised

if P_1 > P_2 && A_reg_eff > 0
    m_dot_reg = gas_flow(P_1, P_2, T1, R_N2, gamma_N2, A_reg_eff);
else
    m_dot_reg = 0;
end

% --- Injector: Liquid N2O from tank -> chamber ---
if Kv_water > 0.5 && m_N2O_liq > 0.001
    dP_inj = P_2 - P_atm;
    if dP_inj > 0
        m_dot_inj = Cd_3 * A_3 * sqrt(2 * rho_L * dP_inj);
    else
        m_dot_inj = 0;
    end
else
    m_dot_inj = 0;
end

% ======== STATE UPDATE (Forward Euler) ========

% Mass updates
m1        = max(0, m1 - m_dot_reg * dt);
m_N2_ull  = m_N2_ull + m_dot_reg * dt;
m_N2O_liq = max(0, m_N2O_liq - m_dot_inj * dt);
m_N2O_total = m_N2O_liq + m_N2O_vap;

% Energy balance
dU_out      = m_dot_inj * h_L * dt;
Cp_N2       = R_N2 * gamma_N2 / (gamma_N2 - 1);
dU_N2_heat  = m_dot_reg * Cp_N2 * (T1 - T_N2O) * dt * 0.1;
U_N2O       = U_N2O - dU_out + dU_N2_heat;

% Solve for new temperature (simplified Newton — 5 iterations max)
if m_N2O_liq > 0.001
    T_N2O = solve_T(T_N2O, U_N2O, m_N2O_total, V_2, ...
                     T_sat_LUT, rhoL_LUT, rhoV_LUT, uL_LUT, uV_LUT, T_min, T_max);

    % Recompute phase split
    rho_L_n = interp1_lut(T_sat_LUT, rhoL_LUT, T_N2O);
    rho_V_n = interp1_lut(T_sat_LUT, rhoV_LUT, T_N2O);
    denom = 1 - rho_V_n / rho_L_n;
    if abs(denom) > 1e-9
        m_N2O_liq = max(0, (m_N2O_total - rho_V_n * V_2) / denom);
    end
    m_N2O_vap = max(0, m_N2O_total - m_N2O_liq);
end

% ======== OUTPUTS ========
P_2_bar   = P_2 / 1e5;
P_HP_bar  = P_1 / 1e5;
m_dot_L   = m_dot_inj;
m_3       = m_N2O_liq;
m_1       = m1;
T_N2O_out = T_N2O;

end

% ======== LOCAL FUNCTIONS ========

function y = interp1_lut(x_vec, y_vec, x_q)
%#codegen
% Linear interpolation with clamping (codegen compatible)
    n = length(x_vec);
    if x_q <= x_vec(1)
        y = y_vec(1);
        return;
    end
    if x_q >= x_vec(n)
        y = y_vec(n);
        return;
    end
    % Binary search
    lo = 1; hi = n;
    while hi - lo > 1
        mid = floor((lo + hi) / 2);
        if x_vec(mid) <= x_q
            lo = mid;
        else
            hi = mid;
        end
    end
    frac = (x_q - x_vec(lo)) / (x_vec(hi) - x_vec(lo));
    y = y_vec(lo) + frac * (y_vec(hi) - y_vec(lo));
end

function m_dot = gas_flow(P_up, P_down, T_up, R, gamma, Area)
%#codegen
    PR = P_down / P_up;
    PR_crit = (2/(gamma+1))^(gamma/(gamma-1));
    if PR <= PR_crit
        C = sqrt(gamma * (2/(gamma+1))^((gamma+1)/(gamma-1)));
        m_dot = Area * P_up / sqrt(R * T_up) * C;
    else
        m_dot = Area * P_up / sqrt(R * T_up) * ...
                sqrt(2*gamma/(gamma-1) * (PR^(2/gamma) - PR^((gamma+1)/gamma)));
    end
    m_dot = max(0, m_dot);
end

function T_new = solve_T(T_g, U_tgt, m_tot, V_tank, T_vec, rhoL_vec, rhoV_vec, uL_vec, uV_vec, T_min, T_max)
%#codegen
    T = T_g;
    for k = 1:5
        T = max(T_min, min(T_max, T));
        rL = interp1_lut(T_vec, rhoL_vec, T);
        rV = interp1_lut(T_vec, rhoV_vec, T);
        uL = interp1_lut(T_vec, uL_vec, T);
        uV = interp1_lut(T_vec, uV_vec, T);

        d = 1 - rV / rL;
        if abs(d) < 1e-12; break; end
        ml = max(0, min(m_tot, (m_tot - rV * V_tank) / d));
        mv = m_tot - ml;
        U_c = ml * uL + mv * uV;
        res = U_tgt - U_c;
        if abs(res) < 1.0; break; end

        dT = 0.05;
        Tp = min(T_max, T + dT);
        rLp = interp1_lut(T_vec, rhoL_vec, Tp);
        rVp = interp1_lut(T_vec, rhoV_vec, Tp);
        uLp = interp1_lut(T_vec, uL_vec, Tp);
        uVp = interp1_lut(T_vec, uV_vec, Tp);
        dp = 1 - rVp / rLp;
        if abs(dp) < 1e-12; break; end
        mlp = max(0, min(m_tot, (m_tot - rVp * V_tank) / dp));
        mvp = m_tot - mlp;
        Up = mlp * uLp + mvp * uVp;
        dUdT = (Up - U_c) / dT;
        if abs(dUdT) < 1e-3; break; end

        step = max(-2, min(2, res / dUdT));
        T = T + step;
    end
    T_new = max(T_min, min(T_max, T));
end
