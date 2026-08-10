function [P_3_pa, P_4_pa, m_dot_L, m_dot_N2] = fcn(P_1_pa, P_2_pa, T, K_v_1, K_v_2, Cd_3, K_v_4, rho_L, R_N2, Tst, rhost, A_3, Pst)
%#codegen

% This function solves for the intermediate pressures and mass flows in a
% fluid system. It replaces a non-codegen-compatible 'fsolve' call with
% two independent 1D Newton-Raphson solvers.

% --- Solver Parameters ---
MAX_ITER = 50;  % Maximum iterations for Newton's method
TOLERANCE = 1e-7; % Convergence tolerance
h = 1e-6;       % Step size for numerical derivative
P_atm_pa = 101325;
% --- Input Conversions ---
% Convert input pressures from Pascals (pa) to bar for K_v calcs
P_1 = P_1_pa * 1e-5;
P_2 = P_2_pa * 1e-5;
% Note: P_atm_pa is passed directly to calc_m_dot_b, which expects Pascals

% --- Problem Setup ---
% Calculate Specific Gravity for liquid flow
SG = rho_L / 1000.0;

% --- 1. Solve for Liquid-Side Pressure P_3 ---
% We need to find P_3 (in bar) such that: liquid_error(P_3) = 0
% where liquid_error = calc_m_dot_a(P_3) - calc_m_dot_b(P_3)

% Define the error function for the liquid side
% All parameters are passed anonymously
liquid_error_fun = @(p3) (calc_m_dot_a(p3, P_2, K_v_2, rho_L, SG) - ...
                          calc_m_dot_b(p3, Cd_3, A_3, rho_L, P_atm_pa));

% Initial guess for P_3 (in bar)
x0_P3 = 1.0; 

% Call the Newton solver
P_3_bar = newton_solver(liquid_error_fun, x0_P3, MAX_ITER, TOLERANCE, h);

% --- 2. Solve for Gas-Side Pressure P_4 ---
% We need to find P_4 (in bar) such that: gas_error(P_4) = 0
% where gas_error = calc_m_dot_c(P_4) - calc_m_dot_d(P_4)

% Define the error function for the gas side
gas_error_fun = @(p4) (calc_m_dot_c(p4, P_1, K_v_1, rhost, T, Pst, R_N2, Tst) - ...
                       calc_m_dot_d(p4, P_2, K_v_4, rhost, T, Pst, R_N2, Tst));
                       
% Initial guess for P_4 (in bar)
x0_P4 = 1.0; 

% Call the Newton solver
P_4_bar = newton_solver(gas_error_fun, x0_P4, MAX_ITER, TOLERANCE, h);

% --- 3. Calculate Final Outputs ---
% Convert pressures back to Pascals
P_3_pa = P_3_bar * 1e5;
P_4_pa = P_4_bar * 1e5;

% Calculate the final mass flow rates using the solved pressures
m_dot_L  = calc_m_dot_a(P_3_bar, P_2, K_v_2, rho_L, SG);
m_dot_N2 = calc_m_dot_c(P_4_bar, P_1, K_v_1, rhost, T, Pst, R_N2, Tst);

end

% =========================================================================
% --- NESTED HELPER FUNCTIONS (CODEGEN-COMPATIBLE) ---
% =========================================================================

function x_root = newton_solver(fun, x0, max_iter, tol, h)
% A simple 1D Newton-Raphson solver using a numerical derivative.
% It finds x such that fun(x) = 0.
    
    x_root = x0;
    for i = 1:max_iter
        % Calculate function value at current point
        f_val = fun(x_root);
        
        % Check for convergence
        if abs(f_val) < tol
            return; % Solution found
        end
        
        % Calculate function value at a small step h away
        f_val_h = fun(x_root + h);
        
        % Calculate numerical derivative (Jacobian)
        f_deriv = (f_val_h - f_val) / h;
        
        % Check for division by zero (flat gradient)
        if abs(f_deriv) < 1e-10
            % Cannot converge, return last best guess
            return;
        end
        
        % Newton's step
        x_new = x_root - f_val / f_deriv;
        
        % Check for step convergence
        if abs(x_new - x_root) < tol
            x_root = x_new;
            return; % Solution found
        end
        
        x_root = x_new;
    end
    % If loop finishes, max iterations were reached. Return last value.
end


function m_dot_a = calc_m_dot_a(P_3, P_2, K_v_2, rho_L, SG)
% Water Control Valve (m_dot_a)
% All pressures in bar
    delta_P = P_2 - P_3;
    % Use sign(delta_P) to handle potential flow reversal safely
    Q = K_v_2 * (delta_P / SG) / sqrt(abs(delta_P / SG) + 1e-9); 
    m_dot_a = Q * rho_L * (1/3600); % Convert standard flow rate to mass flow rate
end


function m_dot_b = calc_m_dot_b(P_3_bar, Cd_3, A_3, rho_L, P_atm_pa)
% Water Orifice (m_dot_b)
% P_3_bar is in bar, P_atm_pa is in Pascals
    
    % Convert P_3 from bar to Pascals for this equation
    P_3_pa = P_3_bar * 1e5;
    delta_P_pa = P_3_pa - P_atm_pa;
    
    % This formulation from your original code correctly handles flow
    % direction using sign(delta_P) * sqrt(abs(delta_P))
    m_dot_b = Cd_3 * A_3 * 2 * rho_L * delta_P_pa / sqrt(abs(2 * rho_L * delta_P_pa) + 1e-9);
end


function m_dot_c = calc_m_dot_c(P_4, P_1, K_v_1, rhost, T, Pst, R_N2, Tst)
% Regulator Valve (m_dot_c)
% All pressures in bar
    Q_N = 0.0;
    if (P_4 >= 0.528 * P_1)
        % Non-choked flow
        arg = (P_1 - P_4) * P_4 / (rhost * T);
        Q_N = K_v_1 * 514 * arg / sqrt(abs(arg) + 1e-9);
    else
        % Choked flow
        Q_N = K_v_1 * 257 * P_1 / sqrt(abs(rhost * T) + 1e-9);
    end
    m_dot_c = Q_N * (Pst / (R_N2 * Tst)) * (1/3600); % Convert standard flow rate to mass flow rate
end


function m_dot_d = calc_m_dot_d(P_4, P_2, K_v_4, rhost, T, Pst, R_N2, Tst)
% Check Valve (m_dot_d)
% All pressures in bar
    Q_N = 0.0;
    if (P_2 >= 0.528 * P_4)
        % Non-choked flow
        arg = (P_4 - P_2) * P_2 / (rhost * T);
        Q_N = K_v_4 * 514 * arg / sqrt(abs(arg) + 1e-9);
    else
        % Choked flow
        Q_N = K_v_4 * 257 * P_4 / sqrt(abs(rhost * T) + 1e-9);
    end
    m_dot_d = Q_N * (Pst / (R_N2 * Tst)) * (1/3600); % Convert standard flow rate to mass flow rate
end