%% SCRIPT TO PLOT E-REG TANK DRAIN SIMULATION RESULTS
%
% This script assumes:
% 1. A 'SimulationOutput' object named 'results' is in the workspace.
% 2. Signal names below match those in logsout.
% 3. 'time' and 'setpoint' arrays exist in the workspace (from Init script).
%

close all; % Close all existing figure windows

%-------------------------------------------------------
%------------- Signal Configuration --------------------
%-------------------------------------------------------

% Exact signal names found in logsout:
NOS_Pressure_Var  = 'P_HP [bar]';    % Matches 'P_HP [bar]' in logsout
Fuel_Pressure_Var = 'P_tank [bar]';  % Matches 'P_tank [bar]' in logsout
Reg_Pressure_Var  = 'P_4';           % Placeholder if logsout name differs

Valve_Angle_Var   = 'Valve angle';
Setpoint_Var      = 'setpoint';
N2_Mass_Var       = 'm_1';
Fuel_Mass_Var     = 'm_3';

%-------------------------------------------------------
%------------------- Plotting --------------------------
%-------------------------------------------------------

%% --- Plot 1: System Pressures (N2, Fuel, Reg) ---
try
    % Access 'logsout' dataset
    ds = results.logsout;

    % --- Helper Function to Extract Values safely ---
    % (Inline logic to avoid external file dependency)

    % NOS (N2)
    elem_nos = ds.get(NOS_Pressure_Var);
    if isa(elem_nos, 'Simulink.SimulationData.Dataset')
        elem_nos = elem_nos{1};
    end
    ts_nos = elem_nos.Values;

    % Fuel (Prop Tank)
    elem_fuel = ds.get(Fuel_Pressure_Var);
    if isa(elem_fuel, 'Simulink.SimulationData.Dataset')
        elem_fuel = elem_fuel{1};
    end
    ts_fuel = elem_fuel.Values;

    % Regulator
    ts_reg = [];
    try
        elem_reg = ds.get(Reg_Pressure_Var);
        if isa(elem_reg, 'Simulink.SimulationData.Dataset')
            elem_reg = elem_reg{1};
        end
        ts_reg = elem_reg.Values;
    catch
        % disp(['Signal not found: ' Reg_Pressure_Var]);
    end

    % Setpoint (from Logsout) - fallback Check
    ts_setpoint = [];
    try
        elem_sp = ds.get(Setpoint_Var);
        if isa(elem_sp, 'Simulink.SimulationData.Dataset')
            elem_sp = elem_sp{1};
        end
        ts_setpoint = elem_sp.Values;
    catch
        % Ignore if not in logsout
    end

    figure('Name', 'System Pressures');
    hold on;

    % Plot N2 (High Pressure)
    plot(ts_nos.Time, ts_nos.Data, 'r-', 'LineWidth', 1.5, 'DisplayName', 'N2');

    % Plot Fuel (Prop Tank)
    plot(ts_fuel.Time, ts_fuel.Data, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fuel (Prop)');

    % Plot Regulator Output
    if ~isempty(ts_reg)
        plot(ts_reg.Time, ts_reg.Data, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Reg Out');
    end

    % Plot Setpoint (Priority: Workspace > Logsout)
    if exist('setpoint', 'var') && exist('time', 'var')
        plot(time, setpoint, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Desired Tank Pressure');
    elseif ~isempty(ts_setpoint)
        plot(ts_setpoint.Time, ts_setpoint.Data, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Desired Tank Pressure');
    else
        disp('Note: Setpoint variable not found in workspace or logs.');
    end

    title('System Pressures');
    xlabel('Time (s)');
    ylabel('Pressure (Pa / Bar check units)');
    legend('Location', 'best', 'Interpreter', 'none');
    grid on;
    hold off;

    % Save the plot
    saveas(gcf, 'pressure_plot.png');
    disp('Saved pressure_plot.png');

catch ME
    warn_msg = sprintf('Could not plot pressures. %s', ME.message);
    warning(warn_msg);
end

%% --- Plot 2: Mass Flows (Optional / Existing) ---
try
    if isprop(results, 'fuelflow') && isprop(results, 'nitflow')
        ts_fuelflow = results.fuelflow;
        ts_nitflow  = results.nitflow;

        figure('Name', 'Mass Flows');
        yyaxis left;
        plot(ts_fuelflow.Time, ts_fuelflow.Data, 'g-', 'LineWidth', 1.5);
        ylabel('Fuel Flow (kg/s)');

        yyaxis right;
        plot(ts_nitflow.Time, ts_nitflow.Data, 'm-', 'LineWidth', 1.5);
        ylabel('Nitrogen Flow (kg/s)');

        title('Fuel and Nitrogen Mass Flow');
        xlabel('Time (s)');
        legend({'Fuel Flow', 'Nitrogen Flow'}, 'Location', 'best');
        grid on;
        % Save the plot
        saveas(gcf, 'mass_flow.png');
        disp('Saved mass_flow.png');
    end
catch
    % disp('Skipping Mass Flow plot.');
end

%% --- Plot 3: Tank Filled Volumes (Fuel & N2) ---
try
    % Access 'logsout' dataset
    ds = results.logsout;
    rho_L = 1000; % Propellant Density [kg/m^3]

    % --- Fuel Mass (m_3) ---
    ts_fuel_mass = [];
    elem_fuel = ds.get(Fuel_Mass_Var);
    if isa(elem_fuel, 'Simulink.SimulationData.Dataset')
        elem_fuel = elem_fuel{1};
    end
    ts_fuel_mass = elem_fuel.Values;

    % Convert to Liters: Mass (kg) / Density (kg/m^3) * 1000 (L/m^3)
    fuel_vol_L = (ts_fuel_mass.Data / rho_L) * 1000;

    % --- N2 Mass (m_1) ---
    ts_n2_mass = [];
    elem_n2 = ds.get(N2_Mass_Var);
    if isa(elem_n2, 'Simulink.SimulationData.Dataset')
        elem_n2 = elem_n2{1};
    end
    ts_n2_mass = elem_n2.Values;

    % Plotting
    figure('Name', 'Tank Levels');

    yyaxis left
    plot(ts_fuel_mass.Time, fuel_vol_L, 'b-', 'LineWidth', 1.5);
    ylabel('Fuel Volume [L] (Prop Tank)');
    ylim([0, max(fuel_vol_L)*1.1]); % Set reasonable limit

    yyaxis right
    plot(ts_n2_mass.Time, ts_n2_mass.Data, 'r-', 'LineWidth', 1.5);
    ylabel('N2 Mass [kg] (HP Tank)');

    title('Tank Filled Levels');
    xlabel('Time (s)');
    legend({'Fuel Volume', 'N2 Mass'}, 'Location', 'best');
    grid on;

    saveas(gcf, 'tank_levels.png');
    disp('Saved tank_levels.png');

catch ME
    warn_msg = sprintf('Could not plot Tank Levels. %s', ME.message);
    warning(warn_msg);
end

%% --- Plot 4: Servo Demand & Valve Angle ---
try
    % Access 'logsout' dataset
    ds = results.logsout;
    Servo_Demand_Var = 'servo_demand'; % Expected signal name

    % --- Servo Demand ---
    ts_servo_demand = [];
    try
        elem_sd = ds.get(Servo_Demand_Var);
        if isa(elem_sd, 'Simulink.SimulationData.Dataset')
            elem_sd = elem_sd{1};
        end
        ts_servo_demand = elem_sd.Values;
    catch
        disp(['Signal not found: ' Servo_Demand_Var]);
    end

    % --- Valve Angle (reuse checking) ---
    ts_valve_angle = [];
    try
        elem_va = ds.get(Valve_Angle_Var);
        if isa(elem_va, 'Simulink.SimulationData.Dataset')
            elem_va = elem_va{1};
        end
        ts_valve_angle = elem_va.Values;
    catch
        disp(['Signal not found: ' Valve_Angle_Var]);
    end

    % --- Servo Angle Real (Actuator) ---
    ts_servo_angle_real = [];
    try
        elem_sar = ds.get('servo angle');
        if isa(elem_sar, 'Simulink.SimulationData.Dataset')
            elem_sar = elem_sar{1};
        end
        ts_servo_angle_real = elem_sar.Values;
    catch
        % disp('Signal not found: servo angle');
    end

    if ~isempty(ts_servo_demand)
        figure('Name', 'Servo Demand');
        hold on;
        plot(ts_servo_demand.Time, ts_servo_demand.Data, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Servo Demand');

        if ~isempty(ts_valve_angle)
            plot(ts_valve_angle.Time, ts_valve_angle.Data, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Valve Angle (Flow)');
        end

        if ~isempty(ts_servo_angle_real)
            plot(ts_servo_angle_real.Time, ts_servo_angle_real.Data, 'g--', 'LineWidth', 1.2, 'DisplayName', 'Servo Angle (Actuator)');
        end

        title('Servo Diagnostics: Demand vs Actuator vs Valve');
        xlabel('Time (s)');
        ylabel('Angle [deg]');
        ylim([-10, 100]); % Crop spikes to show relevant 0-90 deg range
        legend('Location', 'best');
        grid on;
        hold off;

        saveas(gcf, 'servo_demand.png');
        disp('Saved servo_demand.png');
    else
        disp('Skipping Servo Demand plot (signal missing).');
    end

catch ME
    warn_msg = sprintf('Could not plot Servo Demand. %s', ME.message);
    warning(warn_msg);
end

disp('Plotting script finished.');