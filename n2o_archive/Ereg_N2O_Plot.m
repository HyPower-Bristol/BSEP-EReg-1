function Ereg_N2O_Plot(results)
% Ereg_N2O_Plot
% Visualizes the output from EReg_Tank_Drain_N2O_Sim

% results is a struct with fields:
% P_HP_bar, P_tank_bar, m_1, m_3, setpoint (All structs with .Time and .Data)

% Unpack for easy access
t = results.P_HP_bar.Time;
P1 = results.P_HP_bar.Data;
P2 = results.P_tank_bar.Data;
m1 = results.m_1.Data;
m3 = results.m_3.Data; % Liquid N2O Mass
sp = results.setpoint.Data;

% Figure 1: Pressures
figure('Name', 'System Pressures (N2O)');
plot(t, P1, 'r-', 'LineWidth', 1.5); hold on;
plot(t, P2, 'b-', 'LineWidth', 1.5);
plot(t, sp, 'k--', 'LineWidth', 1.2);

title('Pressures: N2 Source vs N2O Run Tank');
xlabel('Time [s]');
ylabel('Pressure [bar]');
legend('HP N2', 'Run Tank (N2O)', 'Setpoint');
ylim([0, 300]);
grid on;

saveas(gcf, 'n2o_pressure_plot.png');

% Figure 2: Masses
figure('Name', 'Propellant Masses');
yyaxis left
plot(t, m3, 'g-', 'LineWidth', 1.5);
ylabel('N2O Liquid Mass [kg]');

yyaxis right
plot(t, m1, 'm-', 'LineWidth', 1.5);
ylabel('N2 Gas Mass [kg]');

title('Tank Mass Depletion');
xlabel('Time [s]');
legend('N2O Liquid', 'N2 Source');
grid on;

saveas(gcf, 'n2o_mass_plot.png');

% Figure 3: Mass Flows
figure('Name', 'Mass Flow Rates');
plot(t, results.m_dot_reg.Data, 'r-', 'LineWidth', 1.5); hold on;
plot(t, results.m_dot_inj.Data, 'b-', 'LineWidth', 1.5);

title('Mass Flow Rates');
xlabel('Time [s]');
ylabel('Mass Flow [kg/s]');
legend('Regulator (N2)', 'Injector (N2O)');
grid on;
saveas(gcf, 'n2o_flow_plot.png');

% Figure 4: Valve Position
figure('Name', 'Valve Position');
plot(t, results.valve_pos.Data, 'k-', 'LineWidth', 1.5);
title('Regulator Valve Position');
xlabel('Time [s]');
ylabel('Angle [deg]');
ylim([0, 100]);
grid on;
saveas(gcf, 'n2o_valve_plot.png');

disp('Plots saved: n2o_pressure_plot.png, n2o_mass_plot.png, n2o_flow_plot.png, n2o_valve_plot.png');

end
