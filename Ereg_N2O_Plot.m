function Ereg_N2O_Plot(results)
% Ereg_N2O_Plot
% Visualises the output from EReg_Tank_Drain_N2O_Sim
%
% Produces 7 figures:
%   1. System Pressures (full scale)
%   2. Run Tank Pressure ZOOMED (tracking detail + relight)
%   3. N2O Temperature
%   4. Propellant Masses
%   5. Mass Flow Rates
%   6. Valve Position
%   7. N2O Quality

%% Unpack
t   = results.P_HP_bar.Time;
P1  = results.P_HP_bar.Data;
P2  = results.P_tank_bar.Data;
sp  = results.setpoint.Data;
m1  = results.m_1.Data;
m3  = results.m_3.Data;

has_extended = isfield(results, 'T_N2O');

if has_extended
    T_N2O     = results.T_N2O.Data;
    P_vap     = results.P_vap.Data;
    m_vap     = results.m_N2O_vap.Data;
    m_N2_ull  = results.m_N2_ull.Data;
    quality   = results.quality.Data;
    T_HP      = results.T_HP.Data;
end

%% ---- Figure 1: System Pressures (Full Scale) ----
figure('Name', 'System Pressures (Full)', 'Position', [50 500 800 400]);
hold on;
plot(t, P1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'HP N2');
plot(t, P2, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Run Tank (Total)');
plot(t, sp, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Setpoint');
if has_extended
    plot(t, P_vap, 'm:', 'LineWidth', 1.2, 'DisplayName', 'N2O Vapour Pressure');
end
title('System Pressures (Full Scale)');
xlabel('Time [s]'); ylabel('Pressure [bar]');
legend('Location', 'northeast'); grid on; hold off;
saveas(gcf, 'n2o_pressure_plot.png');

%% ---- Figure 2: Run Tank Pressure ZOOMED ----
figure('Name', 'Run Tank Pressure (Zoomed)', 'Position', [100 450 900 450]);
hold on;

% Shade the relight region if detectable
% (Look for run valve closing mid-sim by checking setpoint flatness)
rv = results.m_dot_inj.Data;

plot(t, P2, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Run Tank Pressure');
plot(t, sp, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Setpoint');
if has_extended
    plot(t, P_vap, 'm:', 'LineWidth', 1.5, 'DisplayName', 'Vapour Pressure');
end

% Tracking error
err = P2 - sp';
err = err(:);

% Find relight region (where injector flow drops to zero mid-sim)
% Shade it for visual clarity
inj_flow = results.m_dot_inj.Data;
% Simple detection: find regions after t=2s where flow is zero
is_zero = (inj_flow < 0.001) & (t > 2);
transitions = diff([0; is_zero; 0]);
starts = find(transitions == 1);
ends   = find(transitions == -1) - 1;

for k = 1:length(starts)
    if starts(k) <= length(t) && ends(k) <= length(t)
        t_start = t(starts(k));
        t_end   = t(ends(k));
        if (t_end - t_start) > 0.1 && (t_end - t_start) < 3
            % This looks like a relight pause
            yl = ylim;
            patch([t_start t_end t_end t_start], ...
                  [yl(1) yl(1) yl(2) yl(2)], ...
                  [1 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
                  'DisplayName', 'Relight Pause');
        end
    end
end

% Set y-axis to zoom around the operating region
P2_min = min(min(P2), min(sp)) - 3;
P2_max = max(max(P2), max(sp)) + 3;
ylim([max(0, P2_min), P2_max]);

title('Run Tank Pressure - Tracking Detail');
xlabel('Time [s]'); ylabel('Pressure [bar]');
legend('Location', 'best'); grid on; hold off;
saveas(gcf, 'n2o_pressure_zoomed.png');

%% ---- Figure 3: Tracking Error ----
figure('Name', 'Tracking Error', 'Position', [150 400 800 350]);
plot(t, err, 'r-', 'LineWidth', 1.2);
yline(0, 'k--', 'LineWidth', 0.8);
yline(1, 'k:', 'LineWidth', 0.5);
yline(-1, 'k:', 'LineWidth', 0.5);
yline(3, 'r:', 'LineWidth', 0.5);
yline(-3, 'r:', 'LineWidth', 0.5);
title('Pressure Tracking Error (P_{tank} - Setpoint)');
xlabel('Time [s]'); ylabel('Error [bar]');
legend('Error', 'Zero', '\pm1 bar (SS spec)', '\pm3 bar (Transient spec)', ...
       'Location', 'best');
grid on;
saveas(gcf, 'n2o_tracking_error.png');

%% ---- Figure 4: N2O Temperature ----
if has_extended
    figure('Name', 'Fluid Temperatures', 'Position', [200 350 800 400]);
    yyaxis left
    plot(t, T_N2O, 'b-', 'LineWidth', 1.5);
    ylabel('N2O Liquid Temperature [K]');

    yyaxis right
    plot(t, T_HP, 'r-', 'LineWidth', 1.5);
    ylabel('HP N2 Temperature [K]');

    title('Fluid Temperatures');
    xlabel('Time [s]');
    legend('N2O Liquid', 'HP N2 Gas', 'Location', 'best');
    grid on;
    saveas(gcf, 'n2o_temperature_plot.png');
end

%% ---- Figure 5: Propellant Masses ----
figure('Name', 'Propellant Masses', 'Position', [250 300 800 450]);
if has_extended
    yyaxis left
    plot(t, m3, 'b-', 'LineWidth', 1.5, 'DisplayName', 'N2O Liquid'); hold on;
    plot(t, m_vap, 'c--', 'LineWidth', 1.2, 'DisplayName', 'N2O Vapour');
    ylabel('N2O Mass [kg]');

    yyaxis right
    plot(t, m1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'N2 Source');
    plot(t, m_N2_ull, 'r--', 'LineWidth', 1.0, 'DisplayName', 'N2 in Ullage');
    ylabel('N2 Mass [kg]');
else
    yyaxis left
    plot(t, m3, 'g-', 'LineWidth', 1.5);
    ylabel('N2O Liquid Mass [kg]');
    yyaxis right
    plot(t, m1, 'm-', 'LineWidth', 1.5);
    ylabel('N2 Gas Mass [kg]');
end
title('Tank Mass Depletion');
xlabel('Time [s]');
legend('Location', 'best'); grid on;
saveas(gcf, 'n2o_mass_plot.png');

%% ---- Figure 6: Mass Flow Rates ----
figure('Name', 'Mass Flow Rates', 'Position', [300 250 800 400]);
hold on;
plot(t, results.m_dot_reg.Data, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Regulator (N2 in)');
plot(t, results.m_dot_inj.Data, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Injector (N2O out)');
title('Mass Flow Rates');
xlabel('Time [s]'); ylabel('Mass Flow [kg/s]');
legend('Location', 'best'); grid on; hold off;
saveas(gcf, 'n2o_flow_plot.png');

%% ---- Figure 7: Valve Position ----
figure('Name', 'Valve Position', 'Position', [350 200 800 350]);
plot(t, results.valve_pos.Data, 'k-', 'LineWidth', 1.5);
title('Regulator Valve Position');
xlabel('Time [s]'); ylabel('Angle [deg]');
ylim([0, max(results.valve_pos.Data)*1.3 + 5]);
grid on;
saveas(gcf, 'n2o_valve_plot.png');

%% ---- Figure 8: N2O Quality ----
if has_extended
    figure('Name', 'N2O Quality', 'Position', [400 150 800 350]);
    plot(t, quality * 100, 'g-', 'LineWidth', 1.5);
    title('N2O Vapour Mass Fraction (Quality)');
    xlabel('Time [s]'); ylabel('Quality [%]');
    ylim([0, max(quality*100)*1.2 + 1]);
    grid on;
    saveas(gcf, 'n2o_quality_plot.png');
end

fprintf('Plots saved.\n');

end
