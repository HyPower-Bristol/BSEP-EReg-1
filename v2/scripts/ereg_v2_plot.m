function ereg_v2_plot(results, outdir)
%EREG_V2_PLOT Regenerate the four reference graphs from a v2 sim run.
% Usage: run ereg_v2_init first, then ereg_v2_plot(results).
if nargin < 1, results = evalin('base', 'results'); end
here = fileparts(mfilename('fullpath'));
if nargin < 2, outdir = fullfile(here, '..', 'output'); end
if ~exist(outdir, 'dir'), mkdir(outdir); end
addpath(here);
P = ereg_params();

ls_ = results.logsout;
g = @(nm) ls_.getElement(nm).Values;
P_hp   = g('P_HP [bar]');
P_tank = g('P_tank [bar]');
dem    = g('servo_demand');
vang   = g('Valve angle');
sang   = g('servo angle');
m1     = g('m_1');
m3     = g('m_3');
fuel   = results.fuelflow;
nit    = results.nitflow;
setp   = results.setpoint_out;

% 1 - pressures
f = figure('Visible', 'off', 'Position', [50 50 900 500]); hold on; grid on;
plot(P_hp.Time, P_hp.Data, 'r', 'LineWidth', 1.4);
plot(P_tank.Time, P_tank.Data, 'b', 'LineWidth', 1.4);
plot(setp.Time, setp.Data, 'k--', 'LineWidth', 1.1);
xlabel('Time (s)'); ylabel('Pressure [bar]'); title('System Pressures');
legend({'N2 (HP)', sprintf('%s tank', P.fluid.name), 'Setpoint'}, 'Location', 'best');
saveas(f, fullfile(outdir, 'pressure_plot.png')); close(f);

% 2 - mass flows (twin axis)
f = figure('Visible', 'off', 'Position', [50 50 900 500]);
yyaxis left
plot(fuel.Time, fuel.Data, 'g-', 'LineWidth', 1.4);
ylabel(sprintf('%s Flow (kg/s)', P.fluid.name)); ylim([0 max(fuel.Data) * 1.15]);
yyaxis right
plot(nit.Time, nit.Data, 'm-', 'LineWidth', 1.4);
ylabel('Nitrogen Flow (kg/s)'); ylim([0 max(nit.Data) * 1.15]);
grid on; xlabel('Time (s)'); title('Propellant and Nitrogen Mass Flow');
saveas(f, fullfile(outdir, 'mass_flow.png')); close(f);

% 3 - servo diagnostics
f = figure('Visible', 'off', 'Position', [50 50 900 500]); hold on; grid on;
plot(dem.Time, dem.Data, 'b-', 'LineWidth', 1.3);
plot(vang.Time, vang.Data, 'r--', 'LineWidth', 1.3);
plot(sang.Time, sang.Data, 'g--', 'LineWidth', 1.3);
ylim([-10 100]);
xlabel('Time (s)'); ylabel('Angle [deg]');
title('Servo Diagnostics: Demand vs Actuator vs Valve');
legend({'Servo Demand', 'Valve Angle (Flow)', 'Servo Angle (Actuator)'}, 'Location', 'best');
saveas(f, fullfile(outdir, 'servo_demand.png')); close(f);

% 4 - tank levels (twin axis)
f = figure('Visible', 'off', 'Position', [50 50 900 500]);
yyaxis left
plot(m3.Time, m3.Data / P.fluid.rho_L * 1000, 'b-', 'LineWidth', 1.4);
ylabel(sprintf('%s Volume [L] (Prop Tank)', P.fluid.name));
ylim([0 max(m3.Data / P.fluid.rho_L * 1000) * 1.1]);
yyaxis right
plot(m1.Time, m1.Data, 'r-', 'LineWidth', 1.4);
ylabel('N2 Mass [kg] (HP Tank)');
grid on; xlabel('Time (s)'); title('Tank Filled Levels');
saveas(f, fullfile(outdir, 'tank_levels.png')); close(f);

fprintf('plots saved to %s\n', outdir);
end
