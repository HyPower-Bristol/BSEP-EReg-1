function ereg_plot_dual(results, outdir)
%EREG_PLOT_DUAL Pressure and flow plots for the dual-branch run.
here = fileparts(mfilename('fullpath'));
if nargin < 2, outdir = fullfile(here, '..', 'output', 'dual'); end
if ~exist(outdir, 'dir'), mkdir(outdir); end
ls_ = results.logsout;
g = @(n) ls_.getElement(n).Values;

Pfu = g('P_tank_fu [bar]'); Pox = g('P_tank_ox [bar]'); Php = g('P_HP [bar]');
spf = results.setpoint_out_fu; spo = results.setpoint_out_ox;

f = figure('Visible', 'off', 'Position', [50 50 950 520]);
yyaxis left
hold on; grid on;
plot(Pfu.Time, Pfu.Data, 'b-', 'LineWidth', 1.4);
plot(Pox.Time, Pox.Data, 'g-', 'LineWidth', 1.4);
plot(spf.Time, spf.Data, 'b--', 'LineWidth', 0.9);
plot(spo.Time, spo.Data, 'g--', 'LineWidth', 0.9);
ylabel('LP tank pressure [bar]'); ylim([0 70]);
yyaxis right
plot(Php.Time, Php.Data, 'r-', 'LineWidth', 1.4);
ylabel('HP bottle [bar]'); ylim([0 320]);
xlabel('Time (s)');
title('Dual EReg: shared HP bottle, IPA (50 bar) + N2O (55 bar)');
legend({'IPA tank', 'N2O tank', 'IPA setpoint', 'N2O setpoint', 'HP N2'}, 'Location', 'southeast');
saveas(f, fullfile(outdir, 'dual_pressures.png')); close(f);

mLf = g('m_dot_L_fu_clamped'); mLo = g('m_dot_L_ox_clamped');
mNf = g('m_dot_N2_fu'); mNo = g('m_dot_N2_ox');
f = figure('Visible', 'off', 'Position', [50 50 950 520]);
yyaxis left
hold on; grid on;
plot(mLf.Time, squeeze(mLf.Data), 'b-', 'LineWidth', 1.4);
plot(mLo.Time, squeeze(mLo.Data), 'g-', 'LineWidth', 1.4);
ylabel('Propellant flow [kg/s]');
yyaxis right
hold on;
plot(mNf.Time, squeeze(mNf.Data), 'b-.', 'LineWidth', 1.0);
plot(mNo.Time, squeeze(mNo.Data), 'g-.', 'LineWidth', 1.0);
ylabel('N2 draw [kg/s]');
xlabel('Time (s)'); title('Dual EReg: propellant flows and N2 draws');
legend({'IPA flow', 'N2O flow', 'N2 draw (fu)', 'N2 draw (ox)'}, 'Location', 'northwest');
saveas(f, fullfile(outdir, 'dual_flows.png')); close(f);
fprintf('dual plots saved to %s\n', outdir);
end
