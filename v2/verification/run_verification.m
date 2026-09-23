function run_verification(v2_root)
%RUN_VERIFICATION Full-loop 1:1 gate - sim v2 in compat mode and compare every
% logged signal against the v1 baseline. Acceptance: rel <= 1e-6 on all
% signals. Writes v2/output/verification_report.txt + 4 overlay PNGs.
if nargin < 1, v2_root = fileparts(fileparts(fileparts(mfilename('fullpath')))); end
vdir = fullfile(v2_root, 'v2');
addpath(fullfile(vdir, 'scripts'), fullfile(vdir, 'verification'), ...
    fullfile(vdir, 'src', 'controller'), fullfile(vdir, 'src', 'plant'), ...
    fullfile(vdir, 'models'));

S = load(fullfile(vdir, 'verification', 'baseline', 'baseline_v1.mat'));
b = S.baseline;

% run v2 in the base workspace (model params resolve there)
evalin('base', sprintf('run(''%s'')', fullfile(vdir, 'scripts', 'ereg_v2_init.m')));
results = evalin('base', 'results');
ls_ = results.logsout;
names = ls_.getElementNames;

pairs = { ...
    'P_tank_bar',   'P_tank [bar]'; ...
    'P_HP_bar',     'P_HP [bar]'; ...
    'servo_demand', 'servo_demand'; ...
    'servo_angle',  'servo angle'; ...
    'valve_angle',  'Valve angle'; ...
    'valve_area',   'Normalised Valve Area'; ...
    'm_dot_L',      'm_dot_L'; ...
    'm_dot_N2',     'm_dot_N2'; ...
    'm_1',          'm_1'; ...
    'm_2',          'm_2'; ...
    'm_3',          'm_3'; ...
    'T_gas',        'T'};
rows = cell(size(pairs, 1), 1);
allpass = true;
v2sig = struct();
for i = 1:size(pairs, 1)
    bf = pairs{i, 1}; sn = pairs{i, 2};
    assert(isfield(b, bf), 'baseline missing %s', bf);
    idx = find(strcmp(names, sn));
    assert(~isempty(idx), 'v2 logsout missing "%s"', sn);
    el = ls_.getElement(idx(1));
    tv = el.Values.Time; vv = squeeze(el.Values.Data);
    v2sig.(bf) = struct('t', tv, 'v', vv);
    [rows{i}, pass] = compare_signals(sn, b.(bf).t, b.(bf).v, tv, vv);
    allpass = allpass && pass;
end

outdir = fullfile(vdir, 'output');
if ~exist(outdir, 'dir'), mkdir(outdir); end
rpt = fullfile(outdir, 'verification_report.txt');
fid = fopen(rpt, 'w');
fprintf(fid, 'EReg v2 vs v1 baseline (commit %s) - compat mode\n%s\n\n', ...
    S.manifest.commit(1:9), char(datetime('now')));
cellfun(@(r) fprintf(fid, '%s\n', r), rows);
fprintf(fid, '\nOverall: %s (acceptance rel <= 1e-6 per signal)\n', pf(allpass));
fclose(fid);
cellfun(@(r) fprintf('%s\n', r), rows);

overlay(outdir, 'overlay_pressure', 'System Pressures', 'Pressure [bar]', ...
    {b.P_HP_bar, v2sig.P_HP_bar, 'P HP', []; b.P_tank_bar, v2sig.P_tank_bar, 'P tank', []});
overlay(outdir, 'overlay_massflow', 'Mass Flows', 'Flow [kg/s]', ...
    {b.m_dot_L, v2sig.m_dot_L, 'm dot L', []; b.m_dot_N2, v2sig.m_dot_N2, 'm dot N2 (x10)', 10});
overlay(outdir, 'overlay_servo', 'Servo Diagnostics', 'Angle [deg]', ...
    {b.servo_demand, v2sig.servo_demand, 'demand', []; b.valve_angle, v2sig.valve_angle, 'valve', []; ...
     b.servo_angle, v2sig.servo_angle, 'servo', []});
overlay(outdir, 'overlay_tanks', 'Tank Contents', 'Mass [kg]', ...
    {b.m_3, v2sig.m_3, 'm 3', []; b.m_1, v2sig.m_1, 'm 1 (x100)', 100});

fprintf('Overall: %s - report: %s\n', pf(allpass), rpt);
assert(allpass, 'verification FAILED - see %s', rpt);
end

function s = pf(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function overlay(outdir, fname, ttl, ylab, series)
f = figure('Visible', 'off', 'Position', [50 50 900 500]);
hold on; grid on;
for i = 1:size(series, 1)
    sc = 1;
    if size(series, 2) > 3 && ~isempty(series{i, 4}), sc = series{i, 4}; end
    plot(series{i, 1}.t, sc * series{i, 1}.v, '--', 'LineWidth', 2.2, ...
        'DisplayName', [series{i, 3} ' v1']);
    plot(series{i, 2}.t, sc * series{i, 2}.v, '-', 'LineWidth', 0.9, ...
        'DisplayName', [series{i, 3} ' v2']);
end
xlabel('Time [s]'); ylabel(ylab); title([ttl ' - v1 (dashed) vs v2 (solid)']);
legend('Location', 'best');
saveas(f, fullfile(outdir, [fname '.png']));
close(f);
end
