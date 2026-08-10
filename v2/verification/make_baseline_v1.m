function make_baseline_v1(v1_dir, out_file)
%MAKE_BASELINE_V1 Run the v1 model and capture the reference dataset.
% Runs EReg_Tank_Drain_Init.m in the MAIN worktree (the committed
% graph-producing state) and saves all logged signals to baseline_v1.mat.
if nargin < 1, v1_dir = 'D:\GitHub\BSEP-EReg-1'; end
if nargin < 2
    out_file = fullfile(fileparts(mfilename('fullpath')), 'baseline', 'baseline_v1.mat');
end

old = cd(v1_dir);
restore = onCleanup(@() cd(old));
% Run in the BASE workspace: sim() resolves the model's Repeating Sequence /
% MATLAB Fcn parameters there, not in this function's workspace.
evalin('base', sprintf('run(''%s'')', fullfile(v1_dir, 'EReg_Tank_Drain_Init.m')));
results = evalin('base', 'results');

ls_ = results.logsout;
names = ls_.getElementNames;
wanted = { ...
    'P_tank [bar]',          'P_tank_bar'; ...
    'P_HP [bar]',            'P_HP_bar'; ...
    'servo_demand',          'servo_demand'; ...
    'servo angle',           'servo_angle'; ...
    'Valve angle',           'valve_angle'; ...
    'Normalised Valve Area', 'valve_area'; ...
    'm_dot_L',               'm_dot_L'; ...
    'm_dot_N2',              'm_dot_N2'; ...
    'm_1',                   'm_1'; ...
    'm_2',                   'm_2'; ...
    'm_3',                   'm_3'; ...
    'T',                     'T_gas'};
baseline = struct();
for i = 1:size(wanted, 1)
    idxs = find(strcmp(names, wanted{i, 1}));
    if isempty(idxs)
        warning('baseline: logsout signal "%s" not found', wanted{i, 1});
        continue
    end
    el = ls_.getElement(idxs(1));
    baseline.(wanted{i, 2}) = struct('t', el.Values.Time, 'v', squeeze(el.Values.Data));
end

for tw = {'fuelflow', 'nitflow', 'setpoint_out', 'lowpressure_out', 'Servo_angle', 'Valve_Angle2'}
    try
        v = results.(tw{1});
        if isa(v, 'timeseries')
            baseline.(tw{1}) = struct('t', v.Time, 'v', squeeze(v.Data));
        end
    catch
        warning('baseline: To Workspace var "%s" not in results', tw{1});
    end
end

% Grid + headline sanity checks against the known-good reference graphs.
t = baseline.P_tank_bar.t;
assert(abs(t(2) - t(1) - 1/500) < 1e-12, 'unexpected solver step %.6g', t(2) - t(1));
pt_end  = baseline.P_tank_bar.v(end);
ff_max  = max(baseline.m_dot_L.v);
nf_max  = max(baseline.m_dot_N2.v);
va_max  = max(baseline.valve_angle.v);
fprintf('baseline: %d samples, P_tank(end)=%.4f bar, max m_dot_L=%.4f kg/s, max m_dot_N2=%.5f kg/s, max valve angle=%.2f deg\n', ...
    numel(t), pt_end, ff_max, nf_max, va_max);
assert(abs(pt_end - 2.97) < 0.05,   'P_tank settle %.3f outside reference', pt_end);
assert(abs(ff_max - 0.360) < 0.02,  'fuel flow peak %.4f outside reference', ff_max);
assert(abs(nf_max - 0.00915) < 0.001, 'N2 flow peak %.5f outside reference', nf_max);
assert(abs(va_max - 10.8) < 0.8,    'valve angle peak %.2f outside reference', va_max);

[~, h] = system('git rev-parse HEAD');
[~, d] = system('git status --porcelain');
manifest = struct('commit', strtrim(h), 'dirty', ~isempty(strtrim(d)), ...
    'date', char(datetime('now')), 'matlab', version, 'v1_dir', v1_dir);

outdir = fileparts(out_file);
if ~exist(outdir, 'dir'), mkdir(outdir); end
save(out_file, 'baseline', 'manifest');
if manifest.dirty
    dirtymark = '*';
else
    dirtymark = '';
end
fprintf('baseline saved: %s (commit %s%s)\n', out_file, manifest.commit(1:9), dirtymark);
end
