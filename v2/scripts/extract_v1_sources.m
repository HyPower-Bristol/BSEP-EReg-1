function extract_v1_sources(v1_dir, v2_root)
%EXTRACT_V1_SOURCES Pull the five MATLAB Function scripts out of the v1 model
% verbatim, plus a JSON fixture of the block dialog parameters the v2 build
% must reproduce.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin < 1 || isempty(v1_dir), v1_dir = root; end
if nargin < 2, v2_root = root; end

load_system(fullfile(v1_dir, 'EReg_Tank_DrainUSETHIS.slx'));

charts = find(sfroot, '-isa', 'Stateflow.EMChart');
map = { ...
    'Component Flow Solver', 'component_flow_solver.m'; ...
    'Gas Volume Properties', 'gas_volume_properties.m'; ...
    'Valve Gear Conversion', 'valve_gear_conversion.m'; ...
    'Valve area',            'valve_area_poly.m'; ...
    'Signal normalisation',  'servo_demand_clamp.m'};
outdir = fullfile(v2_root, 'v2', 'src', 'plant');
if ~exist(outdir, 'dir'), mkdir(outdir); end
found = false(size(map, 1), 1);
for c = 1:numel(charts)
    ch = charts(c);
    for k = 1:size(map, 1)
        if contains(ch.Path, map{k, 1}) && ~found(k)
            f = fullfile(outdir, map{k, 2});
            fid = fopen(f, 'w');
            fwrite(fid, ch.Script);
            fclose(fid);
            found(k) = true;
            fprintf('extracted %-26s <- %s\n', map{k, 2}, ch.Path);
        end
    end
end
assert(all(found), 'charts not found: %s', strjoin(map(~found, 1), ', '));

% --- Dialog fixture: the exact block params v2 must reproduce ---
fix = struct();
mdl = 'EReg_Tank_DrainUSETHIS';
pid_params = {'TimeDomain', 'SampleTime', 'Form', 'IntegratorMethod', 'FilterMethod', ...
    'ControllerParametersSource', 'P', 'I', 'D', 'N', 'UseFilter', 'LimitOutput', ...
    'UpperSaturationLimit', 'LowerSaturationLimit', 'SatLimitsSource', 'AntiWindupMode', ...
    'LimitIntegrator', 'UpperIntegratorSaturationLimit', 'LowerIntegratorSaturationLimit', ...
    'InitialConditionForIntegrator', 'InitialConditionForFilter', 'ExternalReset', 'UseKiTs'};

fix.inner_pid = grab(find_system(mdl, 'LookUnderMasks', 'all', 'MaskType', 'PID 1dof'), pid_params);
try
    load_system(fullfile(v1_dir, 'Controller.slx'));
    fix.outer_pid = grab(find_system('Controller', 'LookUnderMasks', 'all', 'MaskType', 'PID 1dof'), pid_params);
catch err
    warning('outer PID fixture skipped: %s', err.message);
end
fix.backlash = grab(find_system(mdl, 'BlockType', 'Backlash'), ...
    {'BacklashWidth', 'InitialOutput', 'ZeroCross', 'SampleTime'});
fix.integrators = grab(find_system(mdl, 'BlockType', 'DiscreteIntegrator'), ...
    {'IntegratorMethod', 'InitialCondition', 'SampleTime', 'gainval', 'LimitOutput'});
fix.switches = grab(find_system(mdl, 'BlockType', 'Switch'), {'Criteria', 'Threshold'});

fixfile = fullfile(v2_root, 'v2', 'verification', 'fixture_v1_dialogs.json');
fid = fopen(fixfile, 'w');
fwrite(fid, jsonencode(fix, 'PrettyPrint', true));
fclose(fid);
fprintf('dialog fixture saved: %s\n', fixfile);
end

function out = grab(blocks, params)
out = struct('path', {}, 'params', {});
for i = 1:numel(blocks)
    blk = blocks{i};
    s = struct();
    for j = 1:numel(params)
        try
            s.(params{j}) = get_param(blk, params{j});
        catch
        end
    end
    out(end+1) = struct('path', blk, 'params', s); %#ok<AGROW>
end
end
