function build_controller_model(vdir)
%BUILD_CONTROLLER_MODEL Programmatically build ereg_controller_model.slx -
% the deployable controller unit: one MATLAB Function block (flight code
% wrapper) plus the servo position Discrete-Time Integrator that breaks the
% valve-angle feedback loop exactly as v1's servo integrator does.
if nargin < 1, vdir = fileparts(fileparts(mfilename('fullpath'))); end
addpath(fullfile(vdir, 'src', 'controller'), fullfile(vdir, 'scripts'));
P = ereg_params();
ereg_fanout(P);   % Ts_ctrl / ctrl_params must resolve during the compile check

mdl = 'ereg_controller_model';
if bdIsLoaded(mdl), close_system(mdl, 0); end
target = fullfile(vdir, 'models', [mdl '.slx']);
if exist(target, 'file'), delete(target); end
new_system(mdl);

set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'EnableMultiTasking', 'off', ...
    'AlgebraicLoopMsg', 'error', ...
    'ModelReferenceNumInstancesAllowed', 'Multi', ...
    'SystemTargetFile', 'ert.tlc', ...
    'GenCodeOnly', 'on');

ins = {'mode_cmd', 'P_set_cmd_bar', 'P_tank_meas_bar', 'P_HP_meas_bar', 'valve_angle_meas_deg'};
for i = 1:numel(ins)
    add_block('simulink/Sources/In1', [mdl '/' ins{i}], ...
        'Position', bpos(1, i), 'SampleTime', 'Ts_ctrl', 'Port', num2str(i));
end

fcnblk = [mdl '/controller'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcnblk, ...
    'Position', [260 60 480 260]);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', fcnblk);
assert(~isempty(ch), 'MATLAB Function chart not found at %s', fcnblk);
ch.Script = fileread(fullfile(vdir, 'src', 'controller', 'ereg_controller_sl_wrapper.m'));
pd = find(ch, '-isa', 'Stateflow.Data', 'Name', 'ctrl_params');
if isempty(pd)
    try, sf('Parse', ch.Id); catch, end %#ok<NOCOM>
    pd = find(ch, '-isa', 'Stateflow.Data', 'Name', 'ctrl_params');
end
assert(~isempty(pd), 'ctrl_params data not created by MATLAB Function parser');
pd.Scope = 'Parameter';

% NOTE: the servo position integrator lives in the TOP model, not here. A
% MATLAB Function block is conservatively direct-feedthrough on every input,
% so keeping the integrator inside this referenced model leaves the parent's
% valve-angle feedback looking like an algebraic loop (artificial-loop
% minimization can't remove it while the diagnostic outputs exist). The
% hardware build integrates speed->position in ereg_controller_hw_step.m.
outs = {'servo_speed_cmd', 'servo_demand_deg', 'state', 'P_set_active_bar'};
for i = 1:numel(outs)
    add_block('simulink/Sinks/Out1', [mdl '/' outs{i}], ...
        'Position', bpos(4, i), 'Port', num2str(i));
end

cph = get_param(fcnblk, 'PortHandles');
for i = 1:numel(ins)
    iph = get_param([mdl '/' ins{i}], 'PortHandles');
    add_line(mdl, iph.Outport(1), cph.Inport(i), 'autorouting', 'on');
end
for i = 1:numel(outs)
    o = get_param([mdl '/' outs{i}], 'PortHandles');
    add_line(mdl, cph.Outport(i), o.Inport(1), 'autorouting', 'on');
end

% synchronous compile check
feval(mdl, [], [], [], 'compile');
feval(mdl, [], [], [], 'term');

save_system(mdl, target);
close_system(mdl, 0);
fprintf('built %s\n', target);
end

function p = bpos(col, row)
x = 40 + (col - 1) * 220;
y = 60 + (row - 1) * 60;
p = [x y x + 60 y + 24];
end
