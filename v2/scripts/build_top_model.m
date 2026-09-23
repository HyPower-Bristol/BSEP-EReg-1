function build_top_model(v2_root, variant)
%BUILD_TOP_MODEL Programmatically build the top model: plant, servo & valve
% mechanics, PT passthrough blocks, external command sources, and the
% referenced controller model. Reproduces v1's signal names and logging marks
% so the verification/plot tooling maps 1:1.
% variant: 'water' (default -> EReg_v2.slx, verbatim v1 physics, also used for
% IPA) or 'n2o' (-> EReg_v2_N2O.slx, equilibrium two-phase N2O tank + Dyer
% injector). Only the Tank Plant internals differ - same interface.
if nargin < 1 || isempty(v2_root), v2_root = fileparts(fileparts(fileparts(mfilename('fullpath')))); end
if nargin < 2, variant = 'water'; end
vdir = fullfile(v2_root, 'v2');
plantdir = fullfile(vdir, 'src', 'plant');
addpath(fullfile(vdir, 'src', 'controller'), fullfile(vdir, 'src', 'plant'), ...
    fullfile(vdir, 'scripts'), fullfile(vdir, 'models'));
if strcmp(variant, 'n2o')
    P = ereg_params('N2O');
    mdl = 'EReg_v2_N2O';
else
    P = ereg_params();
    mdl = 'EReg_v2';
end
ereg_fanout(P);

if bdIsLoaded(mdl), close_system(mdl, 0); end
target = fullfile(vdir, 'models', [mdl '.slx']);
if exist(target, 'file'), delete(target); end
new_system(mdl);
set_param(mdl, ...
    'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts_sim', ...
    'StopTime', 'Sim_Duration', 'EnableMultiTasking', 'off', 'AlgebraicLoopMsg', 'error', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout', 'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on');

h = @(b) get_param(b, 'PortHandles');

%% ---------------- Tank Plant subsystem (shared shell, variant internals) ---
tp = [mdl '/Tank Plant'];
add_block('simulink/Ports & Subsystems/Subsystem', tp, 'Position', [760 40 900 140]);
delete_line(tp, 'In1/1', 'Out1/1');
set_param([tp '/In1'], 'Name', 'reg_valve_kv', 'Position', [40 58 70 72]);
set_param([tp '/Out1'], 'Name', 'P_2_pa', 'Position', [860 58 890 72]);
add_block('simulink/Sources/In1', [tp '/water_valve_kv'], 'Port', '2', 'Position', [40 118 70 132]);
add_block('simulink/Sinks/Out1', [tp '/P_1_pa'], 'Port', '2', 'Position', [860 118 890 132]);

if strcmp(variant, 'n2o')
    build_tank_n2o(tp, plantdir);
else
    build_tank_water(tp, plantdir);
end

%% ---------------- Servo Actuated Valve subsystem ----------------
sv = [mdl '/Servo Actuated Valve'];
add_block('simulink/Ports & Subsystems/Subsystem', sv, 'Position', [400 40 540 140]);
delete_line(sv, 'In1/1', 'Out1/1');
set_param([sv '/In1'], 'Name', 'servo_angle_cmd', 'Position', [40 58 70 72]);
set_param([sv '/Out1'], 'Name', 'reg_valve_kv', 'Position', [640 58 670 72]);
add_block('simulink/Sinks/Out1', [sv '/valve_angle_fb'], 'Port', '2', 'Position', [640 118 670 132]);

gear = [sv '/Valve Gear Conversion'];
add_block('simulink/User-Defined Functions/MATLAB Function', gear, 'Position', [120 40 220 100]);
inject_mlfcn(gear, fullfile(plantdir, 'valve_gear_conversion.m'), {});
add_block('simulink/Discontinuities/Backlash', [sv '/Backlash'], ...
    'BacklashWidth', 'backlash_width', 'InitialOutput', 'backlash_init', ...
    'Position', [270 50 310 90]);
va = [sv '/Valve area'];
add_block('simulink/User-Defined Functions/MATLAB Function', va, 'Position', [360 40 460 100]);
inject_mlfcn(va, fullfile(plantdir, 'valve_area_poly.m'), {});
add_block('simulink/Math Operations/Gain', [sv '/Reg Valve Kv Gain'], ...
    'Gain', 'Kv_1_max', 'Position', [510 55 545 85]);

add_line(sv, getfield(h([sv '/servo_angle_cmd']), 'Outport'), ...
    getfield(h(gear), 'Inport'), 'autorouting', 'on');
lv = add_line(sv, getfield(h(gear), 'Outport'), getfield(h([sv '/Backlash']), 'Inport'), 'autorouting', 'on');
set_param(lv, 'Name', 'Valve angle');
set_param(getfield(h(gear), 'Outport'), 'DataLogging', 'on');
add_line(sv, getfield(h([sv '/Backlash']), 'Outport'), getfield(h(va), 'Inport'), 'autorouting', 'on');
add_line(sv, getfield(h([sv '/Backlash']), 'Outport'), ...
    getfield(h([sv '/valve_angle_fb']), 'Inport'), 'autorouting', 'on');
la = add_line(sv, getfield(h(va), 'Outport'), getfield(h([sv '/Reg Valve Kv Gain']), 'Inport'), 'autorouting', 'on');
set_param(la, 'Name', 'Normalised Valve Area');
set_param(getfield(h(va), 'Outport'), 'DataLogging', 'on');
add_line(sv, getfield(h([sv '/Reg Valve Kv Gain']), 'Outport'), ...
    getfield(h([sv '/reg_valve_kv']), 'Inport'), 'autorouting', 'on');

%% ---------------- PT blocks (passthrough placeholders) ----------------
pt1 = [mdl '/PT_tank'];
add_block('simulink/Ports & Subsystems/Subsystem', pt1, 'Position', [1000 40 1080 80]);
set_param([pt1 '/In1'], 'Name', 'P_true_bar');
set_param([pt1 '/Out1'], 'Name', 'P_meas_bar');
mask = Simulink.Mask.create(pt1);
mask.Description = sprintf(['Pressure transducer placeholder (pure passthrough).\n' ...
    'Upgrade recipe: inside, insert ZOH (Ts_pt) -> add Random Number (seed, noise_std_bar)' ...
    ' -> Quantizer (quant_lsb_bar). Mask params are declared but unused until then.']);
mask.addParameter('Name', 'Ts_pt', 'Prompt', 'PT sample time [s]', 'Value', '0', 'Evaluate', 'off');
mask.addParameter('Name', 'noise_std_bar', 'Prompt', 'Noise std [bar]', 'Value', '0', 'Evaluate', 'off');
mask.addParameter('Name', 'quant_lsb_bar', 'Prompt', 'Quantization LSB [bar]', 'Value', '0', 'Evaluate', 'off');
mask.addParameter('Name', 'seed', 'Prompt', 'Noise seed', 'Value', '0', 'Evaluate', 'off');
pt2 = [mdl '/PT_HP'];
add_block(pt1, pt2, 'Position', [1000 120 1080 160]);

%% ---------------- Sources, controller, top-level wiring ----------------
add_block('simulink/Sources/From Workspace', [mdl '/mode_cmd_src'], ...
    'VariableName', 'mode_cmd_ts', 'SampleTime', 'Ts_sim', 'Interpolate', 'off', ...
    'ZeroCross', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [40 40 140 70]);
add_block('simulink/Sources/From Workspace', [mdl '/P_set_cmd_src'], ...
    'VariableName', 'P_set_cmd_ts', 'SampleTime', 'Ts_sim', 'Interpolate', 'on', ...
    'ZeroCross', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [40 100 140 130]);
add_block('simulink/Sources/From Workspace', [mdl '/run_valve_kv_src'], ...
    'VariableName', 'run_valve_kv_ts', 'SampleTime', 'Ts_sim', 'Interpolate', 'on', ...
    'ZeroCross', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [40 160 140 190]);

ctl = [mdl '/Controller'];
add_block('simulink/Ports & Subsystems/Model', ctl, 'Position', [200 40 340 200]);
set_param(ctl, 'ModelName', 'ereg_controller_model');
% Servo position integrator (controller-side state, hosted at top level to
% break the valve-angle feedback loop; hardware integrates in hw_step).
add_block('simulink/Discrete/Discrete-Time Integrator', [mdl '/servo_pos_integ'], ...
    'Position', [370 55 410 95], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'InitialCondition', '0', 'SampleTime', 'Ts_ctrl');

add_block('simulink/Math Operations/Gain', [mdl '/Pa2bar_tank'], 'Gain', '1e-5', ...
    'Position', [940 45 970 75]);
add_block('simulink/Math Operations/Gain', [mdl '/Pa2bar_HP'], 'Gain', '1e-5', ...
    'Position', [940 125 970 155]);
add_block('simulink/Sinks/To Workspace', [mdl '/tw_setpoint'], 'VariableName', 'setpoint_out', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'Position', [400 260 470 290]);
add_block('simulink/Sinks/To Workspace', [mdl '/tw_lowpressure'], 'VariableName', 'lowpressure_out', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'Position', [1140 40 1210 70]);
add_block('simulink/Sinks/Terminator', [mdl '/t_demand'], 'Position', [400 210 420 230]);
add_block('simulink/Sinks/Terminator', [mdl '/t_state'], 'Position', [400 235 420 255]);

cph = h(ctl); svh = h(sv); tph = h(tp); iph = h([mdl '/servo_pos_integ']);
add_line(mdl, getfield(h([mdl '/mode_cmd_src']), 'Outport'), cph.Inport(1), 'autorouting', 'on');
add_line(mdl, getfield(h([mdl '/P_set_cmd_src']), 'Outport'), cph.Inport(2), 'autorouting', 'on');
lsp = add_line(mdl, cph.Outport(1), iph.Inport(1), 'autorouting', 'on');
set_param(lsp, 'Name', 'servo_speed');
lsa = add_line(mdl, iph.Outport(1), svh.Inport(1), 'autorouting', 'on');
set_param(lsa, 'Name', 'servo angle');
set_param(iph.Outport(1), 'DataLogging', 'on');
lsd = add_line(mdl, cph.Outport(2), getfield(h([mdl '/t_demand']), 'Inport'), 'autorouting', 'on');
set_param(lsd, 'Name', 'servo_demand');
set_param(cph.Outport(2), 'DataLogging', 'on');
lst = add_line(mdl, cph.Outport(3), getfield(h([mdl '/t_state']), 'Inport'), 'autorouting', 'on');
set_param(lst, 'Name', 'state');
set_param(cph.Outport(3), 'DataLogging', 'on');
add_line(mdl, cph.Outport(4), getfield(h([mdl '/tw_setpoint']), 'Inport'), 'autorouting', 'on');
add_line(mdl, svh.Outport(1), tph.Inport(1), 'autorouting', 'on');
add_line(mdl, getfield(h([mdl '/run_valve_kv_src']), 'Outport'), tph.Inport(2), 'autorouting', 'on');
add_line(mdl, tph.Outport(1), getfield(h([mdl '/Pa2bar_tank']), 'Inport'), 'autorouting', 'on');
lpt = add_line(mdl, getfield(h([mdl '/Pa2bar_tank']), 'Outport'), getfield(h(pt1), 'Inport'), 'autorouting', 'on');
set_param(lpt, 'Name', 'P_tank [bar]');
set_param(getfield(h([mdl '/Pa2bar_tank']), 'Outport'), 'DataLogging', 'on');
add_line(mdl, getfield(h([mdl '/Pa2bar_tank']), 'Outport'), ...
    getfield(h([mdl '/tw_lowpressure']), 'Inport'), 'autorouting', 'on');
add_line(mdl, getfield(h(pt1), 'Outport'), cph.Inport(3), 'autorouting', 'on');
add_line(mdl, tph.Outport(2), getfield(h([mdl '/Pa2bar_HP']), 'Inport'), 'autorouting', 'on');
lph = add_line(mdl, getfield(h([mdl '/Pa2bar_HP']), 'Outport'), getfield(h(pt2), 'Inport'), 'autorouting', 'on');
set_param(lph, 'Name', 'P_HP [bar]');
set_param(getfield(h([mdl '/Pa2bar_HP']), 'Outport'), 'DataLogging', 'on');
add_line(mdl, getfield(h(pt2), 'Outport'), cph.Inport(4), 'autorouting', 'on');
add_line(mdl, svh.Outport(2), cph.Inport(5), 'autorouting', 'on');

% synchronous compile check
feval(mdl, [], [], [], 'compile');
feval(mdl, [], [], [], 'term');

save_system(mdl, target);
close_system(mdl, 0);
fprintf('built %s\n', target);
end

%% ======================= water/IPA tank internals =======================
function build_tank_water(tp, plantdir)
h = @(b) get_param(b, 'PortHandles');
gvp = [tp '/Gas Volume Properties'];
add_block('simulink/User-Defined Functions/MATLAB Function', gvp, 'Position', [560 40 720 180]);
inject_mlfcn(gvp, fullfile(plantdir, 'gas_volume_properties.m'), ...
    {'V_1', 'V_2', 'T_0_N2', 'rho_1_0', 'gamma_N2', 'R_N2', 'rho_L'});
fs = [tp '/Component Flow Solver'];
add_block('simulink/User-Defined Functions/MATLAB Function', fs, 'Position', [160 220 320 380]);
inject_mlfcn(fs, fullfile(plantdir, 'component_flow_solver.m'), ...
    {'Cd_3', 'K_v_4', 'rho_L', 'R_N2', 'Tst', 'rhost', 'A_3', 'Pst'});

add_block('simulink/Discrete/Discrete-Time Integrator', [tp '/Int_m1'], ...
    'Position', [460 220 500 260], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'InitialCondition', 'm_1_0', 'SampleTime', 'Ts_sim');
add_block('simulink/Discrete/Discrete-Time Integrator', [tp '/Int_m2'], ...
    'Position', [460 300 500 340], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'InitialCondition', 'm_2_0', 'SampleTime', 'Ts_sim');
add_block('simulink/Discrete/Discrete-Time Integrator', [tp '/Int_m3'], ...
    'Position', [460 380 500 420], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'InitialCondition', 'm_3_0', 'SampleTime', 'Ts_sim');
add_block('simulink/Math Operations/Gain', [tp '/neg_N2'], 'Gain', '-1', 'Position', [390 225 420 255]);
add_block('simulink/Math Operations/Gain', [tp '/neg_L'], 'Gain', '-1', 'Position', [390 385 420 415]);
add_block('simulink/Discontinuities/Saturation', [tp '/sat_L'], ...
    'UpperLimit', 'inf', 'LowerLimit', '0', 'Position', [340 440 370 470]);
add_block('simulink/Sinks/Terminator', [tp '/t_P3'], 'Position', [360 240 380 260]);
add_block('simulink/Sinks/Terminator', [tp '/t_P4'], 'Position', [360 280 380 300]);
add_block('simulink/Sinks/Terminator', [tp '/t_rho1'], 'Position', [740 100 760 120]);
add_block('simulink/Sinks/Terminator', [tp '/t_rho2'], 'Position', [740 130 760 150]);
add_block('simulink/Sinks/To Workspace', [tp '/tw_fuelflow'], 'VariableName', 'fuelflow', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'Position', [420 480 480 510]);
add_block('simulink/Sinks/To Workspace', [tp '/tw_nitflow'], 'VariableName', 'nitflow', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'Position', [420 530 480 560]);

gph = h(gvp); fph = h(fs);
% GVP outputs: 1 P_1, 2 P_2, 3 rho_1, 4 rho_2, 5 T
% FS inputs:   1 P_1_pa, 2 P_2_pa, 3 T, 4 K_v_1, 5 K_v_2
add_line(tp, gph.Outport(1), fph.Inport(1), 'autorouting', 'on');
add_line(tp, gph.Outport(2), fph.Inport(2), 'autorouting', 'on');
lT = add_line(tp, gph.Outport(5), fph.Inport(3), 'autorouting', 'on');
set_param(lT, 'Name', 'T');
set_param(gph.Outport(5), 'DataLogging', 'on');
add_line(tp, gph.Outport(3), getfield(h([tp '/t_rho1']), 'Inport'), 'autorouting', 'on');
add_line(tp, gph.Outport(4), getfield(h([tp '/t_rho2']), 'Inport'), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/reg_valve_kv']), 'Outport'), fph.Inport(4), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/water_valve_kv']), 'Outport'), fph.Inport(5), 'autorouting', 'on');
% FS outputs: 1 P_3_pa, 2 P_4_pa, 3 m_dot_L, 4 m_dot_N2
add_line(tp, fph.Outport(1), getfield(h([tp '/t_P3']), 'Inport'), 'autorouting', 'on');
add_line(tp, fph.Outport(2), getfield(h([tp '/t_P4']), 'Inport'), 'autorouting', 'on');
lL = add_line(tp, fph.Outport(3), getfield(h([tp '/sat_L']), 'Inport'), 'autorouting', 'on');
set_param(lL, 'Name', 'm_dot_L');
set_param(fph.Outport(3), 'DataLogging', 'on');
add_line(tp, getfield(h([tp '/sat_L']), 'Outport'), getfield(h([tp '/neg_L']), 'Inport'), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/sat_L']), 'Outport'), getfield(h([tp '/tw_fuelflow']), 'Inport'), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/neg_L']), 'Outport'), getfield(h([tp '/Int_m3']), 'Inport'), 'autorouting', 'on');
lN = add_line(tp, fph.Outport(4), getfield(h([tp '/neg_N2']), 'Inport'), 'autorouting', 'on');
set_param(lN, 'Name', 'm_dot_N2');
set_param(fph.Outport(4), 'DataLogging', 'on');
add_line(tp, fph.Outport(4), getfield(h([tp '/Int_m2']), 'Inport'), 'autorouting', 'on');
add_line(tp, fph.Outport(4), getfield(h([tp '/tw_nitflow']), 'Inport'), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/neg_N2']), 'Outport'), getfield(h([tp '/Int_m1']), 'Inport'), 'autorouting', 'on');
l1 = add_line(tp, getfield(h([tp '/Int_m1']), 'Outport'), gph.Inport(1), 'autorouting', 'on');
set_param(l1, 'Name', 'm_1');
set_param(getfield(h([tp '/Int_m1']), 'Outport'), 'DataLogging', 'on');
l2 = add_line(tp, getfield(h([tp '/Int_m2']), 'Outport'), gph.Inport(2), 'autorouting', 'on');
set_param(l2, 'Name', 'm_2');
set_param(getfield(h([tp '/Int_m2']), 'Outport'), 'DataLogging', 'on');
l3 = add_line(tp, getfield(h([tp '/Int_m3']), 'Outport'), gph.Inport(3), 'autorouting', 'on');
set_param(l3, 'Name', 'm_3');
set_param(getfield(h([tp '/Int_m3']), 'Outport'), 'DataLogging', 'on');
add_line(tp, gph.Outport(2), getfield(h([tp '/P_2_pa']), 'Inport'), 'autorouting', 'on');
add_line(tp, gph.Outport(1), getfield(h([tp '/P_1_pa']), 'Inport'), 'autorouting', 'on');
end

%% ========================= N2O tank internals ==========================
function build_tank_n2o(tp, plantdir)
% Equilibrium two-phase N2O tank with N2 supercharge + Dyer/NHNE injector.
% States: m_1 (HP bottle N2), m_N2_tank (pad N2), m_N2O (total N2O),
% U_n2o (total internal energy of tank contents).
h = @(b) get_param(b, 'PortHandles');
hp = [tp '/HP Tank Properties'];
add_block('simulink/User-Defined Functions/MATLAB Function', hp, 'Position', [560 40 700 120]);
inject_mlfcn(hp, fullfile(plantdir, 'hp_tank_properties.m'), ...
    {'V_1', 'T_0_N2', 'rho_1_0', 'gamma_N2', 'R_N2'});
nt = [tp '/N2O Tank Properties'];
add_block('simulink/User-Defined Functions/MATLAB Function', nt, 'Position', [560 180 720 460]);
inject_mlfcn(nt, fullfile(plantdir, 'n2o_tank_properties.m'), ...
    {'V_2', 'n2o_lut', 'R_N2', 'cv_N2'});
fs = [tp '/N2O Flow Solver'];
add_block('simulink/User-Defined Functions/MATLAB Function', fs, 'Position', [160 200 320 460]);
inject_mlfcn(fs, fullfile(plantdir, 'n2o_flow_solver.m'), ...
    {'n2o_lut', 'K_v_4', 'Cd_3', 'A_3', 'R_N2', 'Tst', 'rhost', 'Pst'});
ef = [tp '/Energy Flux'];
add_block('simulink/User-Defined Functions/MATLAB Function', ef, 'Position', [340 520 460 600]);
inject_mlfcn(ef, fullfile(plantdir, 'n2o_energy_flux.m'), {'cp_N2'});

ints = {'Int_m1', 'm_1_0'; 'Int_mN2t', 'm_N2tank_0'; 'Int_mN2O', 'm_N2O_0'; 'Int_U', 'U_n2o_0'};
for i = 1:4
    add_block('simulink/Discrete/Discrete-Time Integrator', [tp '/' ints{i, 1}], ...
        'Position', [460, 200 + 70 * (i - 1), 500, 240 + 70 * (i - 1)], ...
        'IntegratorMethod', 'Integration: Forward Euler', ...
        'InitialCondition', ints{i, 2}, 'SampleTime', 'Ts_sim');
end
add_block('simulink/Math Operations/Gain', [tp '/neg_N2'], 'Gain', '-1', 'Position', [390 205 420 235]);
add_block('simulink/Math Operations/Gain', [tp '/neg_L'], 'Gain', '-1', 'Position', [390 345 420 375]);
add_block('simulink/Discontinuities/Saturation', [tp '/sat_L'], ...
    'UpperLimit', 'inf', 'LowerLimit', '0', 'Position', [340 480 370 510]);
terms = {'t_P3', 't_P4', 't_T', 't_mL', 't_mV', 't_PN2', 't_Psat', 't_flag'};
for i = 1:numel(terms)
    add_block('simulink/Sinks/Terminator', [tp '/' terms{i}], ...
        'Position', [780, 180 + 35 * (i - 1), 800, 200 + 35 * (i - 1)]);
end
add_block('simulink/Sinks/To Workspace', [tp '/tw_fuelflow'], 'VariableName', 'fuelflow', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'Position', [420 620 480 650]);
add_block('simulink/Sinks/To Workspace', [tp '/tw_nitflow'], 'VariableName', 'nitflow', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'Position', [420 670 480 700]);

hph = h(hp); nth = h(nt); fsh = h(fs); efh = h(ef);
mark = @(lh, ph, nm) deal_mark(lh, ph, nm);

% HP side: m_1 -> hp props -> P_1 (flow solver in1 + subsystem Out2), T_hp
l = add_line(tp, getfield(h([tp '/Int_m1']), 'Outport'), hph.Inport(1), 'autorouting', 'on');
mark(l, getfield(h([tp '/Int_m1']), 'Outport'), 'm_1');
add_line(tp, hph.Outport(1), fsh.Inport(1), 'autorouting', 'on');
add_line(tp, hph.Outport(1), getfield(h([tp '/P_1_pa']), 'Inport'), 'autorouting', 'on');
add_line(tp, hph.Outport(2), fsh.Inport(3), 'autorouting', 'on');   % T_hp
add_line(tp, hph.Outport(2), efh.Inport(2), 'autorouting', 'on');

% N2O tank state -> equilibrium properties
l = add_line(tp, getfield(h([tp '/Int_mN2O']), 'Outport'), nth.Inport(1), 'autorouting', 'on');
mark(l, getfield(h([tp '/Int_mN2O']), 'Outport'), 'm_N2O');
l = add_line(tp, getfield(h([tp '/Int_mN2t']), 'Outport'), nth.Inport(2), 'autorouting', 'on');
mark(l, getfield(h([tp '/Int_mN2t']), 'Outport'), 'm_N2_tank');
l = add_line(tp, getfield(h([tp '/Int_U']), 'Outport'), nth.Inport(3), 'autorouting', 'on');
mark(l, getfield(h([tp '/Int_U']), 'Outport'), 'U_n2o');

% nt outputs: 1 P_tank_pa, 2 T_tank, 3 m_L, 4 m_V, 5 P_N2_pa, 6 P_sat_pa,
%             7 rho_L_now, 8 h_L_now, 9 s_L_now, 10 flag
add_line(tp, nth.Outport(1), fsh.Inport(2), 'autorouting', 'on');
add_line(tp, nth.Outport(1), getfield(h([tp '/P_2_pa']), 'Inport'), 'autorouting', 'on');
l = add_line(tp, nth.Outport(2), getfield(h([tp '/t_T']), 'Inport'), 'autorouting', 'on');
mark(l, nth.Outport(2), 'T');
l = add_line(tp, nth.Outport(3), getfield(h([tp '/t_mL']), 'Inport'), 'autorouting', 'on');
mark(l, nth.Outport(3), 'm_3');
l = add_line(tp, nth.Outport(4), getfield(h([tp '/t_mV']), 'Inport'), 'autorouting', 'on');
mark(l, nth.Outport(4), 'm_V');
l = add_line(tp, nth.Outport(5), getfield(h([tp '/t_PN2']), 'Inport'), 'autorouting', 'on');
mark(l, nth.Outport(5), 'P_N2');
l = add_line(tp, nth.Outport(6), getfield(h([tp '/t_Psat']), 'Inport'), 'autorouting', 'on');
mark(l, nth.Outport(6), 'P_sat');
add_line(tp, nth.Outport(6), fsh.Inport(9), 'autorouting', 'on');
add_line(tp, nth.Outport(7), fsh.Inport(6), 'autorouting', 'on');   % rho_L_now
add_line(tp, nth.Outport(8), fsh.Inport(7), 'autorouting', 'on');   % h_L_now
add_line(tp, nth.Outport(8), efh.Inport(4), 'autorouting', 'on');
add_line(tp, nth.Outport(9), fsh.Inport(8), 'autorouting', 'on');   % s_L_now
l = add_line(tp, nth.Outport(10), getfield(h([tp '/t_flag']), 'Inport'), 'autorouting', 'on');
mark(l, nth.Outport(10), 'tank_flag');

% valve commands
add_line(tp, getfield(h([tp '/reg_valve_kv']), 'Outport'), fsh.Inport(4), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/water_valve_kv']), 'Outport'), fsh.Inport(5), 'autorouting', 'on');

% flow solver outputs: 1 P_3, 2 P_4, 3 m_dot_L, 4 m_dot_N2
add_line(tp, fsh.Outport(1), getfield(h([tp '/t_P3']), 'Inport'), 'autorouting', 'on');
add_line(tp, fsh.Outport(2), getfield(h([tp '/t_P4']), 'Inport'), 'autorouting', 'on');
l = add_line(tp, fsh.Outport(3), getfield(h([tp '/sat_L']), 'Inport'), 'autorouting', 'on');
mark(l, fsh.Outport(3), 'm_dot_L');
l = add_line(tp, fsh.Outport(4), getfield(h([tp '/neg_N2']), 'Inport'), 'autorouting', 'on');
mark(l, fsh.Outport(4), 'm_dot_N2');
add_line(tp, fsh.Outport(4), getfield(h([tp '/Int_mN2t']), 'Inport'), 'autorouting', 'on');
add_line(tp, fsh.Outport(4), getfield(h([tp '/tw_nitflow']), 'Inport'), 'autorouting', 'on');
add_line(tp, fsh.Outport(4), efh.Inport(1), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/neg_N2']), 'Outport'), getfield(h([tp '/Int_m1']), 'Inport'), 'autorouting', 'on');

% clamped liquid flow: mass + energy must use the SAME flow
add_line(tp, getfield(h([tp '/sat_L']), 'Outport'), getfield(h([tp '/neg_L']), 'Inport'), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/sat_L']), 'Outport'), getfield(h([tp '/tw_fuelflow']), 'Inport'), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/sat_L']), 'Outport'), efh.Inport(3), 'autorouting', 'on');
add_line(tp, getfield(h([tp '/neg_L']), 'Outport'), getfield(h([tp '/Int_mN2O']), 'Inport'), 'autorouting', 'on');

% energy flux -> U integrator
l = add_line(tp, efh.Outport(1), getfield(h([tp '/Int_U']), 'Inport'), 'autorouting', 'on');
mark(l, efh.Outport(1), 'dU');
end

function deal_mark(line_h, port_h, name)
set_param(line_h, 'Name', name);
set_param(port_h, 'DataLogging', 'on');
end

function inject_mlfcn(blockpath, srcfile, param_names)
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blockpath);
assert(~isempty(ch), 'MATLAB Function chart not found at %s', blockpath);
ch.Script = fileread(srcfile);
for k = 1:numel(param_names)
    d = find(ch, '-isa', 'Stateflow.Data', 'Name', param_names{k});
    if isempty(d)
        try, sf('Parse', ch.Id); catch, end %#ok<NOCOM>
        d = find(ch, '-isa', 'Stateflow.Data', 'Name', param_names{k});
    end
    assert(~isempty(d), 'data %s not found in %s', param_names{k}, blockpath);
    d.Scope = 'Parameter';
end
end
