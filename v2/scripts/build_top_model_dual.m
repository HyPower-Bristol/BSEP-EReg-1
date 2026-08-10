function build_top_model_dual(v2_root)
%BUILD_TOP_MODEL_DUAL Build EReg_v2_Dual.slx: one shared HP N2 bottle feeding
% two complete EReg branches simultaneously - fuel (IPA, water-law tank) and
% oxidizer (two-phase N2O) - each with its own controller instance, servo
% valve, PT, and command sequence. Branch plant parameters are bound to
% _fu/_ox base-workspace names by renaming the chart parameter symbols at
% injection time.
if nargin < 1, v2_root = 'D:\GitHub\BSEP-EReg-1-v2'; end
vdir = fullfile(v2_root, 'v2');
plantdir = fullfile(vdir, 'src', 'plant');
addpath(fullfile(vdir, 'src', 'controller'), fullfile(vdir, 'src', 'plant'), ...
    fullfile(vdir, 'scripts'), fullfile(vdir, 'models'));
P = ereg_params_dual();
ereg_fanout_dual(P);

mdl = 'EReg_v2_Dual';
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

%% ---------------- Shared HP bottle ----------------
add_block('simulink/Math Operations/Sum', [mdl '/Sum_draws'], 'Inputs', '++', ...
    'Position', [1250 240 1280 270]);
add_block('simulink/Math Operations/Gain', [mdl '/neg_HP'], 'Gain', '-1', ...
    'Position', [1300 240 1330 270]);
add_block('simulink/Discrete/Discrete-Time Integrator', [mdl '/Int_m1'], ...
    'Position', [1350 235 1390 275], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'InitialCondition', 'm_1_0', 'SampleTime', 'Ts_sim');
hp = [mdl '/HP Tank Properties'];
add_block('simulink/User-Defined Functions/MATLAB Function', hp, 'Position', [1430 220 1550 290]);
inject_mlfcn(hp, fullfile(plantdir, 'hp_tank_properties.m'), ...
    {'V_1', 'T_0_N2', 'rho_1_0', 'gamma_N2', 'R_N2'});
add_block('simulink/Math Operations/Gain', [mdl '/Pa2bar_HP'], 'Gain', '1e-5', ...
    'Position', [1580 200 1610 230]);
pt_hp = [mdl '/PT_HP'];
make_pt(pt_hp, [1640 195 1720 235]);

hph = h(hp);
l = add_line(mdl, getfield(h([mdl '/Int_m1']), 'Outport'), hph.Inport(1), 'autorouting', 'on');
set_param(l, 'Name', 'm_1');
set_param(getfield(h([mdl '/Int_m1']), 'Outport'), 'DataLogging', 'on');
add_line(mdl, getfield(h([mdl '/Sum_draws']), 'Outport'), getfield(h([mdl '/neg_HP']), 'Inport'), 'autorouting', 'on');
add_line(mdl, getfield(h([mdl '/neg_HP']), 'Outport'), getfield(h([mdl '/Int_m1']), 'Inport'), 'autorouting', 'on');
add_line(mdl, hph.Outport(1), getfield(h([mdl '/Pa2bar_HP']), 'Inport'), 'autorouting', 'on');
lhp = add_line(mdl, getfield(h([mdl '/Pa2bar_HP']), 'Outport'), getfield(h(pt_hp), 'Inport'), 'autorouting', 'on');
set_param(lhp, 'Name', 'P_HP [bar]');
set_param(getfield(h([mdl '/Pa2bar_HP']), 'Outport'), 'DataLogging', 'on');

%% ---------------- Two branches ----------------
brs = {'fu', 'ox'};
for b = 1:2
    sfx = brs{b};
    yo = (b - 1) * 500;   % vertical offset for branch B

    % command sources
    add_block('simulink/Sources/From Workspace', [mdl '/mode_cmd_' sfx], ...
        'VariableName', ['mode_cmd_' sfx '_ts'], 'SampleTime', 'Ts_sim', ...
        'Interpolate', 'off', 'ZeroCross', 'off', ...
        'OutputAfterFinalValue', 'Holding final value', 'Position', [40 40+yo 140 70+yo]);
    add_block('simulink/Sources/From Workspace', [mdl '/P_set_cmd_' sfx], ...
        'VariableName', ['P_set_cmd_' sfx '_ts'], 'SampleTime', 'Ts_sim', ...
        'Interpolate', 'on', 'ZeroCross', 'off', ...
        'OutputAfterFinalValue', 'Holding final value', 'Position', [40 100+yo 140 130+yo]);
    add_block('simulink/Sources/From Workspace', [mdl '/run_valve_kv_' sfx], ...
        'VariableName', ['run_valve_kv_' sfx '_ts'], 'SampleTime', 'Ts_sim', ...
        'Interpolate', 'on', 'ZeroCross', 'off', ...
        'OutputAfterFinalValue', 'Holding final value', 'Position', [40 160+yo 140 190+yo]);

    % controller instance + servo position integrator
    ctl = [mdl '/Controller_' upper(sfx)];
    add_block('simulink/Ports & Subsystems/Model', ctl, 'Position', [200 40+yo 340 200+yo]);
    set_param(ctl, 'ModelName', 'ereg_controller_model');
    dti = [mdl '/servo_pos_integ_' sfx];
    add_block('simulink/Discrete/Discrete-Time Integrator', dti, ...
        'Position', [370 55+yo 410 95+yo], 'IntegratorMethod', 'Integration: Forward Euler', ...
        'InitialCondition', '0', 'SampleTime', 'Ts_ctrl');

    % servo + valve mechanics
    sv = [mdl '/Servo Valve ' upper(sfx)];
    build_servo_valve(sv, plantdir, sfx, [440 40+yo 580 140+yo]);

    % LP branch plant
    lp = [mdl '/LP Branch ' upper(sfx)];
    add_block('simulink/Ports & Subsystems/Subsystem', lp, 'Position', [640 40+yo 780 200+yo]);
    delete_line(lp, 'In1/1', 'Out1/1');
    set_param([lp '/In1'], 'Name', 'reg_valve_kv', 'Position', [40 58 70 72]);
    set_param([lp '/Out1'], 'Name', 'P_2_pa', 'Position', [900 58 930 72]);
    add_block('simulink/Sources/In1', [lp '/run_valve_kv'], 'Port', '2', 'Position', [40 118 70 132]);
    add_block('simulink/Sources/In1', [lp '/P_1_pa'], 'Port', '3', 'Position', [40 178 70 192]);
    add_block('simulink/Sources/In1', [lp '/T_hp'], 'Port', '4', 'Position', [40 238 70 252]);
    add_block('simulink/Sinks/Out1', [lp '/n2_draw'], 'Port', '2', 'Position', [900 118 930 132]);
    if b == 1
        build_branch_fuel(lp, plantdir);
    else
        build_branch_ox(lp, plantdir);
    end

    % measurement + sinks
    add_block('simulink/Math Operations/Gain', [mdl '/Pa2bar_' sfx], 'Gain', '1e-5', ...
        'Position', [820 45+yo 850 75+yo]);
    ptt = [mdl '/PT_tank_' sfx];
    make_pt(ptt, [880 40+yo 960 80+yo]);
    add_block('simulink/Sinks/To Workspace', [mdl '/tw_setpoint_' sfx], ...
        'VariableName', ['setpoint_out_' sfx], 'SaveFormat', 'Timeseries', ...
        'SampleTime', '-1', 'Position', [400 260+yo 480 290+yo]);
    add_block('simulink/Sinks/Terminator', [mdl '/t_demand_' sfx], 'Position', [400 210+yo 420 230+yo]);
    add_block('simulink/Sinks/Terminator', [mdl '/t_state_' sfx], 'Position', [400 235+yo 420 255+yo]);

    % wiring
    cph = h(ctl); svh = h(sv); lph = h(lp); dph = h(dti); pth = h(ptt);
    add_line(mdl, getfield(h([mdl '/mode_cmd_' sfx]), 'Outport'), cph.Inport(1), 'autorouting', 'on');
    add_line(mdl, getfield(h([mdl '/P_set_cmd_' sfx]), 'Outport'), cph.Inport(2), 'autorouting', 'on');
    add_line(mdl, cph.Outport(1), dph.Inport(1), 'autorouting', 'on');
    l = add_line(mdl, dph.Outport(1), svh.Inport(1), 'autorouting', 'on');
    set_param(l, 'Name', ['servo angle ' sfx]);
    set_param(dph.Outport(1), 'DataLogging', 'on');
    l = add_line(mdl, cph.Outport(2), getfield(h([mdl '/t_demand_' sfx]), 'Inport'), 'autorouting', 'on');
    set_param(l, 'Name', ['servo_demand_' sfx]);
    set_param(cph.Outport(2), 'DataLogging', 'on');
    l = add_line(mdl, cph.Outport(3), getfield(h([mdl '/t_state_' sfx]), 'Inport'), 'autorouting', 'on');
    set_param(l, 'Name', ['state_' sfx]);
    set_param(cph.Outport(3), 'DataLogging', 'on');
    add_line(mdl, cph.Outport(4), getfield(h([mdl '/tw_setpoint_' sfx]), 'Inport'), 'autorouting', 'on');
    add_line(mdl, svh.Outport(1), lph.Inport(1), 'autorouting', 'on');
    add_line(mdl, getfield(h([mdl '/run_valve_kv_' sfx]), 'Outport'), lph.Inport(2), 'autorouting', 'on');
    add_line(mdl, hph.Outport(1), lph.Inport(3), 'autorouting', 'on');
    add_line(mdl, hph.Outport(2), lph.Inport(4), 'autorouting', 'on');
    add_line(mdl, lph.Outport(2), getfield(h([mdl '/Sum_draws']), 'Inport', {b}), 'autorouting', 'on'); %#ok<GFLD>
    add_line(mdl, lph.Outport(1), getfield(h([mdl '/Pa2bar_' sfx]), 'Inport'), 'autorouting', 'on');
    l = add_line(mdl, getfield(h([mdl '/Pa2bar_' sfx]), 'Outport'), pth.Inport(1), 'autorouting', 'on');
    set_param(l, 'Name', ['P_tank_' sfx ' [bar]']);
    set_param(getfield(h([mdl '/Pa2bar_' sfx]), 'Outport'), 'DataLogging', 'on');
    add_line(mdl, pth.Outport(1), cph.Inport(3), 'autorouting', 'on');
    add_line(mdl, getfield(h(pt_hp), 'Outport'), cph.Inport(4), 'autorouting', 'on');
    add_line(mdl, svh.Outport(2), cph.Inport(5), 'autorouting', 'on');
end

% synchronous compile check
feval(mdl, [], [], [], 'compile');
feval(mdl, [], [], [], 'term');
save_system(mdl, target);
close_system(mdl, 0);
fprintf('built %s\n', target);
end

%% ===================== fuel branch (IPA, water-law) =====================
function build_branch_fuel(lp, plantdir)
h = @(b) get_param(b, 'PortHandles');
tk = [lp '/LP Tank'];
add_block('simulink/User-Defined Functions/MATLAB Function', tk, 'Position', [560 40 700 140]);
inject_mlfcn(tk, fullfile(plantdir, 'lp_tank_water.m'), ...
    {'V_2', 'rho_L_fu', 'R_N2'}, {'rho_L', 'rho_L_fu'});
fs = [lp '/Flow Solver'];
add_block('simulink/User-Defined Functions/MATLAB Function', fs, 'Position', [160 200 320 380]);
inject_mlfcn(fs, fullfile(plantdir, 'component_flow_solver.m'), ...
    {'Cd_3_fu', 'K_v_4_fu', 'rho_L_fu', 'R_N2', 'Tst', 'rhost', 'A_3_fu', 'Pst'}, ...
    {'Cd_3', 'Cd_3_fu'; 'K_v_4', 'K_v_4_fu'; 'rho_L', 'rho_L_fu'; 'A_3', 'A_3_fu'});
add_block('simulink/Discrete/Discrete-Time Integrator', [lp '/Int_m2'], ...
    'Position', [460 220 500 260], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'InitialCondition', 'm_2_0_fu', 'SampleTime', 'Ts_sim');
add_block('simulink/Discrete/Discrete-Time Integrator', [lp '/Int_m3'], ...
    'Position', [460 300 500 340], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'InitialCondition', 'm_3_0_fu', 'SampleTime', 'Ts_sim');
add_block('simulink/Math Operations/Gain', [lp '/neg_L'], 'Gain', '-1', 'Position', [390 305 420 335]);
add_block('simulink/Discontinuities/Saturation', [lp '/sat_L'], ...
    'UpperLimit', 'inf', 'LowerLimit', '0', 'Position', [340 360 370 390]);
add_block('simulink/Sinks/Terminator', [lp '/t_P3'], 'Position', [360 210 380 230]);
add_block('simulink/Sinks/Terminator', [lp '/t_P4'], 'Position', [360 250 380 270]);
add_block('simulink/Sinks/Terminator', [lp '/t_clamp'], 'Position', [500 420 520 440]);

tkh = h(tk); fsh = h(fs);
add_line(lp, getfield(h([lp '/P_1_pa']), 'Outport'), fsh.Inport(1), 'autorouting', 'on');
add_line(lp, tkh.Outport(1), fsh.Inport(2), 'autorouting', 'on');
add_line(lp, tkh.Outport(1), getfield(h([lp '/P_2_pa']), 'Inport'), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/T_hp']), 'Outport'), fsh.Inport(3), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/reg_valve_kv']), 'Outport'), fsh.Inport(4), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/run_valve_kv']), 'Outport'), fsh.Inport(5), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/T_hp']), 'Outport'), tkh.Inport(3), 'autorouting', 'on');
add_line(lp, fsh.Outport(1), getfield(h([lp '/t_P3']), 'Inport'), 'autorouting', 'on');
add_line(lp, fsh.Outport(2), getfield(h([lp '/t_P4']), 'Inport'), 'autorouting', 'on');
l = add_line(lp, fsh.Outport(3), getfield(h([lp '/sat_L']), 'Inport'), 'autorouting', 'on');
set_param(l, 'Name', 'm_dot_L_fu');
set_param(fsh.Outport(3), 'DataLogging', 'on');
l = add_line(lp, getfield(h([lp '/sat_L']), 'Outport'), getfield(h([lp '/neg_L']), 'Inport'), 'autorouting', 'on');
set_param(l, 'Name', 'm_dot_L_fu_clamped');
set_param(getfield(h([lp '/sat_L']), 'Outport'), 'DataLogging', 'on');
add_line(lp, getfield(h([lp '/sat_L']), 'Outport'), getfield(h([lp '/t_clamp']), 'Inport'), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/neg_L']), 'Outport'), getfield(h([lp '/Int_m3']), 'Inport'), 'autorouting', 'on');
l = add_line(lp, fsh.Outport(4), getfield(h([lp '/Int_m2']), 'Inport'), 'autorouting', 'on');
set_param(l, 'Name', 'm_dot_N2_fu');
set_param(fsh.Outport(4), 'DataLogging', 'on');
add_line(lp, fsh.Outport(4), getfield(h([lp '/n2_draw']), 'Inport'), 'autorouting', 'on');
l = add_line(lp, getfield(h([lp '/Int_m2']), 'Outport'), tkh.Inport(1), 'autorouting', 'on');
set_param(l, 'Name', 'm_2_fu');
set_param(getfield(h([lp '/Int_m2']), 'Outport'), 'DataLogging', 'on');
l = add_line(lp, getfield(h([lp '/Int_m3']), 'Outport'), tkh.Inport(2), 'autorouting', 'on');
set_param(l, 'Name', 'm_3_fu');
set_param(getfield(h([lp '/Int_m3']), 'Outport'), 'DataLogging', 'on');
end

%% ===================== oxidizer branch (two-phase N2O) ==================
function build_branch_ox(lp, plantdir)
h = @(b) get_param(b, 'PortHandles');
nt = [lp '/N2O Tank Properties'];
add_block('simulink/User-Defined Functions/MATLAB Function', nt, 'Position', [560 40 720 320]);
inject_mlfcn(nt, fullfile(plantdir, 'n2o_tank_properties.m'), ...
    {'V_2', 'n2o_lut', 'R_N2', 'cv_N2'});
fs = [lp '/Flow Solver'];
add_block('simulink/User-Defined Functions/MATLAB Function', fs, 'Position', [160 200 320 460]);
inject_mlfcn(fs, fullfile(plantdir, 'n2o_flow_solver.m'), ...
    {'n2o_lut', 'K_v_4_ox', 'Cd_3_ox', 'A_3_ox', 'R_N2', 'Tst', 'rhost', 'Pst'}, ...
    {'K_v_4', 'K_v_4_ox'; 'Cd_3', 'Cd_3_ox'; 'A_3', 'A_3_ox'});
ef = [lp '/Energy Flux'];
add_block('simulink/User-Defined Functions/MATLAB Function', ef, 'Position', [340 520 460 600]);
inject_mlfcn(ef, fullfile(plantdir, 'n2o_energy_flux.m'), {'cp_N2'});
ints = {'Int_mN2t', 'm_N2tank_0'; 'Int_mN2O', 'm_N2O_0'; 'Int_U', 'U_n2o_0'};
for i = 1:3
    add_block('simulink/Discrete/Discrete-Time Integrator', [lp '/' ints{i, 1}], ...
        'Position', [460, 200 + 70 * (i - 1), 500, 240 + 70 * (i - 1)], ...
        'IntegratorMethod', 'Integration: Forward Euler', ...
        'InitialCondition', ints{i, 2}, 'SampleTime', 'Ts_sim');
end
add_block('simulink/Math Operations/Gain', [lp '/neg_L'], 'Gain', '-1', 'Position', [390 275 420 305]);
add_block('simulink/Discontinuities/Saturation', [lp '/sat_L'], ...
    'UpperLimit', 'inf', 'LowerLimit', '0', 'Position', [340 480 370 510]);
terms = {'t_P3', 't_P4', 't_T', 't_mL', 't_mV', 't_PN2', 't_Psat', 't_flag', 't_clamp'};
for i = 1:numel(terms)
    add_block('simulink/Sinks/Terminator', [lp '/' terms{i}], ...
        'Position', [780, 180 + 35 * (i - 1), 800, 200 + 35 * (i - 1)]);
end

nth = h(nt); fsh = h(fs); efh = h(ef);
add_line(lp, getfield(h([lp '/Int_mN2O']), 'Outport'), nth.Inport(1), 'autorouting', 'on');
lg(lp, [lp '/Int_mN2O'], 'm_N2O');
add_line(lp, getfield(h([lp '/Int_mN2t']), 'Outport'), nth.Inport(2), 'autorouting', 'on');
lg(lp, [lp '/Int_mN2t'], 'm_N2_tank_ox');
add_line(lp, getfield(h([lp '/Int_U']), 'Outport'), nth.Inport(3), 'autorouting', 'on');
lg(lp, [lp '/Int_U'], 'U_n2o_ox');
add_line(lp, nth.Outport(1), fsh.Inport(2), 'autorouting', 'on');
add_line(lp, nth.Outport(1), getfield(h([lp '/P_2_pa']), 'Inport'), 'autorouting', 'on');
add_line(lp, nth.Outport(2), getfield(h([lp '/t_T']), 'Inport'), 'autorouting', 'on');
set_param(nth.Outport(2), 'DataLogging', 'on');
set_param(get_param(nth.Outport(2), 'Line'), 'Name', 'T_ox');
add_line(lp, nth.Outport(3), getfield(h([lp '/t_mL']), 'Inport'), 'autorouting', 'on');
set_param(nth.Outport(3), 'DataLogging', 'on');
set_param(get_param(nth.Outport(3), 'Line'), 'Name', 'm_3_ox');
add_line(lp, nth.Outport(4), getfield(h([lp '/t_mV']), 'Inport'), 'autorouting', 'on');
add_line(lp, nth.Outport(5), getfield(h([lp '/t_PN2']), 'Inport'), 'autorouting', 'on');
set_param(nth.Outport(5), 'DataLogging', 'on');
set_param(get_param(nth.Outport(5), 'Line'), 'Name', 'P_N2_ox');
add_line(lp, nth.Outport(6), getfield(h([lp '/t_Psat']), 'Inport'), 'autorouting', 'on');
set_param(nth.Outport(6), 'DataLogging', 'on');
set_param(get_param(nth.Outport(6), 'Line'), 'Name', 'P_sat_ox');
add_line(lp, nth.Outport(6), fsh.Inport(9), 'autorouting', 'on');
add_line(lp, nth.Outport(7), fsh.Inport(6), 'autorouting', 'on');
add_line(lp, nth.Outport(8), fsh.Inport(7), 'autorouting', 'on');
add_line(lp, nth.Outport(8), efh.Inport(4), 'autorouting', 'on');
add_line(lp, nth.Outport(9), fsh.Inport(8), 'autorouting', 'on');
add_line(lp, nth.Outport(10), getfield(h([lp '/t_flag']), 'Inport'), 'autorouting', 'on');
set_param(nth.Outport(10), 'DataLogging', 'on');
set_param(get_param(nth.Outport(10), 'Line'), 'Name', 'tank_flag_ox');
add_line(lp, getfield(h([lp '/P_1_pa']), 'Outport'), fsh.Inport(1), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/T_hp']), 'Outport'), fsh.Inport(3), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/T_hp']), 'Outport'), efh.Inport(2), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/reg_valve_kv']), 'Outport'), fsh.Inport(4), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/run_valve_kv']), 'Outport'), fsh.Inport(5), 'autorouting', 'on');
add_line(lp, fsh.Outport(1), getfield(h([lp '/t_P3']), 'Inport'), 'autorouting', 'on');
add_line(lp, fsh.Outport(2), getfield(h([lp '/t_P4']), 'Inport'), 'autorouting', 'on');
add_line(lp, fsh.Outport(3), getfield(h([lp '/sat_L']), 'Inport'), 'autorouting', 'on');
set_param(fsh.Outport(3), 'DataLogging', 'on');
set_param(get_param(fsh.Outport(3), 'Line'), 'Name', 'm_dot_L_ox');
add_line(lp, fsh.Outport(4), getfield(h([lp '/Int_mN2t']), 'Inport'), 'autorouting', 'on');
set_param(fsh.Outport(4), 'DataLogging', 'on');
set_param(get_param(fsh.Outport(4), 'Line'), 'Name', 'm_dot_N2_ox');
add_line(lp, fsh.Outport(4), getfield(h([lp '/n2_draw']), 'Inport'), 'autorouting', 'on');
add_line(lp, fsh.Outport(4), efh.Inport(1), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/sat_L']), 'Outport'), getfield(h([lp '/neg_L']), 'Inport'), 'autorouting', 'on');
set_param(getfield(h([lp '/sat_L']), 'Outport'), 'DataLogging', 'on');
set_param(get_param(getfield(h([lp '/sat_L']), 'Outport'), 'Line'), 'Name', 'm_dot_L_ox_clamped');
add_line(lp, getfield(h([lp '/sat_L']), 'Outport'), getfield(h([lp '/t_clamp']), 'Inport'), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/sat_L']), 'Outport'), efh.Inport(3), 'autorouting', 'on');
add_line(lp, getfield(h([lp '/neg_L']), 'Outport'), getfield(h([lp '/Int_mN2O']), 'Inport'), 'autorouting', 'on');
add_line(lp, efh.Outport(1), getfield(h([lp '/Int_U']), 'Inport'), 'autorouting', 'on');
set_param(efh.Outport(1), 'DataLogging', 'on');
set_param(get_param(efh.Outport(1), 'Line'), 'Name', 'dU_ox');
end

%% ============================ helpers ============================
function lg(~, blk, name)
ph = get_param(blk, 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on');
set_param(get_param(ph.Outport(1), 'Line'), 'Name', name);
end

function build_servo_valve(sv, plantdir, sfx, pos)
h = @(b) get_param(b, 'PortHandles');
add_block('simulink/Ports & Subsystems/Subsystem', sv, 'Position', pos);
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
add_block('simulink/Math Operations/Gain', [sv '/Kv Gain'], 'Gain', 'Kv_1_max', 'Position', [510 55 545 85]);
add_line(sv, getfield(h([sv '/servo_angle_cmd']), 'Outport'), getfield(h(gear), 'Inport'), 'autorouting', 'on');
l = add_line(sv, getfield(h(gear), 'Outport'), getfield(h([sv '/Backlash']), 'Inport'), 'autorouting', 'on');
set_param(l, 'Name', ['Valve angle ' sfx]);
set_param(getfield(h(gear), 'Outport'), 'DataLogging', 'on');
add_line(sv, getfield(h([sv '/Backlash']), 'Outport'), getfield(h(va), 'Inport'), 'autorouting', 'on');
add_line(sv, getfield(h([sv '/Backlash']), 'Outport'), getfield(h([sv '/valve_angle_fb']), 'Inport'), 'autorouting', 'on');
add_line(sv, getfield(h(va), 'Outport'), getfield(h([sv '/Kv Gain']), 'Inport'), 'autorouting', 'on');
add_line(sv, getfield(h([sv '/Kv Gain']), 'Outport'), getfield(h([sv '/reg_valve_kv']), 'Inport'), 'autorouting', 'on');
end

function make_pt(blk, pos)
add_block('simulink/Ports & Subsystems/Subsystem', blk, 'Position', pos);
set_param([blk '/In1'], 'Name', 'P_true_bar');
set_param([blk '/Out1'], 'Name', 'P_meas_bar');
mask = Simulink.Mask.create(blk);
mask.Description = 'Pressure transducer placeholder (passthrough). See single-branch model for upgrade recipe.';
mask.addParameter('Name', 'Ts_pt', 'Prompt', 'PT sample time [s]', 'Value', '0', 'Evaluate', 'off');
mask.addParameter('Name', 'noise_std_bar', 'Prompt', 'Noise std [bar]', 'Value', '0', 'Evaluate', 'off');
mask.addParameter('Name', 'quant_lsb_bar', 'Prompt', 'Quantization LSB [bar]', 'Value', '0', 'Evaluate', 'off');
mask.addParameter('Name', 'seed', 'Prompt', 'Noise seed', 'Value', '0', 'Evaluate', 'off');
end

function inject_mlfcn(blockpath, srcfile, param_names, renames)
src = fileread(srcfile);
if nargin > 3
    for r = 1:size(renames, 1)
        src = regexprep(src, ['\<' renames{r, 1} '\>'], renames{r, 2});
    end
end
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blockpath);
assert(~isempty(ch), 'MATLAB Function chart not found at %s', blockpath);
ch.Script = src;
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
