function ereg_fanout(P)
%EREG_FANOUT Assign every workspace variable the v2 models reference into the
% base workspace (plant MATLAB Fcn parameters, block dialog vars, controller
% params, command timeseries). Single entry point used by the init script,
% the build scripts and check_codegen.
vars = struct( ...
    'Ts_sim', P.Ts_sim, 'Ts_ctrl', P.Ts_ctrl, 'Sim_Duration', P.Sim_Duration, ...
    'V_1', P.V_1, 'V_2', P.V_2, 'T_0_N2', P.T_0_N2, 'rho_1_0', P.rho_1_0, ...
    'gamma_N2', P.gamma_N2, 'R_N2', P.R_N2, 'rho_L', P.rho_L, ...
    'Cd_3', P.Cd_3, 'K_v_4', P.K_v_4, 'Tst', P.Tst, 'rhost', P.rhost, ...
    'A_3', P.A_3, 'Pst', P.Pst, 'Kv_1_max', P.Kv_1_max, ...
    'm_1_0', P.m_1_0, 'm_2_0', P.m_2_0, 'm_3_0', P.m_3_0, ...
    'backlash_width', P.backlash_width, 'backlash_init', P.backlash_init, ...
    'ctrl_params', P.ctrl);
fn = fieldnames(vars);
for i = 1:numel(fn)
    assignin('base', fn{i}, vars.(fn{i}));
end
if isfield(P, 'n2o_lut')   % N2O variant extras
    assignin('base', 'n2o_lut', P.n2o_lut);
    assignin('base', 'cv_N2', P.cv_N2);
    assignin('base', 'cp_N2', P.cp_N2);
    assignin('base', 'm_N2O_0', P.n2o_ic.m_N2O);
    assignin('base', 'm_N2tank_0', P.n2o_ic.m_N2);
    assignin('base', 'U_n2o_0', P.n2o_ic.U);
end
seq = ereg_make_sequence(P);
assignin('base', 'mode_cmd_ts', seq.mode_cmd_ts);
assignin('base', 'P_set_cmd_ts', seq.P_set_cmd_ts);
assignin('base', 'run_valve_kv_ts', seq.run_valve_kv_ts);
end
