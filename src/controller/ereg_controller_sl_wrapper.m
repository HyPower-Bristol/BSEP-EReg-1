function [servo_speed_cmd, servo_demand_deg, state, P_set_active_bar] = ...
    ereg_controller_sl_wrapper(mode_cmd, P_set_cmd_bar, P_tank_meas_bar, ...
                               P_HP_meas_bar, valve_angle_meas_deg, ctrl_params)
%EREG_CONTROLLER_SL_WRAPPER Script of the MATLAB Function block in
% ereg_controller_model.slx (injected at build time by build_controller_model).
% ctrl_params is parameter-scope data resolved from the workspace.
%#codegen
persistent x
if isempty(x)
    x = ereg_controller_init();
end
[servo_speed_cmd, servo_demand_deg, state, P_set_active_bar, x] = ereg_controller_step( ...
    mode_cmd, P_set_cmd_bar, P_tank_meas_bar, P_HP_meas_bar, valve_angle_meas_deg, ...
    x, ctrl_params);
end
