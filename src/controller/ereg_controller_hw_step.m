function [servo_angle_cmd_deg, servo_demand_deg, state_out, P_set_active_bar] = ...
    ereg_controller_hw_step(mode_cmd, P_set_cmd_bar, P_tank_meas_bar, P_HP_meas_bar, ...
                            valve_angle_meas_deg, p)
%EREG_CONTROLLER_HW_STEP Self-contained per-tick entry point for hardware.
%
% Wraps ereg_controller_step with the persistent controller state plus the
% servo position integration that the Simulink model keeps in a separate
% Discrete-Time Integrator block. Call at exactly p.Ts intervals. This is the
% MATLAB Coder codegen entry point (codegen path B):
%   codegen ereg_controller_hw_step -args {0,0,0,0,0, coder.Constant(p)} -config:lib
%#codegen

persistent x pos
if isempty(x)
    x = ereg_controller_init();
    pos = 0;
end

[speed, servo_demand_deg, state_out, P_set_active_bar, x] = ereg_controller_step( ...
    mode_cmd, P_set_cmd_bar, P_tank_meas_bar, P_HP_meas_bar, valve_angle_meas_deg, x, p);

% Forward Euler integrator, pre-update output (matches the Simulink DTI).
servo_angle_cmd_deg = pos;
pos = pos + p.Ts * speed;
end
