function x = ereg_controller_init()
%EREG_CONTROLLER_INIT Zeroed controller state (matches v1 block initial conditions).
%#codegen
x = struct( ...
    'state',       0, ...   % OFF
    'outer_integ', 0, ...   % outer PID integrator (clamped +/-p.int_limit)
    'outer_filt',  0, ...   % outer PID derivative filter state
    'inner_integ', 0, ...   % inner position PI integrator (unclamped)
    'inner_filt',  0);      % inner PID derivative filter state (D=0 so stays 0)
end
