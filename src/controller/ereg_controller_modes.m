function m = ereg_controller_modes()
%EREG_CONTROLLER_MODES Mode command / state values shared by sim and flight code.
%#codegen
m = struct('OFF', 0, 'ARMED', 1, 'PRESSURIZE', 2, 'RUN', 3);
end
