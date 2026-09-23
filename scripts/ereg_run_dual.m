function [results, P] = ereg_run_dual()
%EREG_RUN_DUAL Run the dual-branch mission (IPA fuel + N2O ox, shared HP).
here = fileparts(mfilename('fullpath'));
addpath(here, ...
    fullfile(here, '..', 'src', 'controller'), ...
    fullfile(here, '..', 'src', 'plant'), ...
    fullfile(here, '..', 'models'));
P = ereg_params_dual();
ereg_fanout_dual(P);
results = sim('EReg_v2_Dual', 'StopTime', num2str(P.Sim_Duration));
end
