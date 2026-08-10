%EREG_V2_INIT Set up and run the EReg v2 simulation.
% All tunables live in ereg_params.m - edit there, then run this script.
here = fileparts(mfilename('fullpath'));
addpath(here, ...
    fullfile(here, '..', 'src', 'controller'), ...
    fullfile(here, '..', 'src', 'plant'), ...
    fullfile(here, '..', 'models'));
P = ereg_params();
ereg_fanout(P);
results = sim('EReg_v2', 'StopTime', num2str(P.Sim_Duration));
