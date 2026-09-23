function results = ereg_run(what)
%EREG_RUN One-command entry point for every v2 configuration.
% Each case is fully self-contained: it sets its own parameters, fans out its
% own workspace variables, uses the committed models, and runs its own
% verification and plots. No case depends on another having run first.
%
%   ereg_run('replication')  v1 water-graph 1:1 gate (compat mode; needs
%                            baseline_v1.mat - generate it once with
%                            verification/make_baseline_v1.m)
%   ereg_run('water')        clean-mode water mission + physics checks
%   ereg_run('IPA')          clean-mode IPA mission + physics checks
%   ereg_run('N2O')          flight-scale two-phase N2O mission + checks
%   ereg_run('dual')         shared HP bottle, IPA + N2O branches at once
here = fileparts(mfilename('fullpath'));
addpath(here, ...
    fullfile(here, '..', 'verification'), ...
    fullfile(here, '..', 'src', 'controller'), ...
    fullfile(here, '..', 'src', 'plant'), ...
    fullfile(here, '..', 'models'));

switch lower(what)
    case 'replication'
        run_verification(fileparts(fileparts(here)));
        results = evalin('base', 'results');
    case 'water'
        [results, P] = ereg_run_case('water');
        verify_physics(results, P);
        ereg_v2_plot(results, fullfile(here, '..', 'output', 'water'), P);
    case 'ipa'
        [results, P] = ereg_run_case('IPA');
        verify_physics(results, P);
        ereg_v2_plot(results, fullfile(here, '..', 'output', 'ipa'), P);
    case 'n2o'
        [results, P] = ereg_run_case('N2O');
        verify_physics_n2o(results, P);
        ereg_v2_plot(results, fullfile(here, '..', 'output', 'n2o'), P);
    case 'dual'
        [results, P] = ereg_run_dual();
        verify_physics_dual(results, P);
        ereg_plot_dual(results);
    otherwise
        error('ereg_run: unknown case "%s" (replication|water|IPA|N2O|dual)', what);
end
end
