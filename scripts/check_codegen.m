function check_codegen(vdir)
%CHECK_CODEGEN Prove the controller compiles to C via both paths.
% Path A: Simulink/Embedded Coder build of ereg_controller_model (ert.tlc,
%         code only - no toolchain required).
% Path B: MATLAB Coder lib from ereg_controller_hw_step.m (the pure .m route
%         that proves the flight code is liftable without Simulink).
if nargin < 1, vdir = fileparts(fileparts(mfilename('fullpath'))); end
addpath(fullfile(vdir, 'src', 'controller'), fullfile(vdir, 'scripts'), fullfile(vdir, 'models'));
P = ereg_params();
ereg_fanout(P);

outdir = fullfile(vdir, 'output');
if ~exist(outdir, 'dir'), mkdir(outdir); end
old = cd(outdir);
restore = onCleanup(@() cd(old));

% --- Path A ---
assert(license('test', 'Real-Time_Workshop') == 1, 'Simulink Coder license unavailable');
% license('test') is true for licensed-but-not-installed products, so also
% check the installed product list before relying on ert.tlc.
v = ver;
tgt = 'ert';
if license('test', 'RTW_Embedded_Coder') ~= 1 || ~any(strcmp({v.Name}, 'Embedded Coder'))
    warning('Embedded Coder unlicensed or not installed - falling back to grt.tlc');
    load_system('ereg_controller_model');
    set_param('ereg_controller_model', 'SystemTargetFile', 'grt.tlc');
    tgt = 'grt';
end
slbuild('ereg_controller_model');
cfile = fullfile(outdir, ['ereg_controller_model_' tgt '_rtw'], 'ereg_controller_model.c');
if exist(cfile, 'file')
    fprintf('Path A OK: %s\n', cfile);
else
    d = dir(fullfile(outdir, '*rtw*'));
    fprintf('Path A build finished; generated dirs: %s\n', strjoin({d.name}, ', '));
end

% --- Path B ---
assert(license('test', 'MATLAB_Coder') == 1, 'MATLAB Coder license unavailable');
cfg = coder.config('lib');
cfg.GenCodeOnly = true;
cfg.GenerateReport = true;
hwdir = fullfile(outdir, 'codegen_hw');
codegen('ereg_controller_hw_step', '-args', {0, 0, 0, 0, 0, coder.Constant(P.ctrl)}, ...
    '-config', cfg, '-d', hwdir);
fprintf('Path B OK: %s\n', hwdir);
end
