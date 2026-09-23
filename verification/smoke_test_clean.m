function smoke_test_clean(vdir)
%SMOKE_TEST_CLEAN Exercise the state machine the way v1 never could:
% OFF -> ARMED -> PRESSURIZE (dead-head to 3 bar on the overdamped gain set,
% run valve closed) -> RUN (valve open, v1 gains). Asserts the PRESSURIZE
% phase reaches setpoint without overshoot (dead-head overshoot is permanent -
% the check valve prevents venting) and RUN holds setpoint under flow.
if nargin < 1, vdir = fileparts(fileparts(mfilename('fullpath'))); end
addpath(fullfile(vdir, 'scripts'), fullfile(vdir, 'src', 'controller'), ...
    fullfile(vdir, 'src', 'plant'), fullfile(vdir, 'models'));

P = ereg_params();
P.compat_v1 = false;
P.sequence = { ...
    0.0, 'OFF',        0; ...
    0.5, 'ARMED',      0; ...
    1.0, 'PRESSURIZE', P.Target_Pressure; ...
    6.0, 'RUN',        P.Target_Pressure};
ereg_fanout(P);
results = sim('EReg_v2', 'StopTime', num2str(P.Sim_Duration));

ls_ = results.logsout;
pt = ls_.getElement('P_tank [bar]').Values;
st = ls_.getElement('state').Values;
t = pt.Time; v = squeeze(pt.Data); s = squeeze(st.Data);

press_win = t >= 1.0 & t < 6.0;
run_tail  = t >= 8.0;
p_press_max = max(v(press_win));
p_press_end = v(find(press_win, 1, 'last'));
p_run_mean  = mean(v(run_tail));
p_run_max   = max(v(run_tail));

fprintf('smoke: states seen = [%s]\n', num2str(unique(s)'));
fprintf('smoke: PRESSURIZE max=%.3f end=%.3f bar | RUN tail mean=%.3f max=%.3f bar\n', ...
    p_press_max, p_press_end, p_run_mean, p_run_max);

assert(isequal(unique(s), [0; 1; 2; 3]), 'not all states reached');
assert(p_press_max <= P.Target_Pressure * 1.05, ...
    'PRESSURIZE overshoot: %.3f bar (dead-head overshoot is unrecoverable)', p_press_max);
assert(abs(p_press_end - P.Target_Pressure) <= 0.15, ...
    'PRESSURIZE did not settle at setpoint: %.3f bar', p_press_end);
assert(abs(p_run_mean - P.Target_Pressure) <= 0.2, ...
    'RUN not holding setpoint: mean %.3f bar', p_run_mean);
fprintf('clean-mode smoke test PASSED\n');
end
