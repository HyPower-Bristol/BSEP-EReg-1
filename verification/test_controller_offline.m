function test_controller_offline(baseline_file)
%TEST_CONTROLLER_OFFLINE Bit-exactness test of ereg_controller_step vs v1.
%
% Replays the recorded v1 closed-loop inputs (P_tank, P_HP, setpoint) through
% the v2 controller code plus plain-MATLAB gear/backlash replicas. Because the
% recorded traces were produced by exactly this signal chain, the outputs must
% reproduce the recorded servo_demand / servo angle / Valve angle / valve area
% samples bit-for-bit. Any deviation means the PID discretization is wrong.
here = fileparts(mfilename('fullpath'));
if nargin < 1, baseline_file = fullfile(here, 'baseline', 'baseline_v1.mat'); end
addpath(fullfile(here, '..', 'src', 'controller'));
addpath(fullfile(here, '..', 'src', 'plant'));
addpath(fullfile(here, '..', 'scripts'));

S = load(baseline_file);
b = S.baseline;
P = ereg_params();
p = P.ctrl;
m = ereg_controller_modes();

t = b.P_tank_bar.t;
n = numel(t);
Ts = p.Ts;
assert(abs(t(2) - t(1) - Ts) < 1e-12, 'baseline grid does not match Ts_ctrl');

% Compat mode trace reproducing v1's gating: regulation and servo enable both
% become active at t=1.0; the pre-1.0 ARMED/PRESSURIZE steps are unobservable
% (setpoint and tank pressure are exactly 0 there). The final sample drops to
% OFF because v1's Repeating Sequences wrap to their t=0 value at t=10.
mode = zeros(n, 1);
mode(t >= 0.996 - 1e-9) = m.ARMED;
mode(t >= 0.998 - 1e-9) = m.PRESSURIZE;
mode(t >= 1.000 - 1e-9) = m.RUN;
mode(t >= 10.00 - 1e-9) = m.OFF;

Pset  = b.setpoint_out.v;   % v1's exact per-sample setpoint (2.7 edge + wrap included)
Ptank = b.P_tank_bar.v;
Php   = b.P_HP_bar.v;

x = ereg_controller_init();
pos = 0;
y_bl = P.backlash_init;
half = P.backlash_width / 2;
demand = zeros(n, 1); servo = zeros(n, 1); valve = zeros(n, 1); area = zeros(n, 1);
for k = 1:n
    servo(k) = pos;                      % DTI pre-update output
    g = pos / P.gear_ratio;
    valve(k) = g;
    if g > y_bl + half                   % Simulink Backlash update
        y_bl = g - half;
    elseif g < y_bl - half
        y_bl = g + half;
    end
    [spd, demand(k), ~, ~, x] = ereg_controller_step( ...
        mode(k), Pset(k), Ptank(k), Php(k), y_bl, x, p);
    pos = pos + Ts * spd;
    area(k) = valve_area_poly(y_bl);     % bonus check of area poly + backlash replica
end

ok = true;
ok = check('servo_demand', demand, b.servo_demand.v) && ok;
ok = check('servo angle',  servo,  b.servo_angle.v)  && ok;
ok = check('Valve angle',  valve,  b.valve_angle.v)  && ok;
ok = check('valve area',   area,   b.valve_area.v)   && ok;
assert(ok, 'controller offline test FAILED');
fprintf('controller offline test PASSED\n');
end

function ok = check(name, got, ref)
got = got(:); ref = ref(:);
assert(numel(got) == numel(ref), '%s: length mismatch %d vs %d', name, numel(got), numel(ref));
d = abs(got - ref);
[mx, at] = max(d);
tol = 4 * eps(max(abs(ref)));
ok = mx <= tol;
fprintf('%-14s max|diff| = %.3e (tol %.1e) at sample %d, exact %d/%d  %s\n', ...
    name, mx, tol, at, sum(d == 0), numel(d), ternary(ok, 'OK', 'FAIL'));
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
