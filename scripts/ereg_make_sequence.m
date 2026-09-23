function seq = ereg_make_sequence(P)
%EREG_MAKE_SEQUENCE Build the command timeseries for the From Workspace blocks.
%
% compat_v1 = true : reproduce v1's exact stimulus - arrays on the
%   linspace(0,10,10000) grid (spacing 10/9999 s) linearly interpolated by the
%   2 ms solver, including the ~2.7 bar setpoint / ~0.9 Kv edge sample at
%   t=1.0 and the Repeating Sequence period wrap to 0 at t=10. The mode trace
%   walks OFF->ARMED->PRESSURIZE in the two samples before t=1.0 (unobservable:
%   setpoint and tank pressure are exactly 0 there), hits RUN at t=1.0, and
%   drops to OFF on the final sample to mirror the sequence wrap.
%
% compat_v1 = false : clean stepped commands straight from P.sequence rows
%   {t, mode_name, P_set}. Steps are encoded with duplicated breakpoints so
%   the linearly-interpolating From Workspace blocks produce exact steps.
m = ereg_controller_modes();

if P.compat_v1
    assert(strcmp(P.fluid.name, 'water'), ...
        'compat_v1 replication is defined for water only (got %s)', P.fluid.name);
    num = round(P.Sim_Duration / 0.001);            % v1: Num_Steps = 10000
    tgrid = linspace(0, P.Sim_Duration, num)';      % v1 grid, spacing 10/9999
    setp = zeros(num, 1);
    runv = zeros(num, 1);
    setp(1001:end) = P.Target_Pressure;             % v1: idx 1001.. = active
    runv(1001:end) = 1;
    setp(end) = setp(1);                            % Repeating Sequence wraps
    runv(end) = runv(1);                            % to t=0 value at t=10
    seq.P_set_cmd_ts    = timeseries(setp, tgrid);
    seq.run_valve_kv_ts = timeseries(runv, tgrid);
    tm = [0; 0.996; 0.998; 1.0; P.Sim_Duration];
    md = [m.OFF; m.ARMED; m.PRESSURIZE; m.RUN; m.OFF];
    seq.mode_cmd_ts = timeseries(md, tm);
else
    n = size(P.sequence, 1);
    tr = cell2mat(P.sequence(:, 1));
    md = zeros(n, 1);
    for i = 1:n
        md(i) = m.(P.sequence{i, 2});
    end
    sp = cell2mat(P.sequence(:, 3));
    assert(issorted(tr) && tr(1) == 0, 'sequence must start at t=0 and be sorted');
    % ZOH mode trace (From Workspace block has Interpolate off)
    seq.mode_cmd_ts = timeseries(md, tr);
    % stepped setpoint via duplicated breakpoints (block interpolates linearly)
    [ts, vs] = stepped(tr, sp, P.Sim_Duration);
    seq.P_set_cmd_ts = timeseries(vs, ts);
    % run valve: open only in RUN (externally commanded, mirrors mode table)
    [tv, vv] = stepped(tr, double(md == m.RUN), P.Sim_Duration);
    seq.run_valve_kv_ts = timeseries(vv, tv);
end
end

function [ts, vs] = stepped(t, v, t_end)
ts = t(1); vs = v(1);
for i = 2:numel(t)
    eps_t = 1e-9;
    ts = [ts; t(i) - eps_t; t(i)]; %#ok<AGROW>
    vs = [vs; v(i-1);      v(i)]; %#ok<AGROW>
end
if ts(end) < t_end
    ts = [ts; t_end];
    vs = [vs; vs(end)];
end
end
