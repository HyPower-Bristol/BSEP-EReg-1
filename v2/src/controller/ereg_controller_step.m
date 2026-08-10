function [servo_speed_cmd, servo_demand_deg, state_out, P_set_active_bar, x] = ...
    ereg_controller_step(mode_cmd, P_set_cmd_bar, P_tank_meas_bar, P_HP_meas_bar, ...
                         valve_angle_meas_deg, x, p)
%EREG_CONTROLLER_STEP One control tick of the EReg flight controller.
%
% Fully discrete (forward Euler, sample time p.Ts). In RUN mode this replicates
% the v1 Simulink cascade bit-for-bit: gain-scheduled outer pressure PID ->
% demand gate -> clamp [0,180] -> inner valve-angle PI -> servo speed
% saturation. Expression grouping and summation order intentionally mirror the
% v1 block diagram (IEEE floating point is not associative) - do not "tidy"
% the arithmetic without re-running the bit-exactness test.
%
% The servo position integration lives OUTSIDE this function (Simulink
% Discrete-Time Integrator in ereg_controller_model, or ereg_controller_hw_step
% on hardware) so the valve-angle feedback loop is broken by that state,
% exactly as v1's servo integrator does.
%#codegen

m = ereg_controller_modes();

% --- State machine: externally commanded, no auto-advance, OFF always wins.
% Legal: OFF<->ARMED, ARMED->PRESSURIZE, PRESSURIZE<->RUN, any->OFF.
% Illegal commands hold the current state.
prev = x.state;
next = prev;
if mode_cmd == m.OFF
    next = m.OFF;
elseif mode_cmd == m.ARMED
    if prev == m.OFF || prev == m.ARMED
        next = m.ARMED;
    end
elseif mode_cmd == m.PRESSURIZE
    if prev ~= m.OFF
        next = m.PRESSURIZE;
    end
elseif mode_cmd == m.RUN
    if prev == m.PRESSURIZE || prev == m.RUN
        next = m.RUN;
    end
end

regulating = (next == m.PRESSURIZE) || (next == m.RUN);
was_regulating = (prev == m.PRESSURIZE) || (prev == m.RUN);
if regulating && ~was_regulating
    % Re-arm: fresh outer PID on entering regulation. Unobservable in the v1
    % replication scenario (states are already 0 there). Inner loop is never
    % reset - it tracks the physical valve at all times, as in v1.
    x.outer_integ = 0;
    x.outer_filt  = 0;
end
x.state = next;

% --- Error gating: v1's Switch forces the PID input to 0 when not regulating;
% the PID still executes its state updates every step.
if regulating
    e = P_set_cmd_bar - P_tank_meas_bar;
else
    e = 0;
end

% --- Gain schedule (v1 "PT" block law, same op order): 1x at full HP tank,
% (1+gs_gain)x at empty. p.P_HP_0_bar must be derived as P_1_0*1e-5.
mult = ((p.P_HP_0_bar - P_HP_meas_bar) * p.inv_P_HP_0_bar) * p.gs_gain + 1;

% --- Mode-selected outer gains: PRESSURIZE is tuned overdamped (admit
% pressure, then close, no overshoot at dead-head); RUN carries the v1 gains.
if next == m.PRESSURIZE
    g = p.press;
else
    g = p.run;
end

% --- Outer PID: Parallel form, forward Euler integrator and filter, external
% gain ports (P and I scheduled, D not), no output saturation, integrator
% STATE clamped at +/-p.int_limit. Outputs use pre-update states.
Pk  = g.K_P * mult;
Ik  = g.K_I * mult;
y_D = (g.K_D * e - x.outer_filt) * g.N;
u   = (Pk * e + x.outer_integ) + y_D;
demand = u + p.feedforward;

xi = x.outer_integ + p.Ts * (Ik * e);
x.outer_integ = min(max(xi, -p.int_limit), p.int_limit);
x.outer_filt  = x.outer_filt + p.Ts * y_D;

% --- Demand gate (v1's servo-enable Switch) then clamp to servo range.
if regulating
    demand_gated = demand;
else
    demand_gated = 0;
end
if demand_gated < 0
    demand_c = 0;
elseif demand_gated > 180
    demand_c = 180;
else
    demand_c = demand_gated;
end

% --- Inner valve-angle PI(D): internal gains, forward Euler, output saturated
% to +/-p.servo_speed_max (servo speed, deg/s), no anti-windup (integrator
% keeps winding while saturated - v1 behavior), integrator unclamped.
e_i  = demand_c - valve_angle_meas_deg;
y_Di = (p.inner.K_D * e_i - x.inner_filt) * p.inner.N;
u_i  = (p.inner.K_P * e_i + x.inner_integ) + y_Di;
servo_speed_cmd = min(max(u_i, -p.servo_speed_max), p.servo_speed_max);

x.inner_integ = x.inner_integ + p.Ts * (p.inner.K_I * e_i);
x.inner_filt  = x.inner_filt + p.Ts * y_Di;

% --- Diagnostics / telemetry.
servo_demand_deg = demand;              % pre-gate, matches v1 'servo_demand' log
state_out        = next;
if regulating
    P_set_active_bar = P_set_cmd_bar;
else
    P_set_active_bar = 0;
end
end
