function verify_physics(results, P)
%VERIFY_PHYSICS Fluid-agnostic correctness checks on a clean-mode run.
% Conservation, mass-balance consistency, capacity sanity, regulation quality.
ls_ = results.logsout;
sig = @(n) squeeze(ls_.getElement(n).Values.Data);
t = ls_.getElement('m_1').Values.Time;
m1 = sig('m_1'); m2 = sig('m_2'); m3 = sig('m_3');
pt = sig('P_tank [bar]'); st = sig('state');
mdotL = squeeze(results.fuelflow.Data);   % clamped liquid flow feeding m_3
Ts = t(2) - t(1);

% N2 conservation: everything leaving the HP tank enters the ullage
drift = max(abs((m1 + m2) - (m1(1) + m2(1))));
assert(drift < 1e-9, 'N2 mass not conserved: drift %.3g kg', drift);

% liquid mass balance: m_3 must equal its own forward-Euler integral
m3_pred = m3(1) - [0; cumsum(mdotL(1:end-1))] * Ts;
mb_err = max(abs(m3_pred - m3));
assert(mb_err < 1e-9, 'liquid mass balance broken: %.3g kg', mb_err);

% capacity sanity: liquid always fits, never negative
assert(max(m3) / P.fluid.rho_L <= P.V_2 + 1e-12, 'liquid volume exceeds tank');
assert(min(m3) >= 0, 'negative liquid mass');

% injector flow bound: m_dot <= Cd*A*sqrt(2*rho*P_tank) (P_3 < P_2 always)
bound = P.Cd_3 * P.A_3 * sqrt(2 * P.fluid.rho_L * max(pt) * 1e5);
assert(max(mdotL) <= bound * 1.01, ...
    'liquid flow %.4g exceeds physical bound %.4g kg/s', max(mdotL), bound);

% regulation quality
w_press = (st == 2);
assert(any(w_press), 'PRESSURIZE never entered');
p_press_max = max(pt(w_press));
assert(p_press_max <= P.Target_Pressure * 1.05, ...
    'PRESSURIZE overshoot: %.3f bar', p_press_max);
run_tail = t >= 8.0;
p_run_mean = mean(pt(run_tail));
assert(abs(p_run_mean - P.Target_Pressure) <= 0.2, ...
    'RUN not holding setpoint: mean %.3f bar', p_run_mean);

drained_L = (m3(1) - m3(end)) / P.fluid.rho_L * 1000;
fprintf(['physics OK [%s]: rho=%g kg/m^3, fill %.2f kg | plateau %.4f kg/s ' ...
    '(%.3f L/s) | drained %.2f L | PRESS max %.3f bar | RUN mean %.3f bar\n'], ...
    P.fluid.name, P.fluid.rho_L, m3(1), max(mdotL), ...
    max(mdotL) / P.fluid.rho_L * 1000, drained_L, p_press_max, p_run_mean);
end
