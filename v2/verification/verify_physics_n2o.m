function verify_physics_n2o(results, P)
%VERIFY_PHYSICS_N2O Correctness checks for the two-phase N2O variant.
% Conservation (N2 and N2O mass, energy), Dalton consistency, LUT coherence,
% self-cooling, no regime clamps, regulation quality.
ls_ = results.logsout;
sig = @(n) squeeze(ls_.getElement(n).Values.Data);
t = ls_.getElement('m_1').Values.Time;
m1 = sig('m_1'); mN2t = sig('m_N2_tank'); mN2O = sig('m_N2O');
mL = sig('m_3'); T = sig('T'); U = sig('U_n2o'); dU = sig('dU');
Pt = sig('P_tank [bar]'); PN2 = sig('P_N2'); Psat = sig('P_sat');
flagsig = sig('tank_flag'); st = sig('state');
fuel = squeeze(results.fuelflow.Data);
Ts = t(2) - t(1);

% N2 conservation: bottle + pad constant
drift = max(abs((m1 + mN2t) - (m1(1) + mN2t(1))));
assert(drift < 1e-9, 'N2 not conserved: %.3g kg', drift);

% N2O mass balance vs its own integral
pred = mN2O(1) - [0; cumsum(fuel(1:end-1))] * Ts;
mb = max(abs(pred - mN2O));
assert(mb < 1e-9, 'N2O mass balance broken: %.3g kg', mb);

% energy balance vs integrated flux
Upred = U(1) + [0; cumsum(dU(1:end-1))] * Ts;
eb = max(abs(Upred - U)) / max(abs(U));
assert(eb < 1e-9, 'energy balance broken: rel %.3g', eb);

% Dalton: P_tank = P_N2 + P_sat (same-solve consistency)
dal = max(abs(Pt * 1e5 - (PN2 + Psat)));
assert(dal < 1, 'Dalton inconsistency: %.3g Pa', dal);

% LUT coherence: logged P_sat must equal P_sat(T) from the LUT
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'scripts'));
lc = max(abs(interp1(P.n2o_lut.T, P.n2o_lut.P, T) - Psat));
assert(lc < 1, 'P_sat(T) LUT mismatch: %.3g Pa', lc);

% no regime clamps or LUT-edge pins anywhere
assert(all(flagsig == 0), 'tank solver hit clamps: max flag %d', max(flagsig));

% self-cooling once draining
iRun = find(st == 3, 1);
assert(~isempty(iRun), 'RUN never entered');
dT = T(iRun) - T(end);
assert(dT > 0.1, 'no self-cooling during drain (dT=%.3f K)', dT);

% liquid never runs dry
assert(min(mL) > 0.3, 'tank nearly dried out: min m_L %.3f kg', min(mL));

% regulation
p_press_max = max(Pt(st == 2));
assert(p_press_max <= P.Target_Pressure * 1.05, ...
    'PRESSURIZE overshoot: %.2f bar', p_press_max);
p_run_mean = mean(Pt(t >= 8));
assert(abs(p_run_mean - P.Target_Pressure) <= 1.0, ...
    'RUN not holding setpoint: mean %.2f bar', p_run_mean);

fprintf(['physics OK [N2O]: T %.1f->%.1f K (dT=%.2f), P_sat %.1f->%.1f bar, ' ...
    'pad %.0f g N2, drained %.2f kg, peak flow %.3f kg/s | PRESS max %.2f | RUN mean %.2f bar\n'], ...
    T(1), T(end), dT, Psat(1) / 1e5, Psat(end) / 1e5, mN2t(end) * 1000, ...
    mN2O(1) - mN2O(end), max(fuel), p_press_max, p_run_mean);
end
