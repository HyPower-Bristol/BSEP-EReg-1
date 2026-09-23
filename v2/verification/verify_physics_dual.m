function verify_physics_dual(results, P)
%VERIFY_PHYSICS_DUAL Correctness checks for the dual-branch model:
% cross-branch N2 conservation through the shared HP bottle, per-branch mass
% and energy balances, independent regulation of both setpoints.
ls_ = results.logsout;
sig = @(n) squeeze(ls_.getElement(n).Values.Data);
t = ls_.getElement('m_1').Values.Time;
Ts = t(2) - t(1);

m1 = sig('m_1');
m2fu = sig('m_2_fu'); m3fu = sig('m_3_fu');
mN2ox = sig('m_N2_tank_ox'); mN2O = sig('m_N2O'); m3ox = sig('m_3_ox');
Uox = sig('U_n2o_ox'); dUox = sig('dU_ox');
Tox = sig('T_ox'); flag = sig('tank_flag_ox');
Pfu = sig('P_tank_fu [bar]'); Pox = sig('P_tank_ox [bar]'); Php = sig('P_HP [bar]');
stfu = sig('state_fu'); stox = sig('state_ox');
mLfu = sig('m_dot_L_fu_clamped'); mLox = sig('m_dot_L_ox_clamped');

% Shared-HP N2 conservation: bottle + fuel ullage + ox pad = const
tot = m1 + m2fu + mN2ox;
drift = max(abs(tot - tot(1)));
assert(drift < 1e-9, 'shared-HP N2 not conserved: %.3g kg', drift);

% Per-branch propellant balances
pf = m3fu(1) - [0; cumsum(mLfu(1:end-1))] * Ts;
assert(max(abs(pf - m3fu)) < 1e-9, 'fuel mass balance broken');
po = mN2O(1) - [0; cumsum(mLox(1:end-1))] * Ts;
assert(max(abs(po - mN2O)) < 1e-9, 'N2O mass balance broken');

% Ox energy balance
Up = Uox(1) + [0; cumsum(dUox(1:end-1))] * Ts;
assert(max(abs(Up - Uox)) / max(abs(Uox)) < 1e-9, 'ox energy balance broken');

% Regime health
assert(all(flag == 0), 'ox tank solver clamped (max flag %d)', max(flag));
assert(min(m3fu) > 0.3 && min(m3ox) > 0.3, 'a tank nearly dried out');

% Independent regulation
for c = {{'fuel', Pfu, stfu, P.fu.Target}, {'ox', Pox, stox, P.ox.Target}}
    nm = c{1}{1}; pv = c{1}{2}; st = c{1}{3}; tgt = c{1}{4};
    assert(isequal(unique(st), [0; 1; 2; 3]), '%s: not all states reached', nm);
    pmax = max(pv(st == 2));
    assert(pmax <= tgt * 1.05, '%s PRESSURIZE overshoot: %.2f bar', nm, pmax);
    pmean = mean(pv(t >= 8));
    assert(abs(pmean - tgt) <= 1.0, '%s RUN off setpoint: mean %.2f bar', nm, pmean);
    fprintf('  %s: PRESS max %.2f | RUN mean %.2f (target %g)\n', nm, pmax, pmean, tgt);
end

fprintf(['physics OK [dual]: HP %.0f->%.0f bar, N2 drift %.1e kg | fuel drained %.2f kg | ' ...
    'ox drained %.2f kg, T_ox %.1f->%.1f K, pad %.0f g\n'], ...
    Php(1), Php(end), drift, m3fu(1) - m3fu(end), mN2O(1) - mN2O(end), ...
    Tox(1), Tox(end), mN2ox(end) * 1000);
end
