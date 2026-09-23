function [row, pass, rel] = compare_signals(name, t_ref, v_ref, t_got, v_got, tol)
%COMPARE_SIGNALS Per-signal error metrics on identical time grids (no resampling).
if nargin < 6, tol = 1e-6; end
v_ref = v_ref(:); v_got = v_got(:);
assert(numel(t_ref) == numel(t_got), '%s: sample count %d vs %d', name, numel(t_ref), numel(t_got));
assert(max(abs(t_ref(:) - t_got(:))) < 1e-9, '%s: time grids differ', name);
d = abs(v_ref - v_got);
maxabs = max(d);
rmse = sqrt(mean(d .^ 2));
rng_ = max(v_ref) - min(v_ref);
rel = maxabs / max(rng_, eps);
pass = rel <= tol;
if pass, verdict = 'OK'; else, verdict = 'FAIL'; end
row = sprintf('%-24s max|d|=%.3e  rms=%.3e  rel=%.3e  range=%.3e  exact %5d/%5d  %s', ...
    name, maxabs, rmse, rel, rng_, sum(d == 0), numel(d), verdict);
end
