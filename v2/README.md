# EReg v2

Clean rebuild of the EReg tank-drain simulation. Same physics and control law
as v1 (`EReg_Tank_DrainUSETHIS.slx`, checkpoint commit `d8d1f98`), restructured
so the controller is a single extractable, C-compilable, fully discrete unit.

## Run it

```matlab
cd v2/scripts
ereg_v2_init        % params -> workspace -> sim('EReg_v2')
ereg_v2_plot        % the four graphs -> v2/output/
```

Every tunable lives in **`scripts/ereg_params.m`** — plant geometry, fluid,
gains (per-mode sets), timing, sequence, PT placeholders. Nothing is defined
anywhere else.

## Architecture

```
mode_cmd / P_set_cmd / run_valve_cmd  (From Workspace, external test sequence)
      v
[Controller]  Model Reference: ereg_controller_model.slx
      |         - one MATLAB Function block wrapping src/controller/ereg_controller_step.m
      |         - state machine OFF(0)/ARMED(1)/PRESSURIZE(2)/RUN(3), externally
      |           commanded, never auto-advances, OFF always reachable
      |         - gain-scheduled outer pressure PID (per-mode gains: PRESSURIZE
      |           overdamped, RUN = v1 water gains) + inner valve-angle PI
      |         - outputs servo SPEED; position integration in a top-level DTI
      |           (breaks the valve-angle loop; hardware integrates in
      |           ereg_controller_hw_step.m)
      v
[servo_pos_integ] -> [Servo Actuated Valve] gear /1.7 -> backlash -> area poly -> Kv
      v
[Tank Plant]  verbatim v1 physics (src/plant/*.m): gas volume properties +
      |       two-Newton-solver component flow, 3 discrete mass integrators
      v
[PT_tank / PT_HP]  passthrough placeholders (masked; noise/quantization/rate
                   params declared for the upcoming PT model)
```

The run valve is commanded by the test sequence, not the controller
(`ereg_make_sequence`: open only in RUN rows for clean mode).

## Flight code extraction

`src/controller/ereg_controller_step.m` is the flight logic;
`ereg_controller_hw_step.m` is the per-tick hardware entry point (adds the
persistent state + servo position integration). Two C paths, both proven by
`scripts/check_codegen.m`:

- **Path A**: `slbuild('ereg_controller_model')` with ert.tlc (code only)
- **Path B**: `codegen ereg_controller_hw_step -args {0,0,0,0,0, coder.Constant(P.ctrl)} -config:lib`

Call at exactly `p.Ts` intervals on hardware. Do not "tidy" the arithmetic in
`ereg_controller_step.m` — expression grouping mirrors the v1 block diagram and
is covered by a bit-exactness test.

## Verification (v1 replication gate)

`verification/make_baseline_v1.m` captures the v1 run (main worktree) to
`baseline_v1.mat`. Two gates:

1. `test_controller_offline.m` — replays recorded closed-loop inputs through
   the controller code + gear/backlash replicas: **bit-exact, 0 difference,
   5001/5001 samples** on servo demand, servo angle, valve angle, valve area.
2. `run_verification.m` — full closed-loop sim in compat mode vs baseline
   (acceptance rel <= 1e-6 per signal):

| Signal | max abs diff | rel (to range) |
|---|---|---|
| P_tank [bar] | 2.0e-14 | 6.7e-15 |
| P_HP [bar] | 1.2e-13 | 1.3e-14 |
| servo_demand | 1.5e-11 | 6.8e-15 |
| servo angle | 5.7e-13 | 3.1e-14 |
| Valve angle | 3.4e-13 | 3.1e-14 |
| Normalised Valve Area | 9.0e-15 | 6.1e-14 |
| m_dot_L | 7.2e-12 | 1.2e-11 |
| m_dot_N2 | 1.3e-13 | 1.5e-11 |
| m_1 / m_2 / m_3 | <= 1.6e-14 | <= 1.5e-14 |
| T | 1.4e-12 | 1.9e-14 |

i.e. machine-precision agreement everywhere (differences are float
non-associativity across block execution order, 6-9 orders below acceptance).

Additional gates, both passing:

3. `smoke_test_clean.m` — clean-mode sequence OFF -> ARMED -> PRESSURIZE
   (dead-head, run valve closed: reaches 2.984 bar with zero overshoot on the
   overdamped gain set) -> RUN (valve open: holds 3.02 bar mean).
4. `check_codegen.m` — Path A generated `ereg_controller_model.c` (ert.tlc),
   Path B generated the MATLAB Coder lib from `ereg_controller_hw_step`
   (reports in `v2/output/`).

Compat mode (`P.compat_v1 = true`) reproduces v1's stimulus artifacts exactly:
the `linspace(0,10,10000)` grid whose linear interpolation makes the t=1.0
sample read ~2.7 bar (not 3), and the Repeating Sequence period wrap that
drops all commands to 0 on the final sample. Clean mode uses stepped commands
from `P.sequence` rows.

## Fluids

`ereg_params(fluid_name)` selects the working fluid: `'water'` (default, the
replication fluid) or `'IPA'`. The tank fill is volume-based (`P.V_fill` =
12 L), so water gives exactly the v1 12 kg while IPA gives 9.43 kg (12 kg of
IPA would not fit the 15 L tank). `ereg_run_case('IPA')` runs the standard
clean-mode mission; `verify_physics` checks conservation, mass balance,
capacity, injector-flow bounds and regulation quality for any fluid.

IPA results (no controller retune needed): flow plateau 0.3192 kg/s vs water's
0.3605 — ratio 0.885 vs the injector-theory prediction sqrt(786/1000) = 0.887,
i.e. the model tracks the physics to 0.1%. RUN holds 3.01 bar; dead-head
PRESSURIZE peaks 2.984 bar (no overshoot). Plots in `v2/output/ipa/`.

## N2O (two-phase, flight-scale)

`EReg_v2_N2O.slx` (`build_top_model(root,'n2o')`, run via `ereg_run_case('N2O')`)
replaces the LP tank with an **equilibrium two-phase N2O model with a full
energy balance**: states (m_N2O, m_N2_pad, U_total), per-step bisection on T
closing volume + energy against the CoolProp saturation LUT
(`n2o_archive/n2o_saturation_properties.csv`), P_tank = P_sat(T) + P_N2
(Dalton). Injector flow is **Dyer/NHNE** (k-weighted SPI/HEM, isentropic
downstream state from the LUT) — SPI-dominated under supercharge. Scenario:
300 bar HP bottle, 55 bar setpoint, 288 K, 12 L fill (report flight regime;
the water-rig numbers are meaningless for N2O since P_sat(288 K) = 44.9 bar).

Verified (`test_n2o_offline` + `verify_physics_n2o`): IC round-trip exact,
Dalton partial pressures exact, N2O/N2 mass and energy balances closed to
integrator precision, no regime clamps, honest self-cooling — the tank drops
4.0 K during the 2.6 kg drain with P_sat collapsing 44.9 -> 41.0 bar while
the controller holds 54.98 bar mean (54.83 bar max in dead-head, no
overshoot). N2O gains: report flight values (K_P=16, K_I=17, K_D=8,
gs_gain=2.5); PRESSURIZE pad set (0.5, 0.3, 2). Plots in `v2/output/n2o/`.

Model limitations (deliberate): full thermodynamic equilibrium (no boiling
lag), adiabatic walls, ideal-gas N2 at 300 bar, liquid-only outflow, LUT
bounded to the saturation dome (`tank_flag` reports violations).

## Roadmap seams
- **Dual controllers, shared HP tank**: `ereg_controller_model` already allows
  multiple instances; add a second Model block + LP branch and sum both
  regulator draws into the HP tank mass balance.
- **PT model**: fill in the PT_tank/PT_HP masked subsystems (ZOH at Ts_pt,
  noise via seeded Random Number, Quantizer) — interface already in place.
