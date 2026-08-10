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

## Roadmap seams

- **IPA**: set `P.fluid = struct('name','IPA','rho_L',786)` — done.
- **N2O**: hard (two-phase self-pressurizing liquid + N2 supercharge; scoped
  out of the original report). Seam: swap `src/plant/gas_volume_properties.m`
  for a Dalton's-law variant; prototype material in `n2o_archive/`
  (`EReg_Tank_Drain_N2O_Sim.m`, saturation LUT csv). Own design pass required.
- **Dual controllers, shared HP tank**: `ereg_controller_model` already allows
  multiple instances; add a second Model block + LP branch and sum both
  regulator draws into the HP tank mass balance.
- **PT model**: fill in the PT_tank/PT_HP masked subsystems (ZOH at Ts_pt,
  noise via seeded Random Number, Quantizer) — interface already in place.
