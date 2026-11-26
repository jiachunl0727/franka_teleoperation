# Commissioning software preparation — 2026-11-05

Software preparation is complete for an explicit **single-arm** commissioning
path. This is not physical qualification. No hardware controller or Hand was
started during preparation. Use the [short procedure](HARDWARE_COMMISSIONING.md).

## Changes

- Added immutable `cbf.hardware_mode: blocked|shadow|filtered` and
  `cbf.commissioning_confirmed` selections. Defaults stay blocked/false and CBF
  stays disabled in saved single/dual YAMLs. Existing `operator_confirm` is also
  required for hardware. No runtime parameter update can change the selection.
- Bringup preflights the numerical file and commissioning policy before creating
  owner/Hand actions. Direct runtime independently validates the opt-in before
  connecting. Hardware commissioning requires one arm, asynchronous solving,
  timing and residual diagnostics, <=800 us callback budget, and no inter-arm
  constraints. Dual-arm hardware CBF remains blocked.
- Shadow computes proposals but issues the existing nominal controller's locally
  clipped torque. It does not install CBF reference projection or use holding
  torque. Invalid CBF domains/proposals remain diagnostics; nominal/input validity
  checks and the callback deadline remain enforced. This mode provides no CBF
  protection. Filtered operation retains fresh original-row and actuator checks,
  verified holding where feasible, and rejection when no valid command exists.
- Diagnostics distinguish `output_mode` and `actual_command_verified`. Proposal
  `accepted` does not assert the actual nominal command is CBF-feasible in shadow.
  Hardware timing now records model acquisition, callback entry intervals and
  CPU time beginning before acquisition. Wake-up lateness is explicitly marked
  invalid on hardware. Callback duration ends near return; FCI transport and tiny
  return operations are outside the measured interval.
- Preserved the user's qpOASES selection and constraint values. Corrected a test
  that hardcoded OSQP: it now verifies the parser matches the file's backend,
  while retaining explicit tests for all three native backends/formulations.

## Verification

Release builds passed for the filter, controllers and bringup. The final full
sequential regression passed **207 cases**: 70 numerical, 88 controller/runtime,
49 bringup, no failures or skips. New tests exercise active shadow references and
exact nominal clipping, invalid-domain observation without filtering, immutable
startup selections, explicit hardware opt-in routing, and rejection of invalid
hardware selections/settings before connection. All tests use mock/offline
execution; hardware-mode rejection fixtures fail before a Robot connection.

After the additional instrumentation, the filtered mock timing matrix passed all
12 backend/formulation/arm combinations: **56,021 observed callbacks**, **18,000
retained active samples**, zero >800 us callbacks, no faults and no diagnostic
queue drops. Observed maximum was **330.657 us single-arm** and **567.276 us
dual-arm**. Raw evidence: [filtered matrix](commissioning_filtered_mock_timing.csv).
The single-arm shadow timing matrix passed all six backend/formulation cases:
**18,778 observed callbacks**, 6,000 retained active samples, zero >800 us,
no faults or diagnostic queue drops; maximum **297.444 us**. See
[shadow matrix](commissioning_shadow_mock_timing.csv). Rejected proposals here
are observations, not permission to issue filtered torque.

This recheck is shorter than the earlier [90,000-active-sample report](CALLBACK_BUDGET_REPORT.md).

Profiling uses FIFO mock owner scheduling, 3 ms peer/proposal freshness, a
near-active joint barrier, and the real runtime/numerical path with fixed model
states. It enables self/inter-arm (dual only) and singularity families, but the
separated geometry makes distant rows redundant; at most 52 rows were admitted.
Filter/model diagnostic deadlines are widened to 1 s by the harness, whereas
800 us callback enforcement remains active. Solver iteration/tolerance settings
are those explicitly set by the harness, not a qualification of all saved
settings. These measurements cannot establish a worst-case bound, dense active
collision timing, physical model agreement, braking or tracking stability.

Reproduce in the ROS container, with setup sourced, from `/ws`:

```bash
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
python3 src/teleop_controllers/test/benchmark_avoidance_mock.py \
  --arms 1 2 --backends osqp qpoases piqp --formulations hard elastic \
  --execution-mode asynchronous --output-mode filtered --active-constraint \
  --realtime --samples 1000 --peer-age-us 3000 --budget-us 800 --enforce-budget \
  --output /tmp/filtered.csv
# For shadow: use --arms 1 --output-mode shadow with the same other options.
```

Hardware evidence remains to be collected: exact installation/model and wrist
behavior, physical callback/FCI timing with recording enabled, stopping response,
active-boundary behavior and tracking continuity. Shadow recordings must be
reviewed before applying corrections. An explicit opt-in permits experimental
commissioning; it does not certify these unresolved properties.
