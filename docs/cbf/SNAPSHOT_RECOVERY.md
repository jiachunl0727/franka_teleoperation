# Latest-snapshot directional recovery (experimental)

Dual bringup loads `config/dual_panda/cbf.yaml` directly. It contains the
recovery policy, torque computation setting and shared parameters such as table
height. Single bringup loads `config/single_panda/cbf.yaml` directly.

## Selecting torque computation

The recovery policy and its torque computation are separate settings. The dual profile selects `projection` and the single profile selects `native_qp`. To select the native solver:

```yaml
execution:
  constraint_strategy: snapshot_recovery
  recovery_torque_method: native_qp  # or projection
```

Edit `config/dual_panda/cbf.yaml` for dual arm or
`config/single_panda/cbf.yaml` for single arm. Restart launch after changing
it. The C++ default when omitted remains `projection`, preserving old recovery
profiles. `solver.backend` in the corresponding profile selects the native
backend (currently qpOASES); switching the computation does not change geometry,
clearance, held-arm handling, reference anti-windup or the 600/800 us single and
800/800 us dual guards.

With `native_qp`, the original requested inequalities are assembled first. The
actuator-box optimum and then the exact one-row shortcut are checked against
all requested rows; otherwise the existing selected native solver runs. Only
explicit infeasibility causes a second QP using the existing braking-anchor
recovery bounds. If a native attempt fails its iteration/numerical/verification
checks, the prior verified recovery projection remains a bounded fallback.
An invalid model remains a fault, and all work counts against the same timing
budgets. No unverified native output is sent. Native solves may change torques
and timing; the previous hardware trial used `projection` and is not a hardware
validation of this new computation choice.

`algorithm: recovery_native_qp` identifies the selection (including its shortcuts).
`requested_qp_status`, `recovery_qp_status` (`disabled` when not attempted),
`recovery_projection_fallback`, `iterations` and `analytic_solution` distinguish
actual backend work, explicit infeasibility and backup use. The final status
remains `verified_recovery`; requested residuals retain their existing meaning.
With `projection`, `algorithm` stays `anchored_directional_projection`.

## Behavior

Dual owners use the last data-valid peer sphere positions without extrapolation,
velocity prediction or acceleration inflation. Invalid/unavailable reads retain
the last valid cache; its original timestamp is preserved. The existing 10 ms
peer freshness limit remains. Single-arm recovery has no peer dependency and
adds no inter-arm section to the single-arm configuration.

A geometric or state-limit violation alone does not terminate this mode.
Position, velocity, table, obstacle, self-collision, inter-arm and valid
singularity rows support recovery. Negative position/geometry margins do not
request an autonomous return: their restoring term is clamped at zero, retaining
braking/non-worsening inequalities. Worsening reference increments and stored
tracking lead are projected out at violated/binding boundaries; feasible outward
and tangent commands remain available. Released owners omit inter-arm torque
corrections and retain their ordinary holding/local-safety policy.

With `recovery_torque_method: projection`, the normal requested inequalities are
tried first using bounded weighted
halfspace projections. If that pass certifies all requested rows, its result is
used unchanged. This prevents unnecessarily weakening a table row merely because
the generic braking reference violates it while another feasible torque exists.

If all requested rows cannot be certified in bounded work, recovery uses a
feasible braking reference: `c + M * clip(-braking_gain*dq, acceleration_limits)`,
clipped to the current actuator and torque-slew interval. Requested bounds that
exclude that reference are widened just enough to include it. A bounded second
projection and checked segment toward the braking reference produce an output
that satisfies the enforced recovery rows. No native QP solve is used in the `projection` computation choice. Work is bounded by two passes, each with the configured
`recovery_projection_sweeps` (default six), plus verification.

This is best-effort recovery within actuation limits, not proof that violated
clearance has already been restored. Inertia can prevent an immediate stop of
inward motion, and conflicting constraints can force recovery allowances.
Changing peer positions between snapshots can also violate clearance. Invalid
model/data, stale snapshots, impossible actuator intervals, workspace capacity,
SDK/hardware faults and execution deadlines still stop control. An undefined
singularity metric cannot provide a certified recovery direction and remains a
fault. The 600/800 us single and 800/800 us dual computation guards are retained.

## Diagnostics

`status: verified_recovery` and `algorithm: anchored_directional_projection`
identify this mode. `outside_domain_rows` counts
violated state domains; `recovery_rows` counts bounds widened for the braking
reference. `max_requested_violation` reports the actual output's largest residual
against the requested, un-widened rows. Existing `original_residual`/hard-row
verification refers to the currently enforced rows, including recovery bounds.
Thus `accepted` and `actual_command_verified` do not alone establish that the
configured minimum distance, table height or every requested limit was maintained.

## Table-plane replay on 2026-11-18

The hardware bag `quest_20261117T163522Z_4c9174d5` contains 123.5 s of this
unfinished recovery behavior. Every recorded normal CBF status was accepted and
outgoing-verified, but that did not imply all configured boundaries were met.
Full recorded-pose replay of the configured table capsules finds:

| Arm | Minimum modeled table-geometry surface Z | States below 0.127 m |
| --- | --- | --- |
| Left | 0.154329486 m | 0 / 122665 |
| Right | 0.101407645 m | 13045 / 122848 |

The right modeled surface went 25.592 mm below the real 0.127 m plane, and
0.592 mm below the old active 0.102 m plane (747 states). The lowest capsule is
an existing UMI finger-cover sphere, not the added housing cover. These are
model-based collision-surface heights, not direct physical contact measurements.
Full-rate causal peer alignment finds about 6.34 mm minimum inter-arm modeled
clearance versus 10 mm configured. This is useful recovery/usability evidence,
but not validation of general safety compliance.

Replaying local filtering with 0.127 m changes 7,620 right-arm commands out of 122,848 replayed states.
Recorded poses do not change in a frozen replay: it cannot prove what the new
closed-loop hardware trajectory would have been. Once below both planes, the
non-autonomous recovery rule allows a hold and outward commands; it does not
lift the arm automatically. Full measurements, alignment limits and validation
are recorded in `log/analysis/2026-11-18-recovery-table/RESULTS.md`.
