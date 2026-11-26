# Historical dual-arm CBF review: the two newest 2026-11-16 recordings

Subsequent agreement uses the current sphere approximation, clearance to
clearance +5 mm, and manual hardware tuning. This document preserves the original
diagnosis. The shared and predictive policies discussed later were removed on
2026-11-22; the current implementation supports only independent inter-arm
filtering. The analysis below is retained as historical trial context.

The two stops have the same underlying conflict: independent per-arm safety
filters reserve protection against a worst-case peer acceleration, while the
reference governor does not enforce the pair's joint feasibility. Releasing a
trigger adds an owner veto against inter-arm changes to that arm's holding law.
Holding both triggers removes that veto, but does not remove the conservative
peer model or the finite torque-slew interval. Configuration consolidation does
not resolve this policy conflict.

This review changes configuration selection and enables the requested joint
acceleration constraints. It leaves the collision policy unchanged for discussion.
All analysis/builds/tests were offline or mock; original recordings were read-only.

## Configuration changes

- `src/cbf_safety_filter/config/dual_panda/cbf.yaml` replaces `cbf.left.yaml`
  and `cbf.right.yaml`. Both owners use the same numerical configuration.
- `geometry.files_by_controller` selects `geometry.left.yaml` or
  `geometry.right.yaml` with the existing owner's `controller_id`. Bringup
  preflight and the owner use the same selector. Missing/unknown selectors,
  ambiguous single/dual geometry settings and incomplete maps are rejected.
- Joint velocity is explicitly hard (`relaxation.enabled: false`); this was
  already true in both inspected deployment files. Joint acceleration is now
  enabled and hard, with limits [15, 7.5, 10, 12.5, 15, 20, 20] rad/s².
- Dual bringup selects the shared file once; redundant per-arm CBF selections
  are removed. Surveyed transforms and UMI collision geometry remain distinct.
- `config/cbf.yaml` is the shipped single-arm example. Its unused inter-arm
  section is removed. Custom/test dual profiles can still explicitly configure
  inter-arm constraints; omission defaults to disabled.

The new acceleration setting was **not active in the two recordings**. It adds
constraints and cannot be assumed to improve the demonstrated inter-arm feasibility.

## Evidence and reproduction

| Recording | Context | Initiating owner / cycle | Terminal condition | Terminal modeled sphere surface gap |
|---|---|---|---|---:|
| `quest_20261116T165720Z_3e328f52` | Left held, right active | Left / 29014 | `held_arm_requires_intervention` | 87.17 mm |
| `quest_20261116T172011Z_5b85d4d0` | Both active | Right / 23021 | `infeasible`, stage `solve` | 82.59 mm |

The second-newest recording starts at 17:57:20 CET and the newest at 18:20:11 CET.
The gap is the minimum separation between the modeled covering spheres, across
all body pairs. It is **not** an independently measured fingertip-to-fingertip
distance. The closest pair in both terminal reconstructions has 80 mm summed
sphere radii and 10 mm added clearance. The gap difference is only 4.58 mm;
these two different trajectories/postures do not establish a general stopping
distance or a trigger-dependent distance setting.

Both bags pass SQLite integrity checks and retain their original SHA256 hashes:

```text
165720Z: bd0d20cca5c192b77601ff7989a5331af4cd223d897cad66ddfec109de915d64
172011Z: 7d43f15802af3062591a103bd2e6ac7377c297ff45eb4cb3171c2cdd45c1ada7
```

All 29,327 normal CBF status samples in the older bag and 24,808 in the newer
bag are accepted and outgoing-verified. Terminals are separate, rejected events.
The paired owner's `invalid_model` / `paired_owner_failure` is the propagated
stop reason, not evidence of an independent bad mass matrix.

The [recording audit](../../log/analysis/2026-11-16-dual-cbf/recording-audit.json)
contains transitions, commands-versus-clipped-nominal comparisons and terminal
exports. [Counterfactual results](../../log/analysis/2026-11-16-dual-cbf/counterfactuals.json)
come from the actual C++ filter and constraint builder, restoring recorded q/dq,
M/c, nominal torque, peer sphere states and exact torque bounds. The URDF FNV1a64
`9fd20a8e06bd4e9f` matches both recorded models.

The fast continuation check is:

```bash
python3 log/analysis/2026-11-16-dual-cbf/run_replays.py --assert-continuation
```

It fails deterministically for both symptoms: a nonlocal held-arm correction in
the first case and no accepted torque in the second. This is intentionally a
reproduction check, not a claim that the policy has been fixed. The companion
`run_replays.py` invocation without that flag writes the counterfactual sweep.
The retained probe source is `terminal_replay.cpp` in the same analysis directory.

The bags do not pin the full numerical YAML or the exact internal peer-check
timestamp. Replay uses the archived 10 mm / 15 mm / gains 10 profile and the
peer age at callback entry, with larger **offline-only** timing budgets. Therefore
it isolates causation but cannot exactly reproduce every live residual/torque.
At the older terminal, replay correction is 0.33777 Nm versus logged 0.36106 Nm;
growing age during live preparation plausibly accounts for the difference.
A full continuous closed-loop trajectory replay is not claimed.

## Why the held left arm's torque changes

The held reference is a fixed captured joint position, not a fixed torque.
`PositionHoldController::compute` applies ramped stiffness, velocity damping
and Coriolis compensation:

```text
tau_hold = ramp * K * (q_anchor - q) + D * (0 - dq) + c
```

Torque must vary to counter displacement and damp motion. Actuator magnitude
and slew clipping then apply. The robot handles gravity compensation; recorded
link-side measured torque includes gravity and is not directly comparable with
the gravity-excluding commanded torque.

In the older bag, **all 9,050 left `position_hold` tracking samples equal the
actuator-clipped holding nominal exactly**. There are zero issued CBF corrections
in that mode. Across its hold intervals, maximum measured joint displacement
from the respective captured anchor is about 0.00203 rad. This establishes
bounded observed holding error, not perfect immobility.

The large normal-status `intervention_norm` at release is insufficient evidence
of collision intervention: it compares the candidate against the raw nominal
and therefore also counts actuator clipping. The left release at cycle 20780,
for example, reports 5.3457 Nm intervention norm while `nominal_command_feasible`
is true and the candidate equals the clipped nominal. No inter-arm rows are
retained in that sample. The raw holding law is damping residual release motion.

The terminal is different. After roughly 8.301 s of left holding, the CBF
proposes a genuine inter-arm correction, up to 0.36106 Nm relative to the clipped
hold. The local-only problem accepts the unchanged hold. The owner rejects
that proposed change at `held_arm_requires_intervention`, and the terminal says
`command_issued: false`. Thus the rejected avoidance torque was never issued.

## Why both stops occur far outside 10 mm

The clearance is a geometric component of an acceleration-level barrier, not
a command to allow approach until a 10 mm visual gap. In `constraints.cpp`, the
inter-arm model uses:

```text
T = peer_age + robot_elapsed_period
position_uncertainty = peer_speed_limit * T
velocity_uncertainty = peer_acceleration_limit * T
```

The pair row also reserves worst-case peer acceleration **even with zero age**.
Each local QP chooses only its own seven torques. It has no peer control decision
and cannot count on the peer choosing a helpful acceleration. `PeerSnapshot`
contains geometry, velocity, time and global motion bounds; it contains neither
trigger/hold state nor a commitment to a chosen peer torque or acceleration.

The configured peer bounds are 1 m/s and 5 m/s². At the older terminal the
maximum recorded peer point speed is only 0.00792 m/s. At the newer right
terminal it is 0.000610 m/s: both triggers are engaged, but the peer is nearly
stationary by then. Small measured speed does not remove the 5 m/s² budget.

An illustrative limiting case shows the scale. At rest, with zero age/interval
uncertainty and zero local point acceleration, the implemented inequality needs

```text
alpha1 * alpha2 * (d² - R²) >= 2 * d * A_peer
```

For the closest pair, R = 0.09 m (radii plus clearance), gains 10 and 10, and
A_peer = 5 m/s². This gives d ≈ 0.15296 m, or a **72.96 mm sphere surface gap**,
even in this idealized stationary calculation. Actual age, velocity, local drift,
actuator authority and torque-slew restrictions alter that value. This calculation
explains the broad scale; it is not a fitted stopping-distance prediction.

A counterfactual reducing only the advertised peer acceleration from 5 to
2 m/s² makes the unchanged clipped nominal feasible at both terminal states.
That tests the source of conservatism; deploying such a reduction would also
require a compatible enforced producer bound and a justified motion contract.

## Timing and actuator feasibility distinguish the two events

Holding q/dq, M/c, peer state and the recorded torque box fixed, changing only
the period argument gives:

| Replay period | Older held-left state | Newer active-right state |
|---:|---|---|
| 1 ms | Unchanged hold feasible | Clipped nominal feasible |
| 2 ms | Unchanged hold feasible | Feasible correction, max 0.29824 Nm |
| 3 ms | Unchanged hold feasible | Infeasible |
| 4 ms | Feasible correction, max 0.10780 Nm | Infeasible |
| 5 ms | Feasible correction, max 0.33777 Nm | Infeasible |
| 6 ms | Infeasible | Infeasible |

In both actual terminal reconstructions, removing **only inter-arm rows**
leaves the unchanged clipped nominal feasible. This rules out the local table,
self-collision, joint-position/velocity and point-motion rows as the initiating
conflict in these frozen states.

The older left terminal has a robot elapsed period of 5 ms, 5.956 ms observed
callback interval, and 5.700 ms SDK receive wait. Its callback/filter take only
142.501 / 132.081 us. At entry peer age is 0.190 ms; the uncertainty horizon
therefore jumps from about 1.190 ms under the diagnostic 1 ms case to 5.190 ms.
The QP can meet the resulting row, but only by changing the held arm's law.

The newer right terminal has a robot elapsed period of 4 ms. Its callback-entry
interval is only **169.851 us**, and the SDK reports four received states without
waiting. The previous right tracking interval is about 4.015 ms. Robot state
progression, PC callback spacing and peer age are different measurements; this
terminal is a catch-up update, not a fresh 4 ms receive wait. Its callback/filter
take 298.555 / 290.830 us, and the entry peer age is 0.814 ms.

The exact actuator boxes in both terminals have 0.7 Nm width per joint. The
actuator helper limits slew using `min(elapsed_period, 1 ms)`, whereas the
inter-arm uncertainty uses the full elapsed period. A long reported period thus
expands uncertainty without expanding the available command step. This mismatch
is a real contributor to abrupt loss of feasibility; changing either convention
requires a time/actuator contract rather than a numerical workaround.

For the newer right state at 4 ms, at least one inter-arm row is violated by
**0.095866** even by the best torque anywhere in that box. This is a row residual,
not a distance or torque. It proves infeasibility independent of solver convergence.
The recorded solver's `infeasible` is therefore supported by the reconstructed
inequalities. Neither terminal is a filter deadline overrun or a stale-peer stop.

## The reference governor and no-yield policy do not close the loop

The fresh geometric reference band is clearance plus 15 mm; it does not reach
these 82–87 mm surface gaps, even after entry-age inflation. Both terminal
reconstructions generate zero **fresh geometric** reference normals. The filter
also carries previously binding constraint normals into the next reference
projection; the moving arms do receive such reactive supervision. It would be
incorrect to say the current governor has no effect outside the geometric band.

However, those normals supervise each owner's own reference. They neither
certify the other owner's unchanged holding law nor reserve feasibility for a
future timing/uncertainty jump. In the older recording, the moving right arm
already received 191 issued CBF corrections; the held left received none. In the
newer recording, active left/right received 779/339 corrections relative to their
clipped nominals. All identified binding correction rows in this audit are
inter-arm family 6.

The current no-yield check runs **after** the held owner's QP is solved. It
rejects an inter-arm-driven change rather than communicating the held constraint
to the moving owner's feasible set in advance. When both arms are active, that
veto is absent and both may accept avoidance corrections. They can approach
somewhat further before an owner exhausts its local torque box. Posture, command
history, relative velocity and timing also differ between the recordings; the
4.58 mm difference cannot be attributed solely to the trigger policy.

The separate conservative `hold_correction_is_local` certificate can reject
uncertifiable native-solver cases in general. That is not this older event:
its analytic candidate truly depends on inter-arm rows, and the unchanged local
holding optimum violates those rows.

## Architectural decision for discussion

The key requirement to decide is: **must a released arm retain its captured
holding law while the moving arm takes collision-avoidance responsibility?**
The current owner enforces that preference only as a terminal veto. It needs
to be part of a shared relative-motion feasibility policy.

Two viable directions have different costs:

| Direction | Required contract | Main tradeoff |
|---|---|---|
| Joint safety optimization over both arms' torques | Coherent paired models and actuator intervals; strict held-law equality for a released arm; bounded candidate delivery and owner-side verification | Directly represents relative acceleration and responsibility, but introduces paired computation and synchronization work |
| Independent owners with explicit responsibility allocation | Timestamped hold/active intent plus enforceable acceleration/avoidance commitments, compatible inequalities on both sides, and rules for delayed/failed commitments | Preserves independent local solves, but the distributed feasibility proof and switching protocol are more complex |

My preference for the next design discussion is a shared relative-acceleration
safety decision with explicit held-arm constraints, retaining local FCI ownership
and final command checks. A common reference governor should restrict requested
motion early enough to keep that paired decision feasible. A joint QP alone does
not supply anticipatory reference control or remove delay/model uncertainty.

Either direction must define a justified **future command horizon** separately
from past robot elapsed time, peer age/skew and PC callback spacing, and specify
what happens during missed updates and mode transitions. A fresh peer timestamp
after a reception pause does not prove uninterrupted physical control.

Increasing the geometric activation band, changing gains, lowering peer bounds,
allowing the held arm to yield, or capping the elapsed period would each change
behavior. None alone establishes the missing coordination contract. The two
recordings support discussing that contract before tuning for a 10 mm approach.

## Validation

- Three affected packages build successfully.
- All 10 CBF CTest targets pass, including new shared-profile/invalid-selection tests.
- All 62 bringup tests pass, including shared-file preflight selection.
- Existing inter-arm policy, controller-manager and CBF runtime targets pass.
- A new two-owner mock integration test passes with the shared profile, distinct
  geometry, valid peers, zero soft rows, and 14 hard velocity plus 14 hard
  acceleration rows observed on each owner.
- The frozen-state continuation check still fails as documented: the requested
  architectural policy change has deliberately not been implemented.
