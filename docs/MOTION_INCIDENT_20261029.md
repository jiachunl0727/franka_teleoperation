> Historical incident and pre-removal mitigation record. The CBF implementation and model/joint supervisory policy described below have since been removed. See [current architecture](CONTROLLER_ARCHITECTURE.md).

# Abrupt motion and stalled wrists, 2026-10-29

Hardware operation is stopped. These changes contain observed failure patterns;
restored physical wrist tracking and safe stopping have not been demonstrated.
Do not treat the passing robot-free tests as clearance for another routine trial.
The Hand behavior and operator's saved impedance gains are unchanged by this fix.

## Evidence

The affected controller logs are
`cartesian_impedance_controller_17131_1793278911909.log` and
`cartesian_impedance_controller_17206_1793278941103.log`, preserved with their
model snapshots in `log/diagnostics/abrupt_motion_20261029/`. Recording was in Menu
mode and idle, so no matched raw-controller/tracking bag exists for these trials.
Consequently the logs cannot reconstruct the initiating hand pose/twist or prove
that a velocity spike initiated the movement.

- Trial 17131: reported tracking error grew from 4.16 mm to 98.3 mm and joint speed
  from 0.0228 to 0.697 rad/s. Wrist angles stayed approximately
  `[0.0857, 2.123, 0.7814]` rad. It ended with
  `no_valid_mandatory_table_command`; clearance alone did not establish a feasible
  stopping command for the current downward motion.
- Trial 17206: reported tracking error grew from 32.8 mm to 131.3 mm in about
  0.53 s. Nominal torque **vector norm** reached 181.907 Nm while final commanded
  torque norm was 7.09 Nm. A local repair failure selected fallback, followed by
  Franka `tau_J_range_violation`. Command success rate was 1.0 in the error report.
- The second saved state contains a proximal joint speed of 1.269 rad/s while
  wrist speeds remain approximately `[0.000285, 0.005920, 0.000234]` rad/s.
  Nevertheless `M^-1 * (tau_J_d - coriolis)` predicts wrist accelerations near
  `[-6.96, -6.91, -6.97]` rad/s². Wrist diagonal inertias are approximately
  `[0.0131, 0.0444, 0.00296]` kg m², with substantial coupling.

The wrists are receiving small filtered torques, rather than being omitted from
control. The acceleration model is inconsistent with their observed near-zero
motion. Friction, inner torque tracking and model/payload mismatch are plausible;
the available evidence does not identify their separate contributions. Raising
wrist torque or acceleration ceilings without resolving this disagreement could
create a breakaway surge.

## Reproductions and corrections

1. A small pose change with implausible finite angular feedforward previously
   remained enabled. The manager now rejects linear feedforward above 1 m/s or
   angular feedforward above 6 rad/s and latches protection.
2. Excessive measured speed and reference tracking error previously did not latch
   a stop; release/press could continue. Independent measured-state supervision
   now checks joint speed, tracking error and integrated model/velocity mismatch.
   It remains active regardless of the optional CBF switch. After teleoperation,
   joint speed and model disagreement remain supervised through release/holding.
3. Pose and independently processed velocity previously entered impedance directly,
   without servo-rate motion limits or effort saturation. A single reference
   trajectory now generates pose and twist together, starting from the measured
   pose on engagement/mode change. Speed and acceleration are bounded; native and
   pose-derived feedforward bias the bounded trajectory. The Cartesian wrench is
   norm-limited, then every selected nominal torque (including hold, damping and
   homing) is uniformly scaled if its modeled nominal acceleration exceeds the
   configured ceiling. Uniform
   scaling preserves the nominal effort direction and nullspace projection.
4. Fallback PD torque could overwhelm light wrist inertia even when
   `braking_acceleration` was 2 rad/s². The failing regression measured a modeled
   23.2933 rad/s² after the final local guard. The complete fallback correction,
   including PD feedback, is now bounded in model acceleration coordinates.
5. A motion-protection latch selects bounded model-based braking, then captures
   measured joints once after speed settles. It cannot be cleared by Quest
   release/press, session changes or homing. Restart requires investigation first.
   Its stop command bypasses cached worker corrections even if the worker is
   stale, missing or degraded; the final actuator/table guard still checks it.

The previous ideal-model simulations used the same dynamics for controller and
plant, so they could not detect stalled wrists. The new adversarial regression
uses inertia captured from the failed trial and locks the three wrist joints
while running the actual manager, impedance law, worker and local guard. Both
CBF settings and both commanded rotation signs are exercised. Separate tests
cover each stalled wrist, reference reversals, coherent pose/twist, independent
nominal acceleration limits, effort limits, persistent tracking failure,
latched recovery rejection and braking/holding bounds.

## Configuration and limits

These settings are explicit in the single/dual bringup files and standalone
`teleop_controllers/config/controllers.yaml`. They are positive, finite values;
there is no switch to disable this supervision along with CBF.

| Controller parameter | Default | Meaning |
| --- | --- | --- |
| `reference_linear_speed` | 0.05 m/s | Reference speed norm |
| `reference_angular_speed` | 0.15 rad/s | Reference angular speed norm |
| `reference_linear_acceleration` | 0.15 m/s² | Reference velocity change limit |
| `reference_angular_acceleration` | 0.5 rad/s² | Reference angular velocity change limit |
| `tracking_position_limit` | 0.03 m | Measured-to-governed-reference position error |
| `tracking_orientation_limit` | 0.20 rad | Shortest-arc orientation error |
| `joint_speed_limit` | 0.35 rad/s | Measured per-joint speed trip |
| `nominal_joint_acceleration_limit` | 7 rad/s² | Selected nominal effort's modeled acceleration ceiling |
| `model_velocity_error_limit` | 0.15 rad/s | Model-predicted versus measured velocity change per window |
| `model_error_window_seconds` | 0.05 s | Model comparison window |
| `cartesian_force_limit` | 10 N | Translational Cartesian wrench norm |
| `cartesian_moment_limit` | 1.5 Nm | Rotational Cartesian wrench norm |

A tracking error above half its limit for more than 0.25 s also trips, preventing
persistent reference accumulation before the absolute tracking bound is reached.
Protection requests at most 2 rad/s² modeled braking/holding acceleration.
These are commissioning software ceilings, not certified physical bounds. The
final safety guard may change a nominal/braking command to satisfy current
constraints; mandatory actuator/table feasibility takes priority. Neither the
nominal acceleration cap nor a model-based braking cap establishes actual
physical acceleration or stopping distance under a mismatched plant.

On the first protection trip the non-realtime reporter saves the model under
`/tmp/franka-cbf-hold-arm-ARM-TIMESTAMP.txt`, plus `.motion.txt` with the cause,
requested pose/twist, measured joint torques and gravity. This captures evidence
even when Menu recording is idle. Measured torque/gravity are physical evidence
only for the hardware backend.

## Remaining physical investigation

Wrist tracking is **not verified fixed**. Determine why commanded torque and
observed wrist acceleration disagree, including loaded model/tool settings and
measured torque relative to gravity. Record aligned input, commanded/desired/
measured torque, q/dq and tracking signals during controlled commissioning.
The protection is expected to refuse continued tracking when this disagreement
persists. Do not clear it repeatedly to continue an unstable trial.

## Offline verification

The Docker build/install completed with testing enabled. Fresh case-level XML:
35 Quest, 66 safety and 106 controller cases, **207 total, zero failures/errors or
skips**. This includes an actual mock process that verifies the protection latch,
rearming rejection and saved initiating input with CBF on and off. CTest wrapper
counts are excluded. Logs and `verification.json` are under
`log/diagnostics/abrupt_motion_20261029/`; the final runtime-only rerun is
`protection-runtime-final.log`. The final process check found no running arm
controller. No physical robot or Hand operation was performed.
