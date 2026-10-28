# Panda CBF-QP, asynchronous execution and mandatory table guard

Current behavior (2026-10-27): single- and dual-arm owners run the nominal torque
law at 1 kHz without waiting for the background CBF-QP. Each owner checks its
current candidate locally. Ordinary worker delay/failure selects braking then
joint holding while retaining FCI ownership. The shared world-table guard and
actuator bounds remain active with the optional CBF disabled.

This is the implemented numerical/model policy. Robot-free tests do not certify
physical collision avoidance, table clearance, model accuracy or worst-case
hardware callback timing. The table plane, base transforms, margins, tool model
and stopping response require operator commissioning.

## Runtime ownership and asynchronous handoff

`safety_filter` is linked into `teleop_controllers`; its launch file loads
parameters, not a second robot connection or ROS safety service. Each arm retains
one permanent libfranka owner across Cartesian, orientation-only, position hold,
homing and fallback modes.

- **Owner callback:** obtains current robot/model state and nominal torque,
  publishes a latest-only worker request, reads available worker/peer snapshots,
  performs bounded current-state local checks/repair, and returns torque.
  There is no worker wait or OSQP call in the default callback path.
- **Worker:** consumes the latest requests at a configurable cadence (5 ms by
  default) and solves the single/coupled CBF-QP. It uses a lower FIFO priority
  than the hardware FCI callback. Superseded requests do not form a backlog.
- **Result:** includes original model timestamps, cycles, nominal torque, joint
  state and mode/session/reference epochs. Completion does not refresh source
  age. Duplicate/out-of-order results cannot replace a newer cached result.
- **Acceptance:** source age, completion time, epoch and source-state change
  bounds must pass. Defaults permit at most 0.10 rad joint change and 1 rad/s
  velocity change from the worker source; those two bounds are C++ defaults.
  Current state/constraints and current `tau_J_d` are rechecked independently.

For a usable result, the preferred torque is the current nominal torque plus
`worker_safe - worker_nominal`. This retains current target changes; it does not
blindly replay a previous target torque. A release, session/reference change,
homing transition, controller-mode change or shutdown invalidates the old epoch.
A new epoch has a bounded warmup window while a matching worker result arrives;
that expected wait alone does not immediately disarm every new engagement.

Worker Jacobian derivatives use the actual interval between consumed model
source timestamps. Long history gaps reset derivative history. Callback-side
checks maintain independent history. No previous worker result is assumed safe
solely because it is younger than an age threshold.

### Dual-arm behavior

The background QP remains coupled and requires sufficiently fresh paired states.
Each owner also receives its peer's independently published latest snapshot,
without waiting for the worker. Local inter-arm rows eliminate the other arm's
possible torque over a magnitude/slew-bounded interval, rather than assuming it
will apply a cached command. Peer-state age adds a configurable displacement
reserve to **inter-arm geometry**, not to the current arm's table boundary.

When peer information is missing or stale, tracking is inhibited and the owner
brakes/holds using available local protection. Unavailable peer geometry is not
presented as a verified inter-arm result. Shared stop propagation remains; normal
finishing waits for both owners' finish requests, without requiring a final QP.

## Current-state guard and fallback

`AsyncSafetyController` owns per-arm model/result acceptance and braking/holding.
`CbfSafetyFilter::guard()` rebuilds current constraints without allocating an
OSQP workspace or calling the solver. It verifies a bounded preferred torque;
if it fails, the guard starts from a braking/holding command and performs at most
12 fixed constraint-repair sweeps. Eight sweeps consider the full enabled set;
the last four prioritize mandatory actuator/table rows. No convergence wait or
unbounded iteration is used. A fully feasible preferred torque needs no repair. Before escalating a failed
repair, the actual current robot torque is also bounded and checked against the
current mandatory rows; an old worker torque is never replayed unchecked.

The fallback joint reference decelerates its desired velocity toward zero using
`braking_acceleration`, with model acceleration feedforward and joint PD damping.
After measured velocity settles below 0.02 rad/s and the desired velocity has
reached zero, measured joints are captured once for holding. The anchor does not
follow old Quest targets. There are no autonomous retreat waypoints; safety
constraint corrections can still change the braking torque.

| Condition | Default asynchronous response |
| --- | --- |
| Late, stale, failed or infeasible worker result | Local braking/holding; FCI retained |
| Matching result not yet available for a new context | Bounded warmup holding; no stale target replay |
| Healthy worker after a latched fallback | Keep holding until fresh post-fault trigger release/re-engagement and healthy dwell |
| Secondary CBF shaping cannot all be satisfied | Brake/hold with a mandatory-table/actuator-valid candidate; report residuals and degraded status |
| Isolated invalid model calculation | Bounded model grace, braking/holding and extra displacement reserve |
| Invalid essential joint feedback or expired model grace | Exceptional shared stop |
| No candidate passes mandatory actuator/table bounds after bounded repair | Exceptional shared stop |

A relaxed braking command is **not certified to satisfy every obstacle,
singularity or joint-limit CBF**. The status explicitly distinguishes it from a
fully verified result. Constraint repair is a bounded feasibility heuristic; a
failure to find a mandatory command is not proof that none exists. Such failure
still cannot authorize sending an unchecked table command. Ordinary QP difficulty
alone does not terminate ownership.

Model grace defaults to 3 ms, requires finite current joint feedback and torque
anchors, and permits at most 0.01 rad/0.1 rad/s change from the last good model.
The good model timestamp is never advanced by reuse. This handles isolated model
failures, not indefinite sensor loss. The local reserve increases with joint
change while using model grace.

`cbf.execution: synchronous` retains the former exact-cycle wait for explicit
comparison. Its result/filter deadline and repeated-failure stops remain. With
optional CBF disabled, the mandatory local table/actuator path is used regardless
of the execution selector. The old 800 us filter / 900 us result limits do not
terminate default asynchronous operation. Actual libfranka/robot timing and
communication requirements still apply to the callback itself.

## QP formulation and constraints

The worker minimizes weighted squared deviation from nominal torque:

`min 0.5 * (tau - tau_nominal)^T W (tau - tau_nominal)`

subject to linear torque inequalities `lower <= A*tau <= upper`. There are 14
variables; inactive-arm torques are zero. Franka's gravity-compensated torque
convention gives `qdd = M^-1 (tau - coriolis)`. Tracking is a soft objective;
worker constraints have no slack variables and may be infeasible together.

- **Joint position:** Panda bounds tightened by 0.10 rad, with
  `hddot + 2*zeta*omega*hdot + omega^2*h >= 0`; default omega 8 s^-1, zeta 1.
- **Joint velocity:** 90% of Panda specification ceilings, with first-order
  barrier gain 10 s^-1.
- **Joint acceleration:** model-based limits of 7 rad/s^2 per joint.
- **Collisions:** 14 pinned Panda coarse capsules per arm; table, selected
  self-pairs, inter-arm pairs and optional configured static spheres. Self-pair
  exclusions follow the pinned Panda SRDF. Default added clearances are 0.02 m
  table, 0.03 m self, 0.05 m inter-arm and 0.04 m obstacles. Optional collision
  rows activate within 0.15 m beyond their padded boundary. Collision omega is
  8 s^-1 and damping ratio 1.2. Numerical Jdot and closest-point transitions are
  approximations; the model is not a detailed finger/cable/tool mesh.
- **Singularity:** optional determinant/manipulability barrier described below.
- **Actuators:** mandatory magnitude and rate interval intersection.

A parked arm gets 12 times the torque-deviation weight of a moving arm in the
coupled worker objective. This encourages preserving parked-arm torque without
preventing safety intervention. Priority persists while both are parked close
together and releases after separation.

Pinned OSQP 0.6.3 uses double precision, warm starts, a 100-iteration default,
1e-4 absolute/relative tolerances and independent 1e-3 final row feasibility
checking. A fully feasible nominal torque is the exact objective minimum and
bypasses iterative solving. A finite iteration-limit candidate can be accepted
only after independent bounds checking. Matrix shape/storage is allocated at
configuration; disabled rows are excluded from the fixed sparse solver mapping.
Worker solver failure flags are consumed as degradation information in async
mode, rather than the historical immediate ownership stop.

### Geometry computation and pair pruning

For exact capsule placements, stored centerline boxes, URDF mesh overlays and
world-coordinate exports at the configured home posture, see the
[geometry illustration](figures/cbf_geometry/README.md). The reproducible
robot-free script reads the C++ initializer and checks its 14 definitions
against the pinned URDF collision primitives before rendering.

Disabled collision families and inactive arms skip their geometry calculations.
Only capsules required by an enabled family are transformed. With optional CBF
off, the local guard transforms the 12 table-checked capsules per active arm;
it does not calculate self/inter-arm/obstacle distances. Disabled or unavailable
families report the existing `1e9` minimum-clearance sentinel, meaning **not
evaluated**, not measured free space. The table remains evaluated and enforced.

Self-pairs retain the pinned SRDF exclusions. Fixed base/base inter-arm pairs
are already excluded at configuration; the other 195 possible inter-arm pairs
remain eligible. Additional moving pairs are not permanently excluded based on
an assumed pose or because collision seems unlikely.

For enabled self/inter-arm/obstacle families, a cheap axis-aligned bounding-box
test precedes exact segment/point distances. Box distance is a conservative
lower bound on centerline distance. Exact calculation is skipped only when
that bound exceeds both the padded activation boundary and the best exact
minimum clearance found in this evaluation. Padding includes capsule/obstacle
radii, configured clearance, local reserve and (for inter-arm pairs) peer-age
reserve. A small numerical tolerance favors an exact check at the boundary.
This preserves active rows and exact minimum-clearance diagnostics for enabled
families. Bounds are rebuilt every evaluation, so an approaching pair returns
automatically; deactivation resets its Jacobian derivative history.

Independent feasibility checking and bounded local repair scan current active
rows in their original order, using storage reserved at configuration. Inactive
zero rows remain in the logical layout for fixed solver mapping/history but do
not consume repeated peer-elimination, residual or repair work. Internal C++
`SafetyResult::geometry_work` counters expose transformed capsules, exact distance
calculations and bounding-box rejections without callback allocation.

## Mandatory world-table plane

`table_height` is the horizontal table plane's world z. Both arms use their
surveyed `world_from_base_xyz_xyzw` transforms to evaluate it. The local guard
checks the lowest endpoint of every modeled capsule, subtracting capsule radius,
plane height, `table_minimum_clearance` and the local uncertainty reserve. Logical
links below 2 are excluded as mounting geometry; this does not model arbitrary
fixtures around the base.

Table rows remain present at **every height** in the local guard, even with
`pipeline.cbf_enabled: false`. The guard considers velocity, model acceleration
and torque-rate limits, rather than merely clipping an end-effector target z.
Zero-authority table rows are retained and can cause an exceptional stop.

There is no universal software guarantee that the table can never be hit:
incorrect transforms/geometry, insufficient braking distance, infeasible torque
bounds, disturbances and missed hardware callback deadlines remain relevant.
Choose surveyed plane/margins and validate stopping response before physical use.
The mandatory guard is a substantial change from the old CBF-disabled baseline.

## Nullspace posture and singularity constraint

The nominal Cartesian modes use the configured home joint posture and a
dynamically consistent secondary torque. With `M=L L^T`, `A=J L^-T`, the projector
is `N_tau=L (I-A^+A) L^-1`; fixed-size SVD uses relative rank tolerance 1e-8 and
absolute floor 1e-12. The secondary term combines home stiffness, joint damping
and soft joint-limit repulsion, with a uniform torque cap and entry ramp.
`controller.home_joint_position` is shared with actual homing; per-arm overrides
belong under `arm_settings.<arm>.controller`. This is a posture preference,
subject to the final safety path.

For singularity avoidance, `Js=[J_linear/length; J_angular]`,
`m=det(Js Js^T)`, `h=m/minimum_manipulability^2-1`. The acceleration barrier is
`hddot + 2*omega*hdot + omega^2*h >= 0`, including
`hddot=grad(h)^T M^-1(tau-c) + dq^T Hessian(h) dq`.
Default characteristic length is 0.3 m, minimum manipulability 0.05 and omega
8 s^-1. Measured joint screw axes/origins and the actual end-effector position
construct local perturbed kinematics; the reconstructed Jacobian is checked
against the backend Jacobian. Central differences use 1e-4 rad perturbations.

The worker checks the higher-order domain `h>=0` and `hdot+omega*h>=0`. In async
mode a domain violation degrades tracking and the local guard attempts the
restorative row where its gradient is valid. It does not immediately terminate
ownership solely for being outside that domain. At exact singularity the
manipulability gradient can provide no recovery direction; holding is not an
escape guarantee. Singularities and other secondary barriers are bypassed when
optional CBF is disabled; the mandatory table/actuator guards remain.

## Editable actuator limits, including CBF-disabled operation

`cbf.torque_limit_scale` and `cbf.torque_rate_limit_scale` are fractions in `(0,1]`
of Panda FCI ceilings, applied once. Default 0.35 gives joints 1--4 30.45 Nm,
joints 5--7 4.2 Nm and 350 Nm/s. Controller absolute caps can tighten them; dual
mode uses the stricter configured arm cap. The rate interval is anchored to
current `tau_J_d`, capped at 1 ms elapsed time, including for reused candidates.
An empty magnitude/rate intersection is invalid, not silently overridden.

## Configuration and diagnostics

Combined bringup reads its selected single/dual YAML; standalone launch uses
`safety_filter/config/safety.yaml`. Existing saved overall CBF switches, gains,
actuator scales and base transforms are preserved. Changes require restart.

```yaml
cbf:
  execution: asynchronous             # synchronous retained for comparison
  worker_period_ms: 5.0
  maximum_result_age_ms: 50.0          # from source, not completion
  maximum_worker_state_age_ms: 20.0
  maximum_peer_age_ms: 10.0
  maximum_worker_skew_ms: 5.0
  recovery_dwell_ms: 100.0
  model_grace_ms: 3.0
  braking_acceleration: 2.0            # rad/s^2
  fallback_hold_stiffness: 120.0       # Nm/rad
  fallback_hold_damping: 22.0          # Nm s/rad
  guard_clearance_reserve: 0.005       # m
  peer_motion_bound_mps: 22.0          # peer-age geometry envelope; validate tool/domain
  table_height: 0.0                   # surveyed shared world-z plane
  table_minimum_clearance: 0.02
```

`maximum_failures` governs the historical synchronous failure path; async mode
holds instead of stopping after an arbitrary count of failed solves. Timing
values and model motion bounds are commissioning settings, not delay certificates.

`/ARM/cbf_status` reports local verification status, total intervention from
current nominal torque, residuals, active constraints, table/self/inter-arm
clearances, manipulability, local filter time, worker solve time/status/source
cycles/age, result usability, peer usability, model reuse, braking/holding state,
fallback reason and shared stop reason. Original worker source timestamps remain
visible when its correction is reused. A locally feasible braking command does
not imply that the worker succeeded or tracking is re-enabled.

## Verification boundary

Robot-free tests cover single/dual normal launches, delayed worker cadence,
worker infeasibility with continued ownership, current-state constraints,
mandatory world-table behavior with CBF disabled, stale/out-of-order/context
rejection, braking/holding and explicit recovery, peer loss, isolated model grace,
allocation checks and exceptional table infeasibility. Performance measurements
on fixed fixtures do not establish worst-case hardware timing. Physical FCI,
payload/tool/frame validation, clearance and stopping acceptance remain operator
actions. See `IMPLEMENTATION_PROGRESS.md` for dated test evidence.
