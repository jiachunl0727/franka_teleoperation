# Boundary references and terminal rejection capture

Implemented after the single-Panda filtered trial on 2026-11-11. The operator
requests blocking worsening motion at the table or a singularity boundary,
holding the last accepted reference in blocked directions, and allowing feasible
tangential motion and retreat. This replaces the earlier missing table-reference
supervision; it does not change the torque barrier equations or acceptance policy.

## Reference behavior

Filtered owners prepare current constraints before updating the nominal law.
Table rows now supply the same supervisory reference normals as self-collision
and singularity rows. This also works with table as the only geometric family.
Table normals are supplied when their barrier criteria indicate a binding or
violated boundary; no `boundary_activation_m` setting is used for table rows. A
previously binding table normal also supervises the next reference update. Table
torque rows are all retained, with unchanged coefficients,
bounds and ordering. Shadow observes the filter without applying this governor.

For a joint-space barrier gradient, the task-space normal is

\[
n = (J^\#)^T\nabla_q h,
\qquad n^T\Delta x \ge 0.
\]

For the table, `gradient(h)` is the WORLD vertical row of the capsule-point
Jacobian. Rotations that lower a finger therefore participate in the restriction,
as well as translations. For singularity, the gradient is that of the existing
smooth full-task conditioning barrier. Multiple normals form an intersection of
allowed half-spaces; a direction permitted by one can still be blocked by another.
The bounded projection checks its result and returns a zero increment if the
cone check does not pass. These are local reference directions; fresh torque
inequalities remain authoritative, including joint-nullspace effects.

The manager consumes each change in the incoming pose once. It discards the
worsening part of that increment, rather than repeatedly comparing an accumulated
unreachable absolute target with the held reference. In particular:

- Repeated inward input holds the accepted reference in that direction.
- A small reversal immediately permits retreat, even while the raw mapped target
  remains beyond the blocked pose.
- Feasible tangential components and orientation changes remain available.
- Activation also removes pre-existing inward reference lead. Optional speed /
  acceleration shaping uses the accepted goal, then its output displacement and
  twist are projected again to reject residual inward trajectory velocity.
- Rejected displacement is never automatically pursued later. The resulting
  command/reference offset persists until release/re-engagement resets the anchor.

"Hold" means a Cartesian reference, not a stored torque command. The selected
nominal controller continues recomputing torque from fresh feedback; actuator
clipping, QP constraints and exact outgoing-command verification still apply.
This does not introduce an unchecked backup law. An invalid measured domain,
unverifiable torque, stale state or missed deadline retains the explicit stop
policy. Reference blocking alone does not establish a physical stopping guarantee
or remove model / communication uncertainty.

The inverse-dynamics and original Cartesian laws, configurable gains, optional
shaping, input validity/release gates, physical table height, singularity threshold,
torque magnitude/slew limits, 600/800 us budgets and activation defaults are retained.

## Automatic rejection details

The first rejected constraint is retained separately from valid rows. When a CBF
failure policy is applied, the owner copies a fixed-size failure snapshot. This
includes failures during `prepare()`, before a nominal torque even exists, as well
as solve, final verification and callback-deadline failures. There is no logging,
JSON generation, file I/O or ROS publication in that capture.

The snapshot is serialized once by the ROS timer thread for a protective stop,
or after control unwinds for a fatal model fault:

```text
CBF_PROTECTIVE_STOP_SNAPSHOT {"schema":"cbf_failure_v1", ...}
CBF_CONTROL_TERMINATED_SNAPSHOT {"schema":"cbf_failure_v1", ...}
```

The saved launch log is the primary capture. The same JSON is also published as a
reliable, transient-local `std_msgs/String` on `<arm namespace>/cbf_rejection`.
Default recording profiles automatically include this topic for enabled CBF arms.
For custom topic replacements, add it explicitly. A ROS event can still be lost
at first-cycle startup or process teardown, so retain the launch log even
when recording a bag. Publication and the bounded acknowledgement wait occur
outside the real-time torque callback.

The snapshot records:

- Cycle, state timestamp, robot-reported period, callback-entry interval, elapsed
  callback time, model-acquisition time and available filter-completion time.
- Rejection stage and status, solver/backend/formulation information and model
  fingerprint.
- Joint position/velocity, mass matrix, Coriolis, incoming desired torque, measured
  torque and gravity, plus the exact current lower/upper torque interval.
- Last returned command and its cycle/timestamp, when available. This is command
  history, not proof of hardware acknowledgement.
- The bounded protective torque sent after a recoverable CBF failure, when
  applicable.
- Requested pose and computed reference/nominal/candidate when available.
- The rejected row's family, assembled row index, `h`, `psi`, coefficients and
  bound when a particular row caused the preparation failure.

An assembled row index is not a mesh identifier. Reproduce its geometry using the
saved model/CBF/geometry and joint state. Model-wide failures can have no failed
row. Early failures have `nominal_torque:null`, `reference_position:null` and
`filter_us:null`; nonfinite scalars become JSON null. Candidate residual validity
is explicit. `accepted:false` means the CBF result was rejected. For recoverable
failures, `command_issued:true` and `protective_stop:true` indicate that the
controller's bounded braking-and-hold command was sent while control continued.
Fatal invalid-model faults instead set `control_terminated:true` and issue no
protective command.

## Operator capture workflow

The session `run.sh` already saves stdout/stderr through `tee`; no additional
controller parameter is needed. Keep its complete `launch-*.log`, the closed bag
and the exact configuration/model snapshots. If running a different launch
command directly, preserve its output through `tee` too.

Extract terminal events on the host, substituting the actual new log filename:

```bash
python3 scripts/extract_cbf_rejections.py \
  log/commissioning/2026-11-11-hardware-cbf/launch-filtered-YYYYMMDDTHHMMSSZ.log \
  --output log/commissioning/2026-11-11-hardware-cbf/review/terminal
```

Each event produces a standalone JSON file. A normal run without a terminal CBF
failure has no terminal snapshot. The older completed trial cannot acquire its
missing state retroactively.

## Robot-free verification

Regression tests cover table-only reference blocking, inward/tangential/outward
directions, a virtual singularity threshold, immediate small retreat after blocked
overtravel with either shaping mode, rotational reversal and release/re-engagement.
Another test forces positive table clearance with negative approach-speed psi and
checks that the failed row is captured and cannot authorize a torque. Mock owner
tests require a parseable terminal snapshot for domain, solve-time and callback-time
failures while retaining the nonzero exit/no-fallback behavior.

`scripts/replay_reference_projection.py` runs the actual native constraint
preparation and reference manager with frozen recorded q/dq and mapped inputs,
with shaping off and on. Its probe law deliberately computes no nominal torque:
the result checks reference directions, not new torque acceptance or a physical
closed-loop rollout. `scripts/replay_avoidance.py` separately verifies the unchanged
torque constraints against the recorded nominal torques and incoming intervals.
Hardware behavior still requires a separately reviewed operator-run trial.
