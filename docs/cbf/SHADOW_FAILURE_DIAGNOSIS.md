# Shadow acceptance failures — 2026-11-05

## Contact-trial follow-up

The later corrected shadow trial ended with operator-intended fingertip contact
and a logged `cartesian_reflex`; recording shutdown was clean. Its modeled table
margin remained +49.701 mm, so the ordinary-finger assumption does not establish
clearance for the actual installation. Active proposal acceptance remained 4.93%.
See `log/commissioning/2026-11-05-shadow-corrected/review/contact-review.md`.

That review also exposed an error in the earlier deployment verification: the
controller caller remained compiled against an older `CommandInspection` layout
while the library had the new layout. The trial consequently lacks current-box
fields and has corrupted nominal family IDs. These fields are excluded from the
trial evaluation. The original binaries are preserved; a no-violation mock
regression reproduced the mismatch. A clean frozen-source rebuild repaired it,
and the strengthened schema/value assertions pass with all 212 affected cases.
Selected qpOASES mock timing also passes. The earlier incremental-build/timing
results below must not be treated as proof that the caller/library ABI matched.
No geometry/force threshold was changed or physical trial run in this repair.

The two single-Panda hardware shadow trials exposed a table-coordinate error,
two reproducible command-acceptance failures, and ambiguous diagnostics. The
operator reported smooth, collision-free motion. The numerical fixes preserve
the nominal shadow output and require independent current-row verification;
they do not declare every visibly clear trajectory CBF-feasible.

## Confirmed causes and fixes

### Physical table plane and finger geometry were counted twice

The operator clarified that the tabletop is **0.127 m** above the arm mounting
plane. The trial's **0.190 m** setting included an additional **0.063 m** finger
allowance. Table constraints already apply Hand/finger capsule extents and the
separate 0.005 m clearance, so this raised the guarded surface unnecessarily.

The saved trial configurations remain unchanged as evidence. The active CBF
configuration now uses the physical 0.127 m plane and retains the 0.005 m
clearance. Finger sizes belong in geometry, not in the table coordinate.

Replay of all recorded joint states with the repository's nominal kinematics
removes all **5,836** first-run table-domain failures. Minimum modeled margins
including clearance become **51.528 mm** and **141.000 mm** for runs 1 and 2.
The derivative-domain minima are also positive. This is a kinematic/domain
replay, not a replay of physical dynamics or proof of custom-finger clearance.
The operator-selected ordinary-finger assumption is retained; actual custom
finger mass/centre of mass are still not configured.

### Already feasible actuator-bounded nominal commands waited for a worker

The owner sends the filter a raw nominal law and the current actuator interval.
The former analytical check required the raw law itself to be inside that
interval. A command needing only normal controller clipping therefore entered
the asynchronous proposal path even when its clipped value satisfied every CBF
row. A minimal real-filter regression returns `awaiting_candidate` on all three
backends before the fix, despite a feasible clipped command.

Both execution modes now first inspect the actuator-box projection of the raw
nominal. Positive diagonal torque weights make this the exact box-constrained
optimum. If every current original inequality also holds with zero slack, it
is the full QP optimum and is accepted immediately. If any row fails, the native
QP path remains necessary. The raw objective remains available to that solver;
no constraint or slack-acceptance threshold was relaxed.

### A recent worker proposal could miss a moving actuator interval

An asynchronous solve uses the actuator interval at submission. The next owner
callback rebuilds its interval from the latest robot `tau_J_d`. Directly checking
the old absolute torque could reject a useful recent proposal simply because
its component lay outside the updated interval.

The owner now projects a finite, fresh, same-epoch proposal into the current
actuator interval and independently checks **every current original row** before
acceptance. The tested example moves a 0.25 Nm proposal into a new interval
starting at 0.26 Nm while the current barrier allows 0.5 Nm. It is accepted only
after verification. Tightening the current barrier to exclude 0.26 Nm still
rejects it. Stale, failed or wrong-epoch proposals remain rejected.

Shadow and filtered control now use the same command-epoch transitions,
including controller-law changes between input samples. Shadow still applies
only the nominal controller's actuator clipping; it never issues a CBF proposal.

### Proposal authorization was being read as nominal feasibility

`verify_command()` requires an accepted current transaction. Thus
`actual_command_verified:false` can mean that no proposal was accepted, even
when the nominal command itself satisfies the modeled rows. The old percentage
does not establish a corresponding percentage of unsafe issued commands.

New observation-only fields inspect the clipped nominal independently:

- `nominal_domain_valid` and `nominal_command_feasible`;
- `nominal_max_actuator_violation`, `nominal_max_row_violation` and
  `nominal_worst_family`;
- `current_box_infeasible_rows`, `current_box_max_min_row_violation` and
  `current_box_worst_family`;
- `proposal_residuals_valid`, distinguishing accepted-proposal residuals from
  unpopulated residual slots on failed/awaiting results.

For each original row, minimizing its residual over the current actuator box
provides a direct witness when that row is impossible for every available
torque. Zero individually impossible rows does not prove the combined QP is
feasible. Neither these observations nor current-row inspection confer command
authority; domain, epoch, age, timing and final outgoing-command checks remain.

Family numbers are position=0, velocity=1, acceleration=2, table=3,
obstacle=4, self-collision=5, inter-arm=6, singularity=7, peer-motion=8.

## Reproduction and verification

The two acceptance regressions were run against a saved pre-fix library and
both failed; the corrected library passes them. Additional regressions cover
asymmetric and zero-width actuator intervals with nonuniform weights, both QP
formulations and all backends, the captured first-run minimum-clearance state,
and independent inspection without authorizing an unchecked command.

The focused command is:

```bash
build/cbf_safety_filter/test_async
build/cbf_safety_filter/test_constraints
build/cbf_safety_filter/test_filter
```

Release build, full affected-package regression and mock callback measurements
are retained under `log/commissioning/2026-11-05-shadow/diagnosis/`. All agent-run
validation uses offline or mock execution in network-disabled containers.

Final regression: **212 cases passed**, no failures/skips: 75 numerical,
88 controller/runtime and 49 bringup. The first temporary test container lacked
the project's memory-locking limits, so four preconnection checks stopped at
that prerequisite; rerunning that group with the documented limits passed.

The updated asynchronous mock matrix passed all 12 filtered combinations
(56,784 observed callbacks, 18,000 retained active samples) and all six
single-arm shadow combinations (19,006 observed callbacks, 6,000 retained active
samples). No recorded callback exceeded 800 us, and no process faults or
diagnostic queue drops were observed. Maximum callback durations were
299.318 us single / 523.841 us dual in filtered mock and 390.189 us in shadow
mock. This short fixed-state workload uses the existing harness's widened
filter/model diagnostic deadlines; it does not qualify hardware scheduling,
physical dynamics or dense active collision constraints.

A separate corrected shadow snapshot passed bringup, numerical/geometry and
recording-plan preflight. Run it inside the ROS container with:

```bash
bash /ws/log/commissioning/2026-11-05-shadow-corrected/launch-shadow.sh
```

This is an operator-run physical command, including the retained empty-Hand
startup calibration. The saved source runtime remains CBF-disabled. The original
trial snapshots and bags remain unchanged; the corrected copy pins its own
runtime, numerical and geometry files.

## Remaining limits

The original bags do not contain full per-cycle mass/Coriolis data, inequality
coefficients or rejected-worker slack solutions. They cannot establish an exact
new torque-acceptance percentage after the changes. The original worker
`infeasible` and `slack_budget_rejected` statuses must not all be assigned to
solver defects; a torque-level barrier can conflict with the current actuator
interval even at a collision-free pose. No independent native-solver defect was
established by the solver regression.

The original 6.8/8.05 ms callback-entry gaps and 5/7 ms libfranka periods are
also not fixed by these acceptance changes. Their scheduler/FCI/recording cause
is not isolated by the bags. Callback deadlines, freshness thresholds and
positive-slack rejection remain enforced.

Self-collision, obstacle and singularity families were disabled in both saved
trials, so they did not cause these recorded rejections and are not validated by
this replay. A new operator-run shadow recording is required to measure the
corrected path, inspect independent nominal feasibility, and revisit physical
timing before selecting filtered hardware operation.
