# Historical record: replayable single-arm shadow failures

The optional live CBF failure-capture feature described below has been removed
from the controller runtime. The historical capture format and measurements
remain archived here. The current offline replay reader accepts hard-row-only
`CBF_PROBLEM_V2` snapshots; the older worker/candidate and relaxation format
described below requires conversion before replay.

The completed 2026-11-05 free-space capture is evaluated in
the corresponding hardware review record.
It establishes joint-barrier/slew incompatibility and current-row candidate
rejection, with a large model-response discrepancy. The operator has now supplied
the finger STL; its preserved mesh and local cover are in the same trial's
`finger/` directory. Runtime geometry and control settings remain unchanged.

The hardware acceptance failure remains unresolved. This diagnostic route records
the equations needed to separate it from solver convergence and solver rejection. Finger CAD is not required for that investigation.

The current configuration enables 14 joint-position rows, 14 joint-velocity rows
and 24 table rows. Joint margins and torque-level barrier conditions are added
by this controller; they are not equivalent to Franka accepting an issued command.
Shadow issues nominal actuator-clipped torque, not a CBF proposal. Hardware
acceptance of that command does not prove the CBF's different feasible set is
nonempty. Positive distance alone also does not establish acceleration/braking
feasibility.

## Captured evidence

Each `.qp.txt` stores the current owner's original row coefficients/bounds,
family, domain values, nominal torque and actuator box, host model timestamp
and cycle. Older captures additionally stored relaxation settings, command
epoch, worker status and an optional candidate record. A
paired `.qp.model.txt` stores joint state, mass, Coriolis, incoming desired
torque, poses/Jacobians and raw nominal law in `ROBOT_MODEL_V1` format. The
ordinary bag provides the matching tracking/input context by cycle/timestamp.

These worker and candidate fields describe historical asynchronous captures;
they are not part of the current implementation. Domain-invalid historical
snapshots contain partial rows and must be inspected with their paired model.

## Offline replay

```bash
ros2 run cbf_safety_filter cbf_replay /absolute/cbf.yaml /absolute/problem.qp.txt
# Optional larger OFFLINE iteration budgets, never a runtime deadline override:
ros2 run cbf_safety_filter cbf_replay /absolute/cbf.yaml /absolute/problem.qp.txt --extended
```

The prepared `replay-all.sh` runs both passes over the captured directory. Each valid hard-row problem produces one result per native backend. Current
replay preserves the captured coefficients, objective weights and actuator box;
extended mode increases only the offline iteration budgets.

Interpretation:

- A positive minimum row residual over the actuator box proves that original
  row is impossible for every available torque. The report names the joint and
  upper/lower bound or table capsule endpoint. Zero such rows does not establish
  simultaneous feasibility of all inequalities.
- Feasible witnesses with native rejection, or disagreement between backends,
  identify a numerical/optimization route to investigate.
- A feasible current problem with a rejected historical owner result requires
  checking the archived runtime record separately from native feasibility.
- Model-based predicted acceleration can also be compared with the matching
  recorded state evolution. Hardware model agreement is a separate question
  from whether the captured QP was solved correctly.

## Finger geometry without CAD

A conservative envelope can be built from mounting-to-tip reach, lateral
offsets, width/thickness and the full opening range measured with a ruler or
calipers. The current table filter uses the configured `body_volumes` entries,
excluding the base and Link 1. For Hand geometry, use flange-frame points
(`frame: 7`) with the Hand-to-flange rotation applied. Keep the table plane at
its physical coordinate and clearance separate.
Mass/centre-of-mass/inertia settings must also describe the installed fingers
for torque-level model agreement. None of these measurements is a prerequisite
to replaying the current acceptance failure.

## Software verification

The frozen-source build passes 214 affected case-level regressions. Three paired
mock captures produce 108 replay results, each including candidate context.
Capture-enabled shadow profiling observed 3,152 callbacks, max 236.348 us, zero
>800 us, process faults or diagnostic drops. These checks verify the capture and
analysis path; the hardware acceptance failure remains unresolved.
