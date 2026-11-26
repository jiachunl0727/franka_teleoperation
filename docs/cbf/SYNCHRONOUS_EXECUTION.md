# Synchronous execution and acceleration-model interpretation — 2026-11-05

The subsequent [actuation-model audit](ACTUATION_MODEL_AUDIT.md) identifies
omitted apparent motor/inner-loop dynamics as a leading hypothesis. An offline
sensitivity improves predictions and feasibility, but physical response remains
unvalidated and production dynamics are unchanged.

The operator requested synchronous execution with the other features retained.
`src/cbf_safety_filter/config/cbf.yaml` now selects
`execution.mode: synchronous`. The optional asynchronous path remains available.
Single-arm hardware commissioning permits either validated execution mode;
explicit commissioning confirmation, default-disabled activation, diagnostics,
the 600 us filter deadline, the 800 us callback deadline and the dual-arm hardware
block remain. All joint/table rows, gains, slack policy, actuator limits,
reference processing, nominal laws and Hand/homing settings are retained.

## Meaning of the change

Synchronous execution rebuilds the current rows and solves the current QP inside
the owner callback, then independently verifies the outgoing command. It avoids
worker candidate age/epoch and previous-cycle geometry/actuator-interval changes.
It does not make an infeasible current QP feasible. Solver/model/domain/deadline
failures retain their existing protective handling; shadow still issues the
exact nominal actuator-clipped torque and observes the filter result.

The 150.5 us maximum callback in the preceding hardware trial excluded worker
solver execution. Worker time therefore could not simply be ignored when
choosing synchronous mode. Current measurements cover the selected qpOASES/elastic
52-row configuration. They do not qualify every backend or the larger historical
self-collision/singularity/inter-arm workloads. Increasing enabled families
requires measuring the resulting whole callback again.

The regression also exposed an execution-mode diagnostic error: preparation
failures in async mode were labelled synchronous because the result flag was
only assigned after successful preparation. The result now records its selected
mode immediately; the shadow integration test covers both modes and invalid
domains without changing the nominal command.

## Why acceleration predictions are large

The filter's instantaneous rigid-body model is

`a(tau) = M(q)^-1 * (tau - coriolis(q,dq))`.

It uses the full coupled matrix. This is a candidate-command calculation, not
an acceleration sensor or a long-horizon trajectory simulation.

At recorded cycle 22,810, the joint-7 row of inverse mass is approximately

`[9.999, 1.695, -5.122, 11.068, -147.539, -42.563, 513.005]`.

Multiplying the seven incoming desired torques minus Coriolis by that row gives
these contributions to joint-7 acceleration, in rad/s²:

| Torque from joint | Contribution |
| --- | ---: |
| 1 | 6.19 |
| 2 | 0.62 |
| 3 | -1.42 |
| 4 | 9.98 |
| 5 | 97.93 |
| 6 | 12.47 |
| 7 | 329.60 |
| Total | **455.37** |

The modeled sensitivity at joint 7 corresponds to an effective inertia
`1 / 513.005 = 0.001949 kg m²` when the other torque inputs are held fixed.
A small modeled inertia produces a large modeled acceleration from a modest
torque. Coupling adds to it. Positive-definite mass matrices and large inverse
mass sensitivity are compatible; this is not evidence of a kinematic
singularity or an arithmetic divide-by-zero.

This explains the arithmetic, not why the real motion disagrees. Over the same
surrounding 20 ms, the model predicts an average 455.88 rad/s² using the recorded
desired-torque trajectory, while recorded velocity changes imply 0.4065 rad/s².
Mass/Coriolis are frozen over that short comparison window. The discrepancy
requires investigation of effective actuator/elastic-joint response, residual
friction/external loading and configured Hand/finger load properties. No one
physical cause is established. The missing printed-finger load is a possible
contributor, not a demonstrated explanation for the full discrepancy.

The hardware distinguishes desired torque without gravity from measured
link-side torque and also provides motor/link state separately; desired torque
is not an acceleration measurement. See official
[RobotState definitions](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1RobotState.html).

## Why the filter needs an acceleration response model

The numerical question is: **what acceleration would each candidate torque
produce?** A velocity slope measured before issuing the next command describes
the response to previous inputs. It cannot answer that question by itself.

For velocity safety, `hdot + alpha*h >= 0` contains acceleration. For position
safety, the second-order barrier likewise contains acceleration. Substituting
the candidate-dependent model makes those inequalities constrain torque.
Replacing that model with one fixed measured acceleration would remove the
candidate torque from these rows: every torque would pass or fail together.
It would no longer select a corrective torque based on its predicted effect.
For the underlying construction, see the primary
[high-order CBF paper](https://calinbelta.com/wp-content/uploads/2023/12/High-Order_Control_Barrier_Functions.pdf).

Measured acceleration remains valuable for validation and for constructing a
proper observer/actuator-response model with explicit uncertainty and delay
bounds. A fixed arbitrary correction to the acceleration, clipping the prediction
to the barrier limit, or replacing it with the past slope would not establish
the required command-to-response relationship.

The current mismatch is a reason to correct or replace the model, not evidence
that the physical acceleration reached hundreds of rad/s². Synchronous mode
addresses the separate candidate handoff issue; it does not repair this mismatch.

## Verification evidence

All **222** affected case-level regressions pass, with no failures/errors/skips.
The recorded-state benchmark performs 640 current-cycle solves (20 rounds of
32 paired snapshots), max filter **366.075 us**. Every one of the six feasible
cases passes outgoing verification in every round; the 26 infeasible problems
remain rejected. Those snapshots were selected from failures and do not estimate
the acceptance rate of a synchronous hardware trial.

Four full mock-owner profiles cover shadow/filtered output and analytical/forced
native solving with the exact selected 52-row configuration. Across **13,408**
callbacks, max callback is **269.902 us**, max filter 260.243 us, with zero
callback/filter deadline violations, process faults or diagnostic drops.

Evidence is retained in `log/commissioning/2026-11-05-synchronous/`: configuration
preflight red/green tests, affected-package regressions, the recorded-model
benchmark, full mock-owner profiles and `acceleration-decomposition.json`.
These are offline/mock observations, not a worst-case timing proof or physical
qualification. The preceding hardware failure review remains authoritative for
the 26 infeasible / six feasible captured current problems.

The old forensic launch retains its frozen async configuration. A separate
`log/commissioning/2026-11-05-synchronous/launch-shadow.sh` and configuration copy
select synchronous shadow without altering earlier trial evidence. The operator
has not run that hardware launch as part of this software preparation.
