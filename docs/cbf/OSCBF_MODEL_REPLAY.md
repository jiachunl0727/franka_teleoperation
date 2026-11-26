# Exact OSCBF dynamics replay — 2026-11-06

**Implementation follow-up:** the validated model is now the default native
[runtime dynamics provider](../URDF_DYNAMICS.md). The report below describes
the preceding offline experiment, which itself made no production changes.

Executing the pinned OSCBF dynamics resolves the recorded mathematical
infeasibility in this experiment. With the recorded nominal source, all
**30,445 active samples** pass the current CBF and exact outgoing-torque
verification. All 32 complete native captures become feasible even when only
the mass matrix is replaced, retaining libfranka Coriolis terms.

This is offline evaluation against recorded states and torque history. No
production model provider, controller, CBF implementation or configuration was
changed, and no robot connection/control was performed.

## What differs between the models

The inspected libfranka 0.9.2 adapter calls the model library downloaded from
the robot. `Model::mass(state)` passes joint positions plus the configured total
load inertia, mass and flange-to-load-CoM translation to that library's `M_NE`
function. Runtime evaluation does not identify inertial parameters from the
current motion. The captured matrices do not expose all the library's internal
link parameters or preserve the load-configuration metadata needed to attribute
every difference to one physical parameter.

OSCBF's unmodified `load_panda()` loads its own URDF. Its mass function computes

`M = sum(m_i * Jv_i.T * Jv_i + Jw_i.T * R_i * I_i * R_i.T * Jw_i)`.

The result depends on link masses, CoM positions, rotational inertia tensors,
inertial-frame rotations, joint transforms/axes, and fixed Hand/finger bodies.
It is a dense configuration-dependent matrix, not a chosen diagonal joint
matrix. The URDF gives many link/fixed-body rotational inertias diagonal entries
of 0.1 kg m², including fixed zero-mass bodies. Its fixed attachment assumptions
and load distribution also differ from libfranka's configurable load input.

In this comparison, the captured native kinematics and reconstructed Panda
kinematics agree to below 5.6e-10 in matrix/Jacobian coefficients. Therefore
kinematic differences are excluded as a material cause in this replay. The
large wrist-inertia difference is directly present in the model outputs:
at cycle 22,810, libfranka M77 is 0.0029616103364 kg m² and OSCBF M77 is
0.6000200000000 kg m². Differences cannot be decomposed into only rotational
inertia versus mass/CoM contributions without the manufacturer's corresponding
link parameters.

Sources:

- [Pinned OSCBF mass calculation](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/manipulator.py#L767-L794)
- [Pinned Coriolis calculation](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/manipulator.py#L821-L839)
- [Pinned Panda URDF](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/assets/franka_panda/panda.urdf)
- Local `src/third_party/libfranka_0_9_2/src/model.cpp`,
  `model_library.cpp` and `include/franka/model.h`.

## Executed implementation and independent validation

Source commit: **6082329f8e1d61ee8e875a63a5cd80a14d513e12**.
The complete source archive was downloaded and extracted without modifying its
files. The actual `load_panda()`, `mass_matrix(q)`,
`centrifugal_coriolis_vector(q,dq)` and `gravity_vector(q)` functions were
executed with JAX/JAXlib 0.4.30 and float64 enabled. No added diagonal inertia
or substitute dynamics implementation was used.

PyBullet 3.2.7 independently evaluated the same URDF with file inertias enabled.
Across all 32 captured states, maximum absolute discrepancies were:

| Quantity | OSCBF versus PyBullet |
| --- | ---: |
| Mass-matrix entries | 1.78e-15 kg m² |
| Coriolis entries | 1.79e-14 Nm |
| Gravity entries | 1.07e-14 Nm |

All generated matrices are finite, symmetric and positive definite. The actual
OSCBF gravity-free prediction is `a = M_oscbf^-1 * (u_API - c_oscbf)`: their
total-torque coordinate is `Gamma = u_API + g_oscbf`, so their explicit gravity
term cancels in this conversion. No full gravity term is subtracted twice.

## Comparison on complete native captures

The original 32 model snapshots were paired with exact recorded reference
poses/twists and incoming actuator intervals. Geometry, native frame data,
nominal source, magnitude/slew limits, solver, slack policy and timing settings
were retained. Two substitutions were evaluated:

1. Actual OSCBF M, recorded libfranka c: isolates mass-matrix influence.
2. Actual OSCBF M and c: uses their consistent rigid-body dynamics functions.

The installed production CBF and inverse-dynamics controller performed the
native replays. Both the recorded impedance nominal and a recomputed
inverse-dynamics nominal were checked.

| CBF settings | Recorded M/c baseline, either nominal | OSCBF M only, either nominal | OSCBF M/c, either nominal |
| --- | ---: | ---: | ---: |
| Current source, alpha=10 | 15/32 (46.875%) | 32/32 (100%) | 32/32 (100%) |
| Archived trial, alpha=5 | 6/32 (18.75%) | 32/32 (100%) | 32/32 (100%) |

All acceptances pass verification after final actuator clipping. Independent LP
checks transform the original emitted inequalities to the new matrices and
independently reproduce all 128 configuration/model/state feasibility results.
All 256 outgoing-vector checks satisfy those independent rows and actuator
bounds. The maximum outgoing row residual is -1.80656, and maximum box violation
is zero. This mass-only result demonstrates that changing M alone is sufficient
to remove the mathematical infeasibility in these captures.

## Full active-stream replay

The original bag contains 46,418 tracking samples, of which 30,445 are active
Cartesian teleoperation. OSCBF M/c was evaluated at every active q/dq, making
a full dynamics replay possible without inventing missing libfranka matrices.

The pinned Panda kinematic implementation reconstructs every frame and Jacobian.
Against the 32 complete native models, maximum frame error is 5.57956e-10 and
maximum Jacobian error is 5.57958e-10. Against every full-stream measured pose,
maximum coefficient error is 5.58551e-10. Reconstructing the shared magnitude/
slew clipping reproduces every recorded issued torque: **zero mismatches**.

Every row is rebuilt from the original state and original incoming `tau_J_d`.
This is a synchronous frozen-state replay, including for the archived
asynchronous configuration; it does not reproduce worker scheduling or a
different controller's future physical trajectory.

| CBF settings / nominal source | Bounded nominal satisfies every row and actuator bound | Native accepted + outgoing verified |
| --- | ---: | ---: |
| Current / recorded impedance | 30,445/30,445 (100%) | 30,445/30,445 (100%) |
| Current / recomputed inverse dynamics | 30,445/30,445 (100%) | 30,444/30,445 (99.9967%) |
| Archived / recorded impedance | 30,445/30,445 (100%) | 30,444/30,445 (99.9967%) |
| Archived / recomputed inverse dynamics | 30,445/30,445 (100%) | 30,444/30,445 (99.9967%) |

There are no empty feasible sets in these full-stream problems: every bounded
nominal itself is a feasible witness. Every accepted command equals that bounded
nominal within numerical roundoff, so no additional CBF intervention is needed.

The three isolated authorization failures are distinct from infeasibility:

- Current inverse dynamics, cycle 38,436: model-age guard (`invalid_model`).
- Archived recorded impedance, cycle 22,267: filter time 45,989.534 us exceeded
  the unchanged 600 us budget.
- Archived inverse dynamics, cycle 36,394: model-age guard (`invalid_model`).

The model-age cases have finite nominal/model data and valid freshly built rows,
but the early elapsed-age guard in `solve_prepared()` returns before filling
`total_us`. They are not failed positive-definiteness or row-domain checks.
Each of the three exact data cases was replayed separately under the same
guards; both nominal sources passed in every isolated recheck. The original
full-run rejections remain included in the rates above. No deadline was relaxed
or CBF code changed. Batch replay timing is not hardware timing qualification;
in particular, JAX model generation was offline preprocessing.

## Predicted acceleration

The same recorded issued command at cycle 22,810 gives:

| Joint | Recorded libfranka M/c | Actual OSCBF M/c |
| --- | ---: | ---: |
| 1 | 23.6401 | 2.6400 |
| 2 | 3.8113 | 1.2569 |
| 3 | -16.2392 | -1.7685 |
| 4 | 19.3382 | 3.1378 |
| 5 | -195.4240 | -1.7632 |
| 6 | -66.2892 | -2.3004 |
| 7 | 429.5652 | 2.5279 |

Units: rad/s². With only M replaced, joint-7 prediction is 2.5243 rad/s²; changing
c as well has a small effect at this state. The current joint-7 velocity barrier
allows at most 24.5908 rad/s² here. Across the *entire same actuator box*, the
joint-7 acceleration range changes from [199.5263, 711.2203] with recorded
libfranka M/c to [1.0655, 3.9572] with OSCBF M/c. This directly explains why
the earlier impossible row becomes satisfiable.

Across all 30,445 active issued commands using exact OSCBF M/c:

| Joint | Median absolute prediction | 95th percentile absolute prediction | Maximum absolute prediction |
| --- | ---: | ---: | ---: |
| 1 | 0.7101 | 2.2248 | 3.4502 |
| 2 | 1.2026 | 2.7672 | 4.2730 |
| 3 | 0.7808 | 2.1591 | 5.1008 |
| 4 | 2.5294 | 4.8813 | 7.4387 |
| 5 | 1.0770 | 1.9831 | 4.0672 |
| 6 | 2.0819 | 3.9404 | 6.4795 |
| 7 | 1.2596 | 2.8219 | 5.8795 |

The figure in `log/commissioning/2026-11-06-oscbf-model/joint7-predictions.png`
compares available native-model captures with the full OSCBF prediction stream.
The panels use different vertical scales and show only the joint-7 velocity
barrier; full acceptance checks include every configured row.

## Interpretation and evidence

Changing the mass matrix removes the recorded infeasibility under our existing
limits and strict zero-slack policy. This result does not require changing the
nominal controller or adopting OSCBF's different input bounds/relaxation policy.
It establishes numerical feasibility under their model, not physical calibration
of that URDF. No production provider was activated.

All evidence and runnable analysis files are in
`log/commissioning/2026-11-06-oscbf-model/`:

- `generate_models.py`, pinned source archive, dependency versions and source
  hashes: actual unmodified model execution and independent engine checks.
- `model-summary.json`, `captured-predictions.json`, `oscbf-dynamics.npz`:
  matrices, prediction statistics and the example calculations.
- `oscbf_mass_only/`, `oscbf_mass_coriolis/`: converted model snapshots, exact
  references and native current/archived replay results.
- `native_replay.cpp`, `CMakeLists.txt`, `active-stream.bin`,
  `stream-layout.json`: full native replay with reconstructed frame data.
- `full-*-results.jsonl`, `full-*-validation.json`, `summary.json`: every active
  state, authorization result and geometry/clipping validation.
- `check_independent_rows.py`, `independent-verification.json`: independent LP
  and outgoing-torque verification.
- `guard-*-results.jsonl`: individual rechecks without discarding the full-run
  failures.
- `production-before-sha256.json`, `production-unchanged.json`: all 51 monitored
  production input files preserved, including all 47 CBF package files.
