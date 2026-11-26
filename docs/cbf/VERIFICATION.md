# Software verification and timing, 2026-11-01

For the subsequent collision/singularity implementation and its current measurements, see [AVOIDANCE_VERIFICATION.md](AVOIDANCE_VERIFICATION.md). The baseline timings below predate that extension.

Environment: AMD Ryzen 7 5800U; Linux `5.15.0-1032-realtime`; Ubuntu 22.04 container; GCC 11.4.0; CMake 3.22.1; C++17 Release (`-O3 -DNDEBUG`), pinned dependencies from `third_party.repos`. ROS tests use domain 137 and localhost discovery. CPU affinity/frequency/network and real FCI timing were not commissioned.

Build and regression commands actually run (in `franka-teleop`, `/ws`, sourced ROS Humble and workspace):

```bash
colcon build --base-paths src --packages-select cbf_safety_filter teleop_controllers teleop_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 1
colcon test --base-paths src --executor sequential --packages-select cbf_safety_filter teleop_controllers teleop_bringup --event-handlers console_direct+ --return-code-on-test-failure
build/cbf_safety_filter/benchmark_safety_filter 30 > docs/cbf/benchmark_results.csv
python3 src/teleop_controllers/test/benchmark_cbf_mock.py --samples 1000 --output docs/cbf/mock_callback_results.csv
```

**170 baseline test cases passed before the collision/singularity extension:** 53 library, 76 controller and 41 bringup; no failures or skips.

The current numerical suite includes all six backend/formulation combinations, weighted coupled analytic optima, hard infeasibility, positive/zero slack, penalties, per-row budgets, max-iteration rejection, changing active/slack rows, independent tamper/roundoff checks, invalid models/domains, table/sphere finite differences and incident replays. Controller/bringup tests cover enabled joint+table mock owners, disabled compatibility, single/dual routing, immutable startup parameters, path/config rejection, recording selection and fault exits. Raw command logs are in `log/diagnostics/cbf_20261101/`.

Maximum point-Jacobian central-difference error: **2.003e-10**. Maximum analytical `Jdot*dq` central-difference error: **2.029e-10**, across 15 postures and 9 frames, including stationary/singular cases and a rotated/translated base. Barrier acceleration residuals agree with independently differentiated heights/squared distances within **1e-5**. These are numerical model checks, not physical validation.

The archived incident replay exercises all three backends/both formulations and a locked-wrist constrained plant response using recorded coupled inertias. Four otherwise verified numerical candidates failed the outgoing-command check after actuator-roundoff clipping; the test confirms their remaining physical-row violation exceeds tolerance and requires rejection. No tolerances were loosened and no projection repair was added. The locked-wrist/model disagreement remains unresolved.

## Allocation audit

Test-only glibc interposition counts allocation calls from the library and native shared libraries, after startup, during 20 alternating active-set calls per formulation. The positive-control allocator call is detected. This is a limited numerical-kernel audit, not a complete callback/fault-path proof.

| Backend | Hard allocations /20 calls | Elastic allocations /20 calls | Qualification |
| --- | ---: | ---: | --- |
| OSQP | 0 | 0 | No allocations observed in these cases; hardware unqualified |
| qpOASES | 1298 | 2140 | **Offline/mock-only**: hotstart allocates |
| PIQP | 0 | 0 | No allocations observed in these cases; hardware unqualified |

Owner exception/failure paths, solver rare paths and full libfranka callback execution are not certified allocation/blocking/deadline free. Hardware activation remains blocked for every backend.

## Identical synthetic QP scaling workloads

[Raw measurements](benchmark_results.csv): 60 configurations ×30 changing-nominal solves; 10/30/100/200/400 original CBF-like dense inequalities; hard dimension 7; elastic dimensions `7 + soft_capacity` with 0/25/50% soft capacity (up to 207 variables). Every backend receives identical deterministic normalized rows, bounds, weights and penalties. These synthetic rows exercise numerical scaling; they are not 400 commissioned robot collision barriers. Positive-slack acceptance is explicitly enabled only for this offline benchmark.

OSQP cap 5000 iterations, qpOASES cap 1000 per-cycle work-set recalculations (separate bounded startup bootstrap), PIQP cap 100 iterations. Independent tolerances and every stage mean/p50/p95/p99/max, initialization time, residuals, slack and status counts are retained in CSV. Different native iteration units are not equivalent.

| Backend | QP | Solves | Rejected | Worst total (ms) |
| --- | --- | ---: | ---: | ---: |
| osqp | hard | 150 | 59 | 24.572 |
| osqp | elastic | 450 | 149 | 811.499 |
| qpoases | hard | 150 | 0 | 0.228 |
| qpoases | elastic | 450 | 0 | 10.915 |
| piqp | hard | 150 | 0 | 0.518 |
| piqp | elastic | 450 | 0 | 24.441 |

OSQP hard stress cases reject 59 iteration-limited candidates. Elastic stress cases reject 104 iteration-limited and 45 numerical-status candidates. They are never reported as verified commands. qpOASES and PIQP solve these fixtures but their larger elastic cases still exceed 1 ms. **The synchronous stress timing gate fails.** No asynchronous reuse or alternate solver substitution was introduced. Engineering review is required to choose validated problem capacities/settings/execution and a compatible fault response. Options for review include measured row scaling/solver tuning and bounded soft capacities; reducing slack dimension requires an independently validated equivalent formulation. Dropping required rows or replaying stale torque is not an acceptable timing fix.

## Mock owner profile

[Raw measurements](mock_callback_results.csv): 1000 accepted samples per backend/formulation (6000 total), with 52 joint/table rows, 14 soft slots for elastic QPs, small synthetic Cartesian translation/rotation commands, the actual manager/nominal law/owner/filter/handoff path and off-thread ROS diagnostics. The mock plant has fixed joints, identity inertia and a test table at −1 m. Tests deliberately use relaxed offline deadline/model-age settings to measure software semantics.

| Backend | QP | Owner mean (µs) | p99 (µs) | Observed max (µs) |
| --- | --- | ---: | ---: | ---: |
| osqp | hard | 87.59 | 157.49 | 206.62 |
| osqp | elastic | 197.62 | 498.56 | 781.59 |
| qpoases | hard | 45.66 | 93.67 | 135.25 |
| qpoases | elastic | 67.06 | 125.50 | 267.10 |
| piqp | hard | 143.10 | 273.91 | 331.71 |
| piqp | elastic | 269.50 | 520.37 | 720.43 |

The owner interval begins at its model-state timestamp and ends near the final diagnostic handoff. It includes mock manager/nominal/CBF/queue work, but **excludes actual libfranka model acquisition, FCI networking and a few final return/copy operations**. It is not a measured complete physical libfranka callback. The retained libfranka model adapter still makes its original model calls; no unprofiled optimization was made. Small mock timings do not override the stress failures or establish a hard deadline.

## Release status

- M0–M6: software package, strict configuration, analytical rows, all three native backends/both formulations and default-disabled mock integration delivered.
- M7: scaling/allocation/mock profiling recorded; **real-time release gate not passed**. Physical full-callback profiling remains unperformed.
- M8: software fault injection and historical/locked-wrist model regressions delivered; physical discrepancy and validated backup/stopping region remain unresolved.
- M9: static sphere pairs plus default-disabled self-collision sphere covers implemented; see [extension verification](AVOIDANCE_VERIFICATION.md).
- M10: independent inter-arm filtering now has explicit peer motion bounds, freshness checks and modeled robust rows; see the extension verification. A coupled 14-variable solver is not implemented; physical inter-arm safety is unqualified.
- **No physical FCI or Hand operation, hardware CBF activation, deployment, merge or hardware sign-off was performed.**
