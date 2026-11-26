# Collision/singularity verification report — 2026-11-01

**Historical synchronous baseline.** The subsequent 800 µs optimization and asynchronous implementation are covered in [CALLBACK_BUDGET_REPORT.md](CALLBACK_BUDGET_REPORT.md).

**Implemented for offline/mock use; hardware real-time qualification fails.** New features remain disabled by default and physical CBF activation remains blocked. See the [implementation contract](COLLISION_AND_SINGULARITY.md) and [short hardware procedure](HARDWARE_COMMISSIONING.md).

## Changes and verification

- Hard self/inter-arm distance barriers and full-task singularity barrier, independent seven-joint owner loops, bounded peer-motion exchange and freshness rejection.
- Conservative 43-sphere cover derived from the pinned Franka description, including fixed base, Hand housing and full finger sweep. Explicit source-derived link-pair exclusions; no payload/cable geometry.
- Connected reference intervention preserves feasible sliding and retreat. There is no automatic retreat/restart or unchecked holding torque. Genuine state/feasibility/deadline failure retains stopping.
- New geometric rows are normalized only inside solver matrices. Original-unit rows independently verify outgoing torque. Exact actuator-box redundancy removes provably redundant rows; capacity overflow rejects.
- Analytic singularity gradients and directional Hessian terms agree with independent finite differences at 20 postures (gradient and second-derivative tolerance 1e-7). The metric is checked against the smallest singular value. Relative collision derivatives agree with independent FK differences. Active collision intervention is checked with all three solvers.
- 1000 deterministic adversarial peer perturbations check that the robust inter-arm inequality lower-bounds the actual distance-barrier residual, including position/velocity uncertainty and peer acceleration.
- Tests cover unsafe/singular starts, missing/stale/future peer snapshots, stronger peer acceleration bounds, immutable startup selection, single-use prepared rows, exact outgoing-command verification and connected tangential/retreat projection.
- A mock-discovered clock race was fixed: validation time is captured **after** acquiring a peer snapshot. Fresh peer data must not appear future merely because it arrived after the input-watchdog check. Stale peers now report `stale_peer` explicitly.
- Startup tests cover single-owner inter-arm enablement, asymmetric dual enablement and overlapping base geometry. Asymmetric rejection exposed a lifecycle hang; both owners now reject the incompatible setup, and a failed session cannot wait for a terminated owner to acknowledge completion.

**Final post-fix regression: 186 cases passed** (65 numerical library, 80 controller, 41 bringup), zero failures, errors or skips. No physical FCI/Hand operation or deployment was performed.

Geometry generation and independent source-mesh check:

```bash
python3 scripts/generate_cbf_collision_geometry.py
```

This verifies deterministic primitive transcription and **5724 finger mesh vertex/travel samples** against the generated cover. The enclosing-box construction covers finger triangle interiors as well. It validates the pinned source model, not physical installation dimensions or unmodeled attachments.

## Allocation evidence

The new collision snapshot, all-pair constraint assembly, analytic singularity calculation and reference projection stages observed **zero allocations over 100 calls** after construction. Existing native solver audits remain: OSQP 0/0, qpOASES 1298/2140, PIQP 0/0 allocation calls over 20 hard/elastic solves respectively. Thus qpOASES remains offline/mock-only despite its faster timings. These sampled audits do not certify rare native paths, ROS handoffs, exception handling or a complete physical callback.

## Mock timing setup

AMD Ryzen 7 5800U; Linux 5.15.0-1032-realtime; existing Ubuntu 22.04/ROS Humble container; GCC 11.4, C++17 Release. No CPU isolation, affinity/frequency commissioning or real-time thread-priority qualification was performed. Runs were sequential, without concurrent builds/tests.

Single-arm runs enable self collision and singularity. Dual-arm runs additionally enable inter-arm filtering on both owners, with WORLD bases separated by 1.2 m along Y. All 43×43 cross-arm pairs and covered-point motion bounds are evaluated. At this separated home fixture, new geometric rows are redundant over the torque interval and the QP retains 28 joint rows. These are **not active-contact or dense-active-set timing stress tests**; active rows are exercised separately in numerical tests.

The actual manager, nominal controller, filtering, outgoing verification and diagnostic handoff run in mock. Joints remain fixed, inertia is identity, and small sinusoidal Cartesian commands are sent. Logs confirm active teleoperation in completed runs. The fixture is not a dynamics simulator and cannot demonstrate physical stopping, tracking or edge sliding.

Each completed run retains 1000 samples per arm near its end, after command engagement. Percentiles are empirical. The measured interval starts at the state timestamp and ends near the final diagnostic handoff; it excludes real libfranka model acquisition/FCI/networking and final return copies. It also does not measure callback wakeup lateness. Result serialization runs off-thread but its CPU load remains present.

Comparable numerical settings: 256 inequality capacity; 14 reserved soft slots for elastic; OSQP cap1000, abs/rel1e-8; qpOASES cap100; PIQP cap100, abs/rel1e-8; velocity slack L1 penalty0.1; slack numerical tolerance1e-6. All other hard verification remains1e-6. These are explicit profiling settings, **not the saved default elastic capacity128/penalty100000**. Saved defaults are not qualified by these smaller-workspace measurements. Offline filter/model-age deadlines are widened to1 second to record overruns rather than suppressing their timings.

## Strict peer-age run

[Raw per-arm measurements and fault tails](avoidance_mock_strict.csv). Shipped peer age/skew rejection limits remain **3000 µs**. Ten of twelve configurations completed; dual qpOASES-elastic and PIQP-hard stopped with `stale_peer` before command engagement. Their partial samples are preserved and must not be treated as successful active trials. There were 17,039 retained accepted samples overall and 1222 measured intervals above1 ms. Maximum original hard-row violation was1.33e-8, below1e-6.

Values below take the worse per-arm p99/maximum for dual configurations; partial runs are marked.
| Arms | Backend | QP | p99 µs | Maximum µs | Result |
| ---: | --- | --- | ---: | ---: | --- |
| 1 | osqp | hard | 613.7 | 847.0 | Collected |
| 1 | osqp | elastic | 1473.1 | 1958.3 | Collected |
| 1 | qpoases | hard | 269.9 | 553.5 | Collected |
| 1 | qpoases | elastic | 369.3 | 524.6 | Collected |
| 1 | piqp | hard | 742.5 | 954.7 | Collected |
| 1 | piqp | elastic | 1225.8 | 1459.6 | Collected |
| 2 | osqp | hard | 850.6 | 1445.2 | Collected |
| 2 | osqp | elastic | 1855.2 | 2168.9 | Collected |
| 2 | qpoases | hard | 545.0 | 930.8 | Collected |
| 2 | qpoases | elastic | 644.6 | 810.6 | Partial: stale peer |
| 2 | piqp | hard | 1022.2 | 1231.2 | Partial: stale peer |
| 2 | piqp | elastic | 1508.2 | 1818.7 | Collected |

## Widened-age diagnostic run

[Raw measurements](avoidance_mock_relaxed.csv). A separate **offline-only** run sets peer age/skew rejection to100 ms so all configurations can be measured through command engagement. Every combination collected1000 samples per arm: **18,000 accepted samples**, no process faults,1168 measured intervals above1 ms. Maximum original hard-row violation was1.14e-8; maximum numerical slack was1.14e-8. Widening this gate is not an approved production setting or a timing fix. This run also increases geometric uncertainty bounds when snapshots actually age; it does not pretend old peers are current.

| Arms | Backend | QP | p99 µs | Maximum µs | Result |
| ---: | --- | --- | ---: | ---: | --- |
| 1 | osqp | hard | 530.6 | 815.1 | Collected |
| 1 | osqp | elastic | 1496.2 | 2124.9 | Collected |
| 1 | qpoases | hard | 304.6 | 471.4 | Collected |
| 1 | qpoases | elastic | 396.3 | 575.4 | Collected |
| 1 | piqp | hard | 689.9 | 1047.5 | Collected |
| 1 | piqp | elastic | 1186.7 | 1478.5 | Collected |
| 2 | osqp | hard | 801.9 | 1057.2 | Collected |
| 2 | osqp | elastic | 1881.1 | 2477.6 | Collected |
| 2 | qpoases | hard | 575.3 | 835.0 | Collected |
| 2 | qpoases | elastic | 678.3 | 988.4 | Collected |
| 2 | piqp | hard | 1023.1 | 1456.6 | Collected |
| 2 | piqp | elastic | 1331.3 | 1733.8 | Collected |

## Outcome and remaining hardware gates

**The 1 kHz release gate does not pass.** Some easy separated-home runs exceed1 ms; strict dual freshness can also fail. qpOASES has native allocations even where its observed maximum is below1 ms. Neither means nor short-run maxima establish a hard upper bound. No asynchronous solver/result reuse, silent constraint dropping, relaxed production freshness or automatic restart was introduced to conceal this result.

Before hardware activation: commission actual geometry/base transforms and exclusions; verify physical dynamics/actuation agreement including the historical wrist discrepancy; establish allocation/deadline behavior for a complete physical callback with scheduling margin and active constraints; review the stopping response and recoverable operating domain. Independent bounded-peer filtering can be overly conservative near another arm/base and can reject feasible coordinated configurations. This implementation does not provide a certified invariant backup controller or collision-free emergency stop.

Reproduce inside the existing container, with ROS and the workspace sourced, from `/ws`:

```bash
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
colcon build --base-paths src --packages-select cbf_safety_filter teleop_controllers teleop_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 1
colcon test --base-paths src --executor sequential --packages-select cbf_safety_filter teleop_controllers teleop_bringup --return-code-on-test-failure
python3 src/teleop_controllers/test/benchmark_avoidance_mock.py --samples 1000 --output /tmp/strict.csv
python3 src/teleop_controllers/test/benchmark_avoidance_mock.py --samples 1000 --peer-age-us 100000 --output /tmp/diagnostic.csv
```

The profiling script returns nonzero for incomplete/faulted runs and preserves their data. Inspect the CSV fault column and logs rather than reporting partial measurements as a pass. All profiling controller processes were stopped after collection.
