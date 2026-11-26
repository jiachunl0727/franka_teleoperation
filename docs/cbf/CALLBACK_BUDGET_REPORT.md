# Callback budget and asynchronous QP proposals — 2026-11-01

**Current selection, 2026-11-05:** the operator has requested synchronous solving
for the current single-arm qpOASES/elastic configuration. Both modes remain
available, with all runtime deadlines retained. See the
[synchronous verification report](SYNCHRONOUS_EXECUTION.md). The broader matrix
and previous mode decision below are historical and do not establish timing for
the current selected constraint set.

The optimized synchronous implementation still exceeded the requested 800 µs
callback budget. As authorized, native QP work now runs asynchronously; the
independent owner callbacks retain fresh constraint construction, actuator limits
and final verification. Hardware CBF activation remains blocked, and bringup CBF
switches remain off by default. This report describes mock measurements, not a
worst-case execution-time proof or physical safety qualification.

## Frequency of the original overruns

The completed original diagnostic matrix retained 1,000 samples per arm with
self-collision, singularity, and (for two arms) inter-arm constraints enabled.
Its peer-age allowance was widened to 100 ms **for diagnosis only**, allowing
expensive cases to finish. The separate strict-age matrix contains faulted runs.
Neither establishes an acceptable deployed peer-age setting.

| Backend | Single hard >1 ms | Single elastic >1 ms | Dual hard >1 ms | Dual elastic >1 ms |
|---|---:|---:|---:|---:|
| OSQP | 0% | 7.5% | 0.15% | 12.4% |
| qpOASES | 0% | 0% | 0% | 0% |
| PIQP | 0.1% | 2.9% | 1.1% | 39.5% |

Dual percentages pool 2,000 individual arm callbacks, rather than paired cycles.
Raw evidence: [original complete diagnostic matrix](avoidance_mock_relaxed.csv)
and [original strict-age attempts](avoidance_mock_strict.csv).

A repeat of the slow dual PIQP/elastic case exceeded **800 µs in 1,483/2,000
retained callbacks (74.15%)**, with a maximum of 2,584.012 µs. Its mean was about
920 µs. See [800 µs baseline](callback_budget_baseline.csv).

## Causes and changes

1. **Unnecessary native solves.** The nominal torque is already the exact
   zero-cost optimum when it satisfies all inequalities and actuator bounds with
   zero slack. A checked analytical path now handles that case. Forcing native
   solving remains available for numerical audits.
2. **Unnecessary problem dimensions and geometry work.** Effective slack capacity
   now reflects enabled relaxable families. Self-pair exclusions are precomputed;
   peer geometry is validated once per snapshot. A conservative actuator-box
   bound skips expensive distant-pair work only when the row is provably
   redundant. No required collision rows are removed by a distance heuristic.
3. **Solver latency and scheduling tails.** Native solve time varies with backend,
   formulation and iterations. The analytical optimization reduced the dual mean
   to about 250 µs, but still produced **12/6,400 observed callbacks above 800 µs**,
   with a maximum of 1,361.292 µs. This triggered the authorized asynchronous
   redesign. See [synchronous optimized run](callback_budget_fastpath.csv).
4. **Wake-up latency outside the callback.** Normal-priority asynchronous runs
   stayed within the callback budget in the recorded samples, yet all six dual
   cases stopped on stale peer snapshots. Short callbacks do not guarantee timely
   wake-ups. See [normal-scheduling diagnostic](callback_budget_async_normal.csv).

Diagnostics now distinguish wall callback duration, thread CPU time, wake-up
lateness, observed callback period, constraint work, and worker latency. Diagnostic
queue storage is filled directly, avoiding a second full result copy.

## Execution contract

`execution.mode` selects `synchronous` or `asynchronous`; the development YAML
selects asynchronous. Only native QP assembly/solving moves to a low-priority
worker. It has no model, ROS, hardware, reference or actuator authority.

Each arm keeps its own control loop and bounded latest-only request/proposal
mailboxes. The callback rebuilds rows from current model and peer snapshots.
A proposal must match the command epoch, remain within the configured candidate
age, and pass every current original inequality and current actuator bounds.
The owner handles only numerical roundoff in its existing actuator clipping and
reverifies the exact outgoing command. An old optimal solution is never assumed
to remain optimal or feasible for the current state.

While a proposal is unavailable or fails fresh checks, the selected controller
can compute a new measured-pose hold torque with zero desired twist. The pose
anchor stays fixed during a pending hold; torque is recomputed from fresh state.
That torque must pass the same current rows and actuator box. It is not unchecked
replay, automatic retreat, or a claim that impedance holding guarantees braking.
If no verified command exists, state/peer validity fails, or the callback deadline
expires, the owner stops. No automatic restart is added. Asynchronous mode rejects
configurations allowing positive slack.

The worker never blocks the callback; native iteration limits and worker elapsed
limits reject failed proposals. Worker shutdown joins outside the callback.
Callback-side allocations are audited independently of worker allocations.

## Final verification

The final matrix passed **all 12 configurations** (three backends × two
formulations × single/dual). It retained **90,000 motion-enabled callbacks**
(5,000 per arm/configuration), with **128,385 observed callbacks** overall and
91,805 observed motion-enabled callbacks. There were **zero >800 µs callbacks,
zero >1 ms callbacks, zero process/constraint/watchdog faults, and zero diagnostic
queue drops**. The owner also enforced its 800 µs deadline, independently of the
observer. Maximum observed callback: **693.666 µs**, leaving 106.334 µs below the
requested budget in this run. Maximum single-arm callback: **522.812 µs**.

| Arms | Backend | QP | Active mean µs | Active p99 µs | All-observed max µs |
|---|---|---|---:|---:|---:|
| 1 | OSQP | hard | 76.61 | 164.98 | 414.998 |
| 1 | OSQP | elastic | 78.92 | 169.11 | 522.812 |
| 1 | qpOASES | hard | 72.30 | 145.23 | 300.189 |
| 1 | qpOASES | elastic | 70.92 | 134.86 | 327.281 |
| 1 | PIQP | hard | 71.70 | 147.91 | 287.034 |
| 1 | PIQP | elastic | 79.10 | 198.38 | 449.824 |
| 2 | OSQP | hard | 151.58 | 398.58 | 693.666 |
| 2 | OSQP | elastic | 147.83 | 357.31 | 651.356 |
| 2 | qpOASES | hard | 116.20 | 228.62 | 528.042 |
| 2 | qpOASES | elastic | 116.75 | 227.27 | 566.585 |
| 2 | PIQP | hard | 123.40 | 268.09 | 532.841 |
| 2 | PIQP | elastic | 126.86 | 296.41 | 490.260 |

Dual mean/p99 columns show the worse arm's statistic; maximum covers both arms
and all observed startup/active samples. Raw data:
[final active-only matrix](callback_budget_async_active.csv). Every retained
outgoing command passed current original-row/actuator verification; reported
maximum hard violation and slack were zero in every configuration. At most 28
rows were admitted to these QPs: geometry was evaluated, but distant collision
rows were redundant over the actuator box. The deliberately binding row was a
joint-position barrier. This is not a dense, simultaneously binding collision
stress case. Maximum active-window wake-up lateness was 113.497 µs and observed
period was 1,106.719 µs. Native worker latency reached 3,826.119 µs without making
the callback wait; stale/unsuitable proposals required fresh verified holding.

Verified holds accounted for **41.2–48.3%** of retained samples across the
configurations; the remainder were verified worker proposals. Thus this is a
timing/feasibility pass, **not evidence of uninterrupted tracking**. The deliberately
near-active joint barrier, newly arriving command epochs, worker delay and
fresh actuator-box checks can require holding. Physical smoothness and tracking
quality remain unqualified. The benchmark fails if a configuration obtains no
verified worker proposals, preventing an all-hold false pass.

Earlier exploratory data remains in [the initial short FIFO matrix](callback_budget_async_fifo.csv).
Its observer and publisher shared a process; its window selection is superseded
by the final independent-publisher, active-only benchmark.

Release builds passed for `cbf_safety_filter`, `teleop_controllers` and
`teleop_bringup`. The final sequential regression run passed **193 cases**:
70 numerical/filter, 82 controller/runtime, and 41 bringup, with no failures or
skips. Tests cover fresh-row/epoch/age rejection, worker-owned request storage,
verified-hold fallback and single-use authorization, owner callback deadline
faults, independent dual-arm startup/shutdown, disabled control and launch paths.

The glibc allocation audit recorded **zero calling-thread allocations over 100
asynchronous calls per backend**, and zero allocations over 100 collision/
singularity construction calls. Native qpOASES still allocated 1,298/2,140 times
over 20 hard/elastic solves; these native calls occur on the worker in async mode.
OSQP and PIQP recorded zero in that native steady-state probe. These finite audits
do not certify every owner/controller path as allocation-free.

Formatting and whitespace checks passed. No physical robot/Hand operation was
performed and no mock owners were left running.

## Reproduction and scope

Run in the existing ROS development container after a Release build:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
python3 /ws/src/teleop_controllers/test/benchmark_avoidance_mock.py \
  --arms 1 2 --backends osqp qpoases piqp --formulations hard elastic \
  --execution-mode asynchronous --active-constraint --realtime \
  --samples 5000 --peer-age-us 3000 --budget-us 800 --enforce-budget \
  --output /tmp/cbf_800_async_active.csv
```

The mock owner explicitly requests libfranka-equivalent FIFO priority after
creating non-RT threads; inability to obtain it is an error. The QP worker uses
normal scheduling and lowered priority. No host scheduler or container capability
settings are changed by the benchmark.

The command source runs in a separate spawned process so JSON collection cannot
starve the input watchdog. Retained timing distributions require actual
`motion_enabled` samples. All observed callback durations, including startup and
shutdown, are checked, and the owner enforces the 800 µs deadline independently
of diagnostic delivery. The benchmark records diagnostic queue drops. The
near-active joint barrier forces genuine QP intervention; candidate and holding
counts distinguish worker progress from an all-hold false pass.

Environment: AMD Ryzen 7 5800U, Linux 5.15.0-1032-realtime, ROS Humble,
Release C++17/GCC 11.4 in the existing `franka-teleop` container. Self/inter-arm
and full-task singularity rows use the Franka-derived 43-sphere arm/Hand cover.
The two known WORLD bases are separated by 1.2 m. Peer age/skew remain 3 ms;
proposal age is 3 ms. Profiling uses 256 row capacity, 14 soft-variable capacity,
OSQP 1,000 iterations/1e-8 tolerances, PIQP 100 iterations/1e-8, qpOASES 100
working-set recalculations, and strict rejection of positive slack. These are
explicit profiling settings, not a claim that every shipped solver setting has
identical tracking quality. Filter/model diagnostic deadlines are widened to
1 s in the profiling harness to isolate callback timing; the owner callback
still enforces 800 µs, and peer/candidate freshness limits remain 3 ms. The shipped
600 µs filter and 2 ms model-age defaults are not independently qualified by
this profiling override.

The fixture is a fixed-state mock, not a closed-loop dynamics simulator. It
exercises active inputs and real numerical/runtime paths but cannot establish
collision prevention, braking, stability, tracking quality, or physical dynamics
agreement. Measured callback timing includes the mock owner path through its
final diagnostic snapshot; physical model acquisition, FCI networking and tiny
return operations still require end-to-end hardware measurement. Finite tests on
an RT kernel do not prove an absolute worst-case bound. Worker age rejection can
increase holding frequency and reduce tracking continuity even when timing passes.

Before physical release, retain model/geometry agreement, stopping-response and
physical timing gates. Follow the [short commissioning procedure](HARDWARE_COMMISSIONING.md)
after those gates are reviewed; this implementation does not remove the hardware
activation block or resolve the historical wrist/model discrepancy.
