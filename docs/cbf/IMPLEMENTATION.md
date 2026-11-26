# CBF software implementation, 2026-11-01

**2026-11-05 synchronous selection:** the operator requests synchronous execution
with existing features retained. Both execution modes are supported for explicitly
confirmed single-arm commissioning; the package YAML now selects synchronous.
Current measurements use the selected qpOASES/elastic 52-row setup, not the
historical larger collision/singularity matrix. Filter/callback deadlines and
all constraints remain enforced. See [synchronous report](SYNCHRONOUS_EXECUTION.md).

**2026-11-05 commissioning follow-up:** explicit single-arm shadow/filtered opt-in
is now available; ordinary/default and dual-arm hardware activation remain
blocked. See [procedure](HARDWARE_COMMISSIONING.md). This is software preparation,
not physical qualification; no hardware was operated.

**2026-11-01 timing follow-up:** the authorized asynchronous native-QP proposal path
with fresh callback verification supersedes the synchronous-only timing status
below. See [callback budget report](CALLBACK_BUDGET_REPORT.md). Hardware activation
remains blocked. Earlier milestone counts and timing results below are historical.

Branch: `feature/cbf-safety-layer-20261101`. The supplied roadmap is copied to
[IMPLEMENTATION_ROADMAP.md](IMPLEMENTATION_ROADMAP.md) as the implementation
specification. It does not authorize physical operation. Existing libfranka
working-tree changes are preserved.

Change sequence:

1. Establish controller/bringup baseline. Create/export `cbf_safety_filter` and
   pin native OSQP, qpOASES and PIQP sources with license notices.
2. Validate immutable package configuration and global/per-arm activation;
   resolve paths before launching any owner. Preserve disabled callbacks.
3. Implement analytical point acceleration, joint/table/obstacle barriers and
   domain checks with independent finite-difference references.
4. Assemble common hard/elastic problems, implement all three native adapters,
   independently validate original/augmented rows and explicit slack budgets.
5. Integrate synchronously for mock/offline operation, queue diagnostics,
   exercise faults, record allocation/timing and cross-solver benchmarks.

Hardware CBF activation is blocked in both bringup and direct runtime pending
release review. Ordinary disabled hardware control is unchanged. Self/inter-arm collision and singularity now have default-disabled offline/mock implementations; see [extension contract](COLLISION_AND_SINGULARITY.md). Failure never
reuses stale torque or asserts that unverified damping is CBF safe.

The local libfranka header `franka/control_types.h` explicitly defines torque
commands without gravity and friction. Numerical dynamics use
`M*qdd + coriolis = tau_cmd`; desired `tau_J_d` anchors actuator slew bounds.
This convention does not resolve the historical measured wrist/model mismatch.

External API/version references: [OSQP 0.6.3 C API](https://osqp.org/docs/release-0.6.3/interfaces/C.html),
[qpOASES source/license](https://github.com/coin-or/qpOASES/tree/51d3fbea30142d3acbf40cf7a1c519efc27ea67b),
[PIQP 0.6.3](https://github.com/PREDICT-EPFL/piqp/tree/v0.6.3).
qpOASES is built as an unmodified shared library with its LGPL-2.1 notices and
exact source provenance; OSQP is Apache-2.0, PIQP BSD-2-Clause. Redistribution
must retain corresponding notices and LGPL replacement/source obligations.
PIQP's native solver internally initializes iterates on each solve; its reusable
workspace is not a promise of warm-started iterates.

M0 baseline: the controller/bringup build and 104 existing test cases passed.
M1–M2: all three packages built; 109 tests passed after a symlink-install URI
resolver correction. Logs: `log/diagnostics/cbf_20261101/`. Nominal law sources
were untouched; disabled callback clipping stayed identical.

M3 inspection found a pre-existing **mock fixture only** joint-5 translation
`[-.0825,0,.384]` differing from the pinned URDF `[-.0825,.384,0]`. Corrected the
fixture; physical feedback still uses libfranka and no nominal law is changed.
This does not diagnose or resolve the hardware wrist/model disagreement.

M3–M5: the analytical kinematics/row suite and all 30 parameterized backend/QP
tests plus independent tamper checks pass. PIQP initially stalled when an
unbounded slack used a large **finite** ceiling; the common representation now
uses the native infinity sentinel, verified on all three adapters. No inequalities
were removed or tolerances relaxed to accept unverified candidates.

M6: all six enabled mock backend/formulation cases and three fault-exit cases
pass. Disabled controller and single/dual bringup regressions pass. A zero-L1
fixture exposed PIQP's small positive slack at convergence: the strict policy
correctly rejected it. Zero-slack integration fixtures now use a positive L1
penalty and tight solver tolerances; separate tests retain zero-L1 positive-slack
behavior and verify rejection/degraded acceptance independently.

M7/M8 evidence is in [VERIFICATION.md](VERIFICATION.md), including all six solver
benchmarks, raw CSV artifacts, allocations, numerical errors and release blockers.
The software passes its regression contract; the 1 kHz/hardware release gate does
not pass. No asynchronous redesign or physical activation was attempted.

Final software checks: 170 cases pass (53 library, 76 controller, 41 bringup).
Startup selection descriptors are read-only; an actual ROS parameter-service
test confirms live activation/file edits are rejected. `cbf_validate` now also
preflights the selected native workspace before creating launch owner actions.
The last numerical rebuild/test and startup-preflight checks pass. Formatting,
Python/shell syntax and whitespace checks pass. Test controller processes are
stopped. Nominal controller source and Hand source were not modified.

## Collision/singularity extension

The user confirmed hard self/inter-arm constraints, full-task singularity avoidance, arm/Hand geometry (no cable/payload), independent arm owners and bounded peer motion, and connected boundary intervention preserving feasible tangential/retreat commands. Implementation and validation details: [contract](COLLISION_AND_SINGULARITY.md), [report](AVOIDANCE_VERIFICATION.md). Physical activation remains blocked.
