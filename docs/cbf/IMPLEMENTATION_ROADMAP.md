# C++ CBF Safety Layer — Agent Implementation Roadmap

**Target repository:** [`jiachunl0727/franka_teleoperation`](https://github.com/jiachunl0727/franka_teleoperation)  
**Architecture decision:** **B** — create a **separate `cbf_safety_filter` C++/ROS 2 `ament_cmake` package**, export it as a **C++ library**, and **link it into the existing `teleop_controllers` executable**. Do **not** create a separate torque-command ROS node or inter-process torque handoff.  
**Robot:** Franka Emika Panda, **7 DoF**, with the repository's **pinned libfranka/ROS dependencies and gravity-compensated command convention**. Do not silently substitute an FR3 model.  
**Default activation:** **disabled**.  
**Initial execution:** **synchronous** in the existing libfranka callback **for measurement and software validation only**; physical operation is forbidden until the explicit hardware release gates below are met.  
**Document type:** implementation specification and acceptance checklist for a coding agent.  
**Status:** proposed design; **not** implemented or certified.  
**Revised design requirements (2026-11-01):** Implement **all three configurable native solver backends in the first solver milestone**—**OSQP** (operator-splitting/ADMM), **qpOASES** (online active-set), and **PIQP** (proximal interior-point)—with a shared QP problem and result contract. Implement **hard and elastic/slack QP formulations from the outset**, with explicit per-constraint relaxability and independent reporting of every relaxation. No backend may be deferred to a later milestone merely because another backend succeeds. Version/license/API feasibility must be checked and recorded during M0–M1; substitute an equivalent maintained implementation **only with a documented technical/licensing justification**, preserving the three distinct algorithm families.

---

## 0. Agent operating contract — read before editing

1. **Inspect the current repository first.** These paths/observations were checked during planning, but the repository may have changed. Use the current branch as the source of truth; read `docs/CONTROLLER_ARCHITECTURE.md`, `docs/MOTION_INCIDENT_20261029.md`, and all files named in this roadmap before editing. Do not revive deleted historical CBF code wholesale.
2. **Work on a feature branch.** Keep changes reviewable by milestone and never overwrite unrelated user changes. Do not merge, deploy, connect to physical robots, run hardware launch commands, or enable the CBF in hardware configuration without explicit authorization.
3. **Deliver a complete vertical slice in stages.** At each stage, build and test before progressing. Report actual commands, results, known limitations, **results for all three solver implementations in both supported QP formulations**, and measured timings. Do not claim hardware validation from mocks or offline snapshots.
4. **Preserve the existing nominal controller.** With `cbf.enabled: false`, control selection, impedance gains, homing, hold, damping, input gates and post-controller actuator limiting must preserve current behavior. The installed CBF library alone must not add calculation/communication to the disabled callback path.
5. **Do not bypass safety mechanisms.** Never remove existing actuator, invalid-state or watchdog checks to make a QP feasible. The implementation **must support optional elastic/slack rows from the first QP milestone**, but actuator torque and slew limits, model validity, and other explicitly mandatory rows **remain hard**. A solver success with positive CBF slack is a **relaxed/degraded solution, not a verified hard-CBF-safe torque**. A failed/infeasible QP is **not** a valid torque command. The current braking/holding controller is not automatically a certified CBF-safe fallback.
6. **Keep the hard real-time callback free of** ROS communication, file I/O, logging, YAML parsing, dynamic workspace construction, allocations, blocking locks, heap-dependent operations and uncontrolled solver iteration counts. Allocate and initialize **all three backend workspaces for configured maximum torque/slack/constraint dimensions** at startup, or construct only the selected backend at startup while proving every backend can be selected and initialized there. If a chosen third-party solver cannot meet the no-allocation/no-blocking callback contract, **retain it for offline benchmarks and mark it non-RT-qualified**; never silently deploy it in hardware.
7. **Keep mathematical conventions explicit.** Include the commanded-torque/gravity convention, Jacobian reference frame, local vs world frame, collision geometry offsets, model time stamp, and sign convention `G * tau <= h` in documentation and tests.
8. **Prefer engineering evidence over optimism.** Do not assume 1 kHz just because the previous nominal controller ran at 1 kHz or because a QP benchmark averages <1 ms. Measure **the complete libfranka callback** on the intended hardware under representative and adversarial states.
9. **Limit initial scope.** Joint/actuator constraints and table clearance first; environmental collision spheres next. Self-collision and inter-arm collision are explicitly **later milestones**. No unverified collision constraint may be marked operational.
10. **Make configuration deterministic.** Resolve and validate all config files before opening a hardware owner. Every safety-affecting configuration change requires a deliberate startup/revalidation; do not add live on-the-fly safety gain changes in the first version.

### Definition of "done"

This roadmap is *not* complete when a library builds. It is complete only when the **default-disabled integration** builds and passes existing tests, the selected constraints, **all three solver backends and both hard/elastic QP paths** pass independent offline tests, configuration routing is tested for single/dual arms, the full callback is profiled, and fail/overrun behavior is explicit. A separately documented **hardware release review** is required before any real-robot CBF activation.

---

## 1. Verified repository starting point

Examine these files in the current checkout:

| File | Existing responsibility / planned change |
|---|---|
| `src/teleop_controllers/src/controller_runtime.cpp` | Owns one libfranka callback per arm; `manager.update(...)` returns nominal torque; `actuator_bounds(...)` and final clamping currently occur in the runtime. **Main CBF integration location**. |
| `src/teleop_controllers/src/controller_manager.cpp` | Chooses nominal law (Cartesian, orientation-only, hold, damping, homing, protective). **Do not embed CBF mathematics in the manager**. |
| `src/teleop_controllers/include/teleop_controllers/robot_model.hpp` | `RobotModelState`: `q`, `dq`, `coriolis`, `mass`, previous desired torque, **9** frame poses and **9** frame Jacobians. |
| `src/teleop_controllers/include/teleop_controllers/franka_model.hpp` | Calls `franka::Model` for mass, Coriolis, gravity, frame poses and Jacobians. May already duplicate an end-effector Jacobian calculation: **profile before optimizing**. |
| `src/teleop_controllers/src/cartesian_impedance_controller.cpp` | Existing nominal Cartesian controller using `J.transpose() * wrench + coriolis`, with optional null-space calculation. Do not change its control law in the initial CBF work. |
| `src/teleop_controllers/include/teleop_controllers/actuator_limits.hpp` | Produces intersection of torque magnitude and slew-rate bounds. The **same bounds must be QP constraints**, not only a post-QP clamp. |
| `src/teleop_controllers/CMakeLists.txt`, `package.xml` | Link dependency on `cbf_safety_filter`; no CBF ROS node. |
| `src/teleop_bringup/teleop_bringup/runtime_configuration.py` | Strict `ALLOWED`, `UNIFIED_KEYS`, `feature_document`, global/per-arm validation. Extend for `cbf`. |
| `src/teleop_bringup/launch/cartesian_impedance_controller.launch.py` | Merges global `controller` and `arm_settings.<arm>.controller`; applies `left.` / `right.` parameter prefixes in dual mode. Extend for CBF parameters. |
| `src/teleop_bringup/launch/teleoperation.launch.py` | Unified launch; only accepts `runtime_config` and `operator_confirm`. **Do not add a new top-level launch CLI flag**; activation belongs in YAML. |
| `src/teleop_bringup/config/single_panda.yaml`, `dual_panda.yaml` | Extend with optional `cbf` section, default disabled; support per-arm overrides. |
| `src/teleop_controllers/test/test_package_independence.py` | Test currently asserts the old `safety_filter` package is absent and checks a `"safety_filter"` substring in CMake. **Update narrowly** for the new intentional `cbf_safety_filter` dependency. |
| `src/teleop_bringup/test/test_runtime_configuration.py` | Some tests assert *exact YAML section and key sets*. Update expected schemas/fixtures; retain strict unknown-key rejection. |
| `docs/figures/cbf_geometry/README.md` | Historical 14-capsule Panda geometry reference. Revalidate against the actual tool/robot, frame transforms and clearances. |
| `docs/incidents/2026-10-29/previous_cbf_panda.md` | **Historical removed implementation**, not active design. Useful for lessons about old asynchronous worker, local guard and failure cases. |
| `docs/MOTION_INCIDENT_20261029.md` | **Critical:** historical abrupt motion/stalled wrist model mismatch; physical safety and stopping have not been demonstrated. Hardware activation is prohibited until the discrepancy is investigated. |

**Current architecture reminder:** The upstream libfranka callback computes a nominal torque and locally clips torque magnitude/rate. The prior CBF code was removed. The current mock backend is **not** a dynamic robot simulation.

---

## 2. Architectural specification and dependency rule

```
  ROS2 Quest inputs (existing asynchronous handling)
                    │
                    ▼
       teleop_controllers executable
                    │
       libfranka real-time callback
                    │
      Franka RobotState + franka::Model
                    │
       one coherent model snapshot
                    │
           ControllerManager
                    │
         nominal torque (7x1)
                    │
       actuator magnitude/rate bounds
                    │
                cbf enabled?
              /             \
            no              yes
            │                │
     original clip       cbf_safety_filter library
            │           kinematics -> Lie derivatives
            │           constraints -> configurable OSQP / active-set / IPM QP
            │           independent result checks
            │                │
            └──────┬─────────┘
                   ▼
         final independent validation
                   ▼
          franka::Torques return

  Diagnostics -> existing bounded queue -> non-RT publisher
```

- The new library is a separate **ROS 2 package** for build/distribution purposes, but its numerical code is **ROS-independent** and shares the **same executable and callback** with the existing controller.
- Dependency direction: `teleop_controllers` -> `cbf_safety_filter` -> Eigen / numerical solver / startup parser. **No dependency from CBF back to `teleop_controllers`**, and no dependency on `franka::Model` or ROS in numerical APIs.
- The library receives validated read-only robot data (or safe read-only references/maps with clear lifetime) and the nominal torque. It returns a numerical candidate **plus an explicit status**, never authority to modify controller modes or directly command hardware.
- `teleop_controllers` retains **final authority** over startup authorization, mode transitions, actuator bounds, state validity, protective handling and hardware commands.
- The first design is synchronous, *not a claim that synchronous mode is deadline-safe*. If it fails a timing gate, stop and propose an explicitly reviewed alternative; do not silently introduce asynchronous torque reuse.

---

## 3. Target package tree

```
src/
├── cbf_safety_filter/                         # NEW independent ament_cmake package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   ├── cbf.yaml                           # package default; NOT safety-certified
│   │   └── geometry_panda_umi.yaml                # validated robot-local collision geometry
│   ├── include/cbf_safety_filter/
│   │   ├── types.hpp                         # Eigen fixed-size types / bounded buffers
│   │   ├── configuration.hpp                 # typed, validated immutable settings
│   │   ├── model_view.hpp                    # no franka, ROS or controller headers
│   │   ├── geometry.hpp                      # primitives, attachment frame IDs
│   │   ├── kinematics.hpp                    # collision point data
│   │   ├── lie_derivatives.hpp               # Jdot*qdot / acceleration bias
│   │   ├── constraints.hpp                   # G*tau<=h and domain diagnostics
│   │   ├── qp_problem.hpp                    # hard/soft row metadata, slack indexing
│   │   ├── qp_solver.hpp                     # backend-independent solve interface
│   │   ├── solver_factory.hpp                 # startup-only backend selection
│   │   ├── safety_result.hpp                 # explicit status and validation metrics
│   │   └── safety_filter.hpp                 # startup init + RT evaluation
│   ├── src/
│   │   ├── configuration.cpp
│   │   ├── geometry.cpp
│   │   ├── kinematics.cpp
│   │   ├── lie_derivatives.cpp
│   │   ├── constraints.cpp
│   │   ├── qp_problem.cpp                   # canonical hard/elastic QP assembly
│   │   ├── solver_factory.cpp
│   │   ├── solvers/
│   │   │   ├── osqp_solver.cpp               # operator splitting / ADMM
│   │   │   ├── qpoases_solver.cpp            # online active-set
│   │   │   └── piqp_solver.cpp               # proximal interior-point
│   │   └── safety_filter.cpp
│   ├── test/
│   │   ├── test_configuration.cpp
│   │   ├── test_kinematics.cpp
│   │   ├── test_lie_derivatives.cpp
│   │   ├── test_constraints.cpp
│   │   ├── test_qp.cpp                       # shared hard/elastic fixtures
│   │   ├── test_solver_osqp.cpp
│   │   ├── test_solver_qpoases.cpp
│   │   ├── test_solver_piqp.cpp
│   │   ├── test_slack_semantics.cpp
│   │   ├── test_cross_solver_equivalence.cpp
│   │   ├── test_allocation_free.cpp
│   │   └── benchmark_safety_filter.cpp
│   └── README.md
├── teleop_controllers/                       # existing, update linking + runtime
├── teleop_bringup/                           # existing, update config + launch
└── quest_teleop_interfaces/                  # extend diagnostics only if necessary
```

**Build baseline:** C++17, `ament_cmake`, Eigen3. **Implement and package three pinned native backends:** [OSQP](https://osqp.org/docs/interfaces/C.html) (Apache-2.0, operator splitting), [qpOASES](https://github.com/coin-or/qpOASES) (LGPL-2.1, active-set), and [PIQP](https://github.com/PREDICT-EPFL/piqp) (BSD-2-Clause, proximal interior-point). Record exact versions, binary integration, dependency licensing/notices and ROS 2 build compatibility. Verify that no backend builds or links a separate torque-control process. License obligations, particularly qpOASES LGPL dynamic/static linking and notices, must be reviewed. Do not substitute qpax/Python for the C++ interior-point implementation. A startup-only YAML parser (e.g. `yaml-cpp`) is allowed, but no configuration parsing in the callback. Install both configuration files into the package share directory. Add tests as separate targets.

---

## 4. Configuration design — required behavior

### 4.1 Two levels of configuration

**CBF package-owned settings** (numerical/geometry): `src/cbf_safety_filter/config/cbf.yaml`.  
**Bringup-owned activation and choice of CBF file**: `cbf` section in unified `single_panda.yaml` / `dual_panda.yaml` and `arm_settings.<arm>.cbf` overrides.

**Merge precedence:** per-arm `cbf` overrides > global `cbf` > `{enabled: false, config_file: ""}`.

- `enabled: false`: no filter computation, no QP calls, no CBF geometry work and original controller torque path preserved.
- `enabled: true`: require a readable, valid, explicitly resolved CBF config file. **Reject** empty path, invalid geometry/parameters, unknown keys, or unsupported solver before hardware ownership.
- Use **immutable startup configuration** for v1. No dynamic ROS parameter updates to safety settings during operation.
- Support absolute config paths and optionally `package://cbf_safety_filter/config/cbf.yaml` with a tested resolver. Package URI resolution happens in bringup, before any process begins; do **not** silently fall back when a configured file is missing.
- Avoid contradictory activation flags inside the CBF config: **only bringup owns enable/disable**.

### 4.2 Example package config (illustrative values, not deployable tuning)

**All three solver backends are required to be implemented, built, unit-tested and benchmarked.** `solver.backend` selects *one* backend for a given arm at startup; it does **not** run all three solvers inside each 1 kHz callback. Each arm may select a different backend. The `qp.formulation` setting selects `hard` or `elastic` on **every** backend. Support elastic optimization from the first solver release rather than postponing it.

```yaml
# cbf_safety_filter/config/cbf.yaml -- EXAMPLE ONLY, NOT HARDWARE-CERTIFIED
version: 1
robot:
  model: panda
  joints: 7
  gravity_command_convention: externally_gravity_compensated

execution:
  mode: synchronous
  max_constraints: 256             # maximum CBF rows (excluding actuator box rows)
  max_soft_constraints: 128       # max slack variables; <= max_constraints

objective:
  type: torque_deviation
  joint_weights: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

qp:
  formulation: elastic             # hard | elastic; both supported on every solver
  slack:
    model: per_inequality          # one s_i >= 0 for each explicitly relaxable row
    penalty: l1_plus_l2           # rho_i*s_i + 0.5*eta_i*s_i^2
    require_explicit_opt_in: true # default hard for any row not opted into slack
    acceptance_policy: reject_positive_slack  # reject_positive_slack | allow_bounded_degradation
    # Degraded acceptance is allowed ONLY for offline/mock tests until reviewed;
    # define per-family physical-unit budgets before any hardware use.

solver:
  backend: osqp                    # osqp | qpoases | piqp
  warm_start: true
  osqp:
    max_iterations: 100
    absolute_tolerance: 1.0e-5
    relative_tolerance: 1.0e-5
  qpoases:
    max_working_set_recalculations: 100
    termination_tolerance: 1.0e-6
  piqp:
    max_iterations: 50
    absolute_tolerance: 1.0e-6
    relative_tolerance: 1.0e-6

verification:
  hard_row_tolerance: 1.0e-6
  nonnegative_slack_tolerance: 1.0e-8
  soft_row_tolerance: 1.0e-6
  # All tolerances must be interpreted in documented, unscaled row units.

constraints:
  joint_position:
    enabled: true
    safety_margin_rad: 0.10
    alpha1: 5.0
    alpha2: 5.0
    relaxation:
      enabled: false              # hard unless explicitly opted in
      linear_penalty: 100000.0
      quadratic_penalty: 1.0
      max_accepted_slack: 0.0
  joint_velocity:
    enabled: true
    safety_margin_rad_s: 0.10
    alpha: 5.0
    relaxation:
      enabled: true               # demonstrates initial elastic capability
      linear_penalty: 100000.0
      quadratic_penalty: 1.0
      max_accepted_slack: 0.0     # positive slack rejected by default
  joint_acceleration:
    enabled: false
  table:
    enabled: false                # enable only after geometry validation
    height_m: 0.0                 # example; must be surveyed
    clearance_m: 0.05
    alpha1: 5.0
    alpha2: 5.0
    relaxation:
      enabled: false              # initially hard if enabled
  obstacles:
    enabled: false
    clearance_m: 0.05
    alpha1: 5.0
    alpha2: 5.0
    relaxation:
      enabled: false
  self_collision:
    enabled: false
  inter_arm_collision:
    enabled: false

geometry:
  file: geometry_panda_umi.yaml       # resolve relative to this config file

diagnostics:
  enabled: true
  record_constraint_residuals: true
  record_computation_time: true
  record_slack_values: true

failure_policy:
  solver_failure: request_existing_protective_handling
  infeasible_qp: request_existing_protective_handling
  positive_slack_rejected: request_existing_protective_handling
  invalid_model: terminate_control
  deadline_overrun: request_existing_protective_handling
```

**Config semantics to implement explicitly:**

- Implement all three selector values. Reject unknown backends or unsupported backend/formulation combinations at startup; **no backend may silently fall back to another**. Record the chosen backend/formulation in telemetry.
- Parse and validate *all three backend settings*, even if only one is active. Validate backend-specific iteration/work-set limits and numeric tolerances; translate generic settings into each backend's actual API explicitly (e.g. qpOASES working-set recalculations are **not** the same unit as OSQP iterations).
- `qp.formulation: hard` treats every enabled CBF inequality as hard and **does not** introduce slack variables. The `elastic` formulation adds slack only for rows with `relaxation.enabled: true`; all other rows remain hard. Do not allow `elastic` to silently soften every inequality.
- Test actual positive-slack operation from the outset in the mock/offline suite by setting `qp.slack.acceptance_policy: allow_bounded_degradation` **and** a strictly positive `constraints.<family>.relaxation.max_accepted_slack` for a specifically opted-in family. This is a test of elastic behavior, **not** authorization for physical use. The installed package default should continue rejecting positive slack.
- Never relax torque magnitude/rate bounds, state/model validity, or any explicitly mandatory row. The table/obstacle configuration is **not** permission to silently relax collision clearance. Make relaxability deliberate and testable for every family.
- Positive slack may satisfy the elastic QP while the **original safety inequality is violated**. Report `solved_relaxed`, actual slack and original-row residual; do not report an ordinary `safe` result. The initial configuration `reject_positive_slack` keeps hardware activation out of the design until a reviewed degraded-response policy exists. `allow_bounded_degradation` may be used in offline/mock validation to evaluate bounded soft violations with explicit per-family thresholds and units, without claiming hard-CBF safety.
- For `l1_plus_l2`, require nonnegative finite penalties and a strictly positive quadratic penalty when the backend needs a positive-definite slack Hessian; validate convexity and numerical conditioning. Do not penalize hard actuator constraints via slacks.
- Interpret `max_constraints` as the CBF-row capacity, and `max_soft_constraints` as the maximum dimension of the additional slack decision vector. Preallocate for worst-case `n = 7 + max_soft_constraints`, plus mandatory actuator rows and the `s >= 0` bounds. If workspace capacity would be exceeded, fail explicitly; do not drop inequalities.
- Distinguish convergence tolerance, **independent hard-row verification**, **elastic augmented-row verification**, original unrelaxed CBF violation and **slack acceptance limits**. Preserve raw values in their physical row units; any internal row scaling must be reversed for checking/reporting.
- Validate all numeric values for type/range/finite; reject unknown keys and unsupported combinations. Check robot joint count, frame IDs, available geometry and selected constraint capacity.
- The generic `request_existing_protective_handling` is a **software response request**, not a promise of collision-free braking. Test its mapping into existing controller faults.
- Validate geometry/robot conventions and clearly label settings **not safety-certified** until commissioned.

### 4.3 Bringup examples

Global single-arm default, **off**:

```yaml
# src/teleop_bringup/config/single_panda.yaml
cbf:
  enabled: false
  config_file: package://cbf_safety_filter/config/cbf.yaml
```

Dual-arm, different files/activation per arm:

```yaml
# src/teleop_bringup/config/dual_panda.yaml
cbf:
  enabled: false
  config_file: package://cbf_safety_filter/config/cbf.yaml

arm_settings:
  panda_left:
    cbf:
      enabled: false
      config_file: /absolute/path/to/left_cbf.yaml
  panda_right:
    cbf:
      enabled: false
      config_file: /absolute/path/to/right_cbf.yaml
```

These examples show **configuration behavior**, not permission to operate hardware.

### 4.4 Exact config/launch files to modify

1. `src/teleop_bringup/teleop_bringup/runtime_configuration.py`
   - Add `cbf: {enabled, config_file}` to the global allowed sections.
   - Extend `arm_settings` allowed sections from `controller, hand` to `controller, hand, cbf`.
   - Add strict CBF booleans/path/schema validation and per-arm merged semantics.
   - Preserve strict unknown-key rejection and backward compatibility for old runtime YAML **without** any `cbf` section.
   - Ensure `unified_settings()` / `feature_document()` / `load_runtime()` accept the new section coherently. **Test exact-key fixtures**.
2. `src/teleop_bringup/launch/cartesian_impedance_controller.launch.py`
   - Merge global + per-arm CBF settings, resolve file path, verify config **before** constructing any hardware `Node` actions.
   - Inject `cbf_enabled` and `cbf_config_path` parameters into the **existing** owner node.
   - Respect existing `left.` / `right.` prefixes for dual-arm parameters; use unprefixed parameters for single arm.
3. `src/teleop_bringup/launch/teleoperation.launch.py`
   - Preserve existing interface: **only** `runtime_config` and `operator_confirm`. Keep selection in YAML; do not resurrect the removed `safety_config` CLI argument.
4. `src/teleop_bringup/config/single_panda.yaml`, `dual_panda.yaml` — add **disabled** `cbf` defaults and documented optional per-arm overrides.
5. `src/teleop_controllers/src/controller_runtime.cpp` — consume `cbf_enabled` / `cbf_config_path` via the existing parameter helper, construct/validate the library object **before** `robot.control(...)`.
6. Update `src/teleop_bringup/test/test_runtime_configuration.py`, launch tests, and relevant standalone config tests. Some existing tests check exact sets of YAML keys; adjust assertions intentionally.

**Single responsibility:** `teleop_bringup` selects/resolves file and enable flags; `cbf_safety_filter` validates detailed numerical/geometry contents. Either layer can reject unsafe startup.

---

## 5. Model/data interface contract

### 5.1 CBF library must not import libfranka or controller headers

Define Eigen-based read-only input types, e.g.:

```cpp
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat7 = Eigen::Matrix<double, 7, 7>;
using Mat6x7 = Eigen::Matrix<double, 6, 7>;

struct RobotModelView {
  const Vec7& q;
  const Vec7& dq;
  const Mat7& mass;
  const Vec7& coriolis;
  // Read-only views of frame poses and frame Jacobians.
  // Exact ownership / lifetime must be explicit.
};

struct TorqueBounds {
  Vec7 lower;
  Vec7 upper;
};
```

Do not use this illustrative layout uncritically: the production interface must permit lifetime-safe references/maps and avoid accidental heap copies inside the callback. Wrap the existing `RobotModelState` in `teleop_controllers` using an **adapter**, not a dependency cycle.

### 5.2 Torque/dynamics convention

Current `cartesian_impedance_controller.cpp` computes `J.transpose() * wrench + coriolis` without explicitly adding gravity to the sent torque. For the repository's externally gravity-compensated command convention:

\[
M(q)\ddot q + c(q,\dot q) = \tau_{\mathrm{cmd}},
\qquad
\ddot q=M(q)^{-1}(\tau_{\mathrm{cmd}}-c).
\]

**Verify this convention with the pinned libfranka API and recorded `tau_J_d` values before accepting any CBF row.** Document distinctions among commanded torque, robot desired torque, sensor measured torque and gravity (see `TrackingSample.msg`). The CBF and final torque-rate bounds must use the **same** commanded-torque convention. Solver-independent row construction must also preserve each row's **hard/relaxable classification, original units, and slack-penalty metadata**.

### 5.3 Share, don't recompute

Reuse the model snapshot already created inside the libfranka callback. Profile `franka_model_feedback()`, particularly its separate end-effector Jacobian plus per-frame Jacobian calls. Reuse already computed `M`, `c`, frame transforms and frame Jacobians. Avoid per-CBF calls into `franka::Model`, repeated `M.inverse()` and redundant whole-chain forward kinematics.

### 5.4 Stable mass-matrix computations

Use a **single validated factorization** (`LLT` or suitable alternative) per control cycle, reuse it for all right-hand sides and reject invalid/ill-conditioned models per a documented policy:

\[
B_i = J_iM^{-1},\quad a_{i,0}=\dot J_i\dot q-B_ic,\quad
\ddot p_i=B_i\tau+a_{i,0}.
\]

Implement `B_i` through linear solves, e.g. solve `M X = J_i^T`, then transpose `X`. Do not explicitly invert the mass matrix just to construct these coefficients. Explore reuse of existing controller matrix factorizations **only if** it does not cause lifecycle/reference hazards or alter nominal behavior.

---

## 6. CBF geometry, Lie derivatives and constraint formulation

### 6.1 Recommended first implementation

- Use robot link/frame poses and Jacobians in `RobotModelState`.
- Revalidate the historical 14-capsule Panda geometry from `docs/figures/cbf_geometry/README.md` against the real attachment frames and tool geometry.
- Provide a **static, bounded set** of collision primitives configured at startup.
- For table avoidance, use capsule **endpoint height** barriers (two endpoints per capsule) rather than a dense sphere sampling of each capsule. Both endpoint center heights above the table + radius + margin imply the capsule is above the plane, assuming the geometry is correctly specified.
- For obstacle avoidance, begin with static sphere-vs-sphere geometry and a finite/explicit collision-pair list. Avoid nonsmooth closest-point capsule derivatives in v1.
- All geometry is expressed in explicitly defined frames; do not mix `LOCAL`, `WORLD`, and `LOCAL_WORLD_ALIGNED` Jacobian conventions.

### 6.2 Point Jacobians and acceleration bias

For a world-frame point offset `r` relative to a frame origin:

\[
J_p = J_v - [r]_\times J_\omega,\qquad \dot p=J_p\dot q.
\]

Derive or recursively compute \(\dot J_p\dot q\) analytically. A serial revolute-chain derivative can be built from world joint axes \(z_j\), origins \(o_j\), velocities and:

\[
J_{p,j}=z_j\times(p-o_j),\qquad
\dot J_{p,j}=\dot z_j\times(p-o_j)+z_j\times(\dot p-\dot o_j).
\]

**Do not** use noisy cross-cycle finite differences as the primary real-time derivative. Use central finite differences offline to verify analytical derivatives. If the resulting implementation is too complex, evaluate **Pinocchio frame classical acceleration with zero joint acceleration** as an alternative; first verify its URDF, frames and joint ordering match the pinned Panda model. Do **not** mix inconsistent libfranka and Pinocchio models silently.

### 6.3 Dynamics-affine constraints

All safety requirements must be converted to a single sign convention **before** adding elastic slack:

\[
G(q,\dot q)\tau_{\mathrm{cmd}}\le b(q,\dot q).
\]

**Joint velocity:** relative-degree-one CBFs on \(\dot q\), using \(\ddot q=M^{-1}(\tau-c)\).  
**Joint position:** relative-degree-two barriers, including higher-order barrier state domain checks.  
**Table/collision position:** relative-degree-two CBFs using point Jacobians, \(\dot J_p\dot q\) and the torque-to-point-acceleration map.

For a height barrier \(h(q)=z_p-z_{\mathrm{table}}-r-d\), with gains \(k_1,k_2>0\):

\[
\ddot h+(k_1+k_2)\dot h+k_1k_2 h\ge0.
\]

Given \(n=(0,0,1)^T\), \(\ddot p=B_p\tau+a_{p,0}\), the QP row is:

\[
(-n^TB_p)\tau\le n^Ta_{p,0}+(k_1+k_2)\dot h+k_1k_2h.
\]

For static spherical obstacles, consider a **squared-distance** barrier (avoids norm derivative singularity for nonzero radius; tune gains and scaling appropriately):

\[
h(q)=\|p(q)-o\|^2-R^2.
\]

For all high-order CBFs, validate the initial admissibility conditions and explicitly handle initially unsafe/invalid domains; a feasible instantaneous torque inequality by itself does not certify recovery or future safety.

### 6.4 Optimize for hundreds of constraints

For each cycle:

1. Evaluate model/factorize mass **once**.
2. Evaluate frame/collision-point kinematics **once per relevant robot point**, not per obstacle pair.
3. Cache \(p_i\), \(\dot p_i\), \(J_i\), \(B_i=J_iM^{-1}\), \(a_{i,0}\).
4. For each pair, compute only relative geometry, barrier state and **one or a few** 7-variable inequality rows.
5. Perform bound checks and optional conservative pair selection. Any broad-phase pruning must be proven conservative for the relevant dynamic/sampled-data condition, not based solely on current distance.
6. Construct a bounded/preallocated `G` and `b`; check capacity, finite values and row scaling. Capacity overflow is a **fault**, not permission to drop constraints.

Do not claim that 400 inequalities have 400 independent torque variables: the physical control dimension remains seven. With one slack per relaxable inequality, the **QP decision dimension becomes `7 + m_soft`** even though the physical torque command is still seven-dimensional. Account for memory, backend initialization, dense factorization cost, and constraint counts accordingly. An optional future reduced-elastic formulation is permitted, but **not** as a replacement for the required common initial elastic formulation.

### 6.5 Independent numerical tests

For every constraint family, verify:

- Analytic gradient and acceleration bias against a **separate** finite-difference or generated-reference implementation.
- Sign of inequality for safe/unsafe directions, especially near boundaries.
- Joint/arm indexing, geometry frame, units (m, rad, Nm, s).
- `h >= 0`, higher-order domain checks, nonzero separation assumptions.
- Singular/degenerate geometries and nonfinite model states.
- Tight coupled torque/rate limits and genuinely infeasible states.

---

## 7. QP formulation, **three required solver backends**, and elastic feasibility

### 7.1 Shared hard QP (required from day one)

Let \(\tau\in\mathbb{R}^7\) be the actual commanded torque. The baseline objective is:

\[
\min_{\tau}\quad \tfrac12(\tau-\tau_{\mathrm{nom}})^T W(\tau-\tau_{\mathrm{nom}})
\]

subject to **all enabled CBF rows**, written in the common convention \(G\tau\le b\), and the exact existing `actuator_bounds(...)` magnitude/slew intersection. Use diagonal positive-definite `W` initially. Stanford OSCBF's task-consistent objective is an optional **later** feature, not a prerequisite for the three solver implementations.

**Crucial:** Compute `teleop_controllers::actuator_bounds(...)` **before** the QP; include these exact bounds in every backend's optimization, and retain the owner's independent post-solve actuator check. Do not treat a materially clipped QP solution as verified unless the actual outgoing torque is rechecked against original CBF rows.

### 7.2 Shared elastic/slack QP (also required from day one)

Partition enabled CBF rows into **hard** and explicitly **relaxable** subsets. Let \(m_s\) be the number of relaxable inequalities and \(s\in\mathbb{R}_{\ge0}^{m_s}\) contain **one nonnegative slack per soft row**. Use the same canonical problem on all three solvers:

\[
\begin{aligned}
\min_{\tau,s}\quad &
\tfrac12(\tau-\tau_{\mathrm{nom}})^T W(\tau-\tau_{\mathrm{nom}})
+\sum_{i=1}^{m_s}\rho_i s_i
+\tfrac12\sum_{i=1}^{m_s}\eta_i s_i^2 \\
\text{subject to}\quad &G_h\tau\le b_h,\\
&G_s\tau-s\le b_s,\\
&s\ge0,\\
&\tau_{\min}\le\tau\le\tau_{\max}.
\end{aligned}
\]

- \(G_h\) contains unrelaxable CBF rows; **actuator magnitude/rate constraints are always hard** and can be represented as explicit box bounds.
- \(G_s\) contains only rows with explicit `relaxation.enabled: true`.
- \(\rho_i\ge0\) is a linear (L1-style) violation penalty; \(\eta_i\ge0\) is an optional quadratic regularizer. Choose defaults/validation that give a convex, numerically well-posed QP, including a strictly positive quadratic regularizer if needed for an individual backend.
- For `qp.formulation: hard`, \(m_s=0\) and **the same physical constraints** are hard regardless of `relaxation.enabled` metadata.
- For `elastic`, the solver decision vector has dimension \(7+m_s\); return only the first seven variables as torque, and expose the slack vector **separately** for diagnostics and acceptance decisions.
- Avoid an ambiguous single shared slack for all collision constraints in v1: per-inequality slack is necessary for interpretable violation accounting. A lower-dimensional structured/reduced reformulation may be benchmarked later if mathematically equivalent.
- A QP solved with \(s_i>0\) may violate \(G_s\tau\le b_s\); it is an **elastic/degraded solution**, *not* a hard-CBF-safe torque. Independent verification must evaluate **both** \(G_s\tau-s\le b_s\) and the **original** \(G_s\tau\le b_s\).

### 7.3 Three mandatory C++ solver implementations (same milestone)

Implement, link, initialize, test and benchmark **all three** backends in M5; none is optional or postponed merely because a preferred backend is fast enough:

| Config value | Algorithm family | Initial library choice | Implementation requirements |
|---|---|---|---|
| `osqp` | Operator splitting / ADMM (not an interior-point or active-set method) | [OSQP C API](https://osqp.org/docs/interfaces/C.html) | Pin API/version; startup-only `osqp_setup`; fixed-capacity matrix pattern and update strategy; bounded iterations and warm-start; account for workspace changes with extra slack variables. |
| `qpoases` | Online active-set | [qpOASES](https://github.com/coin-or/qpOASES) | Use a compatible `QProblem`/hot-start approach; bounds and general inequalities; fixed maximum augmented problem size; explicit `nWSR` work-set iteration cap; audit memory, licensing and hot-start transitions. |
| `piqp` | Proximal interior-point | [PIQP](https://github.com/PREDICT-EPFL/piqp) | Use its dense or sparse Eigen-backed C++ API; test problem updates and workspace stability; explicit iteration cap, regularization and termination mapping; validate actual allocation behavior. |

The target is **three maintained backend adapters**, not three settings that all call one algorithm. Keep numerical tolerances/iteration caps backend-specific rather than pretending they have identical internal meaning. Verify PIQP/qpOASES compatibility with the repository's compiler/Eigen/toolchain. If a named library is not compatible or license-acceptable, stop, document the issue, and replace it with a real solver **of the same family** only after review.

### 7.4 One backend-independent problem and result contract

```cpp
// Conceptual interfaces; define exact bounded Eigen/solver storage during M1/M5.
enum class SolverBackend { osqp, qpoases, piqp };
enum class QpFormulation { hard, elastic };
enum class SolverStatus {
  solved, iteration_limit, primal_infeasible, numerical_failure, invalid_problem
};

struct CbfRowMeta {
  bool relaxable;                 // false for actuator and mandatory CBF rows
  int family_id;                 // stable association for reporting/penalties
  double linear_penalty;         // used if relaxable + elastic
  double quadratic_penalty;      // used if relaxable + elastic
  double max_accepted_slack;     // physical row units, policy-dependent
};

struct QpResult {
  Vec7 torque;                   // physical control output only
  SolverStatus solver_status;
  bool augmented_problem_feasible;
  bool all_original_cbf_rows_satisfied;
  bool positive_slack_present;
  double max_hard_violation;
  double max_original_soft_violation;
  double max_augmented_violation;
  double max_slack;
  int iterations;
  // Preallocated slack/row diagnostics with explicit active count.
};

class QpSolver {
 public:
  // Configure/create workspace only at startup; per-cycle solve is bounded.
  virtual QpResult solve(const QpProblem& problem) noexcept = 0;
  virtual ~QpSolver() = default;
};
```

Use a startup-only factory to instantiate the selected solver; **do not instantiate or switch solvers inside a callback**. All backends consume the same canonical Hessian, gradient, hard/soft row metadata and box bounds. The QP assembler, not individual solver adapters, owns slack indexing and construction of the augmented problem.

`noexcept` does **not** prove that a third-party library won't allocate, block or abort. Audit and test backend-specific behavior with both hard and elastic dimensions, rapid active-row changes and warm starts. A backend that cannot meet the real-time contract remains a functioning **offline/benchmark** implementation but is not approved for hardware selection.

### 7.5 Independent verification and explicit slack acceptance

Always check, independently of the optimizer's return status:

1. Finite \(\tau\) and slack values; physical torque magnitude and slew bounds.
2. Hard rows: \(G_h\tau-b_h\le\varepsilon_h\).
3. Nonnegative slack: \(s\ge-\varepsilon_s\).
4. Augmented relaxed rows: \(G_s\tau-s-b_s\le\varepsilon_{\mathrm{aug}}\).
5. **Original CBF residuals**: \(G_s\tau-b_s\), with each row's original units, family, allowed slack and raw violation recorded.
6. A defined `positive_slack` threshold/policy. Distinguish numerical slack below tolerance from intentionally accepted physical relaxation. Never use an implicit global tolerance to conceal a relaxation in collision clearance or other safety limits.

Classify results at least as `solved_hard`, `solved_relaxed`, `rejected_slack_budget`, `infeasible`, `iteration_limit`, and `numerical_failure`. `solved_relaxed` is **not equivalent** to `solved_hard` and must never be labeled `safe` without a separate safety argument. A `solved_relaxed` result may be *accepted under an explicitly configured degraded policy in offline/mock tests*, with a per-row/family maximum and independent validation. Default `reject_positive_slack` must reject such a result through the reviewed owner fault path. No positive-slack hardware acceptance until a separate explicit safety review.

### 7.6 Cross-backend tests and fair benchmarks

For **each** backend, test **both `hard` and `elastic`**, with common problem fixtures, identical original inequalities, objective weights, penalties, accepted slack limits, and feasible/infeasible examples. Cover at least:

- Hard-feasible: same optimum torque (within independently chosen numerical tolerances) and hard-row residuals.
- Hard-infeasible due to competing CBFs vs actuator limits: report infeasible or nonconverged correctly; never publish a fabricated torque.
- Elastic restoration with only opted-in rows relaxed: verify positive slack, augmented feasibility, original CBF violations and strict hard actuator bounds.
- Elastic infeasibility caused by contradictory **hard** rows: even slack must not claim to repair the problem.
- Zero-slack elastic solutions: match hard-QP behavior where both are feasible, within numerical tolerance.
- Penalty sensitivity (L1 and L2), ill-conditioning, varying soft-row count, warm-started sequences, abrupt changes in active constraints and max-iteration termination.
- Backend agreement on status classification **after independent checking**. Distinguish solver-provided status from verified result classification.

Benchmark 10/30/100/200/400 **CBF rows**, with realistic hard/soft ratios, **different numbers of slack variables**, and the **same** reference input/state and logical constraints. **Increase the benchmark workspace capacities appropriately** (for example, `max_constraints >= 400` and a sufficient `max_soft_constraints`); the package-default 256-row example is deliberately too small for the 400-row case. Report individual initialization time, model+constraint evaluation time, QP update time, QP solve time, verification time, total callback time, observed maxima/tails, feasibility and slack statistics. Do not compare a 7-variable hard QP to a \(7+m_s\)-variable elastic QP as though they were the same workload.

**All three backends are deliverables regardless of which is fastest.** The fastest validated backend may become the recommended runtime default, but the config option must remain available for the other two (subject to explicit real-time qualification).

---

## 8. Exact real-time integration contract

### 8.1 Startup (outside callback)

1. Parse ROS params `cbf_enabled` and `cbf_config_path` using existing per-arm parameter conventions.
2. If disabled: do not construct a QP workspace or run CBF math in callback.
3. If enabled: read and validate package config, load geometry, preallocate buffers, construct the configured **hard or elastic** QP capacity and initialize/warm the **selected** OSQP/qpOASES/PIQP backend **before** `robot.control`. All three backends must be independently selectable/testable, but only one runs per arm per callback.
4. Confirm chosen hard/soft row counts and slack capacity, model convention, solver backend, formulation, penalties and slack-acceptance policy. Log effective configuration **outside callback**.
5. Retain existing explicit hardware `operator_confirm` and startup safety requirements.

### 8.2 Callback order (illustrative pseudocode)

```cpp
// libfranka callback, concept only; not certified hardware control code.
const auto feedback = franka_model_feedback(state, model);
const auto nominal = manager.update(feedback.control, now_ns, dt);
if (nominal_has_existing_fatal_fault(nominal)) {
    return existing_fault_path();
}

const auto bounds = actuator_bounds(
    feedback.control.previous_torque,
    configured_torque_limits,
    configured_torque_rate,
    dt);
if (!bounds) {
    return existing_fault_path();
}

if (!cbf_enabled) {
    final_torque = bounds->clamp(nominal.control.nominal);
} else {
    const auto cbf_state = adapt_model_without_allocating(feedback.model);
    const auto result = cbf_filter.compute(
        cbf_state, nominal.control.nominal, *bounds, dt);
    if (!result.hard_and_augmented_constraints_verified() ||
        !result.slack_accepted_by_configured_policy()) {
        // Do not replay previous torque or blindly return zero.
        // Request the separately validated owner fault response.
        return existing_validated_fault_path(result.status);
    }
    final_torque = result.torque;
    if (!final_actuator_and_cbf_check(final_torque, *bounds, result)) {
        return existing_validated_fault_path();
    }
}
return to_franka_torques(final_torque);
```

**Implementation note:** This is a control-flow specification, *not directly compilable code*. Adapt to the actual callback: existing shutdown, homing, protective mode and telemetry semantics are not optional. The `step` lambda currently computes nominal output and final clamping together; refactor carefully with explicit enabled/disabled behavior. Avoid a second pass that computes a different state or actuator interval.

### 8.3 Failure and supervisor contract

Produce explicit statuses such as:

`disabled`, `solved_hard`, `solved_relaxed`, `slack_budget_rejected`, `solver_nonconvergence`, `infeasible`, `invalid_model`, `constraint_violation`, `constraint_capacity_exceeded`, `timing_overrun`. Always attach solver backend, formulation, and original/augmented residuals to numerical results.

Define fault mapping in the **controller owner**, not inside the numerical library. A recommended first-development policy:

| Failure | Software response for development | Important limitation |
|---|---|---|
| Missing/invalid CBF file | Refuse startup | Never silently disable an explicitly enabled filter |
| Nonfinite/inconsistent dynamics or geometry | Existing fatal invalid-state path | Must not output placeholder torque |
| Nonconverged/infeasible QP | Reject unverified candidate; request a **reviewed** protective response | Existing damping/holding may not satisfy collision constraints |
| Valid elastic QP with nonzero slack | Report `solved_relaxed`, actual soft-row violations and slack; apply **startup-configured** acceptance policy | Default reject; bounded degraded acceptance only in offline/mock until separately authorized for hardware |
| Slack exceeds configured family/row budget | Reject candidate and request reviewed fault handling | Solver convergence is not a safety proof |
| Missing CBF constraint or over-capacity | Reject candidate/fault | Never omit a constraint silently |
| Time budget exceeded | Record and request reviewed response | A synchronous solver cannot necessarily be preempted after the deadline |
| CBF disabled | Exact prior nominal/clipping behavior | No CBF safety claim |

**Do not call arbitrary last known torque "safe".** A torque validated at \(q_k,\dot q_k\) may violate the constraints at the next state. Validity is local in state and time.

Separate a **software protective response** from a **formally proved backup safety controller**. The latter requires a recoverable invariant set, bounded faults/model errors, bounded actuation/detection delays and a compatible collision geometry. Do not claim such a guarantee merely because a damping/holding law exists.

### 8.4 Timing/deadline caveat

A callback at 1 kHz does **not** imply an available 1 ms computation budget. Benchmark the actual pinned libfranka/OS/network setup with sufficient margin. Avoid introducing expensive diagnostic serialization or allocations into the callback. If synchronous CBF cannot meet the observed deadline distribution, **stop and document the tradeoff**; do not silently introduce a background solver that replays stale torques.

---

## 9. Diagnostics and observability

Use existing bounded queue/snapshot machinery to publish from non-RT threads. Extend `quest_teleop_interfaces` only if useful; preserve existing consumers and update tests for any message schema changes.

Recommended per-cycle information (lightweight values only in callback):

| Field | Meaning |
|---|---|
| `cbf_enabled`, `cbf_status`, `cbf_solver_backend`, `cbf_qp_formulation` | Activation, chosen backend/formulation and validated result state |
| `cbf_constraint_count` | Number of built inequality rows |
| `cbf_binding_count_estimate` | Optional diagnostic, not the same as enforced count |
| `cbf_max_hard_violation`, `cbf_max_original_soft_violation`, `cbf_max_augmented_violation` | Independently evaluated hard, unrelaxed and elastic-row residuals in original units |
| `cbf_soft_row_count`, `cbf_positive_slack_count`, `cbf_max_slack`, `cbf_sum_slack` | Relaxation scope and amount; zero does not automatically imply domain safety |
| `cbf_slack_acceptance_status`, `cbf_slack_budget_exceeded` | Explicit degraded-policy decision and fault status |
| `cbf_intervention_norm` | \(\|\tau_{\mathrm{filtered}}-\tau_{\mathrm{nom}}\|\) |
| `cbf_solver_iterations` | Numerical workload; qpOASES work-set recalc counter requires explicit semantics |
| `cbf_model_us`, `cbf_kinematics_us`, `cbf_constraints_us`, `cbf_qp_update_us`, `cbf_solver_us`, `cbf_verify_us`, `cbf_total_us` | Timing breakdowns, including elastic assembly and validation |
| `controller_callback_us` | Complete owner callback duration |
| `cbf_fault_count`, `cbf_overrun_count` | Fault and timing tracking |

For disabled mode, retain existing telemetry semantics and minimize overhead. Do not overwrite `nominal_torque`, `commanded_torque`, `robot_desired_torque` or `measured_torque` meanings. Record both the **QP candidate** and the **actual returned command** if they can differ, with clear labels. Do **not** collapse `solved_relaxed` into an ordinary `safe` flag; publish slack and original-row violations (or bounded summaries) without allocating in the callback.

---

## 10. Detailed implementation milestones with exit gates

### M0 — Baseline and safety constraints

**Tasks:** checkout/review current branch; run existing build/tests; record versions, compiler flags and target CPU; verify and pin **OSQP, qpOASES and PIQP** versions/build/link/license constraints; inspect prior incident/model mismatches; document exact current torque and Jacobian conventions; create feature branch.  
**Exit:** baseline CI results and risks recorded. **No real robot launch**.

### M1 — Package foundation and linking

**Tasks:** create `src/cbf_safety_filter` `ament_cmake` package; export a ROS-independent C++ library; write shared hard/soft QP types and startup backend factory interface, package.xml/CMake/tests; add **all three solver dependencies and licenses/notices**; link from `teleop_controllers`; narrowly update package-independence test.  
**Exit:** `colcon build --packages-up-to teleop_controllers teleop_bringup`; baseline tests pass; disabled controller output unchanged.

### M2 — Configuration and launch routing

**Tasks:** package-owned `cbf.yaml` with three backend choices, backend-specific settings, `qp.formulation: hard|elastic`, per-family relaxation penalties and slack acceptance policy; strict startup parser and schema versioning; bringup `cbf` section; per-arm overrides; path resolver; launch parameters for single and dual; startup reject-on-invalid behavior; update strict schema tests.  
**Exit:** disabled by default; global/per-arm precedence proven; bad config fails before opening hardware.

### M3 — Kinematics and derivative reference

**Tasks:** model adapter; frame convention verification; collision-point positions/Jacobians; analytical \(\dot J_p\dot q\); mass-factorization reuse; independent central-difference reference; test fixtures with real recorded model states.  
**Exit:** derivatives pass numeric comparisons in normal, near-boundary and near-singular configurations; no model duplication without justification.

### M4 — First hard/relaxable CBF constraints

**Tasks:** torque/rate bounds (always hard), joint velocity and joint position CBFs, table clearance using capsule endpoints; tag every CBF row as hard or explicitly relaxable with per-row penalty metadata; explicit domain/initial-state checks; row normalization as appropriate; independent original/augmented residual checker.  
**Exit:** sign/frame/unit tests pass; hard and explicitly relaxable rows correctly classified and verified; infeasible states and any positive CBF slack explicitly flagged.

### M5 — **All three native solver backends + hard and elastic QP verification (mandatory)**

**Tasks:** implement the **canonical hard and per-inequality elastic QP assembler**, with fixed-capacity solver data including up to `max_soft_constraints` slack variables. Implement **three complete native adapters: OSQP, qpOASES active-set, and PIQP interior-point**. Implement startup-only backend factory and immutable per-arm `solver.backend` and `qp.formulation` selection. Support each of the **six combinations** (`osqp`/`qpoases`/`piqp` × `hard`/`elastic`); include proper backend-specific settings, workspace reuse, warm/cold starts, structured return statuses, and exhaustive original/augmented hard/soft independent checks. Test zero and nonzero slack, accepted/rejected degraded mode in mock/offline, genuinely hard-infeasible elastic problems, and changing row/decision dimensions. Benchmark all six combinations against identical fixtures.  
**Exit:** **all six combinations** initialize and solve the reference suite; all three backend dependencies are built and selectable; slack values, violated original rows and penalty effects are verified; numerical failure never authorizes unchecked torque; any backend that cannot be proven callback-safe is explicitly tagged **offline-only**, not silently deployed.

### M6 — Optional real-time integration

**Tasks:** instantiate selected backend and hard/elastic problem workspace at startup; insert filter after nominal torque selection and after calculation of actuator bounds; include those bounds in QP; preserve existing homing/hold/protective contract; implement `solved_hard` vs `solved_relaxed` vs rejected-slack fault policy; return only independently checked and acceptance-approved torque when enabled; no CBF work when disabled; queue lightweight solver/slack diagnostics.  
**Exit:** existing controller regression suite passes both disabled and enabled **mock** paths; no undeclared behavior changes.

### M7 — Scaling and end-to-end timing

**Tasks:** benchmark 10/30/100/200/400 CBF constraints on **all three backends, both hard and elastic formulations**, multiple slack ratios and comparable problem settings; measure model, derivative, augmented QP assembly/update, solver, independent validation and complete callback; verify no callback allocations for each backend before declaring RT-qualified; test varied active sets, near-singularities, high velocities, saturation and changing collision sets.  
**Exit:** publish **per-backend/per-formulation** latency distributions, worst observed time, solver status, hard/soft residuals and slack statistics on the **target computer**. Identify which backends are RT-qualified versus offline-only. An observed result alone is not proof of a hard deadline bound.

### M8 — Fault-injection, degraded response and historical regressions

**Tasks:** on **all three backends** test solver timeout/iteration/work-set cap, hard-infeasible and elastically restored QPs, excessive/negative/nonfinite slack, slack-budget rejection, capacity overflow, invalid model, stale peer state, disabled-mode regression, fault latch/restart behavior. Reproduce historical stalled-wrist/motion-incidence states using recorded snapshots and **adversarial dynamics**, not only the ideal mock.  
**Exit:** no unchecked torque is accepted, and positive slack cannot be mistaken for full CBF satisfaction; faults have explicit documented response; prior model discrepancies are tracked as **unresolved physical safety risks** until investigated.

### M9 — Static obstacles and self collision

**Tasks:** sphere obstacle CBFs, pairwise reuse, bounded candidate count; conservative broad phase if justified; independent verification of each barrier; self-collision only with validated geometry/derivatives.  
**Exit:** all new constraints pass tests; scaling results updated; enabled constraint count fits capacity.

### M10 — Dual-arm CBF (separate design review)

**Tasks:** specify and validate combined state \((q_L,q_R)\), peer synchronization, base transforms, state-age bounds and coupled dynamics. A coupled relative-position barrier generally has \(A_L\tau_L + A_R\tau_R\) (14 combined controls), or requires a conservative bounded-peer-input formulation.  
**Exit:** separate system-level proof/test plan and verified synchronization; **do not** claim inter-arm collision safety merely because two independently configured seven-variable filters are enabled.

---

## 11. Verification matrix (minimum required tests)

| Area | Tests |
|---|---|
| Disabled compatibility | Original nominal/clip output, homing, hold, mode changes, input watchdogs, single/dual mock |
| YAML/launch | Missing CBF section, malformed enable, missing/nonexistent path, unknown settings, invalid gains, global override, per-arm override, dual-prefix parameters |
| Kinematics | Point position/Jacobian vs libfranka or independent kinematic reference; attach offset and joint ordering |
| Lie derivatives | Analytical vs central difference; stationary, moving, near-singular, large joint-rate cases |
| Barrier construction | Correct sign/units/relative degree; upper/lower joint limits; table height; approaching and receding obstacle; initial HOCBF domain |
| Actuator limits | QP magnitude+slew interval equals `actuator_bounds`; no post-QP violation after final command selection |
| QP backends | **OSQP + qpOASES + PIQP** each build/select/run in hard and elastic modes; reference optima and objective comparison; repeated warm starts; varying active constraints; backend-specific iteration limits; nonfinite inputs; ill-conditioning |
| Elastic/slack semantics | Explicit per-row relaxability; per-inequality slack \(s\ge0\); hard actuator rows never soften; penalty sensitivity; original vs augmented row residuals; positive slack classification; per-family limits; mock degraded acceptance vs default reject; hard-infeasible elastic case |
| Solver output | Independent hard-row scan plus elastic augmented checks and **original unrelaxed CBF residuals**; solver success alone insufficient |
| Real-time behavior | No allocation/blocking/I/O in callback **for each RT-qualified backend and both formulations**, cold initialization outside, full callback latency distributions on actual control computer; mark noncompliant backend offline-only |
| Fault behavior | Model invalid, capacity overflow, QP failure, nonzero/slack-budget rejection, overrun, fault latched/restart, no stale torque replay |
| Prior incidents | Recorded stalled-wrist/motion mismatch, non-ideal plant response, motor friction/coupling sensitivity |
| Dual-arm | Independent per-arm toggles work; inter-arm remains explicitly disabled/unclaimed until a separate validated implementation |

### Suggested build/test commands (adjust to current workspace)

```bash
# From the ROS 2 workspace root; source the pinned ROS environment first.
colcon build --packages-up-to cbf_safety_filter teleop_controllers teleop_bringup
source install/setup.bash

# Start with the isolated library test suite.
colcon test --packages-select cbf_safety_filter
colcon test-result --verbose

# Then run the controller + bringup integration/regression tests.
colcon test --packages-select teleop_controllers teleop_bringup
colcon test-result --verbose
```

Do not run the physical-robot launch merely as a smoke test. Mock startup alone does not validate plant dynamics, collision clearance, stopping distance or hardware deadlines.

---

## 12. Hardware release gate — explicit blocker

**Do not enable CBF on a physical Panda unless all of the following are reviewed and approved:**

- Reconcile the previously reported commanded-vs-observed wrist acceleration mismatch and abrupt motion. Verify robot model, loaded tool/payload, torque convention and measured state response.
- Verify installation-specific table/obstacle geometry, robot base transforms and collision primitive coverage; account for payload, hand and attached tools.
- Validate correct **hard/soft classifications, chosen solver backend, elastic/slack policy, per-family slack budgets, original vs augmented CBF residuals**, CBF domain, hard actuator torque/rate constraints, numerical acceptance thresholds and solver failure policy. **Positive-slack acceptance is not approved for hardware by the sample config**; any such policy requires separate explicit risk analysis and authorization.
- Measure the complete callback under the **actual** real-time CPU/OS/network/libfranka configuration; establish appropriate deadline margin and fault response.
- Validate the protective stop/braking behavior and its recoverable region under modeled disturbances and hardware limits. Existing damping, hold, zero torque and `MotionFinished` are not universal obstacle-avoidance proofs.
- Use deliberately bounded test trajectories and appropriate independent manufacturer-provided safeguarding under a formal commissioning procedure.
- Record an explicit signed-off hardware validation report. **Do not update the default launch YAML to `cbf.enabled: true` as part of routine software implementation.**

---

## 13. Agent deliverables and reporting format

For each milestone, provide:

1. **Files changed** and a short explanation of the new API/config semantics.
2. **Build and test commands with actual results**; do not state tests passed without running them.
3. **Mathematical validation** for any new constraint/derivative, with numerical error estimates and test cases.
4. **Performance and agreement report for all three solver backends in both hard and elastic modes**, including mean, high percentiles, observed maximum, failure counts, QP assembly/update times, original hard/soft residuals, and slack statistics. Report hardware/CPU, compiler flags and constraints; mark RT-qualified vs offline-only backends.
5. **Unresolved risks**, especially unsafe initial HOCBF states, model mismatch, degenerate geometry, hard-infeasible and elastic-feasible QPs, nonzero slack vs physical CBF safety, backend allocations, and hardware callback timing.
6. **Disabled-mode regression statement** confirming the existing behavior is unchanged.
7. **No hardware activation performed** unless separately authorized and validated.

Prefer small, reviewable commits/PR sections grouped by milestone. If a prerequisite cannot be verified, stop at the last safe software milestone and report the blocker rather than fabricating a success claim.

---

## 14. Summary of non-negotiable design choices

- **Separate C++ ROS 2 package** `cbf_safety_filter`, **linked directly into** the existing controller executable.
- **No extra Python process, no ROS torque transport, no separate libfranka owner**.
- **Config owned by CBF package**; **enable/file selection owned by bringup**, including **per-arm overrides**.
- **Default disabled**, without changes to nominal control behavior.
- **Shared model snapshot**; Eigen-based model adapter; consistent Panda/libfranka conventions.
- **Analytical/recursive** Lie derivative computation initially, independently checked; Pinocchio only if justified and model-matched.
- **Three mandatory native C++ solvers**: OSQP (ADMM), qpOASES (active-set), PIQP (interior-point); **each supports hard and elastic/slack QPs from the start** and is configurable per arm at startup. No backend swaps or solver allocation inside callback.
- **Per-inequality optional elastic slack** with linear/quadratic penalties and explicit acceptance policy; actuator magnitude/slew bounds always hard; `solved_relaxed` never means hard-CBF-safe.
- **Independent hard/augmented/original-row output verification**; never equate solver convergence, positive slack, a previous torque or ordinary damping with proof of safety.
- **All safety-critical fault policy belongs to the existing owner**, not the numerical library.
- **No physical deployment** until prior motion/model incidents and timing/fallback assumptions have been addressed.

**First task for implementing agent:** inspect the current checkout, write a precise change list, establish baseline tests, then implement **M1 (package/export/link) and M2 (config/launch)** with CBF **disabled** and no changes to the actual torque law. Continue to mathematical and solver milestones only after these regressions pass. **M5 must deliver OSQP, qpOASES and PIQP in both hard and elastic modes; deferring any of them contradicts this specification.**
