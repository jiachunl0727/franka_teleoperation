# Optional inverse-dynamics Cartesian controller

The runtime now supplies both Cartesian laws with the native
[OSCBF URDF dynamics model](URDF_DYNAMICS.md). The controller-only comparisons
below preserve the model used at the time of that earlier experiment.

The arm owner supports two teleoperation laws through
`controller.cartesian_control_law`: `cartesian_pd` (the existing default)
and `inverse_dynamics`. The selection applies to ordinary Cartesian and
orientation-only teleoperation. Each arm can override it through
`arm_settings.<arm>.controller`. Release holding, homing, input gates, final
magnitude/slew clipping and optional CBF execution retain their existing paths.
No hardware operation or automatic CBF activation is part of this change.

## Law and gain convention

The new option follows the task/nullspace structure of OSCBF's
[`PoseTaskTorqueController`](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/controllers.py#L151-L176):

\[
\Lambda=(JM^{-1}J^T)^{-1},\quad \bar J=M^{-1}J^T\Lambda,
\]
\[
a=\ddot x_d-K_p e-K_d(J\dot q-\dot x_d),
\]
\[
u=J^T\Lambda a+c+(I-J^T\bar J^T)M
  [K_{p,q}(q_d-q)-K_{d,q}\dot q].
\]

The nullspace term is always active and uses `home_joint_position` as its posture goal. Gravity is supplied by Franka's torque
interface, so the gravity-compensated command `u` omits OSCBF's total-torque `g`.
As in their hardware demo, teleoperation acceleration feedforward is zero.
Explicit in-process Cartesian references may supply acceleration. There is no
additional `Jdot*dq` compensation: this matches the inspected OSCBF law.

Orientation error is exactly OSCBF's base-frame expression
`-0.5 * sum(cross(R_current.col(i), R_desired.col(i)))`. Unlike the existing
rotation-vector error, its magnitude is a sine and it vanishes at 180 degrees.
An uninvertible/ill-conditioned task inertia returns `singular_task_inertia`;
the owner terminates rather than silently substituting a pseudoinverse or
changing to another torque law.

The operator explicitly chose to **preserve physical stiffness**, not reuse
its numerical values as acceleration gains. Therefore the acceleration-feedback
matrices are `Kp = Lambda^-1 K` and `Kd = Lambda^-1 D`. This generalizes OSCBF's
diagonal acceleration gains to full matrices. It preserves its torque structure
but does not use the same numerical gains as their demo.

In particular, with zero acceleration feedforward the task torque simplifies to
`J^T * (-K*e - D*velocity_error)`. The proportional wrench therefore remains at
the configured physical stiffness. Inertia dependence changes the damping and
nullspace torque; it does not introduce unit-inertia proportional feedback.
Changing the model or CBF is not part of this implementation.

## Double diagonalization

Each cycle factors `Lambda = L L^T` and diagonalizes
`L^-1 K L^-T = U diag(omega^2) U^T`. For configurable scalar modal damping ratio
`zeta`, the physical damping is

\[
D=L U\operatorname{diag}(2\zeta\omega)U^T L^T.
\]

With `Phi=L^-T U`, the transformed mass is identity, stiffness is
`diag(omega^2)` and damping is `diag(2*zeta*omega)`. This uses the full coupled
inertia; it is not an elementwise square root. See the double-diagonalization
design in [Albu-Schäffer et al., ICRA 2003](https://www.informatik.uni-bremen.de/agebv/downloads/published/albuschaeffericra03.pdf).

## Configuration

Both combined launch YAMLs contain these settings:

```yaml
controller:
  cartesian_control_law: inverse_dynamics
  inverse_dynamics_stiffness: [750.0, 750.0, 750.0, 75.0, 75.0, 75.0]
  inverse_dynamics_damping_ratio: 1.0
  inverse_dynamics_nullspace_stiffness: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
  inverse_dynamics_nullspace_damping: [6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759]
```

Task stiffness is in base-frame xyz/rotation xyz order: N/m then Nm/rad. Its
defaults match the existing effective stiffness `gain*lambda` (750/75 in the
single/dual launch configurations, 1000/100 in standalone defaults). When the
new stiffness setting is omitted, the runtime derives it from that arm's
legacy `gain_*` and `lambda_*`. Explicit new stiffness values are independent
of later changes to those legacy parameters.

The nullspace gains have acceleration-feedback units s^-2 and s^-1. They are
distinct from the old law's torque-feedback `nullspace_stiffness/damping`.
The new law follows OSCBF's `N_tau M a_joint` term directly: the legacy nominal
joint-limit repulsion, nullspace torque cap and stiffness ramp do not apply to
this term. Common final torque magnitude/slew limits always apply. Optional
Cartesian wrench caps still apply when explicitly enabled.

The startup log identifies the selected law. Existing `TrackingSample`
`lambda_*`/`gain_*` fields describe legacy tuning only; the new matrices are
specified by the recorded launch configuration. Its wire layout is unchanged,
so previous bags remain readable.

## Offline replay

Build and source the controller overlay before running the replay script. The
script reads the bag without starting nodes and pairs each complete captured
model/problem with its exact control-cycle reference, measured pose and limits:

```bash
python3 scripts/replay_inverse_dynamics.py \
  --bag log/commissioning/2026-11-05-cbf-forensics/bags/quest_20261105T150001Z_89a8d8e9 \
  --problems log/commissioning/2026-11-05-cbf-forensics/problems \
  --runtime-config src/teleop_bringup/config/single_panda/single_panda.yaml \
  --cbf-config src/cbf_safety_filter/config/single_panda/cbf.yaml \
  --output log/commissioning/2026-11-06-inverse-dynamics/current-cbf
```

The native executable uses the production controller and CBF. It checks
recorded nominal/model/reference pairing, reconstructs the original actuator
interval, solves both nominal objectives and verifies the exact clipped outgoing
torque. It retains the mass matrix, current rows, actuator interval, solver,
slack policy and deadlines. Historical asynchronous captures are evaluated as
frozen synchronous problems, without proposal scheduling or a simulated plant.

Only 32 cycles from the latest trial have complete captured dynamics. They were
selected from failed cycles. Their acceptance fraction is a **captured-state
replay result**, not a full-trial acceptance estimate or a prediction of the
trajectory produced by running the new law. The full telemetry stream alone
does not contain the missing per-cycle mass matrices and frame Jacobians.

### Results from the 2026-11-05 free-space trial

| CBF configuration | Recorded impedance nominal | New inverse-dynamics nominal |
| --- | ---: | ---: |
| Current source YAML (alpha=10) | 15/32, 46.875% | 15/32, 46.875% |
| Archived trial YAML (alpha=5) | 6/32, 18.75% | 6/32, 18.75% |

Every counted acceptance also passes verification after final actuator clipping.
The new controller does **not improve acceptance on these frozen problems**.
Independent linear programming confirms 15 feasible and 17 infeasible current
problems; it agrees with the native verifier on all 32. Ten current captures
already have an individually impossible row. Under the archived settings the
previous independent analysis found 6 feasible and 26 infeasible problems.

The current/archived differences in alpha and execution mode predate this
implementation. Neither configuration was edited by this work. The CBF package's
47 files were hashed before implementation and checked afterward: no changes or
added files. Acceptance improvements between those two configuration rows must
not be attributed to the new controller.

There are two reasons the replay cannot establish a controller-induced change
in feasibility. First, the selected physical stiffness convention cancels
`Lambda` out of the proportional task wrench. Second, the replay retains each
recorded incoming `tau_J_d`, so the available torque-rate interval stays fixed.
The hard feasible set depends on that interval and the recorded state/model,
not on the objective's nominal torque. A different controller operating over a
new trajectory could change both state and torque history; that behavior cannot
be reconstructed from these sparse models without simulating a different plant
trajectory. No such simulation is presented as recorded hardware acceptance.

Evidence is under `log/commissioning/2026-11-06-inverse-dynamics/`:
`current-cbf/summary.json`, `captured-cbf/summary.json`, their per-cycle
`native-results.jsonl`, the paired replay inputs, and
`independent-feasibility.json` (reproduced by the adjacent
`independent_feasibility.py`). These selected captures must not be compared as
an overall online rate against the original asynchronous full-trial rate.

## Verification

The controller and bringup regression suites pass **158 unique cases** (102
controller, 56 bringup), with zero failures or skips. Coverage includes coupled
modal damping, physical stiffness, the OSCBF task/nullspace structure, invalid
models and task rank loss, zero heap allocations in 1,000 controller evaluations,
configuration validation, single/dual mock operation, shared clipping and
shadow behavior for both laws in synchronous/asynchronous execution.

The complete regression run used an isolated overlay at
`install/inverse-dynamics`. The verified sources are also clean-built into the
normal `build/` and `install/` workspace. Re-source `install/setup.bash` in a new
operator shell after rebuilding; no special executable or alternate launch file
is needed. Existing launch filenames remain compatible with both laws.

Installed-binary mock timing checks exercised shadow and filtered modes with
native QP solving forced, the current 52 rows, and unchanged 600 us filter /
800 us callback budgets. Across **6,749 callbacks** (3,000 active), the maximum
callback was **366.345 us** and maximum filter time **354.152 us**. There were
no deadline violations, diagnostic drops or process faults. These are observed
mock timings, not a hardware timing qualification. The installed native replay
also reproduces the 15/32 current-configuration result for both laws.

Subsequent model comparison: [exact OSCBF dynamics replay](cbf/OSCBF_MODEL_REPLAY.md)
executes their unmodified M/c implementation and changes the model input rather
than only the nominal law. It makes all 32 captures feasible and accepts all
30,445 original active commands under the current CBF. This separate experiment
does not change the controller-only comparison above. Its subsequent
[native runtime implementation](URDF_DYNAMICS.md) is now the default provider.
