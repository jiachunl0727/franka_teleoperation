# CBF safety filter

`cbf_safety_filter` is a ROS-independent C++ library that modifies nominal Panda torque commands to satisfy configured joint, workspace and collision inequalities. It accepts a coherent model snapshot, nominal torque, and actuator magnitude and slew bounds. The controller owner checks the returned status and verifies the exact outgoing torque before sending it.

## Runtime behavior

The filter is synchronous and solves a hard QP with seven torque variables. Its objective is

$$
\min_u \; \frac12 (u-u_n)^\mathsf{T}W(u-u_n)
$$

subject to the current actuator box and all assembled rows $g_i^\mathsf{T}u\le b_i$. The diagonal matrix $W$ contains positive torque-deviation weights. There are no slack variables or per-family relaxation settings.

The filter uses **bounded recovery** for every enabled constraint family. For a margin $h\ge0$, it clamps negative restoring terms so a violated boundary asks for braking or non-worsening motion. It computes a braking anchor from acceleration $-k_b\dot q$, clips acceleration to the configured limits, then clips torque to the current actuator box. When the requested inequalities cannot be jointly satisfied, recovery may widen a row only as far as the braking anchor requires. Every accepted candidate is checked against the recovery rows and actuator bounds. The fallback projection has a fixed sweep limit and receives the same independent verification.

`execution.safety_torque_computation_method` selects `projection` or `native_qp`. Projection performs weighted sequential projections over the hard rows. Native QP solves the requested hard problem first and, on explicit infeasibility, retries with bounded recovery rows; verified projection remains its fallback. OSQP, qpOASES and PIQP are supported for native solves. Analytic actuator-box and single-row solutions are used only when they satisfy every row.

## Constraint families and geometry

The library supports joint position, joint velocity, joint acceleration, table height, configured spherical obstacles, self-collision, inter-arm collision and task-Jacobian singularity constraints. Collision geometry is loaded from YAML and transformed into WORLD coordinates. The single and dual profiles use Panda UMI geometry. Inter-arm collision checking uses the independent policy: each arm checks its local model against the latest valid peer snapshot, subject to age and timestamp-skew limits.

For signed margin $h$, first-order constraints use

$$\dot h+\alpha h\ge0,$$

and second-order constraints use

$$\ddot h+(\alpha_1+\alpha_2)\dot h+\alpha_1\alpha_2h\ge0.$$

The implementation maps each inequality into torque coordinates, assembles a fixed-capacity problem and independently checks original row residuals after solving. A result is accepted only when its status, actuator bounds and hard-row checks all pass.

## Configuration and diagnostics

The single and dual YAML profiles configure the solver backend and tolerances, hard-row capacity, callback budgets, recovery method and gain, enabled constraint families, geometry files, clearances and diagnostic fields. Each profile must also state the fixed selectors `execution.mode: synchronous`, `execution.fallback_strategy: bounded_recovery`, and `qp.formulation: hard`; the parser rejects any other value. `execution.safety_torque_computation_method` selects `projection` or `native_qp`. The parser also rejects unknown keys, duplicate keys, invalid values and incompatible geometry selections.

`objective.type: torque_deviation` selects the weighted squared distance from the nominal torque. `objective.joint_weights` supplies the seven positive diagonal coefficients of $W$: weight $w_j$ multiplies the squared torque change for joint $j$. The shipped profiles use seven ones, so all joints have equal objective weight.

The `solver.backend` field chooses OSQP, qpOASES or PIQP. `warm_start` asks supported solvers to reuse information from the previous callback. Each backend has a per-solve work cap and convergence settings: OSQP and PIQP use `max_iterations`, while qpOASES uses `max_working_set_recalculations`, the cap on active-set updates. Their `absolute_tolerance` and `relative_tolerance` jointly set how small primal and dual residuals must be before the solver reports convergence. Smaller tolerances request a more accurate solve and can take more iterations; the work cap still bounds the solver effort.

`verification.hard_row_tolerance` is the filter's separate post-solve numerical acceptance threshold for hard-row residuals and the QP's actuator-box residual. It does not alter or relax any constraint. Before a command is issued, the filter checks the exact actuator box again.

Runtime diagnostics report status, backend, accepted row count, residuals, intervention magnitude, solver iterations and computation time. Recovery diagnostics include the number of bounds widened to include the braking anchor and the largest resulting violation relative to requested rows.

`cbf_validate` performs configuration validation and constructs the required solver workspace. `cbf_replay` re-solves a saved hard-constraint problem offline using supported native backends.
