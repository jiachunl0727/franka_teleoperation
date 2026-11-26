# Collision and singularity implementation

The later [800 µs callback update](CALLBACK_BUDGET_REPORT.md) adds asynchronous native-QP proposals with fresh current-row verification. It supersedes the synchronous-only execution description below; the geometry and barrier equations are unchanged.

Development implementation, default disabled. Physical CBF activation remains blocked. The short hardware procedure is in [HARDWARE_COMMISSIONING.md](HARDWARE_COMMISSIONING.md). Timing evidence is in [AVOIDANCE_VERIFICATION.md](AVOIDANCE_VERIFICATION.md).

## Execution contract

Each arm retains its existing synchronous seven-torque optimization and independent owner thread. No callback waits for a peer. A bounded latest-only, single-writer/single-reader triple buffer exchanges WORLD collision-point positions, velocities, radii, timestamps and advertised motion bounds. No torque candidate or previous solver result is exchanged. Both arms must enable inter-arm filtering; enabling it on a single owner, or on only one of two owners, rejects startup. A startup barrier seeds both snapshots outside the callbacks. Validation time is captured after peer acquisition; a fresh concurrent publication must not be mistaken for a future snapshot. Stale snapshots report `stale_peer`. A failed owner cannot participate in a normal completion barrier.

The owner prepares fresh constraints and actuator bounds, runs the nominal controller with an optional reference governor, solves once and independently verifies the exact outgoing command. Prepared rows are single-use; preparing another transaction invalidates command authorization. The existing `compute()` remains a convenience wrapper. Model-age and total-time checks include time between preparation and solving. No unchecked holding law, extra torque clipping, automatic restart or retreat controller was introduced.

Near a boundary, reference projection removes inward components while retaining feasible tangential and outward components. It projects both Cartesian displacement and twist, including orientation. Previous-cycle binding-row normals additionally limit reference windup; these are supervisory hints, never reused torque constraints. Current rows always come from fresh feedback. Nonorthogonal normals use bounded repeated half-space projections; a remaining projection violation freezes the reference increment. Cartesian projection cannot represent every joint-nullspace restriction: the torque inequalities remain authoritative. A valid filtered torque keeps the session connected. Invalid models/domains, missing/stale peers, infeasibility, rejected solutions or missed deadlines retain the explicit stop path. This is not a certified collision-free stopping controller.

## Franka geometry adaptation

Source: pinned `LCAS/franka_arm_ros2` commit `bc05b65cf87c20ffa02b890d0f23bd39505fd4aa`, `franka_description/robots/panda_arm.xacro`, `hand.xacro`, and `meshes/visual/finger.dae`, Apache-2.0, Franka Emika GmbH. This is the available Franka **description**, not collision geometry exported by libfranka's dynamics API. The numerical package has no dependency on that ROS description at runtime.

The original geometry audit used enclosing spheres derived from the pinned
Franka primitives. The current single and dual models use the
[reviewed capsule and finger geometry](CAPSULE_BODY_COVER.md).

Body groups 0–8 correspond to Panda links0–8; group9 combines the Hand housing and full finger sweep. Base group0 uses fixed BASE coordinates (`frame: -1`). Model frames0–6 are joint1–7, frame7 flange. Hand coordinates are rotated −45° about flange Z. Frame8 is TCP, not the Hand origin.

The source has no finger collision primitive. The pinned finger visual mesh and 0–40 mm prismatic travel give a conservative swept box in Hand coordinates: x ±0.01050156 m; y ±0.06634549 m; z 0.058533332–0.11229988 m. Three spheres cover an enclosing Y-axis capsule. This includes the space between fingers and is independent of gripper messages. No payload or flexible-cable envelope is included. Source `safety_distance` is zero during transcription; CBF clearance is applied once, separately.

Same-body sphere pairs are omitted. Other omissions are explicit group pairs in `self_exclusions`, adapted from the pinned SRDF adjacency/“Never” policy and its link6–8 default exemption. Hand/finger entries are merged into their swept group. These exclusions and conservatively inflated covers require inspection before hardware use; an SRDF label is not proof for a modified envelope. Every cross-arm sphere pair is considered, including fixed bases and Hands. There is no broad-phase distance cutoff.

The older capsule/point entries remain the existing table/static-obstacle configuration. The new `body_spheres` are the complete arm/Hand covers used by self/inter-arm avoidance. Configured WORLD base transforms must describe the actual installation; namespaces do not imply transforms.

## Hard torque constraints

Self-collision uses `h = ||p_i-p_j||² - (r_i+r_j+clearance)²` with analytic relative Jacobians and `Jdot*dq`. The inequality is `hddot + (alpha1+alpha2) hdot + alpha1*alpha2*h >= 0`. Initial h and `hdot+alpha1*h` must be nonnegative. Self/inter-arm and singularity constraints cannot be softened.

For inter-arm rows, the peer snapshot is not treated as stationary. Each participant enforces componentwise point speed and acceleration boxes with limits `speed/sqrt(3)` and `acceleration/sqrt(3)`, so their Euclidean bounds have an explicit producer. Speed uses first-order barrier rows; acceleration uses hard modeled acceleration rows. A peer snapshot exceeding its advertised speed is rejected.

For age plus the next nominal sampling interval T, peer position uncertainty is bounded by `ep = speed*T` and velocity uncertainty by `ev = acceleration*T`. Relative squared-distance rows use lower bounds on h, hdot and hddot, including worst-case peer acceleration and uncertainty in the controlled point's acceleration coefficient over the exact current actuator box. WORLD transforms, positive finite bounds, peer age and model timestamp skew are checked. Each arm is consequently robust against the advertised peer motion rather than relying on the other arm to share avoidance equally. This can substantially restrict close dual-arm motion, including tangential movement: “permitted” means feasible under all enabled constraints, not unrestricted sliding at arbitrary speed.

These are modeled sampled control constraints. Continuous physical guarantees additionally require bounded geometry/model errors, actuator fidelity, callback/communication delay and stopping behavior. A peer that faults or misses its execution contract invalidates assumptions; both owners then stop rather than continuing on stale state. Configurable age limits are rejection limits, not a guarantee of timely delivery.

An exactly redundant hard geometric row is omitted only when its maximum over the current actuator box is below its bound. Every retained row is checked after solving; omitted rows are satisfied by all commands in that box. No required near row is dropped to meet capacity. Exceeding retained-row capacity rejects the cycle. Geometric rows are equilibrated in the solver representation only; independent verification retains unscaled physical inequalities and tolerances.

## Full-task singularity barrier

Let `J_n = diag(1/L,1/L,1/L,1,1,1) J`, with configurable characteristic length L. The metric `s = 1/trace((J_n J_nᵀ)^-1)` is smooth in the full-rank domain and satisfies `s <= sigma_min(J_n)^2`. Enforcing `s >= minimum_normalized_sigma²` is conservative, including away from repeated singular values; no differentiation of singular vectors is needed.

Joint gradients and the directional second derivative along dq are analytic, using joint axes/origins from the coherent model and matrix-inverse derivatives. No cross-cycle differences or additional robot model queries are used. Rank-deficient/numerically singular Gram matrices reject the domain. The HOCBF uses `h=s-minimum²`; its torque coefficient is `-M^-1 gradient(h)`. Threshold, normalization and gains are provisional configuration, not universal Panda operating limits. This guards kinematic conditioning of the tracked full Cartesian task; it does not diagnose wrist hardware/model disagreement.

## Configuration

The shipped `constraints.self_collision.enabled`, `inter_arm_collision.enabled` and `singularity.enabled` are false. The global bringup CBF switch remains false. Each family can be enabled inside the CBF configuration, subject to the dual-participant rule. Hard/elastic formulations and all three native backends remain selectable. Setting inter-arm acceleration/speed bounds lower increases conservatism and can cause infeasibility; increasing them changes the modeled operating contract. Record both peer states, configuration, solver status, residuals and callback timings when investigating restrictions.
