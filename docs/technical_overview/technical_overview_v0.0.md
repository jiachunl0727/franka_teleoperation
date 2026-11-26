# Franka Panda teleoperation

## 1. Functionality summary

The workspace maps Meta Quest 3 controller motion to one or two seven-joint Franka Panda arms. It supports relative Cartesian tracking, orientation-only tracking, joint holding, homing with predefined robot configuration and actuation of Franka Hand for grasping. A linked control barrier function (CBF) safety filter can constrain arm torque using joint and workspace limits. The pipeline also supports mock execution, experiment recording and offline analysis.

| Package | Functionality |
| --- | --- |
| Quest Android app and quest_teleop | OpenXR acquisition, USB transport, clock synchronization, pose and/or twist mapping and a synthetic pose for testing without a robot |
| quest_teleop_interfaces | customized messages, including typed input, commands, state, timing and calibration contracts |
| teleop_bringup | Single/dual and hardware/mock mode selection, configuration, validation and process composition |
| teleop_controllers | Franka Control Interface (FCI) ownership, URDF dynamics, torque laws, actuator limits, homing and per-cycle tracking telemetry |
| cbf_safety_filter | Joint/workspace constraints, torque verification, independent inter-arm collision checks and numerical replay |
| teleop_gripper | Hand calibration, open width controlled by Quest joystick motion, grasp confirmation and width feedback |
| recording | Continuous or Quest-Menu-controlled rosbag2 recording and tracking export |
| Workspace tooling| Pinned dependencies, Docker environments, visualization, analysis, replay and diagnostics |

![Runtime diagram showing input, feedback, torque and recording paths](figures/technical-overview/runtime.png)

*Figure 1. Solid arrows carry control-path data. Dashed arrows show feedback and recording. Dual launch instantiates two arm paths with a common bridge and, when inter-arm collision avoidance is enabled, an in-process geometry exchange.*

## 2. Quest Android app

### 2.1 Data Acquisition and Reference Frame
The app synchronizes OpenXR actions and obtains each Quest controller’s pose and available twist relative to the headset runtime’s LOCAL coordinate frame. This frame is fixed to the surroundings between recenter events, with gravity-aligned vertical +Y axis, +X axis to the right and −Z axis forward. The mapper transforms motion from these axes into the robot base axes. The app sends trigger, joystick, button and tracking data with a reference-frame generation, sequence and session ID. An status panel in headset displays tracking and USB state.

### 2.2 Sample Timing
The OpenXR location time is the frame’s predicted display time, supplied to OpenXR as the time for which the Quest controller pose is estimated. Interval between consecutive location times is used for estimating twist from pose changes.
Android monotonic acquisition time records when the app starts collecting that frame, just before querying the poses. The bridge converts acquisition time to the PC monotonic clock to check sample age across USB and prevent stale commands. Keeping both fields preserves the time represented by the pose and the time at which the app collected it.

### 2.3 Data Transport and Clock Alignment
Android Debug Bridge (ADB) forwards the app’s full-duplex TCP connection over USB to a listener on the PC. Under protocol version 1, each JSON message has a four-byte size field first and is limited to 4,096 bytes. The app sends tracking samples and clock requests; the PC returns clock replies and recording-state events. The Quest app uses Android text-to-speech to announce recording start and end transitions. With headset send/receive times $q_1,q_4$ and PC receive/send times $p_2,p_3$, the estimate of PC-minus-headset clock offset is

$$
\widehat{\Delta}=\frac{(p_2-q_1)+(p_3-q_4)}{2}.
$$

Initial uncertainty is half the round-trip delay after subtracting PC processing time, plus 0.1 ms. The estimate assumes approximately symmetric transport delays and uncertainty grows by 50 ppm as synchronization ages. Reconnection discards queued samples.

## 3. quest_teleop

### 3.1 USB Bridge
The C++ bridge validates framing, schema, side-specific controls and input order, then publishes aggregate and per-controller ROS state. It converts acquisition time to the PC monotonic clock. The saved bridge settings permit up to 100 ms source age, 10 ms uncertainty, 10 ms future offset and 3 s synchronization age. Disconnects are published as invalid input.

### 3.2 Pose and Twist Mapping
Each mapper captures the Quest controller pose relative to quest headset's LOCAL frame and measured robot pose when motion engages. Let matrix $B$ transform Quest pose vectors into the robot base, with anchor poses $(p_0,R_0)$ and $(p_{r0},R_{r0})$. The mapped pose is

$$
p_d=p_{r0}+s_pB(p-p_0),\qquad
R_d=\operatorname{Exp}\!\big(s_R\operatorname{Log}(BRR_0^\mathsf{T}B^\mathsf{T})\big)R_{r0}.
$$

Current profiles set translation scale $s_p=0.75$, rotation scale $s_R=0.33$ and an equal blend between mapped native and pose-derived twists when native data is valid. Pose derivation uses OpenXR location time intervals. Pose and twist have independent processing chains with bypass, one- or four-pole low-pass filtering, and optional causal septic or quintic interpolation. Trigger hysteresis uses 0.75 to engage and 0.25 to release. Tracking, timing or session faults require a fresh release and re-engagement of Quest trigger to restore teleoperation. Right B and left Y buttons together with trigger select orientation-only tracking.

## 4. quest_teleop_interfaces

| Interface | Contract |
| --- | --- |
| QuestControllerState, QuestButtonState | Timed per-Quest-controller samples with availability, validity, tracking and button values. The right controller reports A and B. The left controller reports X, Y, and Menu. Joystick axes are separate fields in QuestControllerState. The bridge publishes horizontal thumbstick_x, vertical thumbstick_y (both −1 to +1), and thumbstick_active for each controller. The Hand worker uses the horizontal axis and availability. |
| CartesianTeleopCommand | Atomic target pose, desired twist, optional pre-filter requested twist, motion mode, source identity and physical-release flag. |
| ControllerStatus, HomingStatus, TrackingSample | Controller and homing state plus coherent per-cycle references, feedback, torque and timing. |
| GripperStatus, GripperWidth, CalibrateGripper | Franka Hand operation and grasp state, timestamped total opening in metres, and explicit empty-Hand and operator confirmations for calibration. |

## 5. teleop_bringup

The combined launch accepts one argument, `runtime_config`, which selects `single_panda.yaml` or `dual_panda.yaml`. Each profile specifies robot-related information, mock or hardware backend, enabled pipeline components, package configuration files and local overrides. Package-specific settings are placed in the specific package; the bringup YAML references them by `package://` path and can override selected values. Bringup resolves these resources and validates the robot information, controller/Hand/CBF settings and recording plan before starting child launches. Hardware mode requires an IP address for every selected arm. Enabled hardware CBF requires `shadow` or `filtered` mode selection. **shadow** mode forwards nominal torque with unpublished safety filter torque computation for testing, while **filtered** mode publishes filtered torque to the robot; dual-arm owners must agree on this selection. A controller process termination emits launch-wide shutdown, which also closes recording processes.
Standalone launches remain available for bridge, mapping, arm control, Hand, homing and recording. They are used for independent launch of each package.

## 6. teleop_controllers

### 6.1 Execution and Control-Mode Selection
One owner per arm evaluates torque in the libfranka callback or a 1 ms mock loop. ROS commands and monitoring data pass through bounded queues and snapshots.
At startup and after a valid trigger release, the owner captures measured joints and applies ramped joint-PD holding controller. Loss of active input disarms tracking and selects the damping controller, an already parked arm can retain its hold. Holding right A or left X Quest button for at least one second, then releasing with the trigger released, requests homing toward the configured joint posture. Homing cancellation enters controlled deceleration and settling. During dual-arm hardware startup, each arm owner reads fresh robot-state updates while waiting for the robot to confirm that torque control has started. It publishes the arm’s initial collision geometry from those updates before controller callbacks begin.

### 6.2 Rigid-Body Model
The URDF dynamics model supplies mass matrix $M$, Coriolis vector $c$ and kinematic frames. The same selected model is used by nominal torque computation and the CBF safety filter. The Panda UMI URDF models two customized finger attachments at a fixed opening. Hardware pose comes from `RobotState`; its flange-to-tool transform sets the task Jacobian frame. With body mass $m_i$, center-of-mass Jacobians $J_{v,i},J_{\omega,i}$ and inertia $I_i$ in the base frame, the controller computes

$$
M=\sum_i(m_iJ_{v,i}^{\mathsf{T}}J_{v,i}+J_{\omega,i}^{\mathsf{T}}I_iJ_{\omega,i}),
\qquad \ddot q=M^{-1}(u-c).
$$

### 6.3 Nominal Torque Computation
**Cartesian Impedance Control Law**

Let $e_p=p-p_d$ and let $e_R=\operatorname{Log}(RR_d^{\mathsf T})$ be the translation error shortest-rotation orientation error expressed in the robot base frame. With $e=[e_p^{\mathsf T},e_R^{\mathsf T}]^{\mathsf T}$ as the pose error and $e_v=J\dot q-v_d$ as the twist error, the task wrench and nominal torque are

$$
F_{\mathrm{imp}}=-G(e_v+\Lambda_e e),\qquad
\tau_{\mathrm{imp}}=J^{\mathsf T}F_{\mathrm{imp}}+c+\tau_{\mathrm{ns}}.
$$

$G$ and $\Lambda_e$ are diagonal wrench and error-shaping gains. $\tau_{ns}$ represents the nullspace torque. The null-space term attracts joints toward the configured home posture, damps joint velocity, and adds joint-limit repulsion. To keep it from changing the Cartesian task, the controller forms $M=LL^{\mathsf T}$, $A=JL^{-\mathsf T}$ and $P=I-A^+A$, then computes

$$
b=K_{\mathrm{ns}}(q_h-q)-D_{\mathrm{ns}}\dot q+r_{\mathrm{lim}},\qquad
\tau_{\mathrm{ns}}=\gamma\,\operatorname{cap}_{\tau_{\max}}(LPL^{-1}b).
$$

Here $q_h$ is the home posture, $r_{\mathrm{lim}}$ is the joint-limit repulsion vector, $A^+$ is the SVD pseudoinverse, and $\gamma$ ramps the secondary torque from zero to one. The cap scales the projected torque if its largest joint component exceeds the configured limit. Orientation-only mode holds the captured translation while tracking orientation.


**Inverse Dynamics Control Law**

This law computes task inertia $\Lambda=(JM^{-1}J^{\mathsf T})^{-1}$. It uses translation error $p-p_d$ and the implementation's base-frame sine orientation error $e_R=-\frac12\sum_{i=1}^3 R_i\times R_{d,i}$, where $R_i$ and $R_{d,i}$ are columns of current and desired orientation matrices. Let $e$ combine those pose errors and $e_v=J\dot q-v_d$ be the twist error. For configured diagonal stiffness $K$, Cholesky factor $\Lambda=LL^{\mathsf T}$, and damping ratio $\zeta$, the modal damping matrix is calculated as

$$
L^{-1}KL^{-\mathsf T}=V\operatorname{diag}(\omega_i^2)V^{\mathsf T},\qquad
D=LV\operatorname{diag}(2\zeta\omega_i)V^{\mathsf T}L^{\mathsf T}.
$$

Thus the damping coefficient for mode $i$ is $2\zeta\omega_i$. The task acceleration, wrench and torque are

$$
a_d=\ddot{x}_d-\Lambda^{-1}Ke-\Lambda^{-1}De_v,\qquad
F_{\mathrm{id}}=\Lambda a_d,\qquad
\tau_{\mathrm{id}}=J^{\mathsf T}F_{\mathrm{id}}+c+N_\tau M\ddot q_{\mathrm{ns}},
$$

where $N_\tau=I-J^{\mathsf T}\Lambda JM^{-1}$ and $\ddot q_{\mathrm{ns}}=K_q(q_h-q)-D_q\dot q$ provide dynamically consistent home-posture control. Invalid or ill-conditioned task inertia is rejected.

Both control laws can apply configured force and moment limits to the task wrench; reference shaping and tracking-error protection are independently configurable.

### 6.4 Actuation, Protection and Monitoring
Every nominal torque is limited by both magnitude and change from the previous command; the slew interval uses the callback period capped at 1 ms. Current configuration files scale magnitude and rate limits by 0.35 of the maximum value. In CBF shadow mode, nominal clipped torque is sent to robot while filter results are only recorded. In safety filtered mode, the exact outgoing torque is checked against current constraints before issue.
Solver failure, infeasible QP, verification failure or deadline overrun requests the existing protective response: the owner latches motion protection, bypasses the failed filter, and commands bounded braking followed by a low-speed joint hold. In dual mode the protective request is shared by both arm owners. These failures do not directly terminate the controller process. Invalid model data remains a terminating fault.
Telemetry records requested, nominal, outgoing and measured trajectories. The controller's nominal and outgoing commands `tau_J_d` exclude explicit gravity compensation, while robot's sensor torque `tau_J` includes it, because Franka has a built-in convention that computes gravity-compensation automatically.

## 7. cbf_safety_filter

### 7.1 Constraint Construction and Recovery
This ROS-independent C++ library converts joint, point and task derivatives into affine torque inequalities. For a signed constraint $h\ge0$, a first-order row is $\dot h+\alpha h\ge0$. A second-order row is $\ddot h+(\alpha_1+\alpha_2)\dot h+\alpha_1\alpha_2h\ge0, \quad \psi=\dot h+\alpha_1h.$

The runtime uses synchronous hard constraints with bounded recovery for all enabled constraint families.

When the robot is within the safe region and the requested constraints are compatible, the filter enforces the constructed constraints and computes a torque candidate that satisfies those constraints. The torque candidate can be computed with either `projection` or `native_qp` method.
`Projection` starts from nominal torque clipped to actuator limits, then visits the requested constraints one at a time. For each violated constraint, it applies a weighted correction to reduce the violation. It repeats this sequence up to 6 times and checks whether the result satisfies all requested constraints. If not, it repeats the process with bounded-recovery limits. This method does not prove that the requested constraints are infeasible; it switches to recovery if its bounded projection does not produce a candidate that satisfies them.
`native_qp` first uses analytic_nominal shortcuts when enabled: it accepts the clipped nominal torque if it satisfies every requested constraint, or tries an exact weighted correction for a single violated constraint. If neither yields a valid candidate, the configured QP solver finds the torque closest to nominal while satisfying the requested constraints and actuator limits. If the solver reports infeasibility, the filter repeats the process with bounded-recovery limits. If a valid-model QP result cannot be verified, it falls back to the projection method.

When safety constraints conflict or a state is already outside the safety region, a bounded recovery strategy is implemented as fallback for both safety torque computation methods. The bounded recovery strategy allows limited violations to constraints so that the robot can still brake or avoid making the unsafe condition worse. Recovery first anchors deceleration at $-k_b\dot q$, clips it to configured acceleration limits and maps it to torque within the current actuator box. If requested constraints conflict, the recovery path relaxeds constraint bounds only as needed to include that anchor. 

Final torque acceptance requires independent checks against the recovery rows, actuator box and residual tolerances.

### 7.2 Optimization and Verification
The hard quadratic programming (QP) formulation has seven torque variables and minimizes

$$
\frac{1}{2}(u-u_n)^{\mathsf T}W(u-u_n),
$$

where $u_n$ is nominal torque and $W$ is a positive diagonal weight matrix. All configured constraints remain hard, it currently supports no slack variables or per-constraint-family relaxation policies. An actuator-box candidate or a one-row projection can avoid a native QP solve when it satisfies all rows. Otherwise, the configured qpOASES, OSQP or PIQP solver runs synchronously.
The owner independently verifies outgoing torque against current constraints. Solver tolerances govern convergence; and a separate post-solve acceptance-limit is specified for checking row and actuator-box residuals. Candidates from projection and QP solver are checked against the same acceptance limits.

### 7.3 Constraint Families
Constraint families include joint position, velocity and acceleration, table height, self-collision, dual-arm inter-arm collision and task-Jacobian singularity. Joint-position and velocity margins shrink the permitted range around their configured limits. Joint acceleration is bounded explicitly in joint space. Table clearance is measured against robot origin of the world frame of reference. Collision constraints separate between configured geometry volumes, their radii and clearance. A boundary activation parameter specifies a distance for self-collision and inter-arm collision respectively. When the calculated distance between two collision volumes is lower than the sum of the specified activation distance and clearance, pairwise body collision constraints are then constructed and checked. Singularity uses the squared minimum singular value of the characteristic-length-scaled task Jacobian against a configured threshold.

### 7.4 Collision Geometry
Each dual owner loads 15 sphere or capsule volumes for the base, links, housing and fingers. The right base is offset from the left by $(0,-0.750,0)$ m with identity relative orientation. Table checks omit the collision volumes for base and Link 1. Four finger volumes use width feedback no older than 30 ms with an inflated motion interval; without fresh feedback the model covers the full 0–80 mm opening range. The policy for dual inter-arm collision avoidance is `independent`: each filter checks local geometry against the latest valid peer snapshot, rejecting snapshots over configured age or timestamp-skew limits.

![Panda model with collision volumes in an articulated pose](figures/collision_model/robot-collision-geometry.png)

*Figure 2. Collision geometry for one Panda arm: detailed robot and UMI meshes with all collision volumes shown in green.*

### 7.5 Failure Response and Diagnostics
As failure responses, the safety filter require `request_existing_protective_handling` for solver failure, infeasible QP and deadline overrun. At runtime these conditions latch the bounded braking-and-hold response instead of ending the control session. `invalid_model` is configured to terminate control because a trustworthy protective torque cannot be computed from invalid model data. Diagnostics preserve solver status, hard-row residuals, timing, recovery-bound widening and whether a protective command was issued.

## 8. teleop_gripper

### 8.1 Calibration and Command Logic
The Hand worker owns a separate Franka Hand connection and supports a mock backend. Calibration invokes Franka SDK homing only after the service receives both required confirmations.
Moving right-controller joystick horizontally left closes and right opens the gripper; directions are reversed for the left controller. Direction thresholds and hysteresis select configured speeds. Neutral or stale input interrupts an unfinished move, and a fresh neutral sample is required before movement resumes after input loss.

### 8.2 Grasp and Feedback
Closing uses the configured grasp force. A grasp is confirmed only when the Franka SDK reports success, grasp state is present and measured width is within the configured cable interval. Confirmation persists through neutral input or input loss until an explicit opening command. Stop is coordinated with outstanding Hand operations. Gripper width and status are published independently of the arm torque loop.

## 9. recording

Continuous mode starts rosbag2 with the selected topics when the recorder launches with teleoperation bringup combined launch. Menu mode uses validated rising edges from the left Menu button, requires a fresh release after input/session faults, and applies a default 350 ms debounce for avoiding accidental double clicks. A toggle of Menu button starts or stops a separate recorder process.
On normal launch shutdown or arm-owner exit, the recorder asks rosbag2 to stop cleanly with SIGINT signal and waits for it to finish writing the bag. If rosbag2 has not exited after 10 seconds, the recorder sends a stronger stop SIGTERM signal; if it is still running 3 seconds later, the recorder forces it to stop with SIGKILL signal, which may leave the bag incomplete.
SQLite3 is used for data storage. The default topics include Quest input, arm commands and measured/tracking state, Hand events and width, and safety filter status when safety filter is enabled. TrackingSample export produces per-topic CSV files and a manifest with sample counts, queue drops and PC-monotonic time origins.
MATLAB scripts compute tracking error, velocity lag and actuator intervention. PlotJuggler scripts provide live views.

## 10. Workspace and external dependencies

Docker provides ROS 2 Humble in a host-network container with real-time scheduling permissions, alongside separate Android build and MATLAB environments. The environment hook defaults to DDS domain 42 and localhost discovery. Workspace scripts support APK/ADB setup, robot-network profiles and latency tuning, model/geometry generation, benchmark and bag replay, and monitoring.

| Dependency or package | Role |
| --- | --- |
| libfranka 0.9.2 | Arm FCI and Hand API. Local patches allow Hand Stop during an outstanding operation, observe state during arm startup and select fresh queued FCI feedback with a bounded drain. |
| qpOASES, OSQP, PIQP | Active-set, operator-splitting and interior-point QP backend solvers linked or built by the CBF package. |
| franka_description, franka_msgs, franka_semantic_components | Upstream robot descriptions and meshes, message definitions, and ros2_control state/model adapters. |
| franka_hardware, franka_robot_state_broadcaster, franka_control2 | Upstream ros2_control hardware, state broadcasting and control integration. The integrated runtime uses its own direct FCI and Hand owners. |
| franka_bringup, franka_example_controllers, franka_gripper, franka_moveit_config | Upstream launch files, example controllers, gripper node and MoveIt configuration. |
