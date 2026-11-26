# Teleoperation launch configuration inventory

## Configuration layout

Start with `teleop_bringup/config/single_panda/single_panda.yaml` or `teleop_bringup/config/dual_panda/dual_panda.yaml`. Each feature section selects its profile with config_file. The runtime file keeps robot addresses, controller assignments, mapping calibration, model selection, actuator derating, grasp targets, recording choices and activation settings visible.

| Runtime selector | Owning package | File under the owning package |
| --- | --- | --- |
| `quest.config_file` | `quest_teleop` | `config/<mode>/quest.yaml` for bridge and mapper tuning |
| `controller.config_file` | `teleop_controllers` | `config/<mode>/controllers.yaml` for torque laws, hold and homing |
| `hand.config_file` | `teleop_gripper` | `config/<mode>/gripper.yaml` for joystick, motion and watchdog tuning |
| `recording.config_file` | `recording` | `config/<mode>/recording.yaml` for the storage plugin and Menu debounce |
| `cbf.config_file` | `cbf_safety_filter` | `config/<mode>/cbf.yaml` for numerical policy and geometry paths |

Here `<mode>` is `single_panda` or `dual_panda`. Each mode has its own editable file and the launch configs live inside those directories. Combined bringup accepts installed `package://` profile URIs and filesystem paths. Relative profile paths resolve from the runtime file’s directory. CBF retains its existing path and validation rules. Direct controller, Hand and recording launches use each package’s `config/standalone.yaml` by default. These files preserve the controller-only gains, Hand-only defaults (including disabled closing), and the recorder’s default topics and output directory. Combined bringup loads the profiles named in the runtime YAML. Standalone arm routing uses `config/single_panda/arms.yaml` or the optional `config/dual_panda/arms_two_mock.yaml` fixture.

**Precedence.** Launch and executable defaults fill omitted fields. The named profile supplies shared values, then explicit runtime values override them. For mapping, the common Quest profile is followed by `mapper_by_controller.<side>` and then explicit arm fields. For the controller, Hand and CBF selection, `arm_settings.<arm>` applies last. CBF geometry paths stay relative to the file that declares them. The runtime YAML selects the hardware or mock backend.

**How to read the inventory.** Single and Dual show the resolved values for the saved combined templates, including defaults where noted. A dash means there is no applicable saved value. Standalone shows the standalone YAML value when defined. **Runtime** means edit the entry file, **Profile** means edit the referenced feature file, **Derived** means omit the setting to use routing or the built-in default, **Guard** marks an authorization or validation control, and **Fixed** marks a required schema value. **Review** identifies remaining decisions about settings with an active effect or compatibility use. The inventory covers all nine first-party launches and their accepted YAML, including referenced CBF geometry. Third-party launches and executable-only parameters are outside its scope.

## Launch arguments

| Launch file in teleop_bringup | Arguments and launch defaults | Routing and behavior |
| --- | --- | --- |
| `teleoperation.launch.py` | `runtime_config` = **required** | Combined entry. The runtime file selects the backend and profiles. |
| `quest_usb_bridge.launch.py` | `namespace` = `/`<br>`listen_port` = `45454`<br>`bind_address` = `127.0.0.1`<br>`command_timeout_ms` = `100`<br>`max_clock_uncertainty_ms` = `10`<br>`max_clock_sync_age_ms` = `3000`<br>`max_source_future_ms` = `10`<br>`client_receive_timeout_ms` = `500`<br>`recording_status_topic` = `/quest/recording_status` | Bridge only. Namespace scopes the aggregate and per-hand topics. The status topic carries recording events to the headset. |
| `pose_mapper.launch.py` | `namespace` = `/`<br>`controller_id` = `right`<br>`controller_state_topic` = `/quest_usb_bridge/controller_state`<br>`measured_pose_topic` = `/panda_1/teleop_anchor_pose`<br>`command_topic` = `/panda_1/cartesian_teleop_command`<br>`monitor_topic` = `/panda_1/target_pose`<br>`base_frame` = `panda_1/panda_link0`<br>`reference_to_robot_xyzw` = `[0.0, 0.0, 0.0, 1.0]`<br>`translation_scale` = `1.0`<br>`rotation_scale` = `1.0`<br>`feedforward_scale` = `1.0`<br>`velocity_filter_cutoff_hz` = `10.0`<br>`command_timeout_ms` = `100`<br>`max_clock_uncertainty_ms` = `10.0`<br>`max_source_future_ms` = `10.0` | Direct one-mapper entry. The remaining mapper YAML keys are available through quest_teleop and robot_free. The direct velocity cutoff controls the mapper’s velocity filter. |
| `quest_teleop.launch.py` | `arms_config` = `teleop_bringup/config/single_panda/arms.yaml`<br>`mode` = `single`<br>`single_controller` = `right`<br>`start_bridge` = `true`<br>`start_mapper` = `true`<br>`mock_measured_pose` = `false`<br>`namespace` = `/`<br>`listen_port` = `45454`<br>`bind_address` = `127.0.0.1`<br>`command_timeout_ms` = `100`<br>`max_clock_uncertainty_ms` = `10`<br>`max_clock_sync_age_ms` = `3000`<br>`max_source_future_ms` = `10`<br>`client_receive_timeout_ms` = `500`<br>`recording_status_topic` = `/quest/recording_status` | Bridge, mapper and optional synthetic pose publisher. Mode is single, dual or all. |
| `cartesian_pd_controller.launch.py` | `mode` = `single`<br>`single_controller` = `right`<br>`backend` = `mock`<br>`arms_config` = `teleop_bringup/config/single_panda/arms.yaml`<br>`runtime_config` = `teleop_controllers/config/standalone.yaml`<br>`homing_enabled` = `true` | Arm owners and optional CBF. Mode is single or dual. |
| `homing.launch.py` | Inherited from controller launch | Includes the controller launch and forces homing_enabled=true. The other controller arguments and defaults apply. Homing still requires the gesture. |
| `gripper.launch.py` | `mode` = `single`<br>`single_controller` = `right`<br>`backend` = `mock`<br>`arms_config` = `teleop_bringup/config/single_panda/arms.yaml`<br>`runtime_config` = `teleop_gripper/config/standalone.yaml` | Per-arm Hand worker. Startup calibration is always requested for launched Hand workers. Mode is single or dual. |
| `recording.launch.py` | `mode` = `single`<br>`single_controller` = `right`<br>`recording_mode` = `menu`<br>`arms_config` = `teleop_bringup/config/single_panda/arms.yaml`<br>`recording_config` = `recording/config/standalone.yaml`<br>`recording_topics` = `""`<br>`recording_output_root` = `""`<br>`left_controller_state_topic` = `""`<br>`recording_namespace` = `/quest` | Mode selects arms and accepts single, dual or all. recording_mode selects menu or continuous. recording_topics is a JSON list that replaces the configured topic selection. recording_output_root overrides the configured absolute directory. An empty left topic derives it from routing. Mode, namespace and left topic use launch arguments on this standalone path. |
| `robot_free.launch.py` | `arms_config` = `teleop_bringup/config/single_panda/arms.yaml`<br>`start_bridge` = `false`<br>`listen_port` = `45454`<br>`mock_measured_pose` = `false`<br>`recording_enabled` = `false`<br>`recording_config` = `recording/config/standalone.yaml`<br>`recording_topics` = `""`<br>`recording_output_root` = `""` | Runs mapping for all configured arms. If enabled, recording is forced to continuous. The recording overrides follow recording.launch.py. |

## Runtime and standalone YAML

**pipeline**

| Key | Single | Dual | Fallback | Effect or constraint | Location or review status |
| --- | --- | --- | --- | --- | --- |
| `mode` | `single` | `dual` | `single` | Select single- or dual-arm topology. | Runtime |
| `single_controller` | `right` | — | `right` | Select the Quest hand in single-arm mode. | Runtime |
| `backend` | `hardware` | `hardware` | `mock` | Select mock or hardware execution. | Runtime |
| `start_controller` | `true` | `true` | `true` | Start the Panda arm torque owner. | Runtime |
| `homing_enabled` | `true` | `true` | `true` | Enable the homing gesture. | Runtime |
| `start_mapper` | `true` | `true` | `true` | Start per-arm pose/twist mapping. | Runtime |
| `start_hand` | `true` | `true` | `true` | Start the Hand worker and request startup calibration. | Runtime |
| `start_bridge` | `true` | `true` | `false` | Start the Quest USB bridge. | Runtime |
| `mock_measured_pose` | `false` | `false` | `false` | Omitted from normal runtime files. Enable for mapper-only testing with start_controller=false. | Derived |
| `recording_enabled` | `true` | `true` | `true` | Start the configured recording subsystem. | Runtime |

**quest.bridge**

| Key | Single | Dual | Fallback | Effect or constraint | Location or review status |
| --- | --- | --- | --- | --- | --- |
| `namespace` | `/` | `/` | `/` | ROS namespace for the bridge and its raw topics. | Derived |
| `listen_port` | `45454` | `45454` | `45454` | TCP port. Must match the Android/ADB route| Profile|
| `bind_address` | `127.0.0.1` | `127.0.0.1` | `127.0.0.1` | Listener address. Default is loopback. | Profile |
| `command_timeout_ms` | `100` | `100` | `100` | Maximum age of a source sample at the bridge. | Profile |
| `max_clock_uncertainty_ms` | `10` | `10` | `10` | Maximum uncertainty of the clock alignment. | Profile |
| `max_clock_sync_age_ms` | `3000` | `3000` | `3000` | Maximum age of the sync result. | Profile |
| `max_source_future_ms` | `10` | `10` | `10` | Tolerance for a converted source timestamp ahead of the PC clock. | Profile |
| `client_receive_timeout_ms` | `500` | `500` | `500` | Socket receive timeout. | Profile |

**arms.<arm>**

| Key | Single | Dual left | Dual right | Fallback | Effect or constraint | Location or review status |
| --- | --- | --- | --- | --- | --- | --- |
| `robot_ip` | `192.168.3.109` | `192.168.3.101` | `192.168.3.109` | `""` | Unique nonempty address for hardware owners. | Runtime |
| `namespace` | `/panda_1` | `/panda_left` | `/panda_right` | `/<arm>` | Unique ROS namespace and base frame. | Runtime |
| `base_frame` | `panda_1/panda_link0` | `panda_left/panda_link0` | `panda_right/panda_link0` | `<namespace>/panda_link0` | Unique ROS namespace and base frame. | Runtime |
| `controller` | — | `left` | `right` | `right` | Arm-to-Quest assignment. Dual needs one left and one right. The single selector can replace it. | Runtime |
| `controller_enabled` | — | — | — | `false` | Legacy arm YAML field. Only false is accepted. | Review |
| `controller_state_topic` | — | — | — | `Derived from bridge namespace and hand` | Raw input, atomic command, mapped monitoring pose, measured anchor or robot state topic, respectively. target_pose_topic maps to the standalone monitor_topic argument. | Derived |
| `command_topic` | `cartesian_teleop_command` | `cartesian_teleop_command` | `cartesian_teleop_command` | `cartesian_teleop_command` | Raw input, atomic command, mapped monitoring pose, measured anchor or robot state topic, respectively. target_pose_topic maps to the standalone monitor_topic argument. | Derived |
| `target_pose_topic` | `target_pose` | `target_pose` | `target_pose` | `target_pose` | Raw input, atomic command, mapped monitoring pose, measured anchor or robot state topic, respectively. target_pose_topic maps to the standalone monitor_topic argument. | Derived |
| `measured_pose_topic` | `teleop_anchor_pose` | `teleop_anchor_pose` | `teleop_anchor_pose` | `teleop_anchor_pose` | Raw input, atomic command, mapped monitoring pose, measured anchor or robot state topic, respectively. target_pose_topic maps to the standalone monitor_topic argument. | Derived |
| `state_topic` | `robot_state` | `robot_state` | `robot_state` | `robot_state` | Raw input, atomic command, mapped monitoring pose, measured anchor or robot state topic, respectively. target_pose_topic maps to the standalone monitor_topic argument. | Derived |
| `reference_to_robot_xyzw` | `[0.5, 0.5, 0.5, 0.5]` | `[0.5, 0.5, 0.5, 0.5]` | `[0.5, 0.5, 0.5, 0.5]` | `[0.5, 0.5, 0.5, 0.5]` | Calibration quaternion mapping Quest LOCAL to the robot base. | Runtime |
| `translation_scale` | `0.75` | `0.75` | `0.75` | `0.75` | Scale the relative mapped translation and rotation, respectively. | Runtime |
| `rotation_scale` | `0.33` | `0.33` | `0.33` | `0.33` | Scale the relative mapped translation and rotation, respectively. | Runtime |
| `feedforward_scale` | `0.5` | `0.5` | `0.5` | `0.5` | Blend weight for native versus pose-derived twist. | Profile |
| `twist_filter_cutoff_hz` | — | — | — | — | Compatibility alias for old arm YAMLs. It supplies the velocity cutoff only when the explicit key is absent. | Review |
| `pose_filter` | `none` | `none` | `none` | `none` | none, low_pass or four_pole on the corresponding channel. | Profile |
| `velocity_filter` | `low_pass` | `low_pass` | `low_pass` | `low_pass` | none, low_pass or four_pole on the corresponding channel. | Profile |
| `pose_interpolation` | `none` | `none` | `none` | `none` | none, ptp_polynomial or spline on the corresponding channel. | Profile |
| `velocity_interpolation` | `none` | `none` | `none` | `none` | none, ptp_polynomial or spline on the corresponding channel. | Profile |
| `pose_filter_cutoff_hz` | `10.0` | `5.0` | `10.0` | `10.0` | Cutoff for the corresponding channel. Velocity falls back to the legacy key. | Profile |
| `velocity_filter_cutoff_hz` | `5.0` | `5.0` | `10.0` | `10.0` | Cutoff for the corresponding channel. Velocity falls back to the legacy key. | Profile |
| `pose_interpolation_duration_ms` | `40.0` | `40.0` | `40.0` | `40.0` | Transition duration for each interpolated channel. | Profile |
| `velocity_interpolation_duration_ms` | `40.0` | `40.0` | `40.0` | `40.0` | Transition duration for each interpolated channel. | Profile |
| `command_timeout_ms` | `100` | `100` | `100` | `100` | Phase-specific sample age, clock uncertainty and future tolerance. The arm owner also receives these settings. | Profile |
| `max_clock_uncertainty_ms` | `10.0` | `10.0` | `10.0` | `10.0` | Phase-specific sample age, clock uncertainty and future tolerance. The arm owner also receives these settings. | Profile |
| `max_source_future_ms` | `10.0` | `10.0` | `10.0` | `10.0` | Phase-specific sample age, clock uncertainty and future tolerance. The arm owner also receives these settings. | Profile |
| `min_derivative_dt_ms` | `1.0` | `1.0` | `1.0` | `1.0` | Allowed OpenXR interval for pose differencing. | Profile |
| `max_derivative_dt_ms` | `100.0` | `100.0` | `100.0` | `100.0` | Allowed OpenXR interval for pose differencing. | Profile |
| `trigger_engage` | `0.75` | `0.75` | `0.75` | `0.75` | Trigger hysteresis thresholds. The arm owner also receives the release threshold. | Profile |
| `trigger_release` | `0.25` | `0.25` | `0.25` | `0.25` | Trigger hysteresis thresholds. The arm owner also receives the release threshold. | Profile |

The standalone routing files use the same arm schema. `arms.yaml` provides one right arm with an empty IP. `arms_two_mock.yaml` provides left and right mock routes. Both use identity calibration. The two-arm file inherits unit motion scales.

**controller and arm_settings.<arm>.controller**

| Key | Single | Dual shared | Standalone | Effect or constraint | Location or review status |
| --- | --- | --- | --- | --- | --- |
| `dynamics_model_file` | `package://teleop_controllers/config/panda_umi.urdf` | `package://teleop_controllers/config/panda_umi.urdf` | `package://teleop_controllers/config/panda_oscbf.urdf` | Select the URDF model resource shared by the torque law and CBF. Both dual arms inherit the shared model. | Runtime |
| `cartesian_control_law` | `cartesian_pd` | `cartesian_pd` | `cartesian_pd` | Select cartesian_pd or inverse_dynamics. | Runtime |
| `inverse_dynamics_stiffness` | `[750.0, 750.0, 750.0, 75.0, 75.0, 75.0]` | `[750.0, 750.0, 750.0, 75.0, 75.0, 75.0]` | `[1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0]` | Task stiffness and modal damping ratio, then seven-joint posture gains for the inverse-dynamics law. | Profile |
| `inverse_dynamics_damping_ratio` | `1.0` | `1.0` | `1.0` | Task stiffness and modal damping ratio, then seven-joint posture gains for the inverse-dynamics law. | Profile |
| `inverse_dynamics_nullspace_stiffness` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | Task stiffness and modal damping ratio, then seven-joint posture gains for the inverse-dynamics law. | Profile |
| `inverse_dynamics_nullspace_damping` | `[6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759]` | `[6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759]` | `[6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759, 6.324555320336759]` | Task stiffness and modal damping ratio, then seven-joint posture gains for the inverse-dynamics law. | Profile |
| `torque_limit_scale` | `0.35` | `0.35` | `0.35` | Derating factors in (0,1]. The owner intersects scaled Panda ceilings with the configured absolute limits. | Runtime |
| `torque_rate_limit_scale` | `0.35` | `0.35` | `0.35` | Derating factors in (0,1]. The owner intersects scaled Panda ceilings with the configured absolute limits. | Runtime |
| `home_joint_position` | `[0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]` | `[0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]` | `[0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]` | Seven joint coordinates shared by homing and posture control. | Runtime |
| `nullspace_stiffness` | `10.0` | `10.0` | `10.0` | Tune Cartesian PD posture, joint-limit repulsion and posture torque ramp. Posture control is always active in both laws. | Profile |
| `nullspace_damping` | `6.324555320336759` | `6.324555320336759` | `6.324555320336759` | Tune Cartesian PD posture, joint-limit repulsion and posture torque ramp. Posture control is always active in both laws. | Profile |
| `nullspace_joint_limit_gain` | `2.0` | `2.0` | `2.0` | Tune Cartesian PD posture, joint-limit repulsion and posture torque ramp. Posture control is always active in both laws. | Profile |
| `nullspace_joint_limit_margin` | `0.35` | `0.35` | `0.35` | Tune Cartesian PD posture, joint-limit repulsion and posture torque ramp. Posture control is always active in both laws. | Profile |
| `nullspace_max_torque` | `5.0` | `5.0` | `5.0` | Tune Cartesian PD posture, joint-limit repulsion and posture torque ramp. Posture control is always active in both laws. | Profile |
| `nullspace_ramp_seconds` | `0.25` | `0.25` | `0.25` | Tune Cartesian PD posture, joint-limit repulsion and posture torque ramp. Posture control is always active in both laws. | Profile |
| `reference_shaping_enabled` | `false` | `false` | `false` | Enable and tune translational and rotational reference speed and acceleration limits. | Profile |
| `tracking_protection_enabled` | `false` | `false` | `false` | Enable tracking-error protection and set position/orientation thresholds. | Profile |
| `cartesian_wrench_limits_enabled` | `false` | `false` | `false` | Enable Cartesian force and moment caps and set their values. | Profile |
| `reference_linear_speed` | `0.05` | `0.05` | `0.05` | Enable and tune translational and rotational reference speed and acceleration limits. | Profile |
| `reference_angular_speed` | `0.15` | `0.15` | `0.15` | Enable and tune translational and rotational reference speed and acceleration limits. | Profile |
| `reference_linear_acceleration` | `0.15` | `0.15` | `0.15` | Enable and tune translational and rotational reference speed and acceleration limits. | Profile |
| `reference_angular_acceleration` | `0.5` | `0.5` | `0.5` | Enable and tune translational and rotational reference speed and acceleration limits. | Profile |
| `tracking_position_limit` | `0.03` | `0.03` | `0.03` | Enable tracking-error protection and set position/orientation thresholds. | Profile |
| `tracking_orientation_limit` | `0.2` | `0.2` | `0.2` | Enable tracking-error protection and set position/orientation thresholds. | Profile |
| `cartesian_force_limit` | `10.0` | `10.0` | `10.0` | Enable Cartesian force and moment caps and set their values. | Profile |
| `cartesian_moment_limit` | `1.5` | `1.5` | `1.5` | Enable Cartesian force and moment caps and set their values. | Profile |
| `lambda_translation` | `15.0` | `15.0` | `50.0` | Translation/rotation feedback gains for Cartesian PD. | Profile |
| `lambda_rotation` | `15.0` | `15.0` | `40.0` | Translation/rotation feedback gains for Cartesian PD. | Profile |
| `gain_translation` | `50.0` | `50.0` | `20.0` | Translation/rotation feedback gains for Cartesian PD. | Profile |
| `gain_rotation` | `5.0` | `5.0` | `2.5` | Translation/rotation feedback gains for Cartesian PD. | Profile |
| `hold_stiffness` | `[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 75.0]` | `[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 75.0]` | `[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 75.0]` | Joint hold gains and stiffness ramp following startup or trigger release. | Profile |
| `hold_damping` | `[50.0, 50.0, 50.0, 50.0, 20.0, 15.0, 10.0]` | `[50.0, 50.0, 50.0, 50.0, 20.0, 15.0, 10.0]` | `[50.0, 50.0, 50.0, 50.0, 20.0, 15.0, 10.0]` | Joint hold gains and stiffness ramp following startup or trigger release. | Profile |
| `hold_stiffness_ramp_seconds` | `0.25` | `0.25` | `0.25` | Joint hold gains and stiffness ramp following startup or trigger release. | Profile |
| `maximum_torque_rate` | `900.0` | `900.0` | `900.0` | Absolute slew and seven-joint torque ceilings. They can tighten the derated ceilings. | Profile |
| `torque_limits` | `[87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]` | `[87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]` | — | Absolute slew and seven-joint torque ceilings. They can tighten the derated ceilings. | Profile |
| `homing_speed_factor` | `0.15` | `0.15` | `0.15` | Homing trajectory speed, completion timeout, controlled-stop settings and joint PD gains. | Profile |
| `homing_timeout_seconds` | `30.0` | `30.0` | — | Homing trajectory speed, completion timeout, controlled-stop settings and joint PD gains. | Profile |
| `homing_stop_duration_seconds` | `0.5` | `0.5` | — | Homing trajectory speed, completion timeout, controlled-stop settings and joint PD gains. | Profile |
| `homing_stop_acceleration` | `0.75` | `0.75` | — | Homing trajectory speed, completion timeout, controlled-stop settings and joint PD gains. | Profile |
| `homing_stop_timeout_seconds` | `5.0` | `5.0` | — | Homing trajectory speed, completion timeout, controlled-stop settings and joint PD gains. | Profile |
| `homing_stiffness` | `[300.0, 300.0, 300.0, 300.0, 120.0, 100.0, 50.0]` | `[300.0, 300.0, 300.0, 300.0, 120.0, 100.0, 50.0]` | — | Homing trajectory speed, completion timeout, controlled-stop settings and joint PD gains. | Profile |
| `homing_damping` | `[30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0]` | `[30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0]` | — | Homing trajectory speed, completion timeout, controlled-stop settings and joint PD gains. | Profile |

**hand and arm_settings.<arm>.hand**

| Key | Single | Dual | Standalone | Effect or constraint | Location or review status |
| --- | --- | --- | --- | --- | --- |
| `joystick_engage` | `0.4` | `0.4` | `0.4` | Analog direction hysteresis. Joystick neutral interrupts unfinished movement. | Profile |
| `joystick_neutral` | `0.2` | `0.2` | `0.2` | Analog direction hysteresis. Joystick neutral interrupts unfinished movement. | Profile |
| `close_width_m` | `0.0` | `0.0` | `0.0` | Closing/opening targets in metres. Open width −1 uses the calibrated maximum. | Profile |
| `cable_diameter_m` | `0.01` | `0.01` | `0.005` | Installation-specific expected grasp gap, tolerance and force. Force −1 disables closing. | Runtime |
| `cable_diameter_tolerance_m` | `0.0099` | `0.0099` | `0.002` | Installation-specific expected grasp gap, tolerance and force. Force −1 disables closing. | Runtime |
| `grasp_force_n` | `30.0` | `30.0` | `-1.0` | Installation-specific expected grasp gap, tolerance and force. Force −1 disables closing. | Runtime |
| `worker_poll_ms` | `1.0` | `1.0` | `1.0` | Hand worker polling interval. | Profile |
| `open_speed_mps` | `0.07` | `0.07` | `0.1` | Opening and closing speeds. | Profile |
| `close_speed_mps` | `0.03` | `0.03` | `0.02` | Closing speed. | Profile |
| `open_width_m` | `-1.0` | `-1.0` | `-1.0` | Closing/opening targets in metres. Open width −1 uses the calibrated maximum. | Profile |
| `command_timeout_ms` | `100.0` | `100.0` | — | Hand-specific freshness and clock uncertainty gates. | Profile |
| `max_clock_uncertainty_ms` | `10.0` | `10.0` | — | Hand-specific freshness and clock uncertainty gates. | Profile |
| `mock_operation_duration_ms` | `100.0` | `100.0` | — | Mock-only actuation duration and simulated object width. | Profile |
| `mock_object_width_m` | — | — | `0.0` | Mock-only actuation duration and simulated object width. | Profile |

**recording**

| Key | Single | Dual | Standalone | Effect or constraint | Location or review status |
| --- | --- | --- | --- | --- | --- |
| `mode` | `menu` | `menu` | — | menu or continuous. The standalone launch uses recording_mode. | Runtime |
| `namespace` | `/quest` | `/quest` | — | Recorder ROS namespace. The standalone launch uses recording_namespace. | Derived |
| `left_controller_state_topic` | `""` | `""` | — | Menu input override. Empty uses the configured left input or bridge route. | Derived |
| `include_default_topics` | `true` | `true` | `true` | Include the generated core topic list. | Runtime |
| `topics` | `[]` | `[]` | `[]` | Append explicit topics, or provide the full list when defaults are off. | Runtime |
| `output_root` | `/ws/log/quest_bags` | `/ws/log/quest_bags` | `/ws/log/quest_bags` | Absolute output directory in the ROS container. | Runtime |
| `storage_id` | `sqlite3` | `sqlite3` | `sqlite3` | Menu supports only sqlite3. Continuous forwards the plugin name to rosbag2. | Review |
| `menu_debounce_seconds` | `0.35` | `0.35` | — | Menu press debounce. Unified configuration requires at least 0.35 s. | Profile |

**cbf and arm_settings.<arm>.cbf**

| Key | Single | Dual | Fallback | Effect or constraint | Location or review status |
| --- | --- | --- | --- | --- | --- |
| `enabled` | `true` | `true` | `false` | Enable the linked CBF filter for selected arms. | Runtime |
| `config_file` | `package://cbf_safety_filter/config/single_panda/cbf.yaml` | `package://cbf_safety_filter/config/dual_panda/cbf.yaml` | `""` | Required when enabled. Absolute path or package://cbf_safety_filter resource. | Runtime |
| `hardware_mode` | `filtered` | `filtered` | — | shadow or filtered. Enabled hardware CBF requires an explicit choice. Dual arms must match. Mock defaults to filtered. | Runtime |

Per-arm overrides use `arm_settings.<arm>.<section>.<key>` for controller, Hand and CBF only. Each section uses the same keys as the global section and overrides it per arm. The dual YAML selects `panda_umi.urdf` for both arms in the shared `controller` section. A runtime document must provide a nonempty `arms` mapping. Feature-only controller or Hand documents can also contain `arm_settings` and the other validated feature mappings.

## Referenced CBF profiles

The CBF rows show the selected single and dual profiles. Their current settings are compared as saved. A dash can mark an accepted optional field, a policy-specific field, or a field absent when another selector is used.

| Key | Single | Dual | Effect or constraint | Location or review status |
| --- | --- | --- | --- | --- |
| `extends` | — | — | Optional parent YAML relative to the extending file. Maps merge recursively. Cycles and excessive depth are rejected. | Profile |
| `robot.gravity_command_convention` | `externally_gravity_compensated` | `externally_gravity_compensated` | Required schema identifiers or fixed assumptions. | Fixed |
| `robot.joints` | `7` | `7` | Required schema identifiers or fixed assumptions. | Fixed |
| `robot.model` | `panda` | `panda` | Required schema identifiers or fixed assumptions. | Fixed |
| `execution.analytic_nominal` | `true` | `true` | Enable verified analytic shortcuts for feasible nominal or simple rows. | Profile |
| `execution.mode` | `synchronous` | `synchronous` | Required fixed execution mode; synchronous is the only supported value. | Fixed |
| `execution.fallback_strategy` | `bounded_recovery` | `bounded_recovery` | Required fixed recovery strategy; bounded_recovery is the only supported value. | Fixed |
| `execution.max_callback_time_us` | `800` | `800` | Filter/callback computation budgets and maximum model age. Hardware checks require callback budget at most 800 µs. | Guard |
| `execution.max_constraints` | `512` | `512` | Bound the preallocated hard-constraint row capacity. | Profile |
| `execution.max_filter_time_us` | `600` | `800` | Filter/callback computation budgets and maximum model age. Hardware checks require callback budget at most 800 µs. | Guard |
| `execution.max_model_age_us` | `2000` | `2000` | Filter/callback computation budgets and maximum model age. Hardware checks require callback budget at most 800 µs. | Guard |
| `execution.recovery_braking_gain_s_inv` | `20.0` | `20.0` | Braking anchor gain and maximum projection sweeps. | Profile |
| `execution.recovery_projection_sweeps` | `6` | `6` | Braking anchor gain and maximum projection sweeps. | Profile |
| `execution.safety_torque_computation_method` | `native_qp` | `projection` | projection or native_qp for bounded recovery. | Profile |
| `objective.joint_weights` | `[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]` | `[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]` | Seven positive coefficients in the weighted squared distance from nominal torque: 1/2 Σⱼ wⱼ(uⱼ − u_nominal,ⱼ)². | Profile |
| `objective.type` | `torque_deviation` | `torque_deviation` | Required schema identifiers or fixed assumptions. | Fixed |
| `qp.formulation` | `hard` | `hard` | Required fixed QP formulation; hard constraints are the only supported formulation. | Fixed |
| `solver.backend` | `qpoases` | `qpoases` | Selects the numerical QP algorithm. | Profile |
| `solver.osqp.absolute_tolerance` | `1e-05` | `1e-05` | Absolute component of OSQP primal/dual convergence thresholds; smaller is stricter. | Profile |
| `solver.osqp.max_iterations` | `100` | `100` | Maximum iterations for one OSQP solve. | Profile |
| `solver.osqp.relative_tolerance` | `1e-05` | `1e-05` | Relative component of OSQP primal/dual convergence thresholds; smaller is stricter. | Profile |
| `solver.piqp.absolute_tolerance` | `1e-06` | `1e-06` | Absolute component of PIQP primal/dual convergence thresholds; smaller is stricter. | Profile |
| `solver.piqp.max_iterations` | `50` | `50` | Maximum iterations for one PIQP solve. | Profile |
| `solver.piqp.relative_tolerance` | `1e-06` | `1e-06` | Relative component of PIQP primal/dual convergence thresholds; smaller is stricter. | Profile |
| `solver.qpoases.max_working_set_recalculations` | `100` | `100` | Maximum active-set updates allowed in one qpOASES solve. | Profile |
| `solver.qpoases.termination_tolerance` | `1e-06` | `1e-06` | qpOASES stopping threshold for the optimization. | Profile |
| `solver.warm_start` | `true` | `true` | Reuse prior solver state when the selected backend supports it. | Profile |
| `verification.hard_row_tolerance` | `1e-06` | `1e-06` | Post-solve threshold for hard-row and QP box residuals; it is not a safety margin. | Guard |
| `constraints.self_collision.boundary_activation_m` | `0.03` | `0.03` | Clearance band used by conservative self-collision group pruning; does not change collision clearance. | Profile |
| `constraints.inter_arm_collision.alpha1` | — | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.inter_arm_collision.alpha2` | — | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.inter_arm_collision.clearance_m` | — | `0.01` | Geometric clearance for the named family. | Profile |
| `constraints.inter_arm_collision.enabled` | — | `true` | Enable the named constraint family. | Profile |
| `constraints.inter_arm_collision.max_peer_age_us` | — | `10000` | Peer snapshot age and time skew limits. | Guard |
| `constraints.inter_arm_collision.max_peer_skew_us` | — | `10000` | Peer snapshot age and time skew limits. | Guard |
| `constraints.inter_arm_collision.point_acceleration_limit_m_s2` | — | `5.0` | Peer point-motion bounds for applicable policies. | Profile |
| `constraints.inter_arm_collision.point_speed_limit_m_s` | — | `1.0` | Peer point-motion bounds for applicable policies. | Profile |
| `constraints.inter_arm_collision.policy` | — | `independent` | Explicit dual-profile selector. The only supported value is `independent`. | Profile |
| `constraints.inter_arm_collision.reference_activation_m` | — | `0.015` | Distance band used by conservative inter-arm group pruning; does not change collision clearance. | Profile |
| `constraints.joint_acceleration.enabled` | `true` | `true` | Enable the named constraint family. | Profile |
| `constraints.joint_acceleration.limits_rad_s2` | `[15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]` | `[15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]` | Per-joint acceleration bounds used by the torque constraints and bounded braking recovery. | Profile |
| `constraints.joint_position.alpha1` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.joint_position.alpha2` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.joint_position.enabled` | `true` | `true` | Enable the named constraint family. | Profile |
| `constraints.joint_position.lower_limits_rad` | `[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]` | Same | Lower joint position limits, in joint order. | Profile |
| `constraints.joint_position.safety_margin_rad` | `0.1` | `0.1` | Inward buffer from each lower and upper joint position limit. | Profile |
| `constraints.joint_position.upper_limits_rad` | `[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]` | Same | Upper joint position limits, in joint order. | Profile |
| `constraints.joint_velocity.alpha` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.joint_velocity.enabled` | `true` | `true` | Enable the named constraint family. | Profile |
| `constraints.joint_velocity.maximum_limits_rad_s` | `[2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]` | Same | Per-joint absolute velocity limits, in joint order. | Profile |
| `constraints.joint_velocity.safety_margin_rad_s` | `0.1` | `0.1` | Buffer subtracted from each absolute joint velocity limit. | Profile |
| `constraints.obstacles.alpha1` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.obstacles.alpha2` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.obstacles.clearance_m` | `0.05` | `0.05` | Geometric clearance for the named family. | Profile |
| `constraints.obstacles.enabled` | `false` | `false` | Enable the named constraint family. | Profile |
| `constraints.self_collision.alpha1` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.self_collision.alpha2` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.self_collision.clearance_m` | `0.01` | `0.01` | Geometric clearance for the named family. | Profile |
| `constraints.self_collision.enabled` | `true` | `true` | Enable the named constraint family. | Profile |
| `constraints.singularity.alpha1` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.singularity.alpha2` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.singularity.characteristic_length_m` | `0.3` | `0.3` | Length used to scale translational Jacobian rows relative to rotational rows. | Profile |
| `constraints.singularity.enabled` | `true` | `true` | Enable the named constraint family. | Profile |
| `constraints.singularity.minimum_normalized_sigma` | `0.025` | `0.025` | Minimum threshold for the normalized Jacobian singularity measure. | Profile |
| `constraints.table.alpha1` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.table.alpha2` | `10.0` | `10.0` | First- or second-order barrier gain. | Profile |
| `constraints.table.clearance_m` | `0.0005` | `0.0005` | Geometric clearance for the named family. | Profile |
| `constraints.table.enabled` | `true` | `true` | Enable the named constraint family. | Profile |
| `constraints.table.height_m` | `0.127` | `0.127` | Surveyed world-frame table height. Keep visible in installation-specific safety profile. | Profile |
| `geometry.file` | `geometry_panda_umi.yaml` | — | Select a single cover or both hand-indexed covers. Relative paths are relative to the profile that declares them. | Profile |
| `geometry.files_by_controller.left` | — | `geometry.left.yaml` | Select a single cover or both hand-indexed covers. Relative paths are relative to the profile that declares them. | Profile |
| `geometry.files_by_controller.right` | — | `geometry.right.yaml` | Select a single cover or both hand-indexed covers. Relative paths are relative to the profile that declares them. | Profile |
| `diagnostics.enabled` | `true` | `true` | Select diagnostics and recorded fields. Hardware CBF commissioning requires time and residual diagnostics. | Guard |
| `diagnostics.record_computation_time` | `true` | `true` | Select diagnostics and recorded fields. Hardware CBF commissioning requires time and residual diagnostics. | Guard |
| `diagnostics.record_constraint_residuals` | `true` | `true` | Select diagnostics and recorded fields. Hardware CBF commissioning requires time and residual diagnostics. | Guard |
| `failure_policy.deadline_overrun` | `request_existing_protective_handling` | `request_existing_protective_handling` | Required policy identifiers. The parser accepts only the shown values. | Fixed |
| `failure_policy.infeasible_qp` | `request_existing_protective_handling` | `request_existing_protective_handling` | Required policy identifiers. The parser accepts only the shown values. | Fixed |
| `failure_policy.invalid_model` | `terminate_control` | `terminate_control` | Required policy identifiers. The parser accepts only the shown values. | Fixed |
| `failure_policy.solver_failure` | `request_existing_protective_handling` | `request_existing_protective_handling` | Required policy identifiers. The parser accepts only the shown values. | Fixed |

**Referenced collision geometry YAML**

| Key | Role | Location or review status |
| --- | --- | --- |
| `version` | Required value 1. | Fixed |
| `world_from_base_xyz_xyzw` | World-from-base translation and unit quaternion. Surveyed, per-arm installation data. | Profile |
| `capsules[].frame / a / b / radius_m` | Link frame, segment endpoints and radius for the separate table-capsule representation. | Profile |
| `points[].frame / offset / radius_m` | Frame, local offset and radius for configured point checks. | Profile |
| `body_spheres[].frame / offset / radius_m / group` | Link frame, local center, radius and collision group for the sphere representation. | Profile |
| `body_volumes[].frame / offset / radius_m / group / end / opening_axis` | Unified sphere or capsule cover. Optional endpoint and finger-opening axis. The parser includes the optional members in the shared body schema and restricts nonzero spans and axes to body_volumes. | Profile |
| `self_exclusions[]` | Pair of group IDs excluded from self-collision checks. | Profile |
| `obstacles[].center / radius_m` | World-frame static obstacle spheres. | Profile |
| `pairs[].point / obstacle` | Indices selecting a point-to-obstacle pair. | Profile |

## Decisions with direct implementation evidence

| Candidate | Evidence and decision to make |
| --- | --- |
| Legacy twist alias | New profiles and the standalone mapper launch use velocity_filter_cutoff_hz. Old arm YAMLs can still supply a missing velocity cutoff via twist_filter_cutoff_hz. |
| Retired Hand keys removed | close_retry_interval_ms, epsilon_inner_m, epsilon_outer_m, grasp_width_m, grip_click_max_ms, grip_debounce_ms, grip_double_click_ms, grip_press, grip_release had no runtime effect. Remove these keys from old launch profiles before reusing those files. |
| Legacy controller_enabled field | The runtime and standalone arm templates omit it. The validator accepts false in older arm YAMLs. |
| Consolidate topic/default duplication | Combined bringup already derives raw topics and normalized arm topics. Keep custom routes as exceptions. The direct mapper standalone default uses the aggregate raw topic while launch routing derives per-hand topics. |
| Review the homing wrapper | It includes the arm controller launch with homing enabled, which is already the controller launch default. Keep the discoverable entry point if operators use it, or document the main controller launch as its replacement. |
| Keep actuator limits and CBF guard values explicit in resolved settings | Absolute torque/rate limits can narrow the scaled limits. CBF time budgets and hard-row limits affect whether a command can be issued. Verify deployment profiles preserve these semantics. |
| Limit storage choice to the relevant recording mode | Menu rejects all plugins except sqlite3. Continuous accepts a plugin name and forwards it to rosbag2. Consolidating the menu-only presentation need not remove continuous plugin selection. |
| Keep policy-specific CBF keys out of the top-level runtime | The native parser accepts only the parameters used by the synchronous hard-constraint filter. Bounded recovery is fixed behavior, and `independent` is the only inter-arm policy. |

This inventory was checked against the argument declarations in all nine launch files, the Python validators in teleop_bringup, the package YAMLs, and the native CBF schema in `cbf_safety_filter/src/configuration.cpp`. Generated node overrides such as side-prefixed owner parameters, synthesized recorder topic JSON and the fixed startup calibration flag are launch implementation details. Node-only knobs such as `mock_realtime`, synthetic pose coordinates, generic ROS CLI switches and DDS environment variables are outside the launch-facing schema.
