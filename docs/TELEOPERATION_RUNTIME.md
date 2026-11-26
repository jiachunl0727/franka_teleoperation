# Integrated runtime and robot-free checks

Overall bringup requires `runtime_config` as its sole launch argument.
Package-local configs remain the defaults
for standalone features. Configuration/editing and operational commands are in
[README.md](../README.md); schema and ownership are in
[PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md).

For these checks, set `pipeline.backend: mock` in the selected file first; the
current single-arm working configuration selects hardware. Mock state is fixed
and does not simulate physical dynamics, clearance or realtime timing.
Reuse the validated APK/USB setup. Stop earlier launches before starting another
owner/mapper/bridge on the same namespace or port.

## Launch selection

In a sourced ROS container at `/ws`, select single or dual explicitly:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda/single_panda.yaml
```

Stop that launch before using the dual template:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/dual_panda/dual_panda.yaml
```

Set single-left selection in the single file's `pipeline.single_controller`.
Set bridge/Hand/recording/home switches in `pipeline`, not command arguments.
The enabled Hand calibrates its empty fingers at startup. Configure `hand.speed_mps`
for closing, `open_speed_mps` for opening, and `close_width_m`/`open_width_m` for
travel. All widths are full gaps between the fingers.


## Observe behavior

In a second sourced container shell, run one observer at a time:

```bash
ros2 topic echo /panda_1/controller_status
ros2 topic echo /panda_1/cartesian_teleop_command
ros2 topic echo /panda_1/homing_status
ros2 topic echo /panda_1/gripper_events
ros2 topic echo /quest/recording_status
```

For dual substitute `/panda_left` or `/panda_right` for `/panda_1`.

Startup logs distinguish three separate stages for each arm:

- `FCI startup ... mode="Idle"` confirms a state read before requesting torque
  control. It does not confirm that torque callbacks have started.
- `Arm control ready: accepted torque cycle=...` confirms that the arm produced
  a current-cycle, safety-checked torque and measured-state telemetry. An arm
  that still has no accepted cycle after five seconds reports a warning with
  `control_requested`; this is diagnostic only and does not clear robot faults
  or restart control.
- `Hand calibration ready: robot_ip=... maximum_gap_m=...` confirms that the
  Hand's calibration command completed successfully with valid readback.
  Center the assigned joystick after this message, then deflect inward to close.

These stages are independent of Quest engagement. Arm startup holds the
current configuration; it does not automatically move the arm home. The
`Quest input`, `Mapped input` and `Hand input` log lines report changes in input
availability and release gates, limited to one line per type every 500 ms.
`Hand input: neutral_required=1` means a fresh available joystick sample
inside the neutral dead zone is required, including after calibration. A missing
joystick action cannot provide that sample. Arm engagement uses the trigger
and additionally needs fresh tracked pose input.

Desk's Hand panel may show **end-effector not connected** during FCI control.
[Franka's maintainers describe this as expected](https://github.com/frankarobotics/franka_ros/issues/118):
Desk's control path and FCI are distinct. Use the worker's calibration-ready
message and `gripper_events` to diagnose our Hand connection. A successful
firmware `Hand result` followed by a software rejection is a different fault;
it now produces an immediate error with raw width, calibrated maximum width,
endpoint excess and communication status. A fixed **10 micrometre** readback
allowance accepts small endpoint discrepancies and clamps the internal width
to `[0, maximum_width]`. This is a software policy, not a manufacturer accuracy
specification. It never increases commanded travel. Nonfinite values, invalid
maximum width, larger discrepancies and communication failures still require
explicit calibration; none automatically reconnect or retry calibration.

Measured-pose feedback and Quest callbacks run separately from bulk status
publication. `teleop_anchor_pose` publishes at up to 100 Hz with **best-effort,
depth-one QoS**; the mapper requests the same QoS. A slow status subscriber must
not starve these callbacks. Other custom subscribers to the anchor topic must
request best-effort reliability, for example:

```bash
ros2 topic echo /panda_right/teleop_anchor_pose --qos-reliability best_effort
```

The 100 ms input/pose freshness gates remain enforced. Large `source_age_ms`
values indicate delivery delay; increasing stiffness will not clear this gate.
The full-cycle tracking stream retains its separate bounded queue and publisher.

For an unresponsive right arm, in a sourced **ROS container terminal**:

```bash
timeout 3s ros2 topic echo /panda_right/controller_status --once
timeout 3s ros2 topic echo /panda_right/gripper_events --once --qos-durability transient_local
timeout 3s ros2 topic echo /quest_usb_bridge/right/controller_state --once
```

A controller status sample confirms callback output; no sample means input
gating alone has not been established as the cause. Inspect the readiness
messages before changing gains or bypassing release gates. `calibrated: true`
with a nonzero maximum width is the Hand's reported calibration result;
absence of later close/open requests does not itself prove calibration failed.

- Fresh trigger release/press enables only the assigned arm. Input loss, focus
  loss, recenter and USB disconnect disable motion; held trigger cannot resume
  until a valid release/press. The first target anchors at the measured pose.
- Normal trigger release captures all seven measured joint positions once and
  selects `reason: position_hold` with `motion_enabled: false`. The reference
  stays fixed until re-engagement, homing or a fault. Stiffness ramps over
  `hold_stiffness_ramp_seconds` (default 0.25 s); `hold_stiffness` and
  `hold_damping` are seven-element joint gains. Startup and pauses retain a
  fixed local joint anchor even with missing/invalid/stale Quest input. If input
  fails during engagement, capture measured joints once and hold there instead
  of pursuing the old Quest target. Repeated faults and release messages cannot
  move that anchor or reset its stiffness ramp. Fresh release/re-engagement is
  still required. Safety fallback, invalid-state termination, homing priority
  and shutdown retain their separate stop behavior.
  The mock is fixed, so restoring torque under displacement is verified in
  controller tests rather than inferred from mock motion.
- With trigger released, hold right A/left X at least one second then release.
  Only its assigned arm requests home, using `controller.home_joint_position`.
  The fixed mock is at the default home; a different configured home is not
  simulated as physical motion. Trigger/input loss cancels through stopping with no restart.
- Configure Hand force/cable settings, then wait for calibration and center the
  assigned joystick. Right stick left / left stick right closes at fixed speed;
  the opposite direction opens. Neutral stops unfinished motion. Once a cable
  grasp is confirmed, neutral and Quest input loss preserve its force hold.
  After input loss, center the available joystick before accepting new motion.
- Fresh left Menu presses toggle unique recording episodes, even in single-right
  mode. Holding does not repeat; observe recording then idle/bag finalized.
  Input loss leaves a started bag running; normal shutdown finalizes it.
  Homing does not disable recording. Release Menu, wait for `idle`, then press
  again to start another episode. Recorder completion is collected before
  interpreting a fresh press, so an already exited bag cannot consume it against
  a stale `stopping` phase. A press while finalization is actually still running
  is reported and requires a new press after completion.

During trigger-enabled Cartesian/orientation-only control, a dynamically
consistent nullspace term gently favors that same configured home posture and
repels joint limits. Configure its gains, cap and entry ramp under `controller`
(or `arm_settings.<arm>.controller`). Nullspace posture control is always active.
Torque magnitude/rate clipping belongs to `teleop_controllers`. No CBF,
singularity, collision or table constraint is currently executed. See
[the ownership contract](CONTROLLER_ARCHITECTURE.md).

## Hand joystick control and force holding

| Assigned controller | Close (inward) | Open (outward) |
| --- | --- | --- |
| Right | Stick left | Stick right |
| Left | Stick right | Stick left |

These are controller-local horizontal directions; rotating the controller does
not change their mapping. Vertical deflection, side grip and joystick click do
not actuate the Hand. The trigger still controls the arm independently.

Beyond `joystick_engage`, each direction selects a fixed finger speed.
Deflecting farther does not increase speed or select a width. Returning inside
`joystick_neutral` stops any unfinished closing or opening. Direction hysteresis
suppresses threshold chatter. Each gesture submits one motion toward the
configured endpoint: `grasp(close_width_m, speed_mps, force, ...)` for closing
or `move(open_width_m, open_speed_mps)` for opening. Holding continues that
firmware motion; commands are not divided into fixed-distance steps. The
step-based experiment and its position-completion tolerance were reverted.
SDK/firmware stop latency can add travel after neutral; this interface cannot guarantee an exact gap from elapsed time.

```yaml
hand:
  joystick_engage: 0.4
  joystick_neutral: 0.2
  speed_mps: 0.02
  open_speed_mps: 0.1
  open_width_m: -1.0                # Calibrated maximum, or a smaller opening gap.
  close_width_m: 0.0
  cable_diameter_m: 0.005           # Configure for the actual cable.
  cable_diameter_tolerance_m: 0.002 # Accepted compressed gap +/- this amount.
  grasp_force_n: -1.0              # Unset: provide a positive force before closing.
  worker_poll_ms: 1.0
```

All values are available in the package's `gripper.yaml`, combined bringup's
`hand` section and per-arm `arm_settings.<arm>.hand` overrides. Saved finger
speeds/opening widths are preserved. Configure the opening gap above the cable's
accepted width range and within calibrated travel. Cable tolerance must be
smaller than diameter, and its lower gap must exceed the closing limit.
The force is deliberately unset because the cable's permissible force is
unknown. Closing reports the missing setting; opening remains available.

Closing calls `franka::Gripper::grasp` at the configured fixed speed and force.
A zero target is supported: the SDK's outer tolerance includes the actual
cable width. Software additionally requires successful command completion,
`is_grasped=true`, and readback inside the configured cable diameter band.
An empty zero-width closure is not a confirmed cable grasp. An out-of-band or
otherwise unsuccessful grasp is stopped without an automatic retry or force
increase. The `is_grasped` flag is device grasp state, not a force sensor.

**Neutral during unfinished closure cancels it.** It does not finish acquiring
the cable after the operator centers the stick. If a grasp has completed,
neutral preserves the device's force hold. A completion/cancellation gate avoids
sending a redundant stop during final readback, which would release a grasp
that has already succeeded. An explicit outward command releases the confirmed
hold before opening. Neutral then stops any remaining opening travel.

**Quest input loss or app focus loss preserves a confirmed hold**, stops
unfinished finger motion, and requires a fresh neutral sample before accepting
new motion. An inactive/missing joystick, stale timestamp, sequence fault,
session/reference change, and timeout use the same rule. Cartesian pose
occlusion alone does not disable fresh joystick control. A new direction can
replace an unfinished motion only after its stop/command completes and only
while that direction remains current and fresh. Neutral/faults clear queued
intent. Closing requests are never periodically repeated in neutral or while held.

Normal worker shutdown stops unfinished motion and sends no release command
for an idle confirmed hold. Device faults, power loss and firmware behavior
remain outside this software hold policy. Restarting bringup with Hand enabled
still requests empty-hand calibration; prepare the fingers empty as required
by that existing startup procedure.

### Status and recovery

```bash
ros2 topic echo /panda_1/gripper_events --qos-durability transient_local
```

`state: holding` and `grasp_confirmed: true` indicate the last completed,
validated grasp. `commanded_grasp_force_n` is the commanded force, not a measured
force. Width/grasp state are completion snapshots rather than continuous slip
feedback. `neutral_required` exposes the recovery gate. There is no tactile
sensor or automatic slip-triggered force increase.

A Hand communication, invalid-state or stop failure clears calibration and
reports its cause in `message`; ordinary joystick movement cannot reconnect or
clear that fault. Once the worker is idle and the fingers are empty, recover
explicitly in the ROS terminal (use the affected arm namespace):

```bash
ros2 service call /panda_1/calibrate_gripper \
  quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}'
```

Wait for `calibrated: true`, center the joystick, then deflect to move. A physical
manual finger movement does not clear the software fault. Calibration remains
independent of Quest input. Endpoint readback retains its existing 10 micrometre
software allowance; this adds no commanded travel.

### Command timing and compatibility

Closing speed limits the full-travel grasp motion; it does not set stop-command
latency. Centering submits stop promptly, but the blocking SDK call includes
waiting for its shared TCP connection and the robot's acknowledgement. In the
2026-10-28 trial, stop-request input ages were about 1 ms while individual SDK
stop calls took 0.27–4.37 s. At 0.01 m/s, four seconds of continued motion would
add about 40 mm of closing travel. A slower closing speed does not make stop
instantaneous.

The original pinned SDK polls TCP responses while holding the same mutex used
to send requests. A running grasp's receiver can monopolize that mutex and delay
another thread's Stop send. The repository now applies a
[scoped SDK transport patch](../patches/README.md): Hand response waits poll
outside that mutex, and concurrent response lookup/copy/removal stays protected.
The arm connection keeps its existing polling cadence. Apply the patch and
rebuild **libfranka as well as its consumers**, then restart the launch; rebuilding
only `teleop_gripper` does not replace the SDK. The dependency-fetch script
applies the patch automatically for a fresh workspace.

`stop_call_ms` still includes robot acknowledgement and transport time, and does
not measure mechanical stopping. The loopback SDK regression checks that Stop
reaches a fake server before the outstanding motion completes; physical latency
must be checked in the next operator trial. The continuous full-travel Hand path
remains in place; fixed-distance stepping is not used as a workaround.


Quest samples the joystick each OpenXR frame and sends its signed axes plus
availability through USB. A best-effort subscription retains one latest sample, avoiding a reliable
backlog; the input callback dispatches neutral stops before diagnostic publication. Blocking `grasp`, `move`, `stop` and readback run
in independent worker threads, with one serialized motion and its cancellation.
Status/log publication remains outside the control lock. `worker_poll_ms`
controls completion collection/watchdog checks, not the SDK command frequency.
A submission gate cancels before issuing motion if neutral arrives during
initial readback or release of an existing grasp. Bounded stop retries handle
cancellation overtaking an already committed command; they never initiate a
new close or increase force.

`Hand result` logs retain `command_ms`, `readback_ms`, `stop_completion_ms`,
`stop_call_ms`, `stop_calls`, `stop_acknowledged`, `stop_retry_exhausted` and
`grasp_confirmed`. The stop retry window cannot interrupt a blocking SDK call
or guarantee mechanical latency. A failed stop/transport result requires
explicit recovery; it is not hidden by an earlier acknowledgement.

Old side-grip/click/retry settings are accepted with a retirement warning but
have no effect. `grasp_force_n` is active again; `grasp_width_m` and the old
`epsilon_inner_m`/`epsilon_outer_m` keys are replaced by the cable diameter band.
Older APKs that omit axes remain valid for arm control but cannot provide a
neutral sample or actuate the Hand. Rebuild/install the updated APK and rebuild
all active ROS message consumers before restarting the pipeline.

## Startup and stop diagnostics

| Symptom | Meaning and next check |
| --- | --- |
| Missing `runtime_config` | Pass the complete selected YAML path; combined bringup has no default file. |
| Hardware address missing | Each selected hardware arm needs a configured `robot_ip`. For software tests, select the mock backend in YAML. |
| `FCI torque control requires Idle` | Startup feedback reports an active/faulted mode or current errors. Finish the other motion/control session and restore healthy Idle in Desk. The runtime refuses a competing torque-control request and does not recover errors automatically. Historical `last_motion_errors` alone do not block healthy Idle startup. |
| `sequence size exceeds remaining buffer` across unrelated nodes | DDS discovery/serialization traffic, not evidence of a torque or Hand fault. Re-source the updated workspace in every terminal and use its isolated domain/local discovery defaults; see below. |
| `Move command rejected` with mode `Reflex` | The robot rejected entry into torque control. Inspect the startup `current_errors` and `last_motion_errors`, and restore readiness through the operator's normal robot procedure. The runtime does not recover errors automatically. |
| `reason: position_hold`, `motion_enabled: false` | Local joint holding is active at startup or during a pause, including ordinary Quest input faults. Fresh release/re-engagement is required for teleoperation. |
| Input fault or stale command | Check the raw input, mapped command, clock validity and bridge connection; recovery requires a fresh valid trigger release/press. |
| No saved trajectory report data | A live PlotJuggler window does not record topics. Start/finalize an episode and check that it includes `tracking_samples`; see [TRAJECTORY_MONITORING.md](TRAJECTORY_MONITORING.md). |
| Menu does not start the next episode | Inspect `/quest/recording_status` and the `Recording` / `Menu press` log lines. They distinguish finalization, missing publishers, rejected input and an active recording. Homing does not suppress Menu. |
| Only small movements or intermittent response | Compare `Mapper state` and `Motion state` lines: source tracking/clutch readiness, measured-pose freshness, requested versus enabled motion, homing, target error and torque norms are reported separately. A ready FCI connection does not imply active Cartesian tracking. |
| Gripper responds but arm remains still | Arm motion requires the assigned **index trigger**, independently of joystick gripper input. Check `Mapped input: motion_requested=1`. A valid `trigger released` state keeps position holding. |

Controller or Hand process failure stops combined bringup. Keep its terminal
output when investigating a failure. Parameters are loaded at startup; edit
the selected YAML and restart to apply changes.

The launch log may contain only the process exit and shutdown sequence. The
controller's full ROS log is stored separately in the same container, for
example `/root/.ros/log/cartesian_pd_controller_PID_TIMESTAMP.log`.
Read the first `Controller stopped:` message and its timing
lines to identify the initiating failure; subsequent Hand stop requests and
Quest disconnection are expected bringup cleanup.

The recorder's status JSON includes `input_reason` and `release_seen`. Missing
left Menu availability, focus/clock/input loss or order faults still require a
fresh valid release before a new press; these gates are not bypassed by homing.
Recording phase changes and rejected presses are also printed in the launch
terminal. If bag finalization ignores SIGINT for 10 seconds, the recorder sends
SIGTERM, then SIGKILL after another 3 seconds if needed. This prevents permanent
`stopping` lockout; a forcibly terminated/unfinalized bag remains an error and
must be inspected before analysis. No new episode is queued during finalization.
