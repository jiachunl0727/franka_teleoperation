# Franka Panda + Quest 3 teleoperation

Run the Quest 3 USB app and a single or dual **legacy Panda (FER)** through ROS 2
Humble. Use Ubuntu 22.04 with native Docker Engine/Compose v2; real FCI control
requires a PREEMPT_RT host, compatible robot firmware and a wired robot network.
Use your installation's configured robot IPs, mapping, gains and geometry.

New to the repository? The [documentation guide](docs/README.md) links the
architecture, operation, monitoring and development instructions. The current
pipeline holds measured joint position after normal trigger release and uses
controller-owned torque magnitude/rate clipping at 35% of Panda ceilings.
Reference shaping, tracking-error protection and Cartesian force/moment caps
are optional and disabled by default. Enable them separately with
`controller.reference_shaping_enabled`, `controller.tracking_protection_enabled`
and `controller.cartesian_wrench_limits_enabled`. Torque magnitude and slew-rate
clipping remain active independently.

The linked `cbf_safety_filter` library implements joint, table, collision and
singularity constraints. CBF defaults to disabled when omitted. If enabled on
hardware, select `cbf.hardware_mode: shadow` for diagnostics or `filtered` to apply
verified corrections. The saved single- and dual-Panda YAMLs select filtered
mode. See the [commissioning procedure](docs/cbf/HARDWARE_COMMISSIONING.md).

| Terminal | Where commands run |
| --- | --- |
| **Host 1** | PC host: clone and open the ROS container |
| **CONTAINER 1** | That container, `/ws`: build and run teleoperation |
| **Host 2** | PC host, `~/franka_teleoperation`: edit configs and start Quest |
| **CONTAINER 2** | Optional second container shell: standalone commands / launch help |

## 1. Open and build the Docker workspace

**Host 1 — first setup:**

```bash
cd ~
git clone https://github.com/jiachunl0727/franka_teleoperation.git
cd ~/franka_teleoperation
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm --name franka-teleop panda-ros2
```

**CONTAINER 1 — first build:**

```bash
cd /ws
source /opt/ros/humble/setup.bash
bash scripts/fetch_teleop_dependencies.sh
apt-get update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y \
  --skip-keys ament_cmake_clang_format
colcon build --base-paths src --symlink-install --packages-up-to teleop_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

Sourcing this workspace also defaults to `ROS_DOMAIN_ID=42` and
`ROS_LOCALHOST_ONLY=1`. This keeps the local teleoperation graph separate from
unrelated lab DDS traffic. Source the same workspace in every monitoring,
recording and PlotJuggler terminal. Explicit domain values are preserved;
for remote ROS observers, set the same domain and `TELEOP_ROS_LOCALHOST_ONLY=0` before
sourcing on each PC. These settings affect ROS discovery, not direct FCI or the
Quest USB TCP connection. See [network diagnosis](docs/TELEOPERATION_RUNTIME.md#ros-discovery-and-the-quest-connection).

Colcon builds libfranka before its dependent controllers automatically. This
command builds the teleoperation pipeline, excluding the legacy Panda demos.
Libfranka's own tests/examples default to off; an unused `BUILD_TESTING` warning
from libfranka is harmless because that option controls the ROS packages.

The dependency-fetch script also applies the repository's
[Hand Stop transport patch](patches/README.md). For an existing checkout after
updating this repository, rebuild the SDK together with its consumers:

```bash
bash scripts/apply_teleop_dependency_patches.sh
colcon build --base-paths src --symlink-install --packages-up-to teleop_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

Stop any existing teleoperation launch before rebuilding; restart it afterward
so every worker loads the updated library.

To exist the container:
```
docker rm -f franka-teleop
```

## 2. Set the unified configuration

**Host 2:** edit `src/teleop_bringup/config/single_panda/single_panda.yaml` for one
arm or `src/teleop_bringup/config/dual_panda/dual_panda.yaml` for two.
With a symlink install, restart the launch after editing an existing setting.
Rebuild after adding or relocating package assets so the installed profiles are available.

| YAML section | Configure here |
| --- | --- |
| `pipeline` | Backend, arm count and enabled features. In single mode, select the Quest hand. |
| `<feature>.config_file` | Each Quest, controller, Hand, recording and CBF section selects its own file in the owning package’s `config/single_panda/` or `config/dual_panda/` directory. |
| `quest.bridge`, `arms.<arm_name>` | USB port, robot IP, controller assignment, frame and calibration. Custom topics can override the derived routes. |
| `controller`, `hand` | Model, control law, derating, home pose and calibrated grasp settings. |
| `cbf` | Enable CBF, select its numeric profile and set its hardware commissioning mode. |
| `arm_settings.<arm_name>` | Controller, Hand or CBF overrides for a named arm. |
| `recording` | Mode, topics and output folder. |

Named profiles supply shared settings. Explicit runtime fields override them,
and per-arm overrides apply last. See the [configuration inventory](docs/LAUNCH_CONFIGURATION_INVENTORY.md)
for the full parameter list and standalone defaults.

Inspect the selected file and set `pipeline.backend: mock` for robot-free work.
The working single-arm file contains operator hardware settings.
The historical [incident analysis](docs/MOTION_INCIDENT_20261029.md) records the
abrupt-motion trials. Current ownership and removed protections are documented
in [CONTROLLER_ARCHITECTURE.md](docs/CONTROLLER_ARCHITECTURE.md).
Set `pipeline.start_hand: false` to omit fingers,
or `pipeline.homing_enabled: false` to omit home gestures.

The selected YAML controls hardware startup. With `pipeline.backend: hardware`,
launch opens the configured connections and an enabled Hand calibrates at
startup. Use calibrated IPs, gains, and mapping values for your installation.

Use floating-point YAML values and arrays for gains/transforms. The mapping basis
is `[qx,qy,qz,qw]`; the base-to-world transform is `[x,y,z,qx,qy,qz,qw]` in metres.
Hardware dual operation requires distinct robot IPs, namespaces and base frames. The single
template contains one arm; the dual template includes left/right assignments.

## 3. Start the Quest app over USB

Enable Quest developer mode, connect a USB **data** cable and accept the USB
debugging prompt inside the headset. Use one connected Android device.

**Host 2 — first Android build/install:**

```bash
cd ~/franka_teleoperation
docker compose -f docker/android-compose.yaml build
docker compose -f docker/android-compose.yaml run --rm android-build gradle assembleDebug
docker compose -f docker/android-compose.yaml up -d quest-adb-server
docker compose -f docker/android-compose.yaml run --rm quest-adb adb devices -l
bash scripts/quest_adb_reverse.sh 45454
bash scripts/quest_install.sh
```

To enable teloperation without wearing the headset:
```
docker compose -f docker/android-compose.yaml run --rm quest-adb adb shell am broadcast -a com.oculus.vrpowermanager.prox_close
```

To recover teleoperation for wearing headset:
```
docker compose -f docker/android-compose.yaml run --rm quest-adb adb shell am broadcast -a com.oculus.vrpowermanager.automation_disabled
```

`adb devices` must list the authorized headset as `device`. The install helper
also launches the app. Joystick direction control requires the updated APK,
which now sends both joystick axes and their availability. Rebuild/reinstall
this APK, rebuild the ROS message and its consumers, and restart the pipeline.
Older APK packets remain usable for arm motion but cannot actuate the Hand.
The 2026-10-28 APK also reconnects to a restarted bridge without waiting for
new OpenXR tracking frames. Install that update once with
`bash scripts/quest_install.sh`; no motion input is replayed on reconnect.

For subsequent sessions with the updated APK, reuse it:

```bash
docker compose -f docker/android-compose.yaml up -d quest-adb-server
bash scripts/quest_adb_reverse.sh 45454
docker compose -f docker/android-compose.yaml run --rm quest-adb \
  adb shell am start -n org.franka.questteleop/android.app.NativeActivity
```

After USB replug, rerun `bash scripts/quest_adb_reverse.sh 45454`. If an APK
update reports a signing mismatch, the install helper prints the uninstall
command; uninstalling erases that app's data.

## 4. Start teleoperation

Activate **FCI in Desk**, release the Quest triggers and keep the Hand empty
with its calibration stroke clear. Enabling Hand starts one calibration at launch.

**CONTAINER 1**, after setting `pipeline.backend: hardware` in the YAML:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda/single_panda.yaml
```

The file selects single/dual mode and every feature. Only one arm owner per
robot and one USB receiver per port may run. Do not start overlapping standalone
launches. The existing package name is `teleop_bringup`.

Desk may show **end-effector not connected** while FCI owns the Hand; this is
[expected during external control](https://github.com/frankarobotics/franka_ros/issues/118).
Check `Arm control ready` and `Hand calibration ready` for **each** arm in the
launch output. Firmware calibration success alone does not establish that the
Hand worker accepted its readback. See
[startup diagnostics](docs/TELEOPERATION_RUNTIME.md#launch-selection).

| Quest action | Operation |
| --- | --- |
| Release trigger, then hold above its engage threshold (default 0.75) | Engage assigned arm at measured pose; controller motion changes its target |
| Release below its release threshold (default 0.25) | Capture measured joints once and hold the whole arm configuration |
| Right joystick left / right | Close / open the right Hand at fixed speed while deflected |
| Left joystick right / left | Close / open the left Hand at fixed speed while deflected |
| Center joystick | Stop unfinished finger motion; preserve a confirmed cable force hold |
| Right A / left X held at least one second, then released | Home assigned arm with trigger released and homing enabled |
| Fresh left Menu presses | Start/stop an episode with `recording.mode: menu` |

Ctrl+C stops bringup. Input loss/recenter requires trigger release/press before
resuming. The manufacturer stop remains the physical emergency-stop mechanism.

Configure `hand.cable_diameter_m`, `cable_diameter_tolerance_m`, and
`grasp_force_n` for the actual cable. Force defaults to `-1` (unset): closing is
disabled until a positive force is supplied; opening remains available. A 5 mm
cable example is included in the YAML. Set the opening gap larger than the
cable's accepted diameter range. Joystick displacement beyond the engage
threshold selects direction only: fixed `speed_mps`/`open_speed_mps` and held
duration determine travel. Side grip, joystick click and vertical deflection
no longer command the Hand.

Holding a joystick direction runs one continuous firmware motion toward the
configured close/open width at the selected speed. Centering requests stop;
there is no fixed-distance stepping or position-step completion gate.
SDK/firmware cancellation latency can add travel after centering.

Closing uses `grasp()` with the configured force. A successful command,
`is_grasped` readback, and cable-width acceptance confirm a hold; empty closure
and an out-of-range cable are rejected. Neutral cancels an unfinished grasp,
so a brief inward nudge does not continue closing after release. A confirmed
hold persists through neutral or Quest input loss until an outward command.
Input faults stop unfinished motion and require a fresh neutral sample before
new movement. There are no automatic close retries or force increases.

`gripper_events` reports `holding`, `grasp_confirmed`, `neutral_required`, and
`commanded_grasp_force_n` (the configured command, not measured force).
Communication/state faults require explicit empty-hand recalibration. See
[Hand control and recovery](docs/TELEOPERATION_RUNTIME.md#hand-joystick-control-and-force-holding).

Normal trigger release selects joint position holding (`reason: position_hold`,
`motion_enabled: false`). Hold stiffness ramps up over 0.25 s; joint damping
acts immediately, and all output still passes through the shared torque limits.
Tune `controller.hold_stiffness` [Nm/rad], `hold_damping` [Nm s/rad] and
`hold_stiffness_ramp_seconds` in the selected runtime YAML. These are finite
impedance gains, not a mechanical lock; physical response needs commissioning.
Startup and pauses hold a fixed measured joint configuration independently of
Quest readiness. Missing, invalid or stale input cannot discard that anchor
or reset its stiffness ramp. Input loss during engagement captures measured
joints once and holds there; fresh release/re-engagement is still required.
Homing priority, invalid-state termination and shutdown retain
their separate stop policy.

Arm torque/rate caps default to **35% of Panda FCI ceilings**. Edit
`controller.torque_limit_scale` and `controller.torque_rate_limit_scale` in the
selected YAML. Defaults give 30.45 Nm on joints 1–4, 4.2 Nm on joints 5–7 and
350 Nm/s. Absolute controller caps may tighten them. Every nominal mode uses
this package-local clipping; there is no dependency on a safety layer.

For dual, edit `src/teleop_bringup/config/dual_panda/dual_panda.yaml`: set actual distinct
IPs, calibrate each controller mapping, check both surveyed base transforms
in the selected CBF geometry files, and set `pipeline.backend: hardware`. Stop the single
launch, activate FCI for both arms, then **CONTAINER 1**:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/dual_panda/dual_panda.yaml
```

Both enabled Hands calibrate at startup. Dual geometry/timing requires separate
commissioning. Use one dual launch rather than two single-arm owners.

## 5. Overall launch arguments

The combined launch accepts one required argument, `runtime_config`, with no
default path. It selects the single- or dual-Panda YAML. The YAML controls the
mode, backend, enabled features, port, and recording settings.

For mock operation, after keeping `pipeline.backend: mock`, **CONTAINER 1**:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda/single_panda.yaml
```

All other configuration and feature switches must be edited in YAML; there are
no overall command-line overrides or per-package config-file arguments.

## 6. Standalone packages

Standalone launches keep their own package defaults and configuration arguments.
Open **CONTAINER 2** from another host terminal when needed:

```bash
docker exec -it franka-teleop bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Command form: `ros2 launch <package> <launch_file> argument:=value`.

| Package / launch | Standalone configuration / important arguments |
| --- | --- |
| `teleop_bringup quest_usb_bridge.launch.py` | Quest USB bridge only; namespace and bridge parameters |
| `teleop_bringup pose_mapper.launch.py` | Pose mapper only; namespace, topics and mapper parameters |
| `teleop_bringup quest_teleop.launch.py` | Bringup `config/single_panda/arms.yaml`; `arms_config`, `mode`, `single_controller`, bridge/mapper switches and bridge parameters |
| `teleop_bringup cartesian_pd_controller.launch.py` | Controller `controllers.yaml`; `runtime_config`, `arms_config`, backend/mode and homing settings |
| `teleop_bringup homing.launch.py` | Same owner/configuration with homing enabled; no automatic home |
| `teleop_bringup gripper.launch.py` | Package `gripper.yaml`; `runtime_config`, `arms_config`, backend/mode; one startup calibration |
| `teleop_bringup recording.launch.py` | Package `recording.yaml`; `recording_config`, `arms_config`, recording mode/topics/output/namespace |

Show complete arguments for any launch in **CONTAINER 2**:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda/single_panda.yaml --show-args
```

The arm controller separates `controller_runtime.cpp` (ROS, mock/hardware and
FCI ownership), `controller_manager.cpp` (arming and nominal-source selection),
and the Cartesian PD, position hold, fault damping and homing implementations.
All laws share a joint-torque interface and final safety path;
`joint_motion_generator.cpp` remains the homing reference generator. Existing
launch commands and configuration keys are unchanged.

For configuration and controller extension details, see
[docs/PACKAGE_STRUCTURE.md](docs/PACKAGE_STRUCTURE.md#arm-runtime-and-controller-structure).

Select the Cartesian teleoperation law with `controller.cartesian_control_law`:
`cartesian_pd` (existing default) or `inverse_dynamics`. The inverse-dynamics
option uses the OSCBF operational-space structure, preserves configured physical
stiffness, and computes modal damping from task inertia and stiffness. Both laws
use the existing final torque clipping and optional CBF. Parameters, equations
and recorded replay results: [inverse-dynamics controller](docs/INVERSE_DYNAMICS_CONTROLLER.md).

Runtime dynamics now use the pinned OSCBF Panda URDF through a native,
allocation-free M/c/g calculation shared by nominal control and CBF. Select the
file with `controller.dynamics_model_file`; libfranka supplies robot communication
and feedback. See [URDF dynamics](docs/URDF_DYNAMICS.md).
The confirmed `.101` installation uses the augmented 5 g UMI finger model.
[Finger geometry and offline avoidance checks](docs/cbf/UMI_FINGERS_AND_OFFLINE_AVOIDANCE.md)
describe its mounting, full-opening cover and original-bag replay commands.

## Trajectory monitoring and experiment reports

Each arm can set `pose_filter` and `velocity_filter` independently to `none`,
`low_pass`, or `four_pole`. The four-pole filter has a C3 continuous step
response. Each arm can also set `pose_interpolation` and
`velocity_interpolation` to `none`, `ptp_polynomial`, or `spline`.
The septic point-to-point curve is C3 at completed knots; the quintic spline
is C2 there. Separate cutoff and interpolation-duration settings are available
under each arm in the package and bringup YAML files. Interpolation coalesces
new waypoints while a segment runs, so it adds trajectory delay and may skip
intermediate waypoints when input arrives faster than segments complete.

While holding the trigger, hold Y on the left Quest controller or B on the
right to follow orientation while holding the measured end-effector position.
Release Y or B to return to Cartesian PD tracking without releasing
the trigger. The latest live tracking view is `/<arm>/tracking_status`.
The default bag profile records controller input, mapped command, measured
pose, and `/<arm>/tracking_samples`; its CSV exporter selects only trajectory
fields and timing data for the MATLAB report.

In a second ROS container terminal, run
`bash scripts/plotjuggler_tracking.sh /panda_1` and subscribe to
`/panda_1/tracking_status`. For recorded reports, export bags to CSV with
`ros2 run recording export_tracking BAG OUTPUT_DIRECTORY`.

MATLAB analysis runs in the separate optional `docker/matlab-compose.yaml`
container and exports vector PDFs, high-resolution PNGs, metrics and estimated
acceleration/jerk. Torque and diagnostic report pages are omitted. It requires base MATLAB with an eligible license; no host
installation, ROS Toolbox or Signal Processing Toolbox is required. First-time
account/network activation, batch/browser commands and license handover are
documented in [docs/TRAJECTORY_MONITORING.md](docs/TRAJECTORY_MONITORING.md).
Every plot panel includes an explicit legend identifying its signals. Regenerate
reports to update older figures.

Generated recordings, exports, reports and runtime logs live under ignored
`log/`. The existing experiment data was removed during the requested cleanup;
new recordings create their directories automatically. The reusable analysis
code remains in `analysis/matlab/`. For formatting and robot-free validation,
see [DEVELOPMENT.md](docs/DEVELOPMENT.md).

Hand motion and stop commands run outside the arm torque loop. SDK/firmware
latency still affects how quickly a finger stops; held duration is not an exact
position measurement. See [Hand control details](docs/TELEOPERATION_RUNTIME.md#hand-joystick-control-and-force-holding).

The Cartesian modes support a dynamically consistent nullspace posture
preference toward `controller.home_joint_position`, shared with homing. This is
a nominal controller preference. Optional CBF software validation is described in `docs/cbf/IMPLEMENTATION.md`.
See [CONTROLLER_ARCHITECTURE.md](docs/CONTROLLER_ARCHITECTURE.md).

Collision/singularity design and mock evidence: [implementation](docs/cbf/COLLISION_AND_SINGULARITY.md), [verification report](docs/cbf/AVOIDANCE_VERIFICATION.md), [short hardware procedure](docs/cbf/HARDWARE_COMMISSIONING.md).
