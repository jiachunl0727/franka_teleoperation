# Documentation guide

Start with the [repository README](../README.md) to build the workspace, configure
an arm and launch teleoperation. ROS commands run inside the ROS container;
Docker commands run on the host PC. The host repository is mounted at `/ws`.

| What you want to do | Read |
| --- | --- |
| Understand packages and add a controller | [Package architecture](PACKAGE_STRUCTURE.md) |
| Understand trigger release, homing, Hand behavior or startup errors | [Runtime guide](TELEOPERATION_RUNTIME.md) |
| Check mapping without a robot; calibrate the coordinate basis | [Robot-free mapping](ROBOT_FREE_MAPPING.md) |
| Plot live data, record an experiment or create MATLAB reports | [Trajectory monitoring](TRAJECTORY_MONITORING.md) |
| Review the new CBF library, all solver benchmarks and hardware release blockers | [CBF verification](cbf/VERIFICATION.md) |
| Check installed UMI fingers and replay table/singularity avoidance from bags | [Finger model and offline avoidance](cbf/UMI_FINGERS_AND_OFFLINE_AVOIDANCE.md) |
| Change torque caps or understand controller ownership | [Controller ownership and torque clipping](CONTROLLER_ARCHITECTURE.md) |
| Format code, build and run tests | [Development guide](DEVELOPMENT.md) |
| Understand USB messages and timestamps | [Quest USB protocol](quest-usb-protocol.md) |
| Understand the native headset application | [Quest app](VR_APP_PORT.md) |
| Review implemented features and validation limits | [Implementation record](IMPLEMENTATION_PROGRESS.md) |
| Review the current architecture and deferred CBF work | [Current architecture](CONTROLLER_ARCHITECTURE.md) |

The combined-launch configuration files are
[`single_panda.yaml`](../src/teleop_bringup/config/single_panda/single_panda.yaml) and
[`dual_panda.yaml`](../src/teleop_bringup/config/dual_panda/dual_panda.yaml). Inspect the
selected file before running it: these are editable installation settings, and
the single-arm file may already select hardware. Package-local YAML files apply
to standalone launches.

Current behavior includes joint position holding after normal trigger release,
controller-owned actuator clipping and full-cycle tracking telemetry.
The former CBF layer is removed; see the current architecture for protection
scope and launch ownership.

`IMPLEMENTATION_PROGRESS.md` records earlier stages as history. Its older paths,
test counts and behavior descriptions are not current operating instructions.
`AGENTS.md` contains contributor/agent policy and historical decisions.
