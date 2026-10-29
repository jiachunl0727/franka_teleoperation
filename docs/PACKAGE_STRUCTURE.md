# Package structure

See [controller architecture](CONTROLLER_ARCHITECTURE.md) for the current
ownership contract and the removal of CBF functionality.

| Package | Owned functionality |
| --- | --- |
| `teleop_bringup` | All project launch entry points, pipeline schema, arm routing and recording topic plans |
| `quest_teleop` | USB input bridge, pose/twist mapping and mock pose publisher |
| `teleop_controllers` | Controller laws, manager, homing, model adaptation, actuator clipping and libfranka/mock runtime |
| `teleop_gripper` | Franka Hand state machine, calibration and worker |
| `recording` | Menu recorder, recording process lifecycle and tracking export |
| `quest_teleop_interfaces` | Shared ROS message definitions |

Feature runtimes depend on shared message definitions and external libraries,
not on one another's functionality. Cross-package launch/configuration belongs
to `teleop_bringup`. The former `safety_filter` package and its solver dependency
are removed. The legacy `panda_bringup` and `panda_demos` packages are unchanged.

Use `ros2 launch teleop_bringup <launch_file>` for both combined and individual
feature launches. Use `ros2 run <owning_package> <executable>` for direct execution.
