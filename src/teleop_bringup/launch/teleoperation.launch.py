"""Compose the pipeline from an explicitly selected runtime YAML."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from teleop_bringup.configuration import (
    full_topic,
    load_arms,
    select_arms,
)
from teleop_bringup.runtime_configuration import (
    unified_settings,
    load_runtime,
    load_runtime_document,
)

def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    if set(context.launch_configurations) - {"runtime_config"}:
        raise ValueError(
            "overall bringup accepts only runtime_config; set all pipeline options in YAML"
        )
    path = value("runtime_config")
    pipeline, bridge, recording = unified_settings(load_runtime_document(path))
    if pipeline["backend"] not in ("mock", "hardware"):
        raise ValueError("backend must be mock/hardware")
    configured = load_arms(path)
    active = select_arms(configured, pipeline["mode"], pipeline["single_controller"])
    settings = load_runtime(path, configured)
    from teleop_bringup.cbf_configuration import prepare_cbf

    prepare_cbf(settings, active, pipeline["backend"])
    # Validate hardware ownership before starting any child (including Hand).
    if pipeline["backend"] == "hardware" and (
        pipeline["start_controller"] or pipeline["start_hand"]
    ):
        if any(not arm.robot_ip for arm in active):
            raise ValueError("hardware needs each selected robot_ip")
    if pipeline["start_controller"] and pipeline["mock_measured_pose"]:
        raise ValueError(
            "mock_measured_pose requires start_controller:=false to avoid competing pose publishers"
        )
    common = dict(
        arms_config=path,
        mode=pipeline["mode"],
        single_controller=pipeline["single_controller"],
    )
    text = lambda v: str(v).lower() if isinstance(v, bool) else str(v)
    actions = []

    def include(package, launch, args):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(get_package_share_directory(package)) / "launch" / launch)
                ),
                launch_arguments={k: text(v) for k, v in args.items()}.items(),
            )
        )

    if (
        pipeline["start_bridge"]
        or pipeline["start_mapper"]
        or pipeline["mock_measured_pose"]
    ):
        include(
            "teleop_bringup",
            "quest_teleop.launch.py",
            dict(
                common,
                **bridge,
                start_bridge=pipeline["start_bridge"],
                start_mapper=pipeline["start_mapper"],
                mock_measured_pose=pipeline["mock_measured_pose"],
                recording_status_topic=full_topic(
                    "recording_status", recording["namespace"]
                ),
            ),
        )
    if pipeline["start_controller"]:
        include(
            "teleop_bringup",
            "cartesian_pd_controller.launch.py",
            dict(
                common,
                backend=pipeline["backend"],
                runtime_config=path,
                homing_enabled=pipeline["homing_enabled"],
            ),
        )
    if pipeline["start_hand"]:
        include(
            "teleop_bringup",
            "gripper.launch.py",
            dict(
                common,
                backend=pipeline["backend"],
                runtime_config=path,
            ),
        )
    if pipeline["recording_enabled"]:
        # Check recording schema/topics before any process can be started.
        from teleop_bringup.recording_configuration import recording_plan

        recording_plan(path, active)
        include(
            "teleop_bringup",
            "recording.launch.py",
            dict(
                common,
                recording_config=path,
                recording_mode=recording["mode"],
                left_controller_state_topic=recording["left_controller_state_topic"],
                recording_namespace=recording["namespace"],
            ),
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "runtime_config",
                description="Required path to the single- or dual-Panda runtime YAML",
            ),
            OpaqueFunction(function=setup),
        ]
    )
