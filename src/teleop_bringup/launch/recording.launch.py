"""Menu episode recording or continuous recording with an explicit topic profile."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from teleop_bringup.configuration import (
    load_arms,
    select_arms,
    launch_bool,
)
from teleop_bringup.runtime_configuration import (
    unified_settings,
    load_runtime_document,
    bridge_controller_topic,
)
import json
from launch.actions import LogInfo
from teleop_bringup.recording_configuration import recording_plan


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    source = load_runtime_document(value("recording_config"))
    debounce = (
        unified_settings(source)[2]["menu_debounce_seconds"]
        if "pipeline" in source
        else source.get("menu_debounce_seconds", 0.35)
    )
    configured = load_arms(value("arms_config"))
    mode = value("mode")
    active = (
        configured
        if mode == "all"
        else select_arms(configured, mode, value("single_controller"))
    )
    plan = recording_plan(
        value("recording_config"),
        active,
        value("recording_topics"),
        value("recording_output_root"),
    )
    if value("recording_mode") == "continuous":
        return [
            LogInfo(msg=f"Recording selected topics to {plan.output}"),
            Node(
                package="recording",
                executable="menu_recorder",
                name="continuous_recorder",
                namespace=value("recording_namespace"),
                output="screen",
                parameters=[
                    dict(
                        recording_mode="continuous",
                        topics_json=json.dumps(plan.topics),
                        output_root=str(Path(plan.output).parent),
                        output_directory=plan.output,
                        storage_id=plan.command[4],
                    )
                ],
            ),
        ]
    if value("recording_mode") != "menu":
        raise ValueError("recording_mode must be menu/continuous")
    inactive = [
        a.namespace for a in configured if a.name not in {b.name for b in active}
    ]
    left = value("left_controller_state_topic") or next(
        (
            a.parameters["controller_state_topic"]
            for a in configured
            if a.parameters["controller_id"] == "left"
        ),
        bridge_controller_topic(source, "left"),
    )
    return [
        Node(
            package="recording",
            executable="menu_recorder",
            namespace=value("recording_namespace"),
            output="screen",
            parameters=[
                dict(
                    recording_mode="menu",
                    controller_state_topic=left,
                    topics_json=json.dumps(plan.topics),
                    inactive_namespaces_json=json.dumps(inactive),
                    menu_debounce_seconds=float(debounce),
                    output_root=str(Path(plan.output).parent),
                    storage_id=plan.command[4],
                )
            ],
        )
    ]


def generate_launch_description():
    defaults = dict(
        mode="single",
        single_controller="right",
        recording_mode="menu",
        arms_config=str(
            Path(get_package_share_directory("teleop_bringup")) / "config/single_panda/arms.yaml"
        ),
        recording_config=str(
            Path(get_package_share_directory("recording")) / "config/standalone.yaml"
        ),
        recording_topics="",
        recording_output_root="",
        left_controller_state_topic="",
        recording_namespace="/quest",
    )
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=setup)]
    )
