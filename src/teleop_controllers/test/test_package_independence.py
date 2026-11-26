"""Runtime independence and controller-owned actuator limits."""

from pathlib import Path
import ast
import os
import signal
import subprocess
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from quest_teleop_interfaces.msg import CartesianTeleopCommand, TrackingSample


def test_feature_runtimes_have_no_functional_project_dependencies():
    source = Path(__file__).parents[2]
    features = {"quest_teleop", "teleop_controllers", "teleop_gripper", "recording"}
    forbidden = features | {"teleop_bringup", "safety_filter"}
    assert not (source / "safety_filter").exists()
    for feature in features:
        package = ET.parse(source / feature / "package.xml").getroot()
        for tag in ("depend", "exec_depend", "build_depend", "build_export_depend"):
            dependencies = {node.text for node in package.findall(tag)}
            assert not dependencies & forbidden, feature
            if feature != "teleop_controllers":
                assert "cbf_safety_filter" not in dependencies, feature
        for path in (source / feature).rglob("*.py"):
            if "test" in path.parts or "__pycache__" in path.parts:
                continue
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.ImportFrom) and node.module:
                    imported = node.module.split(".")[0]
                    assert imported not in forbidden - {feature}, (path, imported)
        for path in list((source / feature / "include").rglob("*.hpp")) + list(
            (source / feature / "src").rglob("*.cpp")
        ):
            assert '"safety_filter/' not in path.read_text(), path
    assert (
        "find_package(safety_filter "
        not in (source / "teleop_controllers" / "CMakeLists.txt").read_text()
    )
    cbf = ET.parse(source / "cbf_safety_filter" / "package.xml").getroot()
    assert not {n.text for n in cbf.findall("depend")} & (features | {"teleop_bringup"})


def test_mock_arms_clip_torque_independently_inside_controller_runtime():
    rclpy.init()
    node = Node("clip_observer_" + uuid.uuid4().hex)
    prefix = "/clip_" + uuid.uuid4().hex[:12]
    samples = {side: [] for side in ("left", "right")}
    pubs = {}
    args = [
        "ros2",
        "run",
        "teleop_controllers",
        "cartesian_pd_controller",
        "--ros-args",
        "-p",
        "backend:=mock",
        "-p",
        "mode:=dual",
    ]
    for side, scale, rate in (("left", 0.02, 0.005), ("right", 0.005, 0.001)):
        ns = prefix + "/" + side
        node.create_subscription(
            TrackingSample, ns + "/tracking_samples", samples[side].append, 2048
        )
        pubs[side] = node.create_publisher(
            CartesianTeleopCommand, ns + "/cartesian_teleop_command", 10
        )
        for key, value in {
            "cartesian_control_law": (
                "inverse_dynamics" if side == "left" else "cartesian_pd"
            ),
            "namespace": ns,
            "base_frame": ns[1:] + "/base",
            "reference_shaping_enabled": "true" if side == "left" else "false",
            "tracking_protection_enabled": "false",
            "cartesian_wrench_limits_enabled": "true" if side == "left" else "false",
            "torque_limit_scale": scale,
            "torque_rate_limit_scale": rate,
        }.items():
            args += ["-p", side + "." + key + ":=" + str(value)]
    with tempfile.TemporaryDirectory() as directory:
        log_path = Path(directory) / "controller.log"
        with log_path.open("w") as log:
            process = subprocess.Popen(
                args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            sequence = 0

            def drive(active, duration):
                nonlocal sequence
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    sequence += 1
                    for side, pub in pubs.items():
                        msg = CartesianTeleopCommand()
                        msg.header.frame_id = prefix[1:] + "/" + side + "/base"
                        msg.session_id = "local-clipping"
                        msg.sequence = sequence
                        msg.source_time_pc_monotonic_ns = time.monotonic_ns()
                        msg.source_time_valid = msg.clock_sync_valid = (
                            msg.input_valid
                        ) = True
                        msg.motion_enabled = active
                        msg.trigger_released = not active
                        msg.target_pose.position.x = 0.42 if active else 0.4
                        msg.target_pose.position.z = 0.4
                        msg.target_pose.orientation.w = 1.0
                        pub.publish(msg)
                    until = time.monotonic() + 0.01
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.001)
                    assert process.poll() is None, log_path.read_text()

            try:
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline and not all(
                    samples[s] and pubs[s].get_subscription_count() for s in samples
                ):
                    rclpy.spin_once(node, timeout_sec=0.01)
                    assert process.poll() is None, log_path.read_text()
                assert all(samples.values()), log_path.read_text()
                drive(False, 0.15)
                drive(True, 0.6)
                for side, scale, rate in (
                    ("left", 0.02, 0.005),
                    ("right", 0.005, 0.001),
                ):
                    active = [s for s in samples[side] if s.motion_enabled]
                    assert active, log_path.read_text()
                    assert any(abs(s.nominal_torque[0]) > 87 * scale for s in active)
                    assert any(s.torque_rate_limited for s in samples[side])
                    for sample in samples[side]:
                        assert abs(sample.maximum_torque_rate - 1000 * rate) < 1e-10
                        for j, published in enumerate([87.0] * 4 + [12.0] * 3):
                            assert (
                                abs(sample.torque_limits[j] - published * scale) < 1e-10
                            )
                            assert (
                                abs(sample.commanded_torque[j])
                                <= published * scale + 1e-10
                            )
                            if sample.previous_command_valid:
                                assert (
                                    abs(
                                        sample.commanded_torque[j]
                                        - sample.previous_commanded_torque[j]
                                    )
                                    <= rate + 1e-10
                                )
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()
            assert process.returncode == 0, log_path.read_text()
