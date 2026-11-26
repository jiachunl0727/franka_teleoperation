#!/usr/bin/env python3
"""Profile synthetic mock-owner callbacks. Never accepts a hardware backend."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from quest_teleop_interfaces.msg import CartesianTeleopCommand


def summarize(values):
    values = sorted(values)
    return [sum(values) / len(values)] + [
        values[min(len(values) - 1, int(p * (len(values) - 1)))]
        for p in (0.5, 0.95, 0.99, 1.0)
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if not 100 <= args.samples <= 100000:
        parser.error("samples must be in [100,100000]")
    root = Path(__file__).parents[2] / "cbf_safety_filter"
    phases = [
        "model_us",
        "kinematics_us",
        "constraints_us",
        "assembly_us",
        "update_us",
        "solver_us",
        "verify_us",
        "total_us",
        "controller_callback_us",
    ]
    output = Path(args.output)
    with output.open("w") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "backend",
                "samples",
                "failures",
                "max_hard_violation",
            ]
            + [
                phase + "_" + stat
                for phase in phases
                for stat in ("mean", "p50", "p95", "p99", "max")
            ]
        )
        for backend in ("osqp", "qpoases", "piqp"):
            for formulation in ("hard",):
                rclpy.init()
                node = Node("mock_profile_" + uuid.uuid4().hex)
                ns = "/mock_profile_" + uuid.uuid4().hex
                messages = []
                node.create_subscription(
                    String,
                    ns + "/cbf_status",
                    lambda msg: messages.append(json.loads(msg.data)),
                    2048,
                )
                publisher = node.create_publisher(
                    CartesianTeleopCommand, ns + "/cartesian_teleop_command", 10
                )
                with tempfile.TemporaryDirectory() as directory:
                    directory = Path(directory)
                    config = yaml.safe_load((root / "config/single_panda/cbf.yaml").read_text())
                    geometry = yaml.safe_load((Path(__file__).resolve().parent / "data/release_braking/geometry.yaml").read_text())
                    geometry["capsules"] = geometry["capsules"][:12]
                    geometry_path = directory / "geometry.yaml"
                    geometry_path.write_text(yaml.safe_dump(geometry))
                    config["geometry"]["file"] = str(geometry_path)
                    for family in ("joint_acceleration", "self_collision", "singularity"):
                        config["constraints"][family]["enabled"] = False
                    config["constraints"]["table"].update(enabled=True, height_m=-1)
                    config["solver"]["backend"] = backend
                    config["execution"].update(
                        safety_torque_computation_method="projection",
                        max_constraints=52,
                        max_filter_time_us=1000000,
                        max_model_age_us=1000000,
                    )
                    config["solver"]["osqp"].update(
                        max_iterations=10000,
                        absolute_tolerance=1e-8,
                        relative_tolerance=1e-8,
                    )
                    config["solver"]["piqp"].update(
                        max_iterations=100,
                        absolute_tolerance=1e-8,
                        relative_tolerance=1e-8,
                    )
                    path = directory / "cbf.yaml"
                    path.write_text(yaml.safe_dump(config))
                    log_path = directory / "owner.log"
                    with log_path.open("w") as log:
                        cmd = [
                            "ros2",
                            "run",
                            "teleop_controllers",
                            "cartesian_pd_controller",
                            "--ros-args",
                            "-r",
                            "__ns:=" + ns,
                            "-p",
                            "backend:=mock",
                            "-p",
                            "cbf_enabled:=true",
                            "-p",
                            "cbf_config_path:=" + str(path),
                        ]
                        process = subprocess.Popen(
                            cmd,
                            stdout=log,
                            stderr=subprocess.STDOUT,
                            start_new_session=True,
                        )
                        sequence = 0

                        def drive(active):
                            nonlocal sequence
                            sequence += 1
                            msg = CartesianTeleopCommand()
                            msg.header.frame_id = "panda_1/panda_link0"
                            msg.session_id = "mock-profile"
                            msg.sequence = sequence
                            msg.source_time_pc_monotonic_ns = time.monotonic_ns()
                            msg.source_time_valid = msg.clock_sync_valid = (
                                msg.input_valid
                            ) = True
                            msg.trigger_released = not active
                            msg.motion_enabled = active
                            t = time.monotonic() - start
                            msg.target_pose.position.x = 0.4 + (
                                0.002 * math.sin(2 * math.pi * t) if active else 0
                            )
                            msg.target_pose.position.z = 0.4
                            angle = 0.01 * math.sin(2 * math.pi * t) if active else 0
                            msg.target_pose.orientation.z = math.sin(angle / 2)
                            msg.target_pose.orientation.w = math.cos(angle / 2)
                            publisher.publish(msg)

                        try:
                            start = time.monotonic()
                            while time.monotonic() - start < 10 and (
                                not messages or not publisher.get_subscription_count()
                            ):
                                rclpy.spin_once(node, timeout_sec=0.01)
                                if process.poll() is not None:
                                    raise RuntimeError(log_path.read_text())
                            if not messages:
                                raise RuntimeError(
                                    "missing CBF diagnostics\n" + log_path.read_text()
                                )
                            start = time.monotonic()
                            while time.monotonic() - start < 0.2:
                                drive(False)
                                rclpy.spin_once(node, timeout_sec=0.005)
                            messages.clear()
                            start = time.monotonic()
                            last_sent = 0.0
                            while (
                                len(messages) < args.samples + 100
                                and time.monotonic() - start < 30
                            ):
                                if time.monotonic() - last_sent > 0.01:
                                    drive(True)
                                    last_sent = time.monotonic()
                                rclpy.spin_once(node, timeout_sec=0.002)
                                if process.poll() is not None:
                                    raise RuntimeError(log_path.read_text())
                            samples = messages[100 : 100 + args.samples]
                            if len(samples) != args.samples:
                                raise RuntimeError("insufficient callback samples")
                            writer.writerow(
                                [
                                    backend,
                                    len(samples),
                                    sum(not m["accepted"] for m in samples),
                                    max(m["max_hard_violation"] for m in samples),
                                ]
                                + [
                                    v
                                    for phase in phases
                                    for v in summarize([m[phase] for m in samples])
                                ]
                            )
                            file.flush()
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
                        if process.returncode:
                            raise RuntimeError(log_path.read_text())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
