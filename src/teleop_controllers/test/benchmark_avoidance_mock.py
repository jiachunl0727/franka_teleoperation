#!/usr/bin/env python3
"""Robot-free profiling of single/dual owners with collision and singularity rows.

No hardware backend option. Permissive diagnostic deadlines expose overruns;
reported 1 ms exceedances do not constitute a passing hard-real-time gate.
"""
import argparse
import csv
import json
import math
import multiprocessing
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


def stats(values):
    values = sorted(values)
    return (
        (
            [sum(values) / len(values)]
            + [
                values[min(len(values) - 1, int(p * (len(values) - 1)))]
                for p in (0.5, 0.95, 0.99, 0.999, 1)
            ]
        )
        if values
        else [float("nan")] * 6
    )


def drive_commands(namespaces, arms, active_constraint, stop):
    """Independent source: observer JSON processing cannot starve input freshness."""
    rclpy.init()
    node = Node("avoidance_source_" + uuid.uuid4().hex)
    publishers = [
        node.create_publisher(
            CartesianTeleopCommand, namespace + "/cartesian_teleop_command", 10
        )
        for namespace in namespaces
    ]
    start = time.monotonic()
    sequence = 0
    try:
        while not stop.is_set():
            elapsed = time.monotonic() - start
            sequence += 1
            for i, publisher in enumerate(publishers):
                msg = CartesianTeleopCommand()
                msg.header.frame_id = (
                    "panda_1/panda_link0"
                    if arms == 1
                    else (
                        "panda_left/panda_link0"
                        if i == 0
                        else "panda_right/panda_link0"
                    )
                )
                msg.session_id = "avoidance-profile"
                msg.sequence = sequence
                msg.source_time_pc_monotonic_ns = time.monotonic_ns()
                msg.source_time_valid = msg.clock_sync_valid = msg.input_valid = True
                active = elapsed > 2
                msg.trigger_released = not active
                msg.motion_enabled = active
                msg.target_pose.position.x = 0.4 + (
                    0.002 * math.sin(2 * math.pi * elapsed) if active else 0.0
                )
                msg.target_pose.position.y = (
                    0.002 * math.cos(2 * math.pi * elapsed) if active else 0.0
                )
                msg.target_pose.position.z = 0.4
                msg.target_pose.orientation.w = 1.0
                if active_constraint and active:
                    msg.target_pose.orientation.x = -math.sin(0.03 / 2)
                    msg.target_pose.orientation.w = math.cos(0.03 / 2)
                publisher.publish(msg)
            stop.wait(0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def collect(messages, active_counts, index, msg):
    sample = json.loads(msg.data)
    for key in ("constraints", "candidate_torque", "commanded_torque"):
        sample.pop(key, None)
    messages[index].append(sample)
    active_counts[index] += bool(sample.get("motion_enabled", False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--output", required=True)
    parser.add_argument("--peer-age-us", type=float, default=3000)
    parser.add_argument(
        "--backends",
        nargs="+",
        choices=["osqp", "qpoases", "piqp"],
        default=["osqp", "qpoases", "piqp"],
    )
    parser.add_argument("--arms", nargs="+", type=int, choices=[1, 2], default=[1, 2])
    parser.add_argument("--budget-us", type=float, default=800)
    parser.add_argument("--enforce-budget", action="store_true")
    parser.add_argument("--force-solver", action="store_true")
    parser.add_argument("--active-constraint", action="store_true")
    parser.add_argument(
        "--output-mode", choices=["filtered", "shadow"], default="filtered"
    )
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="require libfranka-equivalent FIFO scheduling on mock owner threads",
    )
    args = parser.parse_args()
    if not 100 <= args.samples <= 100000:
        parser.error("samples must be in [100,100000]")
    package = Path(__file__).parents[2] / "cbf_safety_filter"
    phases = [
        "model_us",
        "kinematics_us",
        "assembly_us",
        "update_us",
        "verify_us",
        "controller_cpu_us",
        "wake_lateness_us",
        "observed_period_us",
        "constraints_us",
        "solver_us",
        "total_us",
        "controller_callback_us",
    ]
    failures = 0
    with Path(args.output).open("w") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "arms",
                "backend",
                "formulation",
                "output_mode",
                "workload",
                "scheduling",
                "arm",
                "requested_samples",
                "samples",
                "process_fault",
                "active_observed",
                "peer_age_limit_us",
                "callback_over_1ms",
                "budget_us",
                "callback_over_budget",
                "observed_callbacks",
                "active_observed_callbacks",
                "all_callback_over_budget",
                "all_callback_max_us",
                "diagnostic_queue_drops",
                "max_hard_violation",
                "max_rows",
            ]
            + [
                f"{p}_{s}"
                for p in phases
                for s in ("mean", "p50", "p95", "p99", "p999", "max")
            ]
            + ["log_tail"]
        )
        for arms in args.arms:
            for backend in args.backends:
                for formulation in ("hard",):
                    rclpy.init()
                    node = Node("avoidance_benchmark_" + uuid.uuid4().hex)
                    ns = "/avoidance_" + uuid.uuid4().hex
                    namespaces = [ns] if arms == 1 else [ns + "/left", ns + "/right"]
                    messages = [[] for _ in range(arms)]
                    active_counts = [0] * arms
                    subscriptions = []
                    for i, namespace in enumerate(namespaces):
                        subscriptions.append(
                            node.create_subscription(
                                String,
                                namespace + "/cbf_status",
                                lambda msg, i=i: collect(
                                    messages, active_counts, i, msg
                                ),
                                2048,
                            )
                        )
                    with tempfile.TemporaryDirectory() as directory:
                        directory = Path(directory)
                        parameters = {
                            "backend": "mock",
                            "mode": "single" if arms == 1 else "dual",
                        }
                        for i in range(arms):
                            config = yaml.safe_load(
                                (package / "config/single_panda/cbf.yaml").read_text()
                            )
                            geometry = yaml.safe_load(
                                (Path(__file__).resolve().parent / "data/release_braking/geometry.yaml").read_text()
                            )
                            # Separate known WORLD bases. No implicit namespace-based transform.
                            if arms == 2:
                                geometry["world_from_base_xyz_xyzw"] = [
                                    0,
                                    1.2 * i,
                                    0,
                                    0,
                                    0,
                                    0,
                                    1,
                                ]
                            gpath = directory / f"geometry_{i}.yaml"
                            gpath.write_text(yaml.safe_dump(geometry))
                            config["geometry"]["file"] = str(gpath)
                            if args.active_constraint:
                                config["constraints"]["joint_position"][
                                    "safety_margin_rad"
                                ] = 0.7155
                            config["constraints"]["self_collision"]["enabled"] = True
                            config["constraints"]["singularity"]["enabled"] = True
                            config["constraints"].setdefault("inter_arm_collision", {}).update(
                                enabled=arms == 2,
                                max_peer_age_us=args.peer_age_us,
                                max_peer_skew_us=args.peer_age_us,
                            )
                            config["execution"].update(
                                safety_torque_computation_method="projection",
                                max_constraints=256,
                                max_filter_time_us=1000000,
                                max_model_age_us=1000000,
                                max_callback_time_us=(
                                    args.budget_us if args.enforce_budget else 1000000
                                ),
                            )
                            config["execution"][
                                "analytic_nominal"
                            ] = not args.force_solver
                            config["solver"]["backend"] = backend
                            config["solver"]["osqp"].update(
                                max_iterations=1000,
                                absolute_tolerance=1e-8,
                                relative_tolerance=1e-8,
                            )
                            config["solver"]["piqp"].update(
                                max_iterations=100,
                                absolute_tolerance=1e-8,
                                relative_tolerance=1e-8,
                            )
                            path = directory / f"cbf_{i}.yaml"
                            path.write_text(yaml.safe_dump(config))
                            prefix = (
                                "" if arms == 1 else ("left." if i == 0 else "right.")
                            )
                            parameters[prefix + "mock_realtime"] = args.realtime
                            parameters[prefix + "cbf_enabled"] = True
                            parameters[prefix + "cbf_commissioning_mode"] = (
                                args.output_mode
                            )
                            parameters[prefix + "cbf_config_path"] = str(path)
                            if arms == 2:
                                parameters[prefix + "namespace"] = namespaces[i]
                        params = directory / "owner.yaml"
                        params.write_text(
                            yaml.safe_dump({"/**": {"ros__parameters": parameters}})
                        )
                        logpath = directory / "owner.log"
                        with logpath.open("w") as log:
                            process = subprocess.Popen(
                                [
                                    "ros2",
                                    "run",
                                    "teleop_controllers",
                                    "cartesian_pd_controller",
                                    "--ros-args",
                                    "-r",
                                    "__ns:=" + ns,
                                    "--params-file",
                                    str(params),
                                ],
                                stdout=log,
                                stderr=subprocess.STDOUT,
                                start_new_session=True,
                            )
                            context = multiprocessing.get_context("spawn")
                            stop = context.Event()
                            source = context.Process(
                                target=drive_commands,
                                args=(namespaces, arms, args.active_constraint, stop),
                            )
                            source.start()
                            fault = False
                            try:
                                start = time.monotonic()
                                while time.monotonic() - start < max(
                                    15, args.samples / 1000 + 12
                                ) and (
                                    time.monotonic() - start < 3
                                    or min(active_counts) < args.samples + 100
                                ):
                                    if process.poll() is not None:
                                        fault = True
                                        break
                                    rclpy.spin_once(node, timeout_sec=0.002)
                            finally:
                                stop.set()
                                source.join(timeout=5)
                                if source.is_alive():
                                    source.terminate()
                                    source.join()
                                if process.poll() is None:
                                    os.killpg(process.pid, signal.SIGINT)
                                try:
                                    process.wait(timeout=10)
                                except subprocess.TimeoutExpired:
                                    os.killpg(process.pid, signal.SIGKILL)
                                    process.wait()
                                    fault = True
                            fault |= bool(process.returncode)
                            active_observed = (
                                "requested=1 enabled=1" in logpath.read_text()
                            )
                            for i, data in enumerate(messages):
                                samples = [
                                    m for m in data if m.get("motion_enabled", False)
                                ][-args.samples :]
                                fault_for_arm = (
                                    fault
                                    or len(samples) < args.samples
                                    or not active_observed
                                )
                                over_budget = sum(
                                    m["controller_callback_us"] > args.budget_us
                                    for m in samples
                                )
                                if args.active_constraint and args.output_mode == "filtered":
                                    fault_for_arm |= not any(
                                        m["accepted"] and m["actual_command_verified"]
                                        for m in samples
                                    )
                                failures += fault_for_arm or (
                                    args.enforce_budget
                                    and any(
                                        m["controller_callback_us"] > args.budget_us
                                        for m in data
                                    )
                                )
                                writer.writerow(
                                    [
                                        arms,
                                        backend,
                                        formulation,
                                        args.output_mode,
                                        (
                                            "active_joint_barrier"
                                            if args.active_constraint
                                            else "separated_home"
                                        ),
                                        "fifo" if args.realtime else "normal",
                                        i,
                                        args.samples,
                                        len(samples),
                                        int(fault_for_arm),
                                        int(active_observed),
                                        args.peer_age_us,
                                        sum(
                                            m["controller_callback_us"] > 1000
                                            for m in samples
                                        ),
                                        args.budget_us,
                                        over_budget,
                                        len(data),
                                        sum(
                                            m.get("motion_enabled", False) for m in data
                                        ),
                                        sum(
                                            m["controller_callback_us"] > args.budget_us
                                            for m in data
                                        ),
                                        max(
                                            (m["controller_callback_us"] for m in data),
                                            default=float("nan"),
                                        ),
                                        max(
                                            (m.get("dropped_samples", 0) for m in data),
                                            default=0,
                                        ),
                                        max(
                                            (m["max_hard_violation"] for m in samples),
                                            default=float("nan"),
                                        ),
                                    ]
                                    + [
                                        v
                                        for p in phases
                                        for v in stats([m[p] for m in samples])
                                    ]
                                    + [
                                        (
                                            logpath.read_text()[-1800:]
                                            if fault_for_arm
                                            else ""
                                        )
                                    ]
                                )
                                output.flush()
                            print(
                                arms,
                                backend,
                                formulation,
                                "FAULT" if fault else "collected",
                                *[len(m) for m in messages],
                                flush=True,
                            )
                    node.destroy_node()
                    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
