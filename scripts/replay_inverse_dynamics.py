#!/usr/bin/env python3
"""Pair recorded references with complete models and replay the actual C++ CBF.

Run in the sourced ROS environment. Opens the bag read-only, never starts ROS
nodes or hardware. Sparse failure-selected captures do not estimate a trial rate.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess

from ament_index_python.packages import get_package_prefix
from rclpy.serialization import deserialize_message
from quest_teleop_interfaces.msg import TrackingSample
import yaml


def pose_values(pose):
    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]


def twist_values(twist):
    return [
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--problems", type=Path, required=True)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--cbf-config", type=Path, required=True)
    parser.add_argument("--arm", default="panda_1")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    problems = sorted(args.problems.glob("*.qp.txt"))
    if not problems:
        parser.error("no complete problem captures")
    selected = {}
    for problem in problems:
        cycle = int(problem.read_text().splitlines()[0].split()[3])
        model = problem.with_suffix(".model.txt")
        if not model.is_file() or cycle in selected:
            raise ValueError("missing or duplicate model capture")
        selected[cycle] = (problem.resolve(), model.resolve())
    bag = args.bag if args.bag.suffix == ".db3" else next(args.bag.glob("*.db3"))
    db = sqlite3.connect(f"file:{bag.resolve()}?mode=ro", uri=True)
    if db.execute("PRAGMA quick_check").fetchone()[0] != "ok":
        raise ValueError("bag integrity check failed")
    topic = f"/{args.arm}/tracking_samples"
    topic_id = db.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()[0]
    references = {}
    total = active = 0
    for (blob,) in db.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY id", (topic_id,)
    ):
        msg = deserialize_message(blob, TrackingSample)
        total += 1
        active += int(msg.motion_enabled)
        if msg.control_cycle in selected:
            if msg.control_cycle in references or not msg.cartesian_reference_valid:
                raise ValueError("duplicate cycle or invalid Cartesian reference")
            references[msg.control_cycle] = msg
    if references.keys() != selected.keys():
        raise ValueError("bag does not contain every captured reference cycle")
    data = yaml.safe_load(args.runtime_config.read_text())
    settings = dict(data.get("controller", {}))
    settings.update(
        data.get("arm_settings", {}).get(args.arm, {}).get("controller", {})
    )
    k = settings.get(
        "inverse_dynamics_stiffness",
        [
            settings.get("lambda_translation", 50.0)
            * settings.get("gain_translation", 20.0)
        ]
        * 3
        + [settings.get("lambda_rotation", 40.0) * settings.get("gain_rotation", 2.5)]
        * 3,
    )
    gains = k + [settings.get("inverse_dynamics_damping_ratio", 1.0)]
    gains += settings.get("inverse_dynamics_nullspace_stiffness", [10.0] * 7)
    gains += settings.get("inverse_dynamics_nullspace_damping", [6.324555320336759] * 7)
    gains += settings.get(
        "home_joint_position", [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
    )
    gains += [
        int(settings.get("cartesian_wrench_limits_enabled", False)),
        settings.get("cartesian_force_limit", 10.0),
        settings.get("cartesian_moment_limit", 1.5),
    ]
    lines = ["CONTROLLER_REPLAY_V2", " ".join(map(str, gains))]
    max_nominal_difference = 0.0
    for cycle in sorted(selected):
        problem, model = selected[cycle]
        msg = references[cycle]
        model_lines = model.read_text().splitlines()
        for line, values in (
            (1, msg.joint_position),
            (2, msg.joint_velocity),
            (5, msg.nominal_torque),
        ):
            stored = list(map(float, model_lines[line].split()))
            delta = max(abs(a - b) for a, b in zip(stored, values))
            if delta > 1e-10:
                raise ValueError(f"recorded model/telemetry mismatch at cycle {cycle}")
            if line == 5:
                max_nominal_difference = max(max_nominal_difference, delta)
        fields = pose_values(msg.measured_pose) + pose_values(msg.reference_pose)
        fields += (
            twist_values(msg.reference_twist)
            + list(msg.torque_limits)
            + [msg.maximum_torque_rate]
        )
        lines.append(
            f"{json.dumps(str(model))} {json.dumps(str(problem))} {cycle} "
            + " ".join(format(v, ".17g") for v in fields)
        )
    replay_input = args.output / "paired-input.txt"
    replay_input.write_text("\n".join(lines) + "\n")
    executable = (
        Path(get_package_prefix("teleop_controllers"))
        / "lib/teleop_controllers/controller_cbf_replay"
    )
    result = subprocess.run(
        [str(executable), str(args.cbf_config.resolve()), str(replay_input.resolve())],
        text=True,
        capture_output=True,
        check=True,
    )
    (args.output / "native-results.jsonl").write_text(result.stdout)
    (args.output / "native-stderr.log").write_text(result.stderr)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    summary = {
        "scope": "frozen recorded states and actuator intervals; failure-selected complete captures",
        "not_a_full_trial_acceptance_estimate": True,
        "bag_tracking_samples": total,
        "bag_active_samples": active,
        "paired_complete_captures": len(references),
        "recorded_nominal_pairing_max_abs_error": max_nominal_difference,
        "cbf_config": str(args.cbf_config.resolve()),
        "cbf_config_sha256": hashlib.sha256(args.cbf_config.read_bytes()).hexdigest(),
        "runtime_config": str(args.runtime_config.resolve()),
        "runtime_config_sha256": hashlib.sha256(
            args.runtime_config.read_bytes()
        ).hexdigest(),
        "settings": settings,
        "execution": "synchronous frozen-problem replay; original constraints, model, solver and budgets",
        "laws": {},
    }
    for name in ("recorded_impedance", "inverse_dynamics"):
        group = [r for r in rows if r["law"] == name]
        accepted = sum(r["outgoing_verified"] for r in group)
        summary["laws"][name] = {
            "samples": len(group),
            "accepted_and_outgoing_verified": accepted,
            "acceptance_percent": 100 * accepted / len(group),
            "bounded_nominal_feasible": sum(
                r["bounded_nominal_feasible"] for r in group
            ),
            "status": dict(Counter(r["status"] for r in group)),
            "samples_with_individually_impossible_rows": sum(
                r["box_impossible_rows"] > 0 for r in group
            ),
            "max_filter_us": max(r["filter_us"] for r in group),
            "max_controller_us": max(r["controller_us"] for r in group),
        }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "settings"}, indent=2))


if __name__ == "__main__":
    main()
