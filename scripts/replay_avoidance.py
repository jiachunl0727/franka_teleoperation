#!/usr/bin/env python3
"""Read original ROS sqlite bags and audit native table/singularity CBFs offline.

Run inside the sourced ROS environment. No nodes, hardware or commands are started.
The explicit offline profiles never modify production activation switches.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def export_bag(bag, arm, target):
    from rclpy.serialization import deserialize_message
    from quest_teleop_interfaces.msg import TrackingSample

    databases = [bag] if bag.suffix == ".db3" else sorted(bag.glob("*.db3"))
    if not databases:
        raise ValueError("no sqlite bag files")
    count, previous_cycle, dropped, cycle_gaps = 0, 0, 0, 0
    manifest = []
    with target.open("wb") as stream:
        stream.write(b"AVOID01\n" + struct.pack("<QQ", 0, 54))
        for path in databases:
            before = digest(path)
            with sqlite3.connect(f"file:{path.resolve()}?mode=ro", uri=True) as db:
                if db.execute("PRAGMA quick_check").fetchone()[0] != "ok":
                    raise ValueError("bag integrity failure")
                topic = db.execute("SELECT id,type FROM topics WHERE name=?", (f"/{arm}/tracking_samples",)).fetchone()
                if not topic or topic[1] != "quest_teleop_interfaces/msg/TrackingSample":
                    raise ValueError("missing/incompatible tracking topic")
                for (blob,) in db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id", (topic[0],)):
                    msg = deserialize_message(blob, TrackingSample)
                    if not msg.robot_torque_valid:
                        raise ValueError("no incoming desired torque for actuator interval")
                    if msg.control_cycle <= previous_cycle:
                        raise ValueError("duplicate/out-of-order cycle or concatenated sessions")
                    if previous_cycle:
                        cycle_gaps += msg.control_cycle - previous_cycle - 1
                    previous_cycle = msg.control_cycle
                    dropped = max(dropped, msg.monitor_queue_drops)
                    pose = msg.measured_pose
                    values = [msg.control_cycle, msg.period_seconds]
                    for key in ["joint_position", "joint_velocity", "robot_desired_torque", "nominal_torque", "commanded_torque", "torque_limits"]:
                        values.extend(getattr(msg, key))
                    values += [msg.maximum_torque_rate, pose.position.x, pose.position.y, pose.position.z,
                               pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                               msg.state_time_pc_monotonic_ns, int(msg.motion_enabled)]
                    if len(values) != 54 or not all(math.isfinite(v) for v in values):
                        raise ValueError("invalid recorded tracking sample")
                    stream.write(struct.pack("<54d", *values))
                    count += 1
            if digest(path) != before:
                raise ValueError("source bag changed during read-only export")
            manifest.append(dict(path=str(path.resolve()), sha256=before))
        stream.seek(8)
        stream.write(struct.pack("<Q", count))
    if not count:
        raise ValueError("empty tracking stream")
    return dict(samples=count, source_bags=manifest, missing_control_cycles=cycle_gaps,
                maximum_reported_monitor_drops=dropped)


def summarize(path):
    groups = {name: dict(samples=0, active_samples=0, domain=Counter(), status=Counter(),
                         verified=0, active_verified=0, corrections=0, impossible=0, max_filter_us=0.)
              for name in ["configured", "table_only", "singularity_only", "combined"]}
    metrics = dict(min_table_h_m=1e100, min_table_psi_m_s=1e100, min_sigma=1e100,
                   min_smooth_sigma=1e100, min_singularity_h=1e100, min_singularity_psi=1e100,
                   table_domain_violations=0, singularity_domain_violations=0,
                   recorded_clipping_mismatches=0, max_pose_error=0.)
    for line in path.open():
        row = json.loads(line)
        for dest, source in [("min_table_h_m", "table_h_min_m"), ("min_table_psi_m_s", "table_psi_min_m_s"),
                             ("min_sigma", "sigma_min"), ("min_smooth_sigma", "singularity_smooth_sigma"),
                             ("min_singularity_h", "singularity_h"), ("min_singularity_psi", "singularity_psi")]:
            if row[source] < metrics[dest]:
                metrics[dest] = row[source]
                metrics[dest + "_cycle"] = row["cycle"]
        metrics["table_domain_violations"] += row["table_h_min_m"] < -1e-10 or row["table_psi_min_m_s"] < -1e-10
        metrics["singularity_domain_violations"] += not row["singularity_valid"] or row["singularity_h"] < -1e-10 or row["singularity_psi"] < -1e-10
        metrics["recorded_clipping_mismatches"] += row["recorded_nominal_clip_difference"] > 1e-10
        metrics["max_pose_error"] = max(metrics["max_pose_error"], row["pose_error"])
        for name, profile in row["profiles"].items():
            g = groups[name]
            g["samples"] += 1
            g["active_samples"] += row["active"]
            g["domain"][profile["domain_status"]] += 1
            g["status"][profile["status"]] += 1
            g["verified"] += profile["verified"]
            g["active_verified"] += row["active"] and profile["verified"]
            g["corrections"] += profile["verified"] and profile["correction_norm"] > 1e-6
            g["impossible"] += profile["impossible_rows"] > 0
            g["max_filter_us"] = max(g["max_filter_us"], profile["filter_us"])
    return dict(metrics=metrics, profiles=groups)


def main():
    from ament_index_python.packages import get_package_prefix
    import yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--arm", default="panda_1")
    parser.add_argument("--cbf-config", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--boundary-probes", action="store_true",
                        help="also run explicitly synthetic rest-state challenges near the actual table/singularity boundaries")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    provenance = export_bag(args.bag, args.arm, args.output / "recorded-input.bin")
    cbf = yaml.safe_load(args.cbf_config.read_text())
    geometry = Path(cbf["geometry"]["file"])
    if not geometry.is_absolute():
        geometry = args.cbf_config.resolve().parent / geometry
    for label, path in [("urdf", args.urdf), ("cbf", args.cbf_config), ("geometry", geometry)]:
        provenance[label] = dict(path=str(path.resolve()), sha256=digest(path))
    executable = Path(get_package_prefix("teleop_controllers")) / "lib/teleop_controllers/avoidance_replay"
    with (args.output / "native-results.jsonl").open("w") as out, (args.output / "native-checks.log").open("w") as err:
        result = subprocess.run([str(executable), str(args.cbf_config.resolve()), str(args.urdf.resolve()),
                                 str((args.output / "recorded-input.bin").resolve())], stdout=out, stderr=err)
    if result.returncode:
        raise RuntimeError(f"native replay/derivative checks failed: {args.output / 'native-checks.log'}")
    summary = dict(scope="frozen recorded states and incoming torque intervals; not a closed-loop rollout or hardware qualification",
                   provenance=provenance, **summarize(args.output / "native-results.jsonl"))
    if args.boundary_probes:
        probe = executable.with_name("avoidance_boundary_probe")
        result = subprocess.run([str(probe), str(args.cbf_config.resolve()), str(args.urdf.resolve()),
                                 str((args.output / "recorded-input.bin").resolve())], text=True, capture_output=True)
        (args.output / "synthetic-boundary-results.jsonl").write_text(result.stdout)
        (args.output / "synthetic-boundary-stderr.log").write_text(result.stderr)
        if result.returncode:
            raise RuntimeError("synthetic boundary intervention check failed; see preserved probe results")
        summary["synthetic_boundary_probes"] = [json.loads(row) for row in result.stdout.splitlines()]
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
