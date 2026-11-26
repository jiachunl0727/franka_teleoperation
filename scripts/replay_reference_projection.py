#!/usr/bin/env python3
"""Read-only recorded reference checks in both shaping modes; no robot or ROS node.

Run inside the sourced ROS environment. The native probe evaluates references
against frozen recorded states; it neither computes nominal torque nor predicts
the physical trajectory after the new reference behavior.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import struct
import subprocess

from replay_avoidance import export_bag


def main():
    from ament_index_python.packages import get_package_prefix
    from rclpy.serialization import deserialize_message
    from quest_teleop_interfaces.msg import TrackingSample

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--arm", default="panda_1")
    parser.add_argument("--cbf-config", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    states = args.output / "recorded-input.bin"
    requests = args.output / "recorded-references.bin"
    provenance = export_bag(args.bag, args.arm, states)
    count = 0
    with requests.open("wb") as stream:
        for source in provenance["source_bags"]:
            path = Path(source["path"])
            with sqlite3.connect(f"file:{path.resolve()}?mode=ro", uri=True) as db:
                topic = db.execute(
                    "SELECT id FROM topics WHERE name=?",
                    (f"/{args.arm}/tracking_samples",),
                ).fetchone()[0]
                for (blob,) in db.execute(
                    "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
                    (topic,),
                ):
                    m = deserialize_message(blob, TrackingSample)
                    p, q, t = (
                        m.requested_pose.position,
                        m.requested_pose.orientation,
                        m.reference_twist,
                    )
                    stream.write(
                        struct.pack(
                            "<13d",
                            p.x,
                            p.y,
                            p.z,
                            q.x,
                            q.y,
                            q.z,
                            q.w,
                            t.linear.x,
                            t.linear.y,
                            t.linear.z,
                            t.angular.x,
                            t.angular.y,
                            t.angular.z,
                        )
                    )
                    count += 1
            if hashlib.sha256(path.read_bytes()).hexdigest() != source["sha256"]:
                raise ValueError("source bag changed during export")
    if count != provenance["samples"]:
        raise ValueError("reference/state count mismatch")
    executable = (
        Path(get_package_prefix("teleop_controllers"))
        / "lib/teleop_controllers/reference_projection_replay"
    )
    with (
        (args.output / "native-results.jsonl").open("w") as out,
        (args.output / "native-checks.log").open("w") as err,
    ):
        result = subprocess.run(
            [
                str(executable),
                str(args.cbf_config),
                str(args.urdf),
                str(states),
                str(requests),
            ],
            stdout=out,
            stderr=err,
        )
    if result.returncode:
        raise RuntimeError(
            f"reference replay failed; inspect {args.output / 'native-checks.log'}"
        )
    summary = dict(
        provenance=provenance,
        profiles=[
            json.loads(line)
            for line in (args.output / "native-results.jsonl").read_text().splitlines()
        ],
    )
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
