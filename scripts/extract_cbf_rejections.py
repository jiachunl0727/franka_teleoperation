#!/usr/bin/env python3
"""Extract CBF failure snapshots from a saved launch log; never opens hardware."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("launch_log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    decoder = json.JSONDecoder()
    count = 0
    markers = (
        "CBF_PROTECTIVE_STOP_SNAPSHOT ",
        "CBF_CONTROL_TERMINATED_SNAPSHOT ",
        "CBF_TERMINAL_SNAPSHOT ",
    )
    for line in args.launch_log.read_text(errors="replace").splitlines():
        marker = next((item for item in markers if item in line), None)
        if marker is None:
            continue
        payload = line.split(marker, 1)[1]
        snapshot, _ = decoder.raw_decode(payload)
        if (
            snapshot.get("schema") not in ("cbf_failure_v1", "cbf_terminal_v1")
            or not snapshot.get("terminal")
        ):
            raise ValueError("Unsupported CBF failure snapshot schema")
        args.output.mkdir(parents=True, exist_ok=True)
        path = args.output / (
            f"cbf-failure-{snapshot['state_time_pc_monotonic_ns']}-"
            f"cycle-{snapshot['cycle']}.json"
        )
        path.write_text(json.dumps(snapshot, indent=2, allow_nan=False) + "\n")
        print(path)
        count += 1
    if not count:
        parser.exit(
            1,
            "No terminal snapshots found; older recordings cannot recover the missing state.\n",
        )


if __name__ == "__main__":
    main()
