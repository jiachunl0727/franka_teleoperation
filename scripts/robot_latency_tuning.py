#!/usr/bin/env python3
"""Reversible host-only latency experiments; never connects to a robot.

Apply one feature at a time, while both arms are idle. EEE changes can briefly
renegotiate the Ethernet link. The local FCI socket check cannot see another PC.
No persistent system configuration or CBF setting is changed.
"""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


INTERFACE = "enp2s0f0"
POLICIES = Path("/sys/devices/system/cpu/cpufreq")


class Host:
    def run(self, *args):
        result = subprocess.run(args, capture_output=True, text=True, check=True, timeout=5)
        return result.stdout

    def fci_connections(self):
        # Include connection establishment, but allow Desk HTTPS and TIME-WAIT.
        sockets = self.run("ss", "-Htan", "(", "dst", "192.168.3.101", "or",
                           "dst", "192.168.3.109", ")")
        return [line for line in sockets.splitlines()
                if line.split()[0] not in ("TIME-WAIT", "CLOSED") and re.search(
                    r"192\.168\.3\.(101|109):(?:1337|1338)\b", line)]

    def idle(self):
        if self.fci_connections():
            raise RuntimeError("Local FCI connection exists; stop the launch first")

    def read(self, feature):
        if feature == "eee":
            output = self.run("ethtool", "--show-eee", INTERFACE)
            status = re.search(r"EEE status:\s+(enabled|disabled)\b", output)
            if not status:
                raise RuntimeError("Cannot read supported EEE state")
            return {"enabled": status[1] == "enabled"}
        policies = sorted(POLICIES.glob("policy[0-9]*"))
        if not policies:
            raise RuntimeError("No CPU frequency policies found")
        return {p.name: (p / "scaling_governor").read_text().strip()
                for p in policies}

    def validate(self, feature, values):
        if feature == "eee":
            if set(values) != {"enabled"} or type(values["enabled"]) is not bool:
                raise RuntimeError("Invalid EEE snapshot")
            return
        actual = self.read(feature)
        if set(values) != set(actual):
            raise RuntimeError("CPU policy set differs from snapshot")
        for name, governor in values.items():
            if not re.fullmatch(r"policy[0-9]+", name) or not isinstance(governor, str):
                raise RuntimeError("Invalid policy snapshot")
            available = (POLICIES / name / "scaling_available_governors").read_text().split()
            if governor not in available:
                raise RuntimeError(f"Governor {governor!r} unavailable for {name}")

    def write(self, feature, values):
        self.validate(feature, values)
        if feature == "eee":
            self.run("ethtool", "--set-eee", INTERFACE, "eee",
                     "on" if values["enabled"] else "off")
        else:
            for name, governor in values.items():
                (POLICIES / name / "scaling_governor").write_text(governor + "\n")


def target(feature, before):
    return {"enabled": False} if feature == "eee" else dict.fromkeys(before, "performance")


def save(path, record, exclusive=False):
    # Refuse to overwrite a saved baseline. Subsequent status updates keep it.
    def write(stream):
        json.dump(record, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())

    if exclusive:
        with path.open("x") as stream:
            write(stream)
        return
    # A killed process must not truncate the only saved restoration baseline.
    fd, name = tempfile.mkstemp(dir=path.parent, prefix=path.name + ".")
    try:
        os.fchmod(fd, path.stat().st_mode & 0o777)
        with os.fdopen(fd, "w") as stream:
            write(stream)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def change(host, feature, desired, rollback):
    """Read back every change; restore original state on any partial failure."""
    try:
        host.write(feature, desired)
        if host.read(feature) != desired:
            raise RuntimeError("Setting readback differs from requested value")
    except Exception as error:
        try:
            host.write(feature, rollback)
            if host.read(feature) != rollback:
                raise RuntimeError("Rollback readback mismatch")
        except Exception as rollback_error:
            raise RuntimeError(f"Change failed ({error}); rollback also failed "
                               f"({rollback_error}). Saved baseline must be restored.") from error
        raise RuntimeError(f"Change failed; original settings restored: {error}") from error


def apply(host, feature, path):
    host.idle()
    before = host.read(feature)
    after = target(feature, before)
    host.validate(feature, before)
    host.validate(feature, after)
    if before == after:
        raise RuntimeError("Requested setting is already active; no new baseline written")
    record = {"version": 1, "feature": feature, "interface": INTERFACE,
              "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
              "before": before, "after": after, "status": "pending"}
    save(path, record, exclusive=True)
    try:
        change(host, feature, after, before)
    except Exception as error:
        record["status"] = "failed"
        record["error"] = str(error)
        save(path, record)
        raise
    record["status"] = "applied"
    save(path, record)
    return record


def restore(host, path):
    record = json.loads(path.read_text())
    if record.get("version") != 1 or record.get("feature") not in ("eee", "performance"):
        raise RuntimeError("Unknown snapshot format")
    if record.get("interface") != INTERFACE:
        raise RuntimeError("Snapshot belongs to a different interface")
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if record.get("boot_id") != boot_id:
        raise RuntimeError("Snapshot is from a different boot; inspect current settings first")
    feature = record["feature"]
    host.validate(feature, record["before"])
    host.idle()
    current = host.read(feature)
    change(host, feature, record["before"], current)
    record["status"] = "restored"
    save(path, record)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("inspect")
    ap = sub.add_parser("apply")
    ap.add_argument("feature", choices=("eee", "performance"))
    ap.add_argument("--state", type=Path, required=True,
                    help="New JSON file for baseline and restoration")
    rp = sub.add_parser("restore")
    rp.add_argument("--state", type=Path, required=True)
    args = parser.parse_args()
    host = Host()
    if args.action == "inspect":
        result = {"interface": INTERFACE, "eee": host.read("eee"),
                  "governors": host.read("performance"),
                  "local_fci_connections": host.fci_connections(), "changed": False}
    else:
        if os.geteuid() != 0:
            raise RuntimeError("Run apply/restore with sudo on the host, outside Docker")
        result = (apply(host, args.feature, args.state) if args.action == "apply"
                  else restore(host, args.state))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
