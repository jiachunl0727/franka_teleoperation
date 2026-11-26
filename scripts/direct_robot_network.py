#!/usr/bin/env python3
"""Temporary, reversible direct Panda links on this commissioning PC.

Apply only after stopping all FCI launches and cabling each Panda directly to
its assigned port. Wi-Fi stays in place. No robot SDK or control is opened.
"""

import argparse
import configparser
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from robot_latency_tuning import Host as TuningHost, save


LINKS = (
    {"side": "left", "dev": "enp2s0f0", "mac": "a8:a1:59:d0:90:ab",
     "source": "192.168.3.49", "robot": "192.168.3.101",
     "uuid": "979f9f32-b459-4ccb-8bba-a7845c010101",
     "original_uuid": "3faeb575-e317-39fc-9c63-c9c9804b4fe6"},
    {"side": "right", "dev": "enp1s0", "mac": "a8:a1:59:d0:90:aa",
     "source": "192.168.3.50", "robot": "192.168.3.109",
     "uuid": "979f9f32-b459-4ccb-8bba-a7845c010109",
     "original_uuid": "c0d5d41d-93fb-34a1-93ac-8f7cd905ecc8"},
)
# Keep addresses tied to robots and device identities tied to physical NICs.
# Separate runtime UUIDs/files let a crossover roll back to the intact normal
# profiles without briefly restoring the building port's DHCP configuration.
CROSSED_LINKS = tuple(
    {**robot, **{key: nic[key] for key in ("dev", "mac", "original_uuid")},
     "uuid": "979f9f32-b459-4ccb-8bba-a7845c01020" + ("1" if i == 0 else "9"),
     "profile_name": "teleop-direct-crossed-" + robot["side"]}
    for i, (robot, nic) in enumerate(zip(LINKS, reversed(LINKS)))
)
LAYOUTS = {"normal": LINKS, "crossed": CROSSED_LINKS}
WIFI = "wlp3s0"
PROFILE_DIR = Path("/run/NetworkManager/system-connections")


def profile_name(link):
    return link.get("profile_name", "teleop-direct-" + link["side"])


def profile_keyfile(link):
    # nmcli 1.36 cannot set connection.uuid via `connection add`. Loading a
    # runtime keyfile preserves the fixed UUID without modifying old profiles.
    return f"""[connection]
id={profile_name(link)}
uuid={link['uuid']}
type=ethernet
interface-name={link['dev']}
autoconnect=false

[ethernet]
mac-address={link['mac']}
mtu=1500

[ipv4]
method=manual
address1={link['source']}/32
route1={link['robot']}/32
route-metric=50
never-default=true
ignore-auto-dns=true
may-fail=false

[ipv6]
method=disabled
"""


def profile_path(link):
    return PROFILE_DIR / (profile_name(link) + ".nmconnection")


class Host(TuningHost):
    def __init__(self, layout="normal"):
        self.layout = layout
        self.links = LAYOUTS[layout]

    def run(self, *args):
        try:
            return subprocess.run(args, capture_output=True, text=True, check=True,
                                  timeout=12, env={**os.environ, "LC_ALL": "C"}).stdout
        except subprocess.CalledProcessError as error:
            detail = (error.stderr or error.stdout or "No error output").strip()
            raise RuntimeError(f"{args[0]} exited {error.returncode}: {detail}") from error

    def boot(self):
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()

    def active(self, dev):
        return self.run("nmcli", "--wait", "5", "-g", "GENERAL.CON-UUID",
                        "device", "show", dev).strip() or None

    def profiles(self):
        return set(self.run("nmcli", "--wait", "5", "-g", "UUID",
                            "connection", "show").splitlines())

    def identity(self):
        for link in self.links:
            dev = link["dev"]
            mac = Path(f"/sys/class/net/{dev}/address").read_text().strip().lower()
            if mac != link["mac"]:
                raise RuntimeError(f"{dev}: physical MAC differs from prepared mapping")
            managed = self.run("nmcli", "--wait", "5", "-g", "GENERAL.NM-MANAGED",
                               "device", "show", dev).strip()
            if managed != "yes":
                raise RuntimeError(f"{dev}: NetworkManager management is required")

    def wifi(self):
        routes = json.loads(self.run("ip", "-j", "-4", "route", "show", "default"))
        if not any(r.get("dev") == WIFI for r in routes):
            raise RuntimeError("No Wi-Fi default route; establish Wi-Fi before changing Ethernet")

    def performance(self):
        if set(self.read("performance").values()) != {"performance"}:
            raise RuntimeError("Keep all CPU governors in performance mode for this comparison")

    def carrier(self, link):
        if Path(f"/sys/class/net/{link['dev']}/carrier").read_text().strip() != "1":
            raise RuntimeError(f"{link['dev']}: no carrier; connect the assigned robot cable first")

    def ready(self, link):
        for attempt in range(16):
            try:
                self.carrier(link)
                return
            except (RuntimeError, OSError):
                if attempt == 15:
                    raise
                time.sleep(0.5)

    def eee(self, link):
        text = self.run("ethtool", "--show-eee", link["dev"])
        if "EEE status: enabled" in text:
            return True
        if "EEE status: disabled" in text:
            return False
        raise RuntimeError(f"Cannot read EEE state on {link['dev']}")

    def set_eee(self, link, enabled):
        self.run("ethtool", "--set-eee", link["dev"], "eee", "on" if enabled else "off")
        if self.eee(link) != enabled:
            raise RuntimeError(f"{link['dev']}: EEE readback failed")

    def create(self, link):
        PROFILE_DIR.mkdir(parents=True, exist_ok=True)
        path = profile_path(link)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        try:
            with os.fdopen(fd, "w") as stream:
                stream.write(profile_keyfile(link))
                stream.flush()
                os.fsync(stream.fileno())
            self.run("nmcli", "--wait", "8", "connection", "load", str(path))
            if link["uuid"] not in self.profiles():
                raise RuntimeError("Loaded profile UUID differs from the prepared UUID")
        except (RuntimeError, OSError, subprocess.SubprocessError):
            path.unlink(missing_ok=True)
            raise

    def has_profile_file(self, link):
        return profile_path(link).exists()

    def remove_profile(self, link):
        path = profile_path(link)
        if path.exists():
            # NetworkManager can normalize a keyfile or add its timestamp after
            # activation. Check ownership identifiers, not byte-for-byte text.
            try:
                data = configparser.ConfigParser()
                data.read_string(path.read_text())
                owned = (data.get('connection', 'uuid') == link['uuid'] and
                         data.get('connection', 'id') == profile_name(link) and
                         data.get('connection', 'interface-name') == link['dev'])
            except configparser.Error:
                owned = False
            if not owned:
                raise RuntimeError(f"{path}: unexpected contents; preserving the file")
        if link["uuid"] in self.profiles():
            self.delete(link["uuid"])
        # Also clean a file left by interruption before `connection load`.
        path.unlink(missing_ok=True)

    def activate(self, uuid, dev):
        self.run("nmcli", "--wait", "8", "connection", "up", "uuid", uuid, "ifname", dev)

    def disconnect(self, dev):
        self.run("nmcli", "--wait", "8", "device", "disconnect", dev)

    def delete(self, uuid):
        self.run("nmcli", "--wait", "5", "connection", "delete", "uuid", uuid)

    def check(self, ping=False):
        self.identity()
        self.wifi()
        self.performance()
        result = []
        for link in self.links:
            self.carrier(link)
            route = json.loads(self.run("ip", "-j", "-4", "route", "get", link["robot"]))[0]
            if (route.get("dev") != link["dev"] or
                    route.get("prefsrc", route.get("src")) != link["source"] or route.get("gateway")):
                raise RuntimeError(f"Wrong direct route/source for {link['robot']}: {route}")
            addresses = json.loads(self.run("ip", "-j", "-4", "address", "show", "dev", link["dev"]))
            expected = any(a.get("local") == link["source"] and a.get("prefixlen") == 32
                           for d in addresses for a in d.get("addr_info", []))
            if not expected or self.active(link["dev"]) != link["uuid"]:
                raise RuntimeError(f"{link['dev']}: prepared direct profile/address is not active")
            speed = self.run("ethtool", link["dev"])
            if "Speed: 1000Mb/s" not in speed or "Duplex: Full" not in speed:
                raise RuntimeError(f"{link['dev']}: expected a 1 Gb/s full-duplex robot link")
            if self.eee(link):
                raise RuntimeError(f"{link['dev']}: EEE must stay disabled for this comparison")
            if ping:
                self.run("ping", "-n", "-I", link["dev"], "-c", "2", "-W", "1", link["robot"])
            result.append({**link, "carrier": True, "eee_enabled": False,
                           "route": route, "ping_checked": ping})
        return {"layout": self.layout, "links": result,
                "wifi_default_present": True, "changed": False}

    def verify(self):
        # Link activation/negotiation can take a moment to settle.
        for attempt in range(8):
            try:
                return self.check(ping=True)
            except (RuntimeError, OSError, subprocess.SubprocessError):
                if attempt == 7:
                    raise
                time.sleep(0.5)


def record_layout(record):
    for name, links in LAYOUTS.items():
        if record.get("mapping") == list(links):
            if record.get("layout", name) != name:
                break
            return name
    raise RuntimeError("Snapshot mapping differs from a prepared layout")


def validate_record(host, record, links=None):
    links = host.links if links is None else links
    layout = record_layout(record)
    if (record.get("schema") != "direct_robot_network_v1" or record.get("mapping") != list(links)
            or record.get("boot_id") != host.boot()):
        raise RuntimeError("Snapshot mapping/boot differs from this prepared setup")
    previous = record.get("previous_layout")
    if previous is not None and (previous not in LAYOUTS or previous == layout):
        raise RuntimeError("Snapshot contains an invalid previous layout")
    previous_by_dev = {x["dev"]: x["uuid"] for x in LAYOUTS[previous]} if previous else {}
    if set(record.get("before", {})) != {link["dev"] for link in links}:
        raise RuntimeError("Incomplete restoration baseline")
    for link in links:
        before = record["before"][link["dev"]]
        allowed = (None, previous_by_dev[link["dev"]]) if previous else (None, link["original_uuid"])
        if (before.get("active_uuid") not in allowed or
                type(before.get("eee")) is not bool):
            raise RuntimeError("Snapshot contains an unexpected original profile or EEE value")
    host.identity()


def restore_settings(host, record, rollback=False):
    # Validate everything before changing anything; never delete foreign profiles.
    validate_record(host, record)
    host.idle()
    for link in host.links:
        if host.active(link["dev"]) not in (None, link["uuid"], link["original_uuid"],
                                             record["before"][link["dev"]]["active_uuid"]):
            raise RuntimeError(f"{link['dev']}: unexpected active profile; no restoration attempted")
    errors = []
    for link in host.links:
        before = record["before"][link["dev"]]
        touched = not rollback or link["dev"] in record.get("touched", [])
        if touched:
            try:
                if before["active_uuid"] and host.active(link["dev"]) != before["active_uuid"]:
                    host.activate(before["active_uuid"], link["dev"])
                elif not before["active_uuid"] and host.active(link["dev"]):
                    host.disconnect(link["dev"])
            except (RuntimeError, OSError, subprocess.SubprocessError) as error:
                errors.append(f"{link['dev']}: {error}")
        # Cleanup and power restoration must still run if DHCP activation fails.
        try:
            host.remove_profile(link)
        except (RuntimeError, OSError, subprocess.SubprocessError) as error:
            errors.append(f"{link['dev']}: {error}")
        if touched:
            try:
                if host.eee(link) != before["eee"]:
                    host.set_eee(link, before["eee"])
            except (RuntimeError, OSError, subprocess.SubprocessError) as error:
                errors.append(f"{link['dev']}: {error}")
    if errors:
        raise RuntimeError("Restoration incomplete: " + "; ".join(errors))


def apply(host, state, previous_layout=None):
    if state.exists():
        raise FileExistsError(f"Saved baseline already exists: {state}. Preserve it; "
                              "use a new --state filename after a failed attempt.")
    host.identity()
    host.idle()
    host.wifi()
    host.performance()
    profiles = host.profiles()
    before = {}
    previous_by_dev = ({x["dev"]: x["uuid"] for x in LAYOUTS[previous_layout]}
                       if previous_layout else {})
    for link in host.links:
        host.carrier(link)
        if previous_layout and previous_by_dev[link["dev"]] not in profiles:
            raise RuntimeError(f"{link['dev']}: previous direct profile is missing; review setup")
        if link["uuid"] in profiles or host.has_profile_file(link):
            raise RuntimeError("A prepared direct profile already exists; restore it first")
        active = host.active(link["dev"])
        # Manual-only profiles can deactivate when the cable is unplugged.
        # Accept that state only while the journaled previous profiles remain
        # installed; preserve actual inactivity in the new rollback baseline.
        allowed = (None, previous_by_dev[link["dev"]]) if previous_layout else (None, link["original_uuid"])
        if active not in allowed:
            raise RuntimeError(f"{link['dev']}: unexpected profile; original setup must be reviewed")
        before[link["dev"]] = {"active_uuid": active, "eee": host.eee(link)}
    record = {"schema": "direct_robot_network_v1", "boot_id": host.boot(),
              "layout": host.layout, "mapping": list(host.links), "before": before,
              "status": "pending", "touched": []}
    if previous_layout:
        record["previous_layout"] = previous_layout
    save(state, record, exclusive=True)
    try:
        for link in host.links:
            host.create(link)
        for link in host.links:
            record["touched"].append(link["dev"])
            save(state, record)
            # Renegotiate before the manual-only profile becomes active;
            # otherwise a carrier flap can deactivate a non-autoconnect profile.
            if host.eee(link):
                host.set_eee(link, False)
            host.ready(link)
            host.activate(link["uuid"], link["dev"])
        record["verification"] = host.verify()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        record["status"] = "failed"
        record["error"] = str(error)
        try:
            restore_settings(host, record, rollback=True)
            record["rollback"] = "restored"
        except (RuntimeError, OSError, subprocess.SubprocessError) as rollback_error:
            record["rollback"] = str(rollback_error)
        save(state, record)
        raise RuntimeError(f"Direct setup failed: {error}; rollback: {record['rollback']}. "
                           f"Baseline retained at {state}") from error
    record["status"] = "applied"
    save(state, record)
    return record


def switch(host, from_state, state):
    """Switch only from a journaled, still-active direct layout; no FCI reads.

    Leave the old profiles and journal intact. Restoring the NEW journal first
    returns to those profiles; the original journal remains the final baseline.
    Do not ping the old layout: the operator has already crossed the cables.
    """
    source = json.loads(from_state.read_text())
    previous = record_layout(source)
    validate_record(host, source, LAYOUTS[previous])
    if source.get("status") != "applied" or previous == host.layout:
        raise RuntimeError("Switch requires an applied journal for the other layout")
    return apply(host, state, previous_layout=previous)


def restore(host, state):
    record = json.loads(state.read_text())
    restore_settings(host, record)
    record["status"] = "restored"
    save(state, record)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    for name in ("plan", "apply", "restore", "switch", "check"):
        p = sub.add_parser(name)
        p.add_argument("--layout", choices=LAYOUTS,
                       help="Default normal; restore otherwise infers layout from its journal")
        if name in ("apply", "restore", "switch"):
            p.add_argument("--state", type=Path, required=True)
        if name == "switch":
            p.add_argument("--from-state", type=Path, required=True)
        if name == "check":
            p.add_argument("--ping", action="store_true")
    args = parser.parse_args()
    layout = args.layout or "normal"
    if args.action == "restore" and args.layout is None:
        layout = record_layout(json.loads(args.state.read_text()))
    host = Host(layout)
    if args.action == "plan":
        host.identity()
        host.wifi()
        result = {"layout": layout, "mapping": list(host.links), "original_active_profiles": {
            link["dev"]: host.active(link["dev"]) for link in host.links}, "changed": False}
    elif args.action == "check":
        result = host.check(ping=args.ping)
    else:
        if os.geteuid() != 0:
            raise RuntimeError("Run apply/switch/restore with sudo on the host, outside Docker")
        if args.action == "switch":
            result = switch(host, args.from_state, args.state)
        else:
            result = apply(host, args.state) if args.action == "apply" else restore(host, args.state)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
