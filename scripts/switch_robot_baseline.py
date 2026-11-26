#!/usr/bin/env python3
"""Read-only check of the saved original switch/EEE/governor configuration."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import direct_robot_network as direct

ROOT = Path(__file__).resolve().parent.parent
SAVED = ROOT / "log/commissioning/2026-11-13-dual-stationary"


def saved_records(host, saved=SAVED):
    paths = {
        "normal": saved / "direct/network-retry-01.json",
        "crossed": saved / "direct/network-crossed-01.json",
        "eee": saved / "review/eee-original.json",
        "governors": saved / "review/governors-original.json",
    }
    records = {key: json.loads(path.read_text()) for key, path in paths.items()}
    for key in ("normal", "crossed"):
        direct.validate_record(host, records[key], direct.LAYOUTS[key])
    for key, feature in (("eee", "eee"), ("governors", "performance")):
        r = records[key]
        if (r.get("version") != 1 or r.get("feature") != feature or
                r.get("interface") != direct.LINKS[0]["dev"] or r.get("boot_id") != host.boot()):
            raise RuntimeError("Original tuning journal has a different schema, feature, interface or boot")
        host.validate(feature, r["before"])
    return records


def check(host, records, ping=False):
    host.identity()
    host.wifi()
    expected_eee = records["eee"]["before"]["enabled"]
    expected_governors = records["governors"]["before"]
    if host.read("eee") != {"enabled": expected_eee}:
        raise RuntimeError("Switch NIC EEE differs from the original setting; run restore_original.sh")
    if host.read("performance") != expected_governors:
        raise RuntimeError("CPU governors differ from the original settings; run restore_original.sh")
    profiles = host.profiles()
    for links in direct.LAYOUTS.values():
        for link in links:
            if link["uuid"] in profiles or host.has_profile_file(link):
                raise RuntimeError("Temporary direct-link profiles remain; run restore_original.sh")
    primary, secondary = direct.LINKS
    host.carrier(primary)
    if host.active(primary["dev"]) != primary["original_uuid"]:
        raise RuntimeError("The original switch-facing NetworkManager profile is not active")
    if host.active(secondary["dev"]) not in (None, secondary["original_uuid"]):
        raise RuntimeError("Unexpected active profile on the former building-network port")
    if host.eee(secondary) != records["normal"]["before"][secondary["dev"]]["eee"]:
        raise RuntimeError("The second NIC EEE differs from its saved original setting")
    addresses = json.loads(host.run("ip", "-j", "-4", "address", "show", "dev", primary["dev"]))
    if not any(a.get("local") == primary["source"] and a.get("prefixlen") == 24
               for d in addresses for a in d.get("addr_info", [])):
        raise RuntimeError("The original 192.168.3.49/24 switch-side address is missing")
    link_state = json.loads(host.run("ip", "-j", "link", "show", "dev", primary["dev"]))
    if link_state[0]["mtu"] != 1500:
        raise RuntimeError("Expected original MTU 1500 on switch-facing NIC")
    speed = host.run("ethtool", primary["dev"])
    if "Speed: 1000Mb/s" not in speed or "Duplex: Full" not in speed:
        raise RuntimeError("Expected original 1 Gb/s full-duplex switch uplink")
    routes = []
    for link in direct.LINKS:
        route = json.loads(host.run("ip", "-j", "-4", "route", "get", link["robot"]))[0]
        if (route.get("dev") != primary["dev"] or route.get("gateway") or
                route.get("prefsrc", route.get("src")) != primary["source"]):
            raise RuntimeError(f"{link['robot']}: expected switch route/source on {primary['dev']}: {route}")
        if ping:
            host.run("ping", "-n", "-I", primary["dev"], "-c", "2", "-W", "1", link["robot"])
        routes.append(route)
    return {"topology": "original shared switch uplink (physical cabling operator-confirmed)",
            "interface": primary["dev"], "eee": {"enabled": expected_eee},
            "governors": expected_governors, "routes": routes, "ping_checked": ping,
            "secondary_active_profile": host.active(secondary["dev"]),
            "secondary_eee_enabled": host.eee(secondary), "temporary_direct_profiles_absent": True,
            "local_fci_connections": host.fci_connections(), "changed": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ping", action="store_true")
    parser.add_argument("--pre-restore", action="store_true",
                        help="Validate journals, identities, Wi-Fi and idle local FCI before restoration")
    args = parser.parse_args()
    host = direct.Host()
    records = saved_records(host)
    if args.pre_restore:
        host.identity()
        host.wifi()
        host.idle()
        result = {"restoration_records_valid": True, "original_eee": records["eee"]["before"],
                  "original_governors": records["governors"]["before"], "changed": False}
    else:
        result = check(host, records, args.ping)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
