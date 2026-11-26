"""Original-setting gates without host mutations or robot communication."""
import json
from pathlib import Path
import tempfile
import unittest

import direct_robot_network as direct
import switch_robot_baseline as baseline


class FakeHost:
    def __init__(self):
        self.eee_enabled = True
        self.governors = {"policy0": "schedutil", "policy1": "schedutil"}
        self.active_profiles = {x["dev"]: x["original_uuid"] for x in direct.LINKS}
        self.existing = set(self.active_profiles.values())
        self.route_device = "enp2s0f0"
        self.calls = []

    def boot(self): return "test-boot"
    def identity(self): pass
    def wifi(self): pass
    def carrier(self, link): pass
    def profiles(self): return self.existing
    def has_profile_file(self, link): return False
    def active(self, dev): return self.active_profiles[dev]
    def eee(self, link): return self.eee_enabled if link["dev"] == "enp2s0f0" else True
    def fci_connections(self): return []
    def validate(self, feature, values): pass
    def read(self, feature):
        return {"enabled": self.eee_enabled} if feature == "eee" else self.governors

    def run(self, *args):
        self.calls.append(args)
        if args[:5] == ("ip", "-j", "-4", "route", "get"):
            return json.dumps([{"dev": self.route_device, "prefsrc": "192.168.3.49"}])
        if args[:5] == ("ip", "-j", "-4", "address", "show"):
            return json.dumps([{"addr_info": [{"local": "192.168.3.49", "prefixlen": 24}]}])
        if args[:4] == ("ip", "-j", "link", "show"):
            return json.dumps([{"mtu": 1500}])
        if args[0] == "ethtool": return "Speed: 1000Mb/s\nDuplex: Full\n"
        if args[0] == "ping": return "2 received"
        raise AssertionError(args)


class BaselineTests(unittest.TestCase):
    def setUp(self):
        self.host = FakeHost()
        self.records = {
            "eee": {"before": {"enabled": True}},
            "governors": {"before": self.host.governors.copy()},
            "normal": {"before": {x["dev"]: {"eee": x["dev"] != "enp2s0f0"} for x in direct.LINKS}},
        }

    def test_shared_switch_routes_and_exact_original_tuning_pass(self):
        result = baseline.check(self.host, self.records, ping=True)
        self.assertFalse(result["changed"])
        self.assertTrue(result["eee"]["enabled"])
        self.assertEqual(set(result["governors"].values()), {"schedutil"})
        self.assertEqual(sum(x[0] == "ping" for x in self.host.calls), 2)

    def test_eee_performance_direct_profiles_or_wrong_route_block_launch(self):
        for fault in ("eee", "performance", "profiles", "route"):
            with self.subTest(fault=fault):
                host = FakeHost()
                if fault == "eee": host.eee_enabled = False
                elif fault == "performance": host.governors["policy0"] = "performance"
                elif fault == "profiles": host.existing.add(direct.CROSSED_LINKS[0]["uuid"])
                else: host.route_device = "enp1s0"
                with self.assertRaises(RuntimeError):
                    baseline.check(host, self.records)
                self.assertFalse(any(x[0] == "ping" for x in host.calls))

    def test_unknown_active_profile_blocks_launch(self):
        self.host.active_profiles["enp2s0f0"] = "foreign-profile"
        with self.assertRaisesRegex(RuntimeError, "original switch-facing"):
            baseline.check(self.host, self.records)

    def test_wrong_boot_journal_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            saved = Path(folder)
            (saved / "direct").mkdir()
            (saved / "review").mkdir()
            for source in (baseline.SAVED / "direct").glob("network-*.json"):
                value = json.loads(source.read_text())
                value["boot_id"] = "different-boot"
                (saved / "direct" / source.name).write_text(json.dumps(value))
            for name in ("eee-original.json", "governors-original.json"):
                value = json.loads((baseline.SAVED / "review" / name).read_text())
                (saved / "review" / name).write_text(json.dumps(value))
            with self.assertRaisesRegex(RuntimeError, "mapping/boot"):
                baseline.saved_records(self.host, saved)


if __name__ == "__main__":
    unittest.main()
