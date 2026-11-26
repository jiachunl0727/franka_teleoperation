"""Test transactions and failure paths without changing host settings."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

import robot_latency_tuning as tuning


class FakeHost(tuning.Host):
    def __init__(self, feature="eee"):
        self.feature = feature
        self.values = ({"enabled": True} if feature == "eee" else
                       {"policy0": "schedutil", "policy1": "ondemand"})
        self.fail_once = False
        self.bad_readback_once = False
        self.active = False
        self.writes = 0

    def idle(self):
        if self.active:
            raise RuntimeError("FCI active")

    def read(self, feature):
        return copy.deepcopy(self.values)

    def validate(self, feature, values):
        pass

    def write(self, feature, values):
        self.writes += 1
        if self.fail_once:
            self.fail_once = False
            self.values[next(iter(values))] = next(iter(values.values()))
            raise RuntimeError("partial hardware write failure")
        if self.bad_readback_once:
            self.bad_readback_once = False
            return
        self.values = copy.deepcopy(values)


class TuningTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "state.json"

    def test_eee_apply_restore(self):
        host = FakeHost()
        record = tuning.apply(host, "eee", self.path)
        self.assertEqual(host.values, {"enabled": False})
        self.assertEqual(record["before"], {"enabled": True})
        tuning.restore(host, self.path)
        self.assertEqual(host.values, {"enabled": True})

    def test_restore_each_original_governor(self):
        host = FakeHost("performance")
        before = host.read("performance")
        tuning.apply(host, "performance", self.path)
        self.assertEqual(set(host.values.values()), {"performance"})
        tuning.restore(host, self.path)
        self.assertEqual(host.values, before)

    def test_partial_failure_rolls_back(self):
        host = FakeHost("performance")
        before = host.read("performance")
        host.fail_once = True
        with self.assertRaisesRegex(RuntimeError, "original settings restored"):
            tuning.apply(host, "performance", self.path)
        self.assertEqual(host.values, before)
        self.assertEqual(json.loads(self.path.read_text())["status"], "failed")

    def test_readback_failure_rolls_back(self):
        host = FakeHost()
        host.bad_readback_once = True
        with self.assertRaisesRegex(RuntimeError, "readback"):
            tuning.apply(host, "eee", self.path)
        self.assertEqual(host.values, {"enabled": True})

    def test_failed_rollback_retains_saved_baseline(self):
        host = FakeHost()

        def fail(*args):
            raise RuntimeError("device unavailable")

        host.write = fail
        with self.assertRaisesRegex(RuntimeError, "rollback also failed"):
            tuning.apply(host, "eee", self.path)
        self.assertEqual(json.loads(self.path.read_text())["before"], {"enabled": True})

    def test_active_fci_blocks_apply_and_restore(self):
        host = FakeHost()
        host.active = True
        with self.assertRaises(RuntimeError):
            tuning.apply(host, "eee", self.path)
        self.assertFalse(self.path.exists())
        self.assertEqual(host.writes, 0)
        host.active = False
        tuning.apply(host, "eee", self.path)
        host.active = True
        with self.assertRaises(RuntimeError):
            tuning.restore(host, self.path)
        self.assertEqual(host.writes, 1)

    def test_baseline_never_overwritten(self):
        host = FakeHost()
        self.path.write_text("preserved")
        with self.assertRaises(FileExistsError):
            tuning.apply(host, "eee", self.path)
        self.assertEqual(host.writes, 0)
        self.assertEqual(self.path.read_text(), "preserved")

    def test_unknown_boot_rejected(self):
        host = FakeHost()
        tuning.apply(host, "eee", self.path)
        record = json.loads(self.path.read_text())
        record["boot_id"] = "another-boot"
        self.path.write_text(json.dumps(record))
        with self.assertRaisesRegex(RuntimeError, "different boot"):
            tuning.restore(host, self.path)
        self.assertEqual(host.writes, 1)

    def test_socket_parser_allows_desk_but_blocks_fci(self):
        host = tuning.Host()
        for state in ("ESTAB", "SYN-SENT", "FIN-WAIT-1"):
            host.run = lambda *args: f"{state} 0 0 192.168.3.49:4567 192.168.3.101:1337"
            with self.assertRaisesRegex(RuntimeError, "FCI"):
                host.idle()
        host.run = lambda *args: "ESTAB 0 0 192.168.3.49:4567 192.168.3.109:443"
        host.idle()

    def test_invalid_eee_snapshot(self):
        with self.assertRaisesRegex(RuntimeError, "Invalid"):
            tuning.Host().validate("eee", {"enabled": "yes"})


if __name__ == "__main__":
    unittest.main()
