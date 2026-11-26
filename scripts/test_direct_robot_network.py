"""Exercise direct-link transactions and route guards without host mutations."""

import copy
import configparser
import json
from pathlib import Path
import tempfile
import sys
import unittest

import direct_robot_network as net


class FakeHost:
    def __init__(self, layout="normal"):
        self.layout = layout
        self.links = net.LAYOUTS[layout]
        self.current = {net.LINKS[0]["dev"]: net.LINKS[0]["original_uuid"],
                        net.LINKS[1]["dev"]: None}
        self.existing = {link["original_uuid"] for link in net.LINKS}
        self.power = {net.LINKS[0]["dev"]: False, net.LINKS[1]["dev"]: True}
        self.fail_activate = None
        self.fail_verify = False
        self.active_fci = False
        self.missing_carrier = False
        self.no_wifi = False
        self.writes = 0
        self.activations = []
        self.fail_create = False

    def identity(self):
        pass

    def boot(self):
        return "test-boot"

    def idle(self):
        if self.active_fci:
            raise RuntimeError("FCI active")

    def wifi(self):
        if self.no_wifi:
            raise RuntimeError("No Wi-Fi")

    def performance(self):
        pass

    def carrier(self, link):
        if self.missing_carrier:
            raise RuntimeError("No carrier")

    def ready(self, link):
        self.carrier(link)

    def active(self, dev):
        return self.current[dev]

    def profiles(self):
        return self.existing.copy()

    def has_profile_file(self, link):
        return False

    def remove_profile(self, link):
        if link['uuid'] in self.existing:
            self.delete(link['uuid'])

    def eee(self, link):
        return self.power[link["dev"]]

    def set_eee(self, link, enabled):
        self.writes += 1
        # Model the carrier-flap hazard: changing EEE on an already active,
        # manual-only direct profile can deactivate that profile.
        if self.power[link["dev"]] != enabled and self.current[link["dev"]] == link["uuid"]:
            self.current[link["dev"]] = None
        self.power[link["dev"]] = enabled

    def create(self, link):
        self.writes += 1
        if self.fail_create:
            raise RuntimeError("connection.uuid cannot be changed")
        self.existing.add(link["uuid"])

    def activate(self, uuid, dev):
        self.writes += 1
        self.activations.append((uuid,dev))
        if uuid == self.fail_activate:
            raise RuntimeError("activation failed")
        self.current[dev] = uuid

    def disconnect(self, dev):
        self.writes += 1
        self.current[dev] = None

    def delete(self, uuid):
        self.writes += 1
        self.existing.remove(uuid)
        for dev,current in self.current.items():
            if current==uuid:self.current[dev]=None

    def verify(self):
        if self.fail_verify or any(self.current[x["dev"]] != x["uuid"] or self.power[x["dev"]]
                                   for x in self.links):
            raise RuntimeError("wrong route or unreachable robot")
        return {"verified": True}


class NetworkTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.state = Path(self.tmp.name) / "baseline.json"
        self.host = FakeHost()

    def assert_original(self, current, power):
        self.assertEqual(self.host.current, current)
        self.assertEqual(self.host.power, power)
        self.assertFalse(any(link["uuid"] in self.host.existing for link in net.LINKS))

    def test_apply_restore_both_profiles_and_eee(self):
        current, power = copy.deepcopy(self.host.current), self.host.power.copy()
        net.apply(self.host, self.state)
        self.assertEqual(set(self.host.power.values()), {False})
        self.assertEqual(self.host.current, {x["dev"]: x["uuid"] for x in net.LINKS})
        net.restore(self.host, self.state)
        self.assert_original(current, power)

    def test_second_activation_failure_restores_first_link(self):
        current, power = self.host.current.copy(), self.host.power.copy()
        self.host.fail_activate = net.LINKS[1]["uuid"]
        with self.assertRaisesRegex(RuntimeError, "rollback: restored"):
            net.apply(self.host, self.state)
        self.assert_original(current, power)
        self.assertEqual(json.loads(self.state.read_text())["status"], "failed")

    def test_create_failure_does_not_bounce_untouched_wired_links(self):
        self.host.current[net.LINKS[1]['dev']] = net.LINKS[1]['original_uuid']
        self.host.fail_create = True
        with self.assertRaisesRegex(RuntimeError, 'connection.uuid'):
            net.apply(self.host,self.state)
        self.assertEqual(self.host.activations, [])

    def test_failed_restore_activation_still_cleans_profile_and_eee(self):
        right=net.LINKS[1]
        self.host.current[right['dev']]=right['original_uuid']
        net.apply(self.host,self.state)
        self.host.fail_activate=right['original_uuid']
        with self.assertRaisesRegex(RuntimeError, 'Restoration incomplete'):
            net.restore(self.host,self.state)
        self.assertTrue(self.host.power[right['dev']])
        self.assertNotIn(right['uuid'],self.host.existing)

    def test_subprocess_error_exposes_actual_stderr(self):
        with self.assertRaisesRegex(RuntimeError, 'specific-profile-error'):
            net.Host().run(sys.executable,'-c',
                           "import sys;sys.stderr.write('specific-profile-error');sys.exit(2)")

    def test_failed_verification_restores_baseline(self):
        current, power = self.host.current.copy(), self.host.power.copy()
        self.host.fail_verify = True
        with self.assertRaisesRegex(RuntimeError, "rollback: restored"):
            net.apply(self.host, self.state)
        self.assert_original(current, power)

    def test_fci_wifi_or_carrier_failure_blocks_all_changes(self):
        for field in ("active_fci", "no_wifi", "missing_carrier"):
            with self.subTest(field=field):
                host = FakeHost()
                setattr(host, field, True)
                with self.assertRaises(RuntimeError):
                    net.apply(host, self.state)
                self.assertEqual(host.writes, 0)
                self.assertFalse(self.state.exists())

    def test_does_not_overwrite_baseline(self):
        self.state.write_text("preserved")
        with self.assertRaises(FileExistsError):
            net.apply(self.host, self.state)
        self.assertEqual(self.host.writes, 0)
        self.assertEqual(self.state.read_text(), "preserved")

    def test_refuses_existing_direct_profiles(self):
        self.host.existing.add(net.LINKS[1]["uuid"])
        with self.assertRaisesRegex(RuntimeError, "already exists"):
            net.apply(self.host, self.state)
        self.assertEqual(self.host.writes, 0)

    def test_snapshot_tampering_rejected_before_mutation(self):
        net.apply(self.host, self.state)
        record = json.loads(self.state.read_text())
        record["before"][net.LINKS[0]["dev"]]["active_uuid"] = "foreign-profile"
        self.state.write_text(json.dumps(record))
        writes = self.host.writes
        with self.assertRaisesRegex(RuntimeError, "unexpected original"):
            net.restore(self.host, self.state)
        self.assertEqual(self.host.writes, writes)

    def test_different_boot_rejected(self):
        net.apply(self.host, self.state)
        self.host.boot = lambda: "new-boot"
        writes = self.host.writes
        with self.assertRaisesRegex(RuntimeError, "mapping/boot"):
            net.restore(self.host, self.state)
        self.assertEqual(self.host.writes, writes)

    def test_foreign_active_profile_not_disconnected(self):
        net.apply(self.host, self.state)
        self.host.current[net.LINKS[1]["dev"]] = "foreign-profile"
        writes = self.host.writes
        with self.assertRaisesRegex(RuntimeError, "unexpected active"):
            net.restore(self.host, self.state)
        self.assertEqual(self.host.writes, writes)

    def test_profile_is_temporary_and_only_adds_host_routes(self):
        for link in net.LINKS:
            profile=configparser.ConfigParser()
            profile.read_string(net.profile_keyfile(link))
            self.assertEqual(profile['connection']['uuid'], link['uuid'])
            self.assertFalse(profile.getboolean('connection','autoconnect'))
            self.assertEqual(profile['ipv4']['address1'],link['source']+'/32')
            self.assertEqual(profile['ipv4']['route1'],link['robot']+'/32')
            self.assertTrue(profile.getboolean('ipv4','never-default'))
            self.assertNotIn('gateway',profile['ipv4'])
            self.assertEqual(net.profile_path(link).parent,Path('/run/NetworkManager/system-connections'))

    def test_create_loads_private_runtime_keyfile_with_fixed_uuid(self):
        original=net.PROFILE_DIR
        net.PROFILE_DIR=Path(self.tmp.name)/'runtime'
        self.addCleanup(setattr,net,'PROFILE_DIR',original)
        host=net.Host();calls=[]
        host.run=lambda *args:calls.append(args) or ''
        host.profiles=lambda:{net.LINKS[0]['uuid']}
        host.create(net.LINKS[0])
        path=net.profile_path(net.LINKS[0])
        self.assertEqual(path.stat().st_mode & 0o777,0o600)
        self.assertEqual(path.read_text(),net.profile_keyfile(net.LINKS[0]))
        self.assertEqual(calls,[('nmcli','--wait','8','connection','load',str(path))])

    def test_failed_load_removes_its_runtime_file(self):
        original=net.PROFILE_DIR
        net.PROFILE_DIR=Path(self.tmp.name)/'runtime'
        self.addCleanup(setattr,net,'PROFILE_DIR',original)
        host=net.Host()
        def fail(*args):raise RuntimeError('load failed')
        host.run=fail
        with self.assertRaisesRegex(RuntimeError,'load failed'):
            host.create(net.LINKS[0])
        self.assertFalse(net.profile_path(net.LINKS[0]).exists())

    def test_cleanup_accepts_networkmanager_timestamp_updates(self):
        original=net.PROFILE_DIR
        net.PROFILE_DIR=Path(self.tmp.name)
        self.addCleanup(setattr,net,'PROFILE_DIR',original)
        link=net.LINKS[0];path=net.profile_path(link)
        path.write_text(net.profile_keyfile(link).replace('autoconnect=false',
                                                        'autoconnect=false\ntimestamp=123'))
        host=net.Host();deleted=[]
        host.profiles=lambda:{link['uuid']}
        host.delete=lambda uid:deleted.append(uid)
        host.remove_profile(link)
        self.assertEqual(deleted,[link['uuid']])
        self.assertFalse(path.exists())

    def test_cleanup_preserves_foreign_runtime_file(self):
        original=net.PROFILE_DIR
        net.PROFILE_DIR=Path(self.tmp.name)
        self.addCleanup(setattr,net,'PROFILE_DIR',original)
        link=net.LINKS[0];path=net.profile_path(link)
        path.write_text(net.profile_keyfile(link).replace(link['uuid'],'foreign-uuid'))
        host=net.Host();deleted=[]
        host.profiles=lambda:{link['uuid']}
        host.delete=lambda uid:deleted.append(uid)
        with self.assertRaisesRegex(RuntimeError,'unexpected contents'):
            host.remove_profile(link)
        self.assertEqual(deleted,[])
        self.assertTrue(path.exists())

    def test_real_route_check_rejects_both_robots_on_one_port(self):
        host = net.Host()
        host.identity = host.wifi = host.performance = lambda: None
        host.carrier = lambda link: None
        host.run = lambda *args: json.dumps([{"dev": "enp2s0f0", "prefsrc": "192.168.3.49"}])
        # Check the second link first to reach the exact shared-subnet mistake.
        host.links = (net.LINKS[1],)
        with self.assertRaisesRegex(RuntimeError, "Wrong direct route/source"):
            host.check()

    def switch_host(self):
        source = net.apply(self.host, self.state)
        self.host.layout = "crossed"
        self.host.links = net.LAYOUTS["crossed"]
        target = self.state.with_name("crossed.json")
        return source, target

    def test_switch_keeps_sources_with_robots_and_preserves_original_journal(self):
        source, target = self.switch_host()
        source_bytes = self.state.read_bytes()
        result = net.switch(self.host, self.state, target)
        self.assertEqual(self.state.read_bytes(), source_bytes)
        self.assertEqual(result["previous_layout"], "normal")
        self.assertEqual(result["status"], "applied")
        for old, new in zip(net.LINKS, self.host.links):
            self.assertEqual((new["source"], new["robot"]), (old["source"], old["robot"]))
            self.assertNotEqual(new["dev"], old["dev"])
            self.assertEqual(self.host.current[new["dev"]], new["uuid"])
            self.assertIn(old["uuid"], self.host.existing)
        net.restore(self.host, target)
        self.assertEqual(self.host.current, {x["dev"]: x["uuid"] for x in net.LINKS})
        self.host.layout, self.host.links = "normal", net.LINKS
        net.restore(self.host, self.state)
        self.assertEqual(self.host.current[net.LINKS[0]["dev"]], net.LINKS[0]["original_uuid"])
        self.assertIsNone(self.host.current[net.LINKS[1]["dev"]])

    def test_failed_crossed_activation_rolls_back_to_normal_profiles(self):
        _, target = self.switch_host()
        before = self.host.current.copy()
        self.host.fail_activate = self.host.links[1]["uuid"]
        with self.assertRaisesRegex(RuntimeError, "rollback: restored"):
            net.switch(self.host, self.state, target)
        self.assertEqual(self.host.current, before)
        self.assertTrue(all(x["uuid"] in self.host.existing for x in net.LINKS))
        self.assertFalse(any(x["uuid"] in self.host.existing for x in self.host.links))

    def test_crossed_verification_failure_restores_normal_profiles_and_eee(self):
        _, target = self.switch_host()
        before, power = self.host.current.copy(), self.host.power.copy()
        self.host.fail_verify = True
        with self.assertRaisesRegex(RuntimeError, "rollback: restored"):
            net.switch(self.host, self.state, target)
        self.assertEqual(self.host.current, before)
        self.assertEqual(self.host.power, power)

    def test_switch_handles_manual_profile_deactivated_by_recabling(self):
        _, target = self.switch_host()
        dev = net.LINKS[1]["dev"]
        self.host.current[dev] = None
        record = net.switch(self.host, self.state, target)
        self.assertIsNone(record["before"][dev]["active_uuid"])
        net.restore(self.host, target)
        self.assertIsNone(self.host.current[dev])
        self.assertTrue(all(x["uuid"] in self.host.existing for x in net.LINKS))

    def test_switch_requires_both_previous_profiles_even_if_inactive(self):
        _, target = self.switch_host()
        previous = net.LINKS[1]
        self.host.current[previous["dev"]] = None
        self.host.existing.remove(previous["uuid"])
        writes = self.host.writes
        with self.assertRaisesRegex(RuntimeError, "previous direct profile"):
            net.switch(self.host, self.state, target)
        self.assertEqual(self.host.writes, writes)
        self.assertFalse(target.exists())

    def test_crossed_route_check_requires_robot_sources_on_crossed_devices(self):
        host = net.Host("crossed")
        host.identity = host.wifi = host.performance = lambda: None
        host.carrier = lambda link: None
        host.eee = lambda link: False
        host.active = lambda dev: next(x["uuid"] for x in host.links if x["dev"] == dev)

        def run(*args):
            if args[:5] == ("ip", "-j", "-4", "route", "get"):
                link = next(x for x in host.links if x["robot"] == args[5])
                return json.dumps([{"dev": link["dev"], "prefsrc": link["source"]}])
            if args[:5] == ("ip", "-j", "-4", "address", "show"):
                link = next(x for x in host.links if x["dev"] == args[-1])
                return json.dumps([{"addr_info": [{"local": link["source"], "prefixlen": 32}]}])
            if args[0] == "ethtool":
                return "Speed: 1000Mb/s\nDuplex: Full\n"
            raise AssertionError(args)

        host.run = run
        result = host.check()
        self.assertEqual(result["layout"], "crossed")
        self.assertEqual([x["route"]["dev"] for x in result["links"]], ["enp1s0", "enp2s0f0"])
        host.links = net.LINKS
        with self.assertRaisesRegex(RuntimeError, "Wrong direct route/source"):
            # Return a valid crossed route while requesting the normal layout.
            host.run = lambda *args: json.dumps([{"dev": "enp1s0", "prefsrc": "192.168.3.49"}])
            host.check()

    def test_switch_rejects_wrong_source_boot_active_profile_or_existing_journal(self):
        for fault in ("boot", "active", "status", "state", "fci"):
            with self.subTest(fault=fault):
                self.host = FakeHost()
                self.state.unlink(missing_ok=True)
                source, target = self.switch_host()
                target.unlink(missing_ok=True)
                if fault == "boot":
                    source["boot_id"] = "previous-boot"
                elif fault == "status":
                    source["status"] = "failed"
                elif fault == "active":
                    self.host.current[net.LINKS[0]["dev"]] = "foreign-profile"
                elif fault == "state":
                    target.write_text("preserved")
                else:
                    self.host.active_fci = True
                self.state.write_text(json.dumps(source))
                writes = self.host.writes
                with self.assertRaises((RuntimeError, FileExistsError)):
                    net.switch(self.host, self.state, target)
                self.assertEqual(self.host.writes, writes)

    def test_original_restore_refuses_while_crossed_layout_is_active(self):
        _, target = self.switch_host()
        net.switch(self.host, self.state, target)
        self.host.layout, self.host.links = "normal", net.LINKS
        writes = self.host.writes
        with self.assertRaisesRegex(RuntimeError, "unexpected active"):
            net.restore(self.host, self.state)
        self.assertEqual(self.host.writes, writes)

    def test_layout_record_detection_accepts_old_normal_journal(self):
        source = copy.deepcopy(net.apply(self.host, self.state))
        source.pop("layout", None)
        self.assertEqual(net.record_layout(source), "normal")
        source["mapping"][0]["dev"] = "foreign-device"
        with self.assertRaisesRegex(RuntimeError, "mapping"):
            net.record_layout(source)


if __name__ == "__main__":
    unittest.main()
