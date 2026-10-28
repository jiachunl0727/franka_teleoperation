"""Workspace network defaults must isolate DDS and preserve explicit choices."""

import os
from pathlib import Path
import subprocess
import unittest


class NetworkEnvironmentTest(unittest.TestCase):
    def source_hook(self, **values):
        env = dict(os.environ)
        for key in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "TELEOP_ROS_LOCALHOST_ONLY"):
            env.pop(key, None)
        env.update(values)
        hook = Path(__file__).parents[1] / "env-hooks/teleop_network.sh.in"
        result = subprocess.run(
            [
                "bash",
                "--noprofile",
                "--norc",
                "-c",
                'source "$1"; printf "%s,%s" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY"',
                "network-test",
                str(hook),
            ],
            env=env,
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout

    def test_defaults_isolate_the_local_graph(self):
        self.assertEqual(self.source_hook(), "42,1")

    def test_explicit_test_and_remote_domains_are_preserved(self):
        self.assertEqual(
            self.source_hook(ROS_DOMAIN_ID="137", ROS_LOCALHOST_ONLY="1"), "137,1"
        )
        self.assertEqual(
            self.source_hook(ROS_DOMAIN_ID="73", TELEOP_ROS_LOCALHOST_ONLY="0"), "73,0"
        )

    def test_empty_values_default_and_repeated_sourcing_preserves_choices(self):
        self.assertEqual(
            self.source_hook(ROS_DOMAIN_ID="", ROS_LOCALHOST_ONLY=""), "42,1"
        )
        self.assertEqual(self.source_hook(ROS_DOMAIN_ID="0"), "0,1")

    def test_underlay_network_default_does_not_enable_lab_discovery(self):
        self.assertEqual(self.source_hook(ROS_LOCALHOST_ONLY="0"), "42,1")
