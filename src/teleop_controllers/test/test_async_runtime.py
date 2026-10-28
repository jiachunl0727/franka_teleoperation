"""Actual asynchronous single/dual mock processes: slow and failing worker paths."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from quest_teleop_interfaces.msg import CartesianTeleopCommand, ControllerStatus, CbfStatus


class AsyncRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def run_mode(self, dual=False, infeasible=False):
        prefix = "/async_" + uuid.uuid4().hex[:12]
        sides = ["left", "right"] if dual else ["right"]
        node = Node("test_" + uuid.uuid4().hex)
        statuses = {side: [] for side in sides}
        controls = {side: [] for side in sides}
        pubs = {}
        executable = Path(get_package_prefix("teleop_controllers")) / "lib/teleop_controllers/cartesian_impedance_controller"
        args = [str(executable), "--ros-args", "-p", "backend:=mock", "-p", "cbf.execution:=asynchronous",
                "-p", "cbf.worker_period_ms:=20.0", "-p", "cbf.maximum_failures:=1"]
        if dual:
            args += ["-p", "mode:=dual"]
        else:
            args += ["-r", "__ns:=" + prefix + "_right"]
        for side in sides:
            ns = prefix + "_" + side
            node.create_subscription(CbfStatus, ns + "/cbf_status", statuses[side].append, 100)
            node.create_subscription(ControllerStatus, ns + "/controller_status", controls[side].append, 100)
            pubs[side] = node.create_publisher(CartesianTeleopCommand, ns + "/cartesian_teleop_command", 10)
            stem = side + "." if dual else ""
            if dual:
                args += ["-p", stem + "namespace:=" + ns]
            args += ["-p", stem + "base_frame:=" + ns[1:] + "/base"]
        if infeasible:
            # Deliberately impossible optional obstacle, while the mandatory
            # table remains clear. Failure must hold, rather than terminate.
            args += ["-p", "cbf.obstacle_centers:=[0.4,0.0,0.4]", "-p", "cbf.obstacle_radii:=[3.0]"]
        with tempfile.TemporaryDirectory() as directory:
            logpath = Path(directory) / "owner.log"
            with logpath.open("w") as log:
                process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                sequence = 0

                def drive(active, duration):
                    nonlocal sequence
                    until = time.monotonic() + duration
                    while time.monotonic() < until:
                        sequence += 1
                        for side in sides:
                            msg = CartesianTeleopCommand()
                            msg.header.frame_id = prefix[1:] + "_" + side + "/base"
                            msg.session_id = "async-test"
                            msg.sequence = sequence
                            msg.reference_space_generation = 1
                            msg.source_time_pc_monotonic_ns = time.monotonic_ns()
                            msg.openxr_location_time_ns = msg.source_time_pc_monotonic_ns
                            msg.clock_uncertainty_ns = 1000000
                            msg.source_time_valid = msg.clock_sync_valid = msg.input_valid = True
                            msg.motion_enabled = active
                            msg.trigger_released = not active
                            msg.target_pose.position.x = .405 if active else .4
                            msg.target_pose.position.z = .4
                            msg.target_pose.orientation.w = 1.0
                            pubs[side].publish(msg)
                        end = time.monotonic() + .01
                        while time.monotonic() < end:
                            rclpy.spin_once(node, timeout_sec=.002)
                        self.assertIsNone(process.poll(), logpath.read_text())

                try:
                    deadline = time.monotonic() + 10
                    while time.monotonic() < deadline and not all(statuses[s] and pubs[s].get_subscription_count() for s in sides):
                        rclpy.spin_once(node, timeout_sec=.02)
                        self.assertIsNone(process.poll(), logpath.read_text())
                    self.assertTrue(all(statuses[s] for s in sides), logpath.read_text())
                    drive(False, .25)
                    drive(True, .6)
                    for side in sides:
                        events = statuses[side]
                        self.assertGreater(events[-1].control_cycle - events[0].control_cycle, 200)
                        self.assertTrue(all(e.stop_reason == "none" and e.mandatory_table_guard_enabled for e in events))
                        self.assertTrue(all(e.execution == "asynchronous" for e in events))
                        self.assertTrue(any(e.worker_age_ms > 1 for e in events))
                        if infeasible:
                            self.assertTrue(any(e.braking_hold_active for e in events))
                            self.assertTrue(any(e.fallback_active for e in events))
                            self.assertFalse(controls[side][-1].motion_enabled)
                        else:
                            self.assertTrue(any(c.motion_enabled for c in controls[side]), logpath.read_text())
                            self.assertTrue(any(e.worker_result_usable for e in events))
                    drive(False, .15)
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                        raise
                    node.destroy_node()
                self.assertEqual(process.returncode, 0, logpath.read_text())

    def test_single_slow_worker(self):
        self.run_mode()

    def test_dual_slow_worker(self):
        self.run_mode(dual=True)

    def test_single_failed_worker_holds_without_losing_ownership(self):
        self.run_mode(infeasible=True)

    def test_dual_failed_worker_holds_without_losing_ownership(self):
        self.run_mode(dual=True, infeasible=True)
