"""Actual non-RT Hand worker with mock backend and guarded calibration service."""

import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import uuid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from quest_teleop_interfaces.msg import (
    GripperStatus,
    QuestControllerState,
    QuestButtonState,
)
from quest_teleop_interfaces.srv import CalibrateGripper


class HandWorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_startup_calibration_once_without_automatic_grasp(self):
        prefix = "/auto_hand_" + uuid.uuid4().hex
        node = Node("test_" + uuid.uuid4().hex)
        statuses = []
        node.create_subscription(
            GripperStatus,
            prefix + "/gripper_events",
            statuses.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        with tempfile.TemporaryDirectory() as directory:
            with (Path(directory) / "worker.log").open("w") as log:
                proc = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "teleop_gripper",
                        "franka_hand_worker",
                        "--ros-args",
                        "-r",
                        "__ns:=" + prefix,
                        "-p",
                        "backend:=mock",
                        "-p",
                        "calibrate_on_start:=true",
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline and not (
                        statuses and statuses[-1].calibrated
                    ):
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertTrue(statuses and statuses[-1].calibrated)
                    deadline = time.monotonic() + 0.3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertFalse(statuses[-1].busy)
                    self.assertEqual(statuses[-1].operations_started, 1)
                    self.assertEqual(statuses[-1].operations_completed, 1)
                    self.assertEqual(statuses[-1].state, "unknown")
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=8)
        node.destroy_node()

    def test_mirrored_joystick_duration_neutral_and_persistent_force_hold(self):
        for controller in ("left", "right"):
            with self.subTest(controller=controller):
                self.exercise_joystick(controller)

    def exercise_joystick(self, controller):
        prefix = "/hand_" + uuid.uuid4().hex
        node = Node("test_" + uuid.uuid4().hex)
        statuses = []
        node.create_subscription(
            GripperStatus, prefix + "/gripper_events", statuses.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        publisher = node.create_publisher(QuestControllerState, prefix + "/raw", 10)
        calibration = node.create_client(CalibrateGripper, prefix + "/calibrate_gripper")
        sequence = 0
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "hand.log"
            with log_path.open("w") as log:
                proc = subprocess.Popen(
                    ["ros2", "run", "teleop_gripper", "franka_hand_worker", "--ros-args",
                     "-r", "__ns:=" + prefix, "-p", "backend:=mock",
                     "-p", "controller_id:=" + controller,
                     "-p", "controller_state_topic:=" + prefix + "/raw",
                     "-p", "grasp_force_n:=5.0", "-p", "cable_diameter_m:=0.005",
                     "-p", "mock_object_width_m:=0.005", "-p", "speed_mps:=0.1",
                     "-p", "mock_operation_duration_ms:=20.0"],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                )

                def wait_for(predicate, timeout=8):
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.005)
                        if predicate():
                            return
                        if proc.poll() is not None:
                            break
                    self.fail("Hand condition timed out:\n" + log_path.read_text())

                def send(inward=0., duration=.08, focused=True, active=True, old_inputs=False):
                    nonlocal sequence
                    end = time.monotonic() + duration
                    while time.monotonic() < end:
                        sequence += 1
                        state = QuestControllerState()
                        state.controller_id = controller
                        state.session_id = "joystick-session"
                        state.sequence = sequence
                        state.source_time_pc_monotonic_ns = time.monotonic_ns()
                        state.source_time_valid = state.clock_sync_valid = True
                        state.clock_uncertainty_ns = 1_000_000
                        state.reference_space = "LOCAL"
                        state.reference_space_generation = 1
                        state.connection_valid = True
                        state.session_focused = focused
                        # Pose tracking is deliberately unavailable: the joystick
                        # remains usable independently of arm tracking/trigger.
                        state.thumbstick_active = active
                        state.thumbstick_x = float(-inward if controller == "right" else inward)
                        state.grip_active = True
                        state.grip_value = 1.0 if old_inputs else 0.
                        button = QuestButtonState()
                        button.name, button.active, button.pressed = "thumbstick", True, old_inputs
                        state.buttons = [button]
                        publisher.publish(state)
                        until = time.monotonic() + .01
                        while time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=.001)

                try:
                    wait_for(lambda: statuses and calibration.service_is_ready()
                             and publisher.get_subscription_count())
                    send(.8)
                    self.assertEqual(statuses[-1].operations_started, 0)
                    request = CalibrateGripper.Request()
                    request.empty_hand_confirm = request.operator_confirm = True
                    future = calibration.call_async(request)
                    wait_for(future.done)
                    self.assertTrue(future.result().accepted)
                    wait_for(lambda: statuses[-1].calibrated and not statuses[-1].busy)
                    send(.8)
                    self.assertEqual(statuses[-1].operations_started, 1)
                    send(0, old_inputs=True)
                    self.assertEqual(statuses[-1].operations_started, 1)
                    send(.5, duration=.1)
                    send(0)
                    wait_for(lambda: not statuses[-1].busy and statuses[-1].state == "stopped")
                    width = statuses[-1].width
                    self.assertGreater(width, .055)
                    self.assertLess(width, .08)
                    send(0, duration=.2)
                    self.assertAlmostEqual(statuses[-1].width, width)
                    send(.9, duration=.1)
                    send(.9, focused=False)
                    wait_for(lambda: not statuses[-1].busy and statuses[-1].neutral_required)
                    count = statuses[-1].operations_started
                    send(.9)
                    self.assertEqual(statuses[-1].operations_started, count)
                    send(0)
                    send(.9, duration=1.0)
                    wait_for(lambda: statuses[-1].grasp_confirmed and not statuses[-1].busy)
                    self.assertEqual(statuses[-1].state, "holding")
                    self.assertAlmostEqual(statuses[-1].width, .005)
                    self.assertAlmostEqual(statuses[-1].commanded_grasp_force_n, 5.)
                    send(0, duration=.2)
                    count = statuses[-1].operations_started
                    wait_for(lambda: statuses[-1].neutral_required, timeout=2)
                    self.assertTrue(statuses[-1].grasp_confirmed)
                    self.assertEqual(statuses[-1].operations_started, count)
                    send(-.8)
                    self.assertEqual(statuses[-1].operations_started, count)
                    send(0)
                    send(-.8, duration=.1)
                    send(0)
                    wait_for(lambda: not statuses[-1].busy and statuses[-1].state == "stopped")
                    self.assertFalse(statuses[-1].grasp_confirmed)
                    self.assertGreater(statuses[-1].width, .005)
                    self.assertLess(statuses[-1].width, .04)
                    self.assertEqual(statuses[-1].commanded_grasp_force_n, 0.)
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=8)
                    node.destroy_node()
                self.assertEqual(proc.returncode, 0, log_path.read_text())

    def test_hardware_guard_prevents_connection(self):
        result = subprocess.run(
            [
                "ros2",
                "run",
                "teleop_gripper",
                "franka_hand_worker",
                "--ros-args",
                "-p",
                "backend:=hardware",
                "-p",
                "robot_ip:=192.0.2.1",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("operator_confirm", result.stderr)

    def test_slow_reporting_cannot_delay_release_or_joystick_open(self):
        library = Path(os.environ["HAND_TEST_SLOW_PUBLISH_LIBRARY"])
        self.assertTrue(library.is_file())
        prefix = "/fast_hand_" + uuid.uuid4().hex
        node = Node("test_" + uuid.uuid4().hex)
        statuses = []
        node.create_subscription(
            GripperStatus, prefix + "/gripper_events", statuses.append, 10
        )
        pub = node.create_publisher(QuestControllerState, prefix + "/raw", 10)
        sequence = 0

        def drive(duration, grip=0.0, joystick=False):
            nonlocal sequence
            until, next_send = time.monotonic() + duration, 0
            while time.monotonic() < until:
                if time.monotonic() >= next_send:
                    next_send = time.monotonic() + 0.01
                    sequence += 1
                    s = QuestControllerState()
                    s.controller_id, s.session_id = "right", "slow-report"
                    s.sequence = sequence
                    s.source_time_pc_monotonic_ns = time.monotonic_ns()
                    s.reference_space = "LOCAL"
                    s.reference_space_generation = 1
                    s.source_time_valid = s.clock_sync_valid = True
                    s.connection_valid = s.session_focused = s.grip_active = True
                    s.grip_value = 0.0
                    s.thumbstick_active = True
                    s.thumbstick_x = .8 if joystick else -float(grip)
                    b = QuestButtonState()
                    b.name, b.active, b.pressed = "thumbstick", True, joystick
                    s.buttons = [b]
                    pub.publish(s)
                rclpy.spin_once(node, timeout_sec=0.001)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hand.log"
            with path.open("w") as log:
                proc = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "teleop_gripper",
                        "franka_hand_worker",
                        "--ros-args",
                        "-r",
                        "__ns:=" + prefix,
                        "-p",
                        "backend:=mock",
                        "-p",
                        "calibrate_on_start:=true",
                        "-p",
                        "mock_operation_duration_ms:=10.0",
                        "-p",
                        "grasp_force_n:=5.0",
                        "-p",
                        "controller_state_topic:=" + prefix + "/raw",
                    ],
                    env={**os.environ, "LD_PRELOAD": str(library)},
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline and not (
                        statuses
                        and statuses[-1].calibrated
                        and pub.get_subscription_count()
                    ):
                        drive(0.02)
                    self.assertTrue(
                        statuses and statuses[-1].calibrated, path.read_text()
                    )
                    drive(0.15)
                    drive(0.15, 1.0)
                    drive(0.02, 0.0, True)
                    drive(1.5)
                    self.assertIn(statuses[-1].state, ("stopped", "open"), path.read_text())
                    text = path.read_text()
                    self.assertIn("Hand request: operation=open", text)
                    result = re.search(
                        r"Hand result: operation=close .*stopped=1 width_m=([0-9.e+-]+)",
                        text,
                    )
                    self.assertIsNotNone(result, text)
                    # 150 ms at 20 mm/s should close about 3 mm, even though
                    # every status publish is deliberately delayed by 400 ms.
                    self.assertGreater(float(result.group(1)), 0.075, text)
                    stop = re.search(
                        r"Hand request: operation=stop .*input_age_ms=([0-9.]+)", text
                    )
                    self.assertIsNotNone(stop, text)
                    self.assertLess(float(stop.group(1)), 100, text)
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=8)
                    node.destroy_node()
                self.assertEqual(proc.returncode, 0, path.read_text())


if __name__ == "__main__":
    unittest.main()
