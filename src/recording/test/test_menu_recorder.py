"""Recorder completion races and bounded finalization, without a bag subprocess."""

import io
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.parameter import Parameter
from recording.menu_recorder import MenuRecorder
from quest_teleop_interfaces.msg import QuestControllerState


class MenuRecorderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.node = MenuRecorder(
            parameter_overrides=[Parameter("output_root", value=self.temp.name)]
        )
        self.node.output = str(Path(self.temp.name) / "previous")
        Path(self.node.output).mkdir()
        self.node.log = io.StringIO()
        self.node.process = Mock(pid=100000000)
        self.node.process.poll.return_value = None

    def tearDown(self):
        if self.node.log:
            self.node.log.close()
        self.node.log = self.node.process = None
        self.node.destroy_node()
        self.temp.cleanup()

    def test_fresh_click_collects_finished_bag_before_deciding_whether_to_start(self):
        self.node.phase = "stopping"
        self.node.process.poll.return_value = 0
        (Path(self.node.output) / "metadata.yaml").write_text(
            "rosbag2_bagfile_information: {}"
        )
        self.node.gate.update = Mock(return_value=True)  # A fresh, accepted press.
        self.node.start = Mock()
        self.node.on_input(QuestControllerState())
        self.node.start.assert_called_once()

    def test_stuck_finalization_escalates_once_and_can_be_restarted_after_exit(self):
        self.node.phase = "recording"
        with patch("recording.menu_recorder.os.killpg") as kill:
            with patch(
                "recording.menu_recorder.time.monotonic_ns", return_value=1000000000
            ):
                self.node.stop()
            self.assertEqual(kill.call_args.args[1], signal.SIGINT)
            with patch(
                "recording.menu_recorder.time.monotonic_ns", return_value=12000000000
            ):
                self.node.tick()
                self.node.tick()
            self.assertEqual(kill.call_count, 2)
            self.assertEqual(kill.call_args.args[1], signal.SIGTERM)
            with patch(
                "recording.menu_recorder.time.monotonic_ns", return_value=16000000000
            ):
                self.node.tick()
                self.node.tick()
            self.assertEqual(kill.call_count, 3)
            self.assertEqual(kill.call_args.args[1], signal.SIGKILL)
        self.node.process.poll.return_value = -signal.SIGKILL
        self.node.tick()
        self.assertEqual(self.node.phase, "error")
        self.assertIsNone(self.node.process)
