"""Reconnect the native TCP client without OpenXR samples or a physical Quest."""

import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time
import unittest

from test_telemetry import read_exact

ROOT = Path(__file__).resolve().parents[1]


class UsbReconnectTest(unittest.TestCase):
    def test_server_restart_reconnects_without_tracking_or_command_replay(self):
        with tempfile.TemporaryDirectory() as directory, socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(3)
            binary = str(Path(directory) / "usb_reconnect_fixture")
            subprocess.run(
                [
                    "g++",
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-pthread",
                    f"-DQUEST_BRIDGE_PORT={server.getsockname()[1]}",
                    "-I",
                    str(ROOT / "test/stubs"),
                    "-I",
                    str(ROOT / "app/src/main/cpp"),
                    str(ROOT / "test/usb_reconnect_fixture.cpp"),
                    "-o",
                    binary,
                ],
                check=True,
            )
            proc = subprocess.Popen([binary])
            sessions = 0
            syncs = 0
            try:
                while proc.poll() is None:
                    conn, _ = server.accept()
                    sessions += 1
                    with conn:
                        conn.settimeout(3)
                        while True:
                            try:
                                (size,) = struct.unpack("!I", read_exact(conn, 4))
                                self.assertTrue(0 < size <= 4096)
                                packet = json.loads(read_exact(conn, size))
                            except EOFError:
                                break
                            # No sample was submitted, so reconnection must
                            # carry clock requests only, never a motion command.
                            self.assertEqual(packet["type"], "clock_sync_request")
                            syncs += 1
                            reply = dict(
                                version=1,
                                type="clock_sync_reply",
                                sync_id=packet["sync_id"],
                                quest_send_time_ns=packet["quest_send_time_ns"],
                                pc_receive_time_ns=str(time.monotonic_ns()),
                                pc_send_time_ns=str(time.monotonic_ns()),
                            )
                            body = json.dumps(reply, separators=(",", ":")).encode()
                            conn.sendall(struct.pack("!I", len(body)) + body)
                            if sessions == 1:
                                break  # Restart the peer after its first clock handshake.
                    if sessions >= 2:
                        break
                self.assertEqual(proc.wait(timeout=3), 0)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait()
            self.assertEqual(sessions, 2)
            self.assertGreaterEqual(syncs, 3)
