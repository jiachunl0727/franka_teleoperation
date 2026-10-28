"""Exercise the actual linked SDK against a loopback-only Hand protocol server."""

import os
import socket
import struct
import subprocess
import threading
import time

import pytest


HEADER = struct.Struct("<HII")


def receive(connection, size):
    data = b""
    while len(data) < size:
        part = connection.recv(size - len(data))
        if not part:
            raise EOFError("client disconnected")
        data += part
    return data


def request(connection):
    command, identifier, size = HEADER.unpack(receive(connection, HEADER.size))
    assert HEADER.size <= size <= 128
    receive(connection, size - HEADER.size)
    return command, identifier


def reply(command, identifier, payload):
    return HEADER.pack(command, identifier, HEADER.size + len(payload)) + payload


@pytest.mark.parametrize("mode", ["concurrent", "short_reply", "disconnect"])
def test_sdk_stop_does_not_wait_for_motion_completion(mode):
    errors = []
    dispatch_delays = []
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 1338))
        listener.listen(1)
        listener.settimeout(5)

        def server():
            try:
                connection, _ = listener.accept()
                with connection:
                    connection.settimeout(2)
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    command, identifier = request(connection)
                    assert command == 0  # Connect, protocol version 3.
                    connection.sendall(reply(command, identifier, struct.pack("<HH", 0, 3)))
                    if mode != "concurrent":
                        command, identifier = request(connection)
                        assert command == 4
                        if mode == "short_reply":
                            connection.sendall(reply(command, identifier, b""))
                        return
                    for cycle in range(12):
                        motion, motion_id = request(connection)
                        assert motion == (3 if cycle % 2 else 2)
                        started = time.monotonic()
                        command, identifier = request(connection)
                        dispatch_delays.append(time.monotonic() - started)
                        assert command == 4  # No motion response until Stop arrives.
                        stopped = reply(command, identifier, struct.pack("<H", 0))
                        aborted = reply(motion, motion_id, struct.pack("<H", 3))
                        data = stopped + aborted if cycle % 2 else aborted + stopped
                        # Exercise partial headers/payloads and both reply orders.
                        connection.sendall(data[:3])
                        time.sleep(0.001)
                        connection.sendall(data[3:11])
                        time.sleep(0.001)
                        connection.sendall(data[11:])
            except Exception as error:
                errors.append(error)

        worker = threading.Thread(target=server, daemon=True)
        worker.start()
        command = [os.environ["HAND_TRANSPORT_CLIENT"]]
        if mode != "concurrent":
            command.append(mode)
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
        worker.join(timeout=3)
    assert not worker.is_alive()
    assert not errors, errors
    assert result.returncode == 0, result.stderr
    if mode == "concurrent":
        assert len(dispatch_delays) == 12
        assert max(dispatch_delays) < 0.2, dispatch_delays
        assert max(map(float, result.stdout.splitlines())) < 200, result.stdout
        print(f"Maximum loopback Stop completion: {max(map(float, result.stdout.splitlines())):.3f} ms")
