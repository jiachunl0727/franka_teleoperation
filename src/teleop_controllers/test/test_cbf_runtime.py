"""Software-only CBF owner integration for the hard-constraint filter."""

from pathlib import Path
import json
import os
import signal
import subprocess
import tempfile
import time
import uuid
import yaml
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters


def configuration(root, backend):
    package = Path(__file__).parents[2] / "cbf_safety_filter"
    data = yaml.safe_load((package / "config/single_panda/cbf.yaml").read_text())
    geometry = yaml.safe_load((Path(__file__).resolve().parent / "data/release_braking/geometry.yaml").read_text())
    # The historical fixture also contains twelve finger entries.
    geometry["capsules"] = geometry["capsules"][:12]
    geometry_path = root / "geometry.yaml"
    geometry_path.write_text(yaml.safe_dump(geometry))
    data["geometry"]["file"] = str(geometry_path)
    data["execution"].update(
        safety_torque_computation_method="projection",
        max_constraints=52,
        max_filter_time_us=100000,
        max_model_age_us=100000,
        max_callback_time_us=100000,
    )
    data["constraints"]["table"].update(enabled=True, height_m=-1)
    # This fixture's 52-row workspace covers joint/table rows only, independent
    # of the operator's saved activation settings for other families.
    for family in ("joint_acceleration", "obstacles", "self_collision", "inter_arm_collision", "singularity"):
        data["constraints"].setdefault(family, {})["enabled"] = False
    data["solver"]["backend"] = backend
    data["solver"]["osqp"].update(
        max_iterations=10000, absolute_tolerance=1e-8, relative_tolerance=1e-8
    )
    data["solver"]["piqp"].update(
        max_iterations=100, absolute_tolerance=1e-8, relative_tolerance=1e-8
    )
    path = root / "cbf.yaml"
    path.write_text(yaml.safe_dump(data))
    return path, data


@pytest.mark.parametrize("backend", ["osqp", "qpoases", "piqp"])
def test_enabled_mock_reports_verified_hard_constraints(
    backend
):
    rclpy.init()
    node = Node("cbf_observer_" + uuid.uuid4().hex)
    namespace = "/cbf_" + uuid.uuid4().hex
    messages = []
    node.create_subscription(
        String,
        namespace + "/cbf_status",
        lambda msg: messages.append(json.loads(msg.data)),
        100,
    )
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        path, _ = configuration(root, backend)
        with (root / "owner.log").open("w") as log:
            args = [
                "ros2",
                "run",
                "teleop_controllers",
                "cartesian_pd_controller",
                "--ros-args",
                "-r",
                "__ns:=" + namespace,
                "-p",
                "backend:=mock",
                "-p",
                "cbf_enabled:=true",
                "-p",
                "cbf_config_path:=" + str(path),
            ]
            process = subprocess.Popen(
                args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            try:
                until = time.monotonic() + 10
                while (
                    time.monotonic() < until
                    and len(messages) < 10
                    and process.poll() is None
                ):
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert len(messages) >= 10, (root / "owner.log").read_text()
                assert all(message["fci_timing"] is None for message in messages)
                assert all("group_pruned_pairs" in message for message in messages)
                if backend == "osqp":
                    client = node.create_client(
                        SetParameters,
                        namespace
                        + "/panda_cartesian_pd_controller/set_parameters",
                    )
                    assert client.wait_for_service(timeout_sec=3)
                    request = SetParameters.Request(
                        parameters=[
                            Parameter(
                                name="cbf_commissioning_mode",
                                value=ParameterValue(
                                    type=ParameterType.PARAMETER_STRING,
                                    string_value="filtered",
                                ),
                            ),
                            Parameter(
                                name="cbf_enabled",
                                value=ParameterValue(
                                    type=ParameterType.PARAMETER_BOOL, bool_value=False
                                ),
                            ),
                            Parameter(
                                name="cbf_config_path",
                                value=ParameterValue(
                                    type=ParameterType.PARAMETER_STRING,
                                    string_value="/changed.yaml",
                                ),
                            ),
                        ]
                    )
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                    assert future.done()
                    assert all(
                        not result.successful for result in future.result().results
                    )
                for msg in messages:
                    assert msg["backend"] == backend
                    assert msg["status"] == "solved_hard" and msg["accepted"]
                    assert msg["rows"] == 52
                    assert msg["max_hard_violation"] <= 1e-6
                    assert msg["nominal_domain_valid"]
                    assert msg["nominal_command_feasible"]
                    assert msg["nominal_max_actuator_violation"] == 0
                    assert msg["nominal_max_row_violation"] <= 1e-6
                    assert msg["nominal_worst_family"] == -1
                    assert msg["current_box_infeasible_rows"] == 0
                    assert msg["current_box_worst_family"] == -1
                    assert msg["current_box_max_min_row_violation"] == 0
                    assert msg["proposal_residuals_valid"]
                    assert len(msg["constraints"]) == 52
                    assert max(map(abs, msg["commanded_torque"])) < 1e-5
                    assert msg["controller_callback_us"] >= msg["total_us"]
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()
            assert process.returncode == 0, (root / "owner.log").read_text()


@pytest.mark.parametrize("fault", ["overrun", "callback_overrun"])
def test_recoverable_rejection_latches_protective_stop_without_terminating(fault):
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        path, data = configuration(root, "osqp")
        if fault == "overrun":
            data["execution"]["max_filter_time_us"] = 1e-9
        elif fault == "callback_overrun":
            data["execution"]["max_callback_time_us"] = 1e-9
        path.write_text(yaml.safe_dump(data))
        args = [
            "ros2",
            "run",
            "teleop_controllers",
            "cartesian_pd_controller",
            "--ros-args",
            "-p",
            "backend:=mock",
            "-p",
            "cbf_enabled:=true",
            "-p",
            "cbf_config_path:=" + str(path),
        ]
        log_path = root / "owner.log"
        with log_path.open("w") as log:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                marker = "CBF_PROTECTIVE_STOP_SNAPSHOT "
                deadline = time.monotonic() + 10
                output = ""
                while time.monotonic() < deadline:
                    output = log_path.read_text()
                    if marker in output or process.poll() is not None:
                        break
                    time.sleep(0.02)
                assert marker in output, output
                assert process.poll() is None, output
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        output = log_path.read_text()
        assert process.returncode == 0, output
        marker = "CBF_PROTECTIVE_STOP_SNAPSHOT "
        captures = [json.loads(line.split(marker, 1)[1]) for line in output.splitlines()
                    if marker in line]
        assert len(captures) == 1, output
        terminal = captures[0]
        assert terminal["terminal"] and terminal["command_issued"]
        assert terminal["protective_stop"] and not terminal["control_terminated"]
        assert len(terminal["protective_command"]) == 7
        assert not terminal["accepted"]
        assert terminal["cycle"] >= 1
        assert len(terminal["q"]) == len(terminal["dq"]) == 7
        assert len(terminal["torque_lower"]) == len(terminal["torque_upper"]) == 7
        assert terminal["model_fnv1a64"]
        assert terminal["callback_us"] > 0
        assert terminal["callback_cpu_us"] > 0
        assert terminal["callback_cpu"] >= 0
        assert terminal["filter_timing_valid"]
        phases = terminal["filter_phases_us"]
        assert all(value >= 0 for value in phases.values())
        assert phases["solver"] <= terminal["filter_us"]


@pytest.mark.parametrize("case", ["single", "asymmetric", "overlapping_bases"])
def test_inter_arm_participant_and_world_geometry_contract(case):
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        path, data = configuration(root, "osqp")
        data["constraints"].setdefault("inter_arm_collision", {})["enabled"] = True
        path.write_text(yaml.safe_dump(data))
        args = [
            "ros2",
            "run",
            "teleop_controllers",
            "cartesian_pd_controller",
            "--ros-args",
            "-p",
            "backend:=mock",
        ]
        if case == "single":
            args += ["-p", "cbf_enabled:=true", "-p", "cbf_config_path:=" + str(path)]
            expected = "inter-arm filtering requires two enabled participants"
        else:
            args += [
                "-p",
                "mode:=dual",
                "-p",
                "left.cbf_enabled:=true",
                "-p",
                "left.cbf_config_path:=" + str(path),
            ]
            if case == "asymmetric":
                expected = "inter-arm filtering must be enabled on both arms"
            else:
                args += [
                    "-p",
                    "right.cbf_enabled:=true",
                    "-p",
                    "right.cbf_config_path:=" + str(path),
                ]
                expected = "CBF torque rejected: invalid_domain"
        process = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True,
        )
        try:
            if case == "overlapping_bases":
                time.sleep(1)
                assert process.poll() is None
                os.killpg(process.pid, signal.SIGINT)
                output, _ = process.communicate(timeout=10)
                assert process.returncode == 0, output
                assert "CBF_PROTECTIVE_STOP_SNAPSHOT" in output
            else:
                output, _ = process.communicate(timeout=15)
                assert process.returncode != 0
                assert expected in output
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def test_independent_dual_profile_routes_geometry_in_both_mock_owners(tmp_path):
    package = Path(__file__).parents[2] / "cbf_safety_filter"
    source = package / "config/dual_panda/cbf.yaml"
    data = yaml.safe_load(source.read_text())
    for side in ("left", "right"):
        geometry_path = source.parent / data["geometry"]["files_by_controller"][side]
        data["geometry"]["files_by_controller"][side] = str(geometry_path)
    data["execution"].update(max_filter_time_us=100000, max_callback_time_us=100000,
                             max_model_age_us=100000)
    data["constraints"]["inter_arm_collision"]["policy"] = "independent"
    path = tmp_path / "independent.yaml"
    path.write_text(yaml.safe_dump(data))
    received = {side: [] for side in ("left", "right")}
    namespace = "/cbf_independent_" + uuid.uuid4().hex
    rclpy.init()
    node = Node("independent_cbf_observer_" + uuid.uuid4().hex)
    args = ["ros2", "run", "teleop_controllers", "cartesian_pd_controller",
            "--ros-args", "-p", "backend:=mock", "-p", "mode:=dual"]
    for side in received:
        node.create_subscription(
            String, namespace + "/" + side + "/cbf_status",
            lambda msg, side=side: received[side].append(json.loads(msg.data)), 100,
        )
        args += ["-p", side + ".namespace:=" + namespace + "/" + side,
                 "-p", side + ".cbf_enabled:=true",
                 "-p", side + ".cbf_config_path:=" + str(path)]
    with (tmp_path / "owner.log").open("w") as log:
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            until = time.monotonic() + 8
            while time.monotonic() < until and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.02)
                if all(len(rows) >= 5 for rows in received.values()):
                    break
            assert all(len(rows) >= 5 for rows in received.values()), (tmp_path / "owner.log").read_text()
            for rows in received.values():
                for row in rows:
                    assert row["accepted"] and row["actual_command_verified"]
                    assert row["inter_arm_enabled"] and row["peer_valid"]
                    assert row["table_height_m"] == data["constraints"]["table"]["height_m"]
                    assert row["recovery_policy"] == "bounded_recovery"
                    assert row["status"] == "verified_recovery"
                    assert sum(c["family"] == 3 for c in row["constraints"]) == 24
                    assert sum(c["family"] == 1 for c in row["constraints"]) == 14
                    assert sum(c["family"] == 2 for c in row["constraints"]) == 14
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


@pytest.mark.parametrize("invalid_domain", [False, True])
@pytest.mark.parametrize("control_law", ["cartesian_pd", "inverse_dynamics"])
def test_shadow_observes_without_changing_nominal_output(
    invalid_domain, control_law
):
    from quest_teleop_interfaces.msg import CartesianTeleopCommand, TrackingSample
    import math

    rclpy.init()
    node = Node("shadow_test_" + uuid.uuid4().hex)
    namespace = "/shadow_" + uuid.uuid4().hex
    statuses, tracking = [], []
    node.create_subscription(
        String,
        namespace + "/cbf_status",
        lambda m: statuses.append(json.loads(m.data)),
        2048,
    )
    node.create_subscription(
        TrackingSample, namespace + "/tracking_samples", tracking.append, 2048
    )
    publisher = node.create_publisher(
        CartesianTeleopCommand, namespace + "/cartesian_teleop_command", 10
    )
    try:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path, data = configuration(root, "piqp")
            data["execution"]["max_constraints"] = 256
            data["constraints"]["self_collision"]["enabled"] = True
            data["constraints"]["singularity"]["enabled"] = True
            if invalid_domain:
                data["constraints"]["table"]["height_m"] = 10
            else:
                data["constraints"]["joint_position"]["safety_margin_rad"] = 0.7155
            path.write_text(yaml.safe_dump(data))
            with (root / "owner.log").open("w") as log:
                process = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "teleop_controllers",
                        "cartesian_pd_controller",
                        "--ros-args",
                        "-r",
                        "__ns:=" + namespace,
                        "-p",
                        "backend:=mock",
                        "-p",
                        "cartesian_control_law:=" + control_law,
                        "-p",
                        "cbf_enabled:=true",
                        "-p",
                        "cbf_commissioning_mode:=shadow",
                        "-p",
                        "cbf_config_path:=" + str(path),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    start, last_send, sequence = time.monotonic(), 0, 0
                    while time.monotonic() - start < 8:
                        assert process.poll() is None, (root / "owner.log").read_text()
                        now = time.monotonic()
                        if now - last_send > 0.01:
                            sequence += 1
                            msg = CartesianTeleopCommand()
                            msg.header.frame_id = "panda_1/panda_link0"
                            msg.session_id = "shadow-test"
                            msg.sequence = sequence
                            msg.source_time_pc_monotonic_ns = time.monotonic_ns()
                            msg.source_time_valid = msg.clock_sync_valid = (
                                msg.input_valid
                            ) = True
                            msg.motion_enabled = now - start > 2
                            msg.trigger_released = not msg.motion_enabled
                            msg.target_pose.position.x = 0.402
                            msg.target_pose.position.z = 0.4
                            msg.target_pose.orientation.x = -math.sin(0.03 / 2)
                            msg.target_pose.orientation.w = math.cos(0.03 / 2)
                            publisher.publish(msg)
                            last_send = now
                        rclpy.spin_once(node, timeout_sec=0.001)
                        if sum(m.motion_enabled for m in tracking) > 30:
                            break
                    active = [m for m in tracking if m.motion_enabled]
                    assert len(active) > 30
                    assert any(max(map(abs, m.nominal_torque)) > 0.01 for m in active)
                    for m in active:
                        assert m.reference_pose.position.x == pytest.approx(
                            m.requested_pose.position.x
                        )
                        assert m.reference_pose.position.z == pytest.approx(
                            m.requested_pose.position.z
                        )
                        assert m.reference_pose.orientation.x == pytest.approx(
                            m.requested_pose.orientation.x
                        )
                        for nominal, previous, limit, actual in zip(
                            m.nominal_torque,
                            m.previous_commanded_torque,
                            m.torque_limits,
                            m.commanded_torque,
                        ):
                            slew = m.maximum_torque_rate * min(m.period_seconds, 0.001)
                            expected = min(
                                min(limit, previous + slew),
                                max(max(-limit, previous - slew), nominal),
                            )
                            assert actual == pytest.approx(expected, abs=1e-12)
                    assert statuses and all(
                        m["output_mode"] == "shadow" for m in statuses
                    )
                    assert all(m["model_acquisition_us"] >= 0 for m in statuses)
                    if invalid_domain:
                        assert all(
                            m["status"] == "invalid_domain"
                            and not m["actual_command_verified"]
                            for m in statuses
                        )
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                assert process.returncode == 0, (root / "owner.log").read_text()
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "parameters, expected",
    [
        ([], "requires cbf_commissioning_mode shadow or filtered"),
        (["cbf_commissioning_mode:=shadow"], "requires timing/residual diagnostics"),
        (["cbf_commissioning_mode:=filtered"], "requires timing/residual diagnostics"),
        (["cbf_commissioning_mode:=blocked"], "CBF output mode must be shadow or filtered"),
        (["cbf_commissioning_mode:=bypass"], "CBF output mode must be shadow or filtered"),
    ],
)
def test_hardware_commissioning_rejects_before_connection(parameters, expected):
    with tempfile.TemporaryDirectory() as directory:
        path, data = configuration(Path(directory), "osqp")
        data["execution"]["mode"] = "synchronous"
        path.write_text(yaml.safe_dump(data))
        args = [
            "ros2",
            "run",
            "teleop_controllers",
            "cartesian_pd_controller",
            "--ros-args",
            "-p",
            "backend:=hardware",
            "-p",
            "robot_ip:=127.0.0.1",
            "-p",
            "cbf_enabled:=true",
            "-p",
            "cbf_config_path:=" + str(path),
        ]
        for parameter in parameters:
            args += ["-p", parameter]
        completed = subprocess.run(args, capture_output=True, text=True, timeout=10)
        assert completed.returncode != 0
        assert expected in completed.stdout + completed.stderr


@pytest.mark.parametrize(
    "right_enabled,right_mode",
    [(False, "filtered"), (True, "shadow")],
)
def test_dual_hardware_commissioning_rejects_pair_before_connection(
    right_enabled, right_mode
):
    args = [
        "ros2", "run", "teleop_controllers", "cartesian_pd_controller",
        "--ros-args", "-p", "backend:=hardware",
        "-p", "mode:=dual", "-p", "left.cbf_enabled:=true",
        "-p", "left.cbf_commissioning_mode:=filtered",
        "-p", "right.cbf_enabled:=" + str(right_enabled).lower(),
        "-p", "right.cbf_commissioning_mode:=" + right_mode,
    ]
    completed = subprocess.run(args, capture_output=True, text=True, timeout=10)
    assert completed.returncode != 0
    assert "dual hardware CBF requires both arms enabled in the same output mode" in completed.stdout + completed.stderr


@pytest.mark.parametrize("torque_method", ["projection", "native_qp"])
def test_bounded_recovery_stays_active_inside_clearance_and_accepts_retreat(tmp_path, torque_method):
    from quest_teleop_interfaces.msg import CartesianTeleopCommand, TrackingSample
    package = Path(__file__).parents[2] / "cbf_safety_filter"
    data = yaml.safe_load((package / "config/dual_panda/cbf.yaml").read_text())
    data["execution"].update(safety_torque_computation_method=torque_method, max_constraints=512)
    data["constraints"]["inter_arm_collision"]["policy"] = "independent"
    data["execution"].update(max_filter_time_us=100000, max_callback_time_us=100000,
                             max_model_age_us=100000)
    # This is a recovery/control integration test, not a scheduling benchmark.
    data["constraints"]["inter_arm_collision"].update(max_peer_age_us=100000,
                                                       max_peer_skew_us=100000)
    for family in ("table", "self_collision", "singularity"):
        data["constraints"][family]["enabled"] = False
    for side, y in (("left", 0), ("right", -.085)):
        geometry = dict(version=1, world_from_base_xyz_xyzw=[0,y,0,0,0,0,1],
                        capsules=[], points=[], obstacles=[], pairs=[], self_exclusions=[],
                        body_spheres=[dict(frame=7, offset=[0,0,0], radius_m=.04, group=9)])
        path = tmp_path / (side + ".yaml")
        path.write_text(yaml.safe_dump(geometry))
        data["geometry"]["files_by_controller"][side] = str(path)
    path = tmp_path / "recovery.yaml"
    path.write_text(yaml.safe_dump(data))
    namespace = "/bounded_recovery_" + uuid.uuid4().hex
    rclpy.init()
    node = Node("bounded_recovery_test_" + uuid.uuid4().hex)
    statuses, tracking = [], []
    node.create_subscription(String, namespace + "/left/cbf_status",
                             lambda m: statuses.append(json.loads(m.data)), 4096)
    node.create_subscription(TrackingSample, namespace + "/left/tracking_samples", tracking.append, 4096)
    publisher = node.create_publisher(CartesianTeleopCommand,
                                      namespace + "/left/cartesian_teleop_command", 10)
    args = ["ros2", "run", "teleop_controllers", "cartesian_pd_controller", "--ros-args",
            "-p", "backend:=mock", "-p", "mode:=dual"]
    for side in ("left", "right"):
        args += ["-p", side + ".namespace:=" + namespace + "/" + side,
                 "-p", side + ".cbf_enabled:=true", "-p", side + ".cbf_config_path:=" + str(path),
                 "-p", side + ".mock_realtime:=true"]
    with (tmp_path / "owner.log").open("w") as log:
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            start, last_send, sequence = time.monotonic(), 0., 0
            reference = None
            while time.monotonic() - start < 5:
                assert process.poll() is None, (tmp_path / "owner.log").read_text()
                rclpy.spin_once(node, timeout_sec=.001)
                if tracking and reference is None:
                    reference = tracking[-1].measured_pose
                now = time.monotonic()
                if reference is not None and now-last_send > .01:
                    sequence += 1
                    msg = CartesianTeleopCommand()
                    msg.header.frame_id = tracking[-1].header.frame_id
                    msg.session_id = "recovery-test"
                    msg.sequence = sequence
                    msg.source_time_pc_monotonic_ns = time.monotonic_ns()
                    msg.source_time_valid = msg.clock_sync_valid = msg.input_valid = True
                    msg.motion_enabled = now-start > 1
                    msg.trigger_released = not msg.motion_enabled
                    msg.target_pose.position.x = reference.position.x
                    msg.target_pose.position.z = reference.position.z
                    msg.target_pose.position.y = reference.position.y + (-.02 if now-start < 3 else .02)
                    msg.target_pose.orientation = reference.orientation
                    publisher.publish(msg)
                    last_send = now
            active = [m for m in tracking if m.motion_enabled]
            assert len(active) > 30
            assert any(m.reference_pose.position.y > reference.position.y + .001 for m in active)
            assert any(s["outside_domain_rows"] > 0 for s in statuses if s["motion_enabled"])
            assert all(s["accepted"] and s["actual_command_verified"] for s in statuses)
            assert all(s["recovery_policy"] == "bounded_recovery" for s in statuses)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


@pytest.mark.parametrize("torque_method", ["projection", "native_qp"])
def test_single_bounded_recovery_requires_no_peer(tmp_path, torque_method):
    path, data = configuration(tmp_path, "qpoases")
    data["execution"].update(safety_torque_computation_method=torque_method,max_constraints=512)
    path.write_text(yaml.safe_dump(data))
    namespace="/single_recovery_"+uuid.uuid4().hex
    rclpy.init()
    node=Node("single_recovery_observer_"+uuid.uuid4().hex)
    received=[]
    node.create_subscription(String,namespace+"/cbf_status",
                             lambda m: received.append(json.loads(m.data)),100)
    with (tmp_path/"owner.log").open("w") as log:
        process=subprocess.Popen(["ros2","run","teleop_controllers","cartesian_pd_controller",
            "--ros-args","-r","__ns:="+namespace,"-p","backend:=mock","-p","cbf_enabled:=true",
            "-p","cbf_config_path:="+str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            until=time.monotonic()+8
            while time.monotonic()<until and process.poll() is None and len(received)<5:
                rclpy.spin_once(node,timeout_sec=.02)
            assert len(received)>=5,(tmp_path/"owner.log").read_text()
            assert all(x["accepted"] and x["actual_command_verified"] for x in received)
            assert all(x["recovery_policy"] == "bounded_recovery" and not x["inter_arm_enabled"] for x in received)
            assert all(x["algorithm"]==("bounded_recovery_native_qp" if torque_method=="native_qp"
                                         else "bounded_recovery_projection") for x in received)
            assert all(x["requested_qp_status"]==("solved_hard" if torque_method=="native_qp"
                                                   else "disabled") for x in received)
            assert all(x["table_height_m"]==data["constraints"]["table"]["height_m"] for x in received)
        finally:
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
            node.destroy_node();rclpy.shutdown()
