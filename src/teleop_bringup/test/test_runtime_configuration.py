"""Unified configuration routing, strict validation and standalone compatibility."""

import copy
from pathlib import Path
import tempfile
import unittest
import yaml
from teleop_bringup.configuration import load_arms, select_arms
from teleop_bringup.runtime_configuration import (
    ALLOWED,
    BRIDGE_DEFAULTS,
    PIPELINE_DEFAULTS,
    RECORDING_DEFAULTS,
    load_runtime,
    load_runtime_document,
    unified_settings,
)


class RuntimeConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "runtime.yaml"
        self.data = {
            "pipeline": {},
            "arms": {"arm": {"namespace": "/arm", "controller": "right"}},
        }

    def write(self, data):
        self.path.write_text(yaml.safe_dump(data))
        return self.path

    def test_package_defaults_and_unified_sections(self):
        arms = load_arms(self.write(self.data))
        settings = load_runtime(self.path, arms)
        self.assertEqual(settings, {})
        self.assertEqual(
            arms[0].parameters["controller_state_topic"],
            "/quest_usb_bridge/right/controller_state",
        )
        standalone = {"controller": {"gain_translation": 7.0}, "arm_settings": {}}
        self.assertEqual(load_runtime(self.write(standalone), arms), standalone)
        self.assertEqual(len(load_arms(self.write({"arms": self.data["arms"]}))), 1)
        pipeline, _, _ = unified_settings(self.data)
        self.assertEqual(pipeline["backend"], "mock")

    def test_single_template_exposes_only_mode_relevant_settings(self):
        path = Path(__file__).parents[2] / "teleop_bringup/config/single_panda/single_panda.yaml"
        data = yaml.safe_load(path.read_text())
        self.assertNotIn("profiles", data)
        for section in ("quest", "controller", "hand", "recording", "cbf"):
            self.assertIn("/config/single_panda/", data[section]["config_file"])
        self.assertNotIn("mock_measured_pose", data["pipeline"])
        self.assertNotIn("gain_translation", data["controller"])
        self.assertNotIn("worker_poll_ms", data["hand"])
        self.assertNotIn("velocity_filter_cutoff_hz", data["arms"]["panda_1"])
        arms = load_arms(path)
        settings = load_runtime(path, arms)
        pipeline, bridge, recording = unified_settings(load_runtime_document(path))
        self.assertFalse(pipeline["mock_measured_pose"])
        self.assertEqual(set(settings["controller"]), ALLOWED["controller"])
        self.assertEqual(set(settings["hand"]), ALLOWED["hand"] - {"mock_object_width_m"})
        # The template is edited for deployment; schema/selection checks must
        # not require an operator's saved backend to remain the mock default.
        self.assertEqual(pipeline["mode"], "single")
        self.assertTrue(pipeline["start_bridge"])
        self.assertEqual(bridge["listen_port"], 45454)
        self.assertEqual(recording["mode"], "menu")
        self.assertNotIn("arm_settings", data)
        self.assertNotIn("controller", data["arms"]["panda_1"])
        self.assertTrue(
            all(
                "controller_enabled" not in a and "operator_confirm" not in a
                for a in data["arms"].values()
            )
        )
        left = select_arms(arms, "single", "left")[0]
        self.assertEqual(left.parameters["controller_id"], "left")

    def test_dual_template_requires_no_physical_geometry_for_mock(self):
        path = Path(__file__).parents[2] / "teleop_bringup/config/dual_panda/dual_panda.yaml"
        data = yaml.safe_load(path.read_text())
        self.assertNotIn("profiles", data)
        for section in ("quest", "controller", "hand", "recording", "cbf"):
            self.assertIn("/config/dual_panda/", data[section]["config_file"])
        # Installation YAML is operator-editable. Exercise mock selection using
        # a temporary copy, without changing saved IPs or surveyed transforms.
        data["pipeline"]["backend"] = "mock"
        for arm in data["arms"].values():
            arm["robot_ip"] = ""
        path = self.write(data)
        arms = select_arms(load_arms(path), "dual", "right")
        self.assertEqual(
            [a.parameters["controller_id"] for a in arms], ["left", "right"]
        )
        self.assertEqual(data["pipeline"]["mode"], "dual")
        self.assertNotIn("single_controller", data["pipeline"])
        self.assertEqual(
            set(data["pipeline"]), set(PIPELINE_DEFAULTS) - {"single_controller", "mock_measured_pose"}
        )
        self.assertTrue(
            all(
                "controller_enabled" not in a and "operator_confirm" not in a
                for a in data["arms"].values()
            )
        )
        self.assertEqual(data["pipeline"]["backend"], "mock")
        self.assertEqual(len({a.namespace for a in arms}), 2)
        self.assertEqual(len({a.parameters["base_frame"] for a in arms}), 2)
        self.assertTrue(all(not a.robot_ip for a in arms))
        settings = load_runtime(path, arms)
        self.assertEqual(
            settings["controller"]["dynamics_model_file"],
            "package://teleop_controllers/config/panda_umi.urdf",
        )
        for arm in arms:
            local = settings.get("arm_settings", {}).get(arm.name, {})
            self.assertNotIn("dynamics_model_file", local.get("controller", {}))
        self.assertEqual(set(settings["controller"]), ALLOWED["controller"])
        self.assertEqual(set(settings["hand"]), ALLOWED["hand"] - {"mock_object_width_m"})
        self.assertEqual([a.parameters["velocity_filter_cutoff_hz"] for a in arms], [5.0, 10.0])

    def test_profile_layers_and_derived_routes(self):
        root = self.path.parent
        (root / "quest.yaml").write_text(yaml.safe_dump({
            "bridge": {"namespace": "/profile_bridge", "listen_port": 45400},
            "mapper": {"translation_scale": 0.5, "velocity_filter_cutoff_hz": 12.0},
            "mapper_by_controller": {"left": {"velocity_filter_cutoff_hz": 7.0}},
        }))
        (root / "controller.yaml").write_text("controller: {gain_translation: 10.0, gain_rotation: 3.0}\n")
        (root / "recording.yaml").write_text("output_root: /profile_bags\n")
        data = {
            "pipeline": {"mode": "dual"},
            "quest": {
                "config_file": "quest.yaml",
                "bridge": {"listen_port": 45454},
            },
            "controller": {"config_file": "controller.yaml", "gain_translation": 20.0},
            "recording": {"config_file": "recording.yaml", "output_root": "/override_bags"},
            "arms": {"a": {"controller": "left", "translation_scale": 0.75},
                     "b": {"controller": "right"}},
            "arm_settings": {"b": {"controller": {"gain_translation": 25.0}}},
        }
        path = self.write(data)
        arms = select_arms(load_arms(path), "dual")
        pipeline, bridge, recording = unified_settings(load_runtime_document(path))
        settings = load_runtime(path, arms)
        self.assertEqual(pipeline["backend"], "mock")
        self.assertEqual(bridge["listen_port"], 45454)
        self.assertEqual(recording["output_root"], "/override_bags")
        self.assertEqual([a.parameters["translation_scale"] for a in arms], [0.75, 0.5])
        self.assertEqual([a.parameters["velocity_filter_cutoff_hz"] for a in arms], [7.0, 12.0])
        self.assertEqual(arms[0].parameters["controller_state_topic"],
                         "/profile_bridge/quest_usb_bridge/left/controller_state")
        self.assertEqual(settings["controller"], {"gain_translation": 20.0, "gain_rotation": 3.0})
        self.assertEqual(settings["arm_settings"]["b"]["controller"], {"gain_translation": 25.0})

    def test_bad_profiles_and_retired_hand_keys_are_rejected(self):
        (self.path.parent / "bad_hand.yaml").write_text("hand: {grip_press: 0.5}\n")
        for section, reference in (
            ("hand", "bad_hand.yaml"),
            ("quest", "package://quest_teleop/../quest.yaml"),
            ("controller", None),
        ):
            runtime = dict(self.data, **{section: {"config_file": reference}})
            with self.assertRaises(ValueError):
                load_runtime_document(self.write(runtime))
        with self.assertRaises(ValueError):
            load_runtime_document(self.write({"controller": {"config_file": "controller.yaml"}}))
        with self.assertRaises(ValueError):
            load_runtime(self.write({"hand": {"grip_press": 0.5}}), [])

    def test_namespaced_bridge_and_per_arm_settings(self):
        self.data.update(
            quest={"bridge": {"namespace": "/input"}},
            controller={"gain_translation": 5.0},
            arm_settings={
                "arm": {
                    "controller": {"gain_translation": 3.0},
                }
            },
        )
        arms = load_arms(self.write(self.data))
        self.assertEqual(
            arms[0].parameters["controller_state_topic"],
            "/input/quest_usb_bridge/right/controller_state",
        )
        left = select_arms(arms, "single", "left")[0]
        self.assertEqual(
            left.parameters["controller_state_topic"],
            "/input/quest_usb_bridge/left/controller_state",
        )
        self.assertEqual(
            load_runtime(self.path, arms)["arm_settings"]["arm"]["controller"][
                "gain_translation"
            ],
            3.0,
        )

    def test_actuator_scales_belong_to_controller_and_validate(self):
        self.data["controller"] = {
            "torque_limit_scale": 0.2,
            "torque_rate_limit_scale": 0.1,
        }
        arms = load_arms(self.write(self.data))
        self.assertEqual(
            load_runtime(self.path, arms)["controller"], self.data["controller"]
        )
        for key in ("torque_limit_scale", "torque_rate_limit_scale"):
            for invalid in (0.0, -1.0, 1.01, float("nan"), float("inf"), True, "0.35"):
                self.data["controller"] = {key: invalid}
                self.write(self.data)
                with self.assertRaises(ValueError):
                    load_runtime(self.path, arms)

    def test_joystick_and_cable_settings_support_per_arm_overrides(self):
        self.data["hand"] = dict(
            cable_diameter_m=0.005,
            cable_diameter_tolerance_m=0.002,
            grasp_force_n=-1.0,
            joystick_engage=0.4,
            joystick_neutral=0.2,
            mock_object_width_m=0.005,
        )
        self.data["arm_settings"] = {
            "arm": {"hand": {"grasp_force_n": 5.0, "cable_diameter_m": 0.01}}
        }
        path = self.write(self.data)
        arms = load_arms(path)
        configured = load_runtime(path, arms)
        self.assertEqual(configured["hand"], self.data["hand"])
        self.assertEqual(
            configured["arm_settings"]["arm"]["hand"]["cable_diameter_m"], 0.01
        )

    def test_invalid_schema_and_config_confirmation_are_rejected(self):
        changes = [
            dict(extra={}),
            dict(pipeline={"operator_confirm": True}),
            dict(pipeline={"start_hand": "true"}),
            dict(safety={"torque_limit_scale": 0.2}),
            dict(pipeline={"backend": "other"}),
            dict(quest={"bridge": {"typo": 3}}),
            dict(quest={"bridge": {"listen_port": 0}}),
            dict(quest={"bridge": {"listen_port": True}}),
            dict(quest={"bridge": {"namespace": "bad name"}}),
            dict(recording={"typo": 3}),
            dict(recording={"menu_debounce_seconds": float("nan")}),
            dict(controller={"typo": 3}),
            dict(arm_settings={"unknown": {}}),
        ]
        for change in changes:
            with self.subTest(change=change):
                data = copy.deepcopy(self.data)
                data.update(change)
                with self.assertRaises(ValueError):
                    path = self.write(data)
                    arms = load_arms(path)
                    load_runtime(path, arms)


if __name__ == "__main__":
    unittest.main()


def test_nullspace_home_settings_validate(tmp_path):
    path = tmp_path / "runtime.yaml"
    home = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
    data = {
        "pipeline": {},
        "arms": {"arm": {"namespace": "/arm", "controller": "right"}},
        "controller": {"home_joint_position": home},
        "arm_settings": {
            "arm": {"controller": {"home_joint_position": [0.2] + home[1:]}}
        },
    }
    path.write_text(yaml.safe_dump(data))
    arms = load_arms(path)
    result = load_runtime(path, arms)
    assert result["arm_settings"]["arm"]["controller"]["home_joint_position"][0] == 0.2
    import pytest

    for section, key, invalid in [
        ("controller", "nullspace_enabled", False),
        ("controller", "nullspace_stiffness", 0),
        ("controller", "home_joint_position", [0.0] * 6),
    ]:
        changed = copy.deepcopy(data)
        changed[section][key] = invalid
        path.write_text(yaml.safe_dump(changed))
        with pytest.raises(ValueError):
            load_runtime(path, arms)


def test_motion_protection_settings_reject_disabled_or_nonfinite_limits(tmp_path):
    import pytest

    path = tmp_path / "motion_limits.yaml"
    fields = (
        "reference_linear_speed",
        "reference_angular_speed",
        "reference_linear_acceleration",
        "reference_angular_acceleration",
        "tracking_position_limit",
        "tracking_orientation_limit",
        "cartesian_force_limit",
        "cartesian_moment_limit",
    )
    for key in fields:
        for value in (0, -1, float("nan"), float("inf"), True):
            path.write_text(yaml.safe_dump({"controller": {key: value}}))
            with pytest.raises(ValueError, match="finite and positive"):
                load_runtime(path, [])


def test_removed_cbf_execution_settings_are_not_silently_accepted(tmp_path):
    import pytest

    path = tmp_path / "obsolete.yaml"
    for key in (
        "execution",
        "worker_period_ms",
        "maximum_result_age_ms",
        "recovery_dwell_ms",
        "model_grace_ms",
        "braking_acceleration",
        "fallback_hold_stiffness",
        "maximum_failures",
    ):
        path.write_text(yaml.safe_dump({"cbf": {key: 1}}))
        with pytest.raises(ValueError, match="unknown or invalid cbf setting"):
            load_runtime(path, [])


def test_optional_controller_switches_validate_and_support_per_arm_overrides(tmp_path):
    import pytest

    path = tmp_path / "optional.yaml"
    switches = (
        "reference_shaping_enabled",
        "tracking_protection_enabled",
        "cartesian_wrench_limits_enabled",
    )
    source = Path(__file__).parents[2]
    for config in (
        "teleop_bringup/config/single_panda/single_panda.yaml",
        "teleop_bringup/config/dual_panda/dual_panda.yaml",
        "teleop_controllers/config/standalone.yaml",
    ):
        saved = load_runtime_document(source / config)
        assert all(saved["controller"][key] is False for key in switches)
    for switch in switches:
        for enabled in (False, True):
            data = {
                "controller": {switch: enabled},
                "arm_settings": {"arm": {"controller": {switch: not enabled}}},
            }
            path.write_text(yaml.safe_dump(data))
            configured = load_runtime(path, [type("Arm", (), {"name": "arm"})()])
            assert configured["controller"][switch] is enabled
            assert (
                configured["arm_settings"]["arm"]["controller"][switch] is not enabled
            )
        for invalid in ("false", 0, 1, None):
            path.write_text(yaml.safe_dump({"controller": {switch: invalid}}))
            with pytest.raises(ValueError, match="must be true/false"):
                load_runtime(path, [])
            path.write_text(
                yaml.safe_dump(
                    {"arm_settings": {"arm": {"controller": {switch: invalid}}}}
                )
            )
            with pytest.raises(ValueError, match="must be true/false"):
                load_runtime(path, [type("Arm", (), {"name": "arm"})()])


def test_inverse_dynamics_parameters_and_per_arm_selection():
    from teleop_bringup.runtime_configuration import validate_avoidance
    import pytest

    valid = {
        "cartesian_control_law": "inverse_dynamics",
        "inverse_dynamics_stiffness": [750.0] * 3 + [75.0] * 3,
        "inverse_dynamics_damping_ratio": 1.0,
        "inverse_dynamics_nullspace_stiffness": [10.0] * 7,
        "inverse_dynamics_nullspace_damping": [6.324555320336759] * 7,
    }
    validate_avoidance(valid, "controller")
    for key, values in {
        "cartesian_control_law": ["inverse", 1, None],
        "inverse_dynamics_stiffness": [[1.0] * 5, [0.0] * 6, [float("nan")] * 6],
        "inverse_dynamics_damping_ratio": [True, 0, float("inf")],
        "inverse_dynamics_nullspace_stiffness": [[1.0] * 6, [-1.0] * 7],
        "inverse_dynamics_nullspace_damping": [[True] * 7, [float("nan")] * 7],
    }.items():
        for value in values:
            with pytest.raises(ValueError):
                validate_avoidance({key: value}, "controller")
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "runtime.yaml"
        path.write_text(
            yaml.safe_dump(
                {
                    "pipeline": {},
                    "arms": {"arm": {"namespace": "/arm", "controller": "right"}},
                    "controller": valid,
                    "arm_settings": {
                        "arm": {
                            "controller": {
                                "cartesian_control_law": "cartesian_pd"
                            }
                        }
                    },
                }
            )
        )
        settings = load_runtime(path, load_arms(path))
        assert settings["controller"]["cartesian_control_law"] == "inverse_dynamics"
        assert (
            settings["arm_settings"]["arm"]["controller"]["cartesian_control_law"]
            == "cartesian_pd"
        )


def test_runtime_urdf_model_resource_validation():
    from teleop_bringup.runtime_configuration import validate_avoidance
    import pytest

    for resource in (
        "package://teleop_controllers/config/panda_oscbf.urdf",
        "/tmp/custom_panda.urdf",
    ):
        validate_avoidance({"dynamics_model_file": resource}, "controller")
    for invalid in (
        True,
        1,
        "",
        "relative.urdf",
        "package://missing",
        "package:///file.urdf",
    ):
        with pytest.raises(ValueError):
            validate_avoidance({"dynamics_model_file": invalid}, "controller")
