"""Validate feature settings and per-arm overrides shared by feature launches."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from teleop_bringup.configuration import _DEFAULTS as MAPPER_DEFAULTS, read_mapping

ALLOWED = {
    "controller": {
        "dynamics_model_file",
        "cartesian_control_law",
        "inverse_dynamics_stiffness",
        "inverse_dynamics_damping_ratio",
        "inverse_dynamics_nullspace_stiffness",
        "inverse_dynamics_nullspace_damping",
        "home_joint_position",
        "nullspace_stiffness",
        "nullspace_damping",
        "nullspace_joint_limit_gain",
        "nullspace_joint_limit_margin",
        "nullspace_max_torque",
        "nullspace_ramp_seconds",
        "reference_shaping_enabled",
        "tracking_protection_enabled",
        "cartesian_wrench_limits_enabled",
        "reference_linear_speed",
        "reference_angular_speed",
        "reference_linear_acceleration",
        "reference_angular_acceleration",
        "tracking_position_limit",
        "tracking_orientation_limit",
        "cartesian_force_limit",
        "cartesian_moment_limit",
        "lambda_translation",
        "lambda_rotation",
        "gain_translation",
        "gain_rotation",
        "maximum_torque_rate",
        "torque_limit_scale",
        "torque_rate_limit_scale",
        "torque_limits",
        "homing_speed_factor",
        "homing_timeout_seconds",
        "homing_stop_duration_seconds",
        "homing_stop_acceleration",
        "homing_stop_timeout_seconds",
        "homing_stiffness",
        "homing_damping",
        "hold_stiffness",
        "hold_damping",
        "hold_stiffness_ramp_seconds",
    },
    "cbf": {"enabled", "config_file", "hardware_mode"},
    "hand": {
        "close_speed_mps",
        "open_speed_mps",
        "grasp_force_n",
        "cable_diameter_m",
        "cable_diameter_tolerance_m",
        "joystick_engage",
        "joystick_neutral",
        "mock_object_width_m",
        "open_width_m",
        "close_width_m",
        "worker_poll_ms",
        "command_timeout_ms",
        "max_clock_uncertainty_ms",
        "mock_operation_duration_ms",
    },
}


def validate_avoidance(values, section):
    """Reject malformed new control/safety settings before opening an owner."""
    import math

    if "dynamics_model_file" in values:
        from pathlib import Path

        resource = values["dynamics_model_file"]
        if (
            not isinstance(resource, str)
            or not resource
            or not (
                Path(resource).is_absolute()
                or (
                    resource.startswith("package://")
                    and len(resource[10:].split("/", 1)) == 2
                    and all(resource[10:].split("/", 1))
                )
            )
        ):
            raise ValueError(
                "controller.dynamics_model_file must be absolute or a package resource"
            )

    if "cartesian_control_law" in values and values["cartesian_control_law"] not in (
        "cartesian_pd",
        "inverse_dynamics",
    ):
        raise ValueError(
            "controller.cartesian_control_law must be cartesian_pd/inverse_dynamics"
        )
    for key, count, allow_zero in (
        ("inverse_dynamics_stiffness", 6, False),
        ("inverse_dynamics_nullspace_stiffness", 7, True),
        ("inverse_dynamics_nullspace_damping", 7, True),
    ):
        if key in values:
            value = values[key]
            if (
                not isinstance(value, list)
                or len(value) != count
                or any(
                    isinstance(v, bool)
                    or not isinstance(v, (int, float))
                    or not math.isfinite(v)
                    or v < 0
                    or (v == 0 and not allow_zero)
                    for v in value
                )
            ):
                raise ValueError(
                    f"controller.{key} requires {count} finite {'nonnegative' if allow_zero else 'positive'} values"
                )

    for key in (
        "reference_shaping_enabled",
        "tracking_protection_enabled",
        "cartesian_wrench_limits_enabled",
    ):
        if key in values and not isinstance(values[key], bool):
            raise ValueError(f"{section}.{key} must be true/false")
    prefix = "nullspace_"
    for key, value in values.items():
        if key.startswith(prefix) or key in {
            "reference_linear_speed",
            "reference_angular_speed",
            "reference_linear_acceleration",
            "reference_angular_acceleration",
            "tracking_position_limit",
            "tracking_orientation_limit",
            "cartesian_force_limit",
            "cartesian_moment_limit",
            "inverse_dynamics_damping_ratio",
        }:
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{section}.{key} must be finite and positive")
    for key in ("torque_limit_scale", "torque_rate_limit_scale"):
        if key in values:
            value = values[key]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or not 0 < value <= 1
            ):
                raise ValueError(f"controller.{key} must be finite and in (0, 1]")
    if section == "controller" and "home_joint_position" in values:
        home = values["home_joint_position"]
        if (
            not isinstance(home, list)
            or len(home) != 7
            or any(
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
                for v in home
            )
        ):
            raise ValueError("home_joint_position requires seven finite joint angles")


def load_runtime(path, arms):
    data = feature_document(load_runtime_document(path))
    if set(data) - (set(ALLOWED) | {"arm_settings"}):
        raise ValueError("unknown runtime configuration section")
    for key, allowed in ALLOWED.items():
        section = data.get(key, {})
        if not isinstance(section, dict) or set(section) - allowed:
            raise ValueError(f"unknown or invalid {key} setting")
        if key == "controller":
            validate_avoidance(section, key)
        if key == "cbf":
            from teleop_bringup.cbf_configuration import validate_selection

            validate_selection(section)
    local = data.get("arm_settings", {})
    if not isinstance(local, dict) or set(local) - {a.name for a in arms}:
        raise ValueError("arm_settings contains unknown arm names")
    for name, settings in local.items():
        if not isinstance(settings, dict) or set(settings) - {
            "controller",
            "hand",
            "cbf",
        }:
            raise ValueError(f"{name}: invalid arm_settings")
        for section in ("controller", "hand", "cbf"):
            values = settings.get(section, {})
            if not isinstance(values, dict) or set(values) - ALLOWED[section]:
                raise ValueError(f"{name}: invalid {section} override")
            if section == "controller":
                validate_avoidance(values, section)
            if section == "cbf":
                from teleop_bringup.cbf_configuration import validate_selection

                validate_selection(values)
    from teleop_bringup.cbf_configuration import merged_selection

    for arm in arms:
        merged_selection(data, arm.name)
    return data


# A document with `pipeline` is the combined-bringup schema. Standalone feature
# documents retain their original schemas and installed package defaults.
PIPELINE_DEFAULTS = dict(
    mode="single",
    single_controller="right",
    backend="mock",
    start_controller=True,
    homing_enabled=True,
    start_mapper=True,
    start_hand=True,
    start_bridge=False,
    mock_measured_pose=False,
    recording_enabled=True,
)
BRIDGE_DEFAULTS = dict(
    namespace="/",
    listen_port=45454,
    bind_address="127.0.0.1",
    command_timeout_ms=100,
    max_clock_uncertainty_ms=10,
    max_clock_sync_age_ms=3000,
    max_source_future_ms=10,
    client_receive_timeout_ms=500,
)
RECORDING_DEFAULTS = dict(
    mode="menu",
    namespace="/quest",
    left_controller_state_topic="",
    include_default_topics=True,
    topics=[],
    output_root="/ws/log/quest_bags",
    storage_id="sqlite3",
    menu_debounce_seconds=0.35,
)
UNIFIED_KEYS = set(ALLOWED) | {"pipeline", "quest", "arms", "arm_settings", "recording"}

PROFILE_SECTIONS = ("quest", "controller", "hand", "recording")


def _profile_path(reference, runtime_path):
    if not isinstance(reference, str) or not reference:
        raise ValueError("profile path must be a nonempty string")
    if reference.startswith("package://"):
        package, _, relative = reference[len("package://") :].partition("/")
        asset = Path(relative)
        if not package or not relative or asset.is_absolute() or ".." in asset.parts:
            raise ValueError(f"invalid package profile path: {reference}")
        return Path(get_package_share_directory(package)) / asset
    return runtime_path.parent / Path(reference).expanduser()


def _profile_mapping(value, name, allowed=None):
    if not isinstance(value, dict) or (allowed is not None and set(value) - allowed):
        raise ValueError(f"unknown or invalid {name} setting")
    return value


def load_runtime_document(path):
    """Resolve package profiles into the existing runtime schema before validation."""
    path = Path(path).expanduser().resolve()
    data = read_mapping(path)
    for section in PROFILE_SECTIONS:
        inline = data.get(section)
        if not isinstance(inline, dict) or "config_file" not in inline:
            continue
        if "pipeline" not in data:
            raise ValueError("profile references require a combined runtime with a pipeline mapping")
        reference = inline.pop("config_file")
        source = read_mapping(_profile_path(reference, path))
        if section in ("controller", "hand"):
            _profile_mapping(source, f"{section} profile", {section})
            if section not in source:
                raise ValueError(f"{section} profile requires a {section} mapping")
            defaults = _profile_mapping(
                source[section], f"{section} profile", ALLOWED[section]
            )
            data[section] = {**defaults, **inline}
        elif section == "recording":
            defaults = _profile_mapping(source, "recording profile", set(RECORDING_DEFAULTS))
            data[section] = {**defaults, **inline}
        else:
            _profile_mapping(source, "quest profile", {"bridge", "mapper", "mapper_by_controller"})
            quest = _profile_mapping(inline, "quest", {"bridge"})
            bridge = _profile_mapping(
                source.get("bridge", {}), "bridge profile", set(BRIDGE_DEFAULTS)
            )
            inline = _profile_mapping(quest.get("bridge", {}), "quest.bridge")
            data["quest"] = {"bridge": {**bridge, **inline}}
            mapper_keys = set(MAPPER_DEFAULTS) | {"twist_filter_cutoff_hz"}
            mapper = _profile_mapping(source.get("mapper", {}), "mapper profile", mapper_keys)
            by_controller = _profile_mapping(
                source.get("mapper_by_controller", {}),
                "mapper_by_controller",
                {"left", "right"},
            )
            for side, values in by_controller.items():
                _profile_mapping(values, f"{side} mapper profile", mapper_keys)
            arms = _profile_mapping(data.get("arms", {}), "arms")
            for name, values in arms.items():
                arm = _profile_mapping(values, f"{name} arm")
                side = arm.get("controller", "right")
                if side not in ("left", "right"):
                    raise ValueError(f"{name}: controller must be left or right")
                arms[name] = {**mapper, **by_controller.get(side, {}), **arm}
    return data


def unified_settings(data):
    """Validate the combined document before any launch can open hardware."""
    if set(data) - UNIFIED_KEYS:
        raise ValueError("unknown unified runtime configuration section")
    for key in UNIFIED_KEYS:
        if key in data and not isinstance(data[key], dict):
            raise ValueError(f"{key} must be a mapping")
    if not data.get("arms"):
        raise ValueError("unified runtime requires a nonempty arms mapping")
    raw = data.get("pipeline", {})
    if set(raw) - set(PIPELINE_DEFAULTS):
        raise ValueError("unknown pipeline setting")
    pipeline = dict(PIPELINE_DEFAULTS, **raw)
    for key, default in PIPELINE_DEFAULTS.items():
        if isinstance(default, bool) and not isinstance(pipeline[key], bool):
            raise ValueError(f"pipeline.{key} must be true/false")
    for key, values in dict(
        mode=("single", "dual"),
        single_controller=("left", "right"),
        backend=("mock", "hardware"),
    ).items():
        if pipeline[key] not in values:
            raise ValueError(f"invalid pipeline.{key}")
    quest = data.get("quest", {})
    if set(quest) - {"bridge"} or not isinstance(quest.get("bridge", {}), dict):
        raise ValueError(
            "quest supports a bridge mapping; mapper settings belong to arms"
        )
    raw = quest.get("bridge", {})
    if set(raw) - set(BRIDGE_DEFAULTS):
        raise ValueError("unknown Quest bridge setting")
    bridge = dict(BRIDGE_DEFAULTS, **raw)
    for key, default in BRIDGE_DEFAULTS.items():
        if isinstance(default, int) and (
            isinstance(bridge[key], bool) or not isinstance(bridge[key], int)
        ):
            raise ValueError(f"quest.bridge.{key} must be an integer")
        if isinstance(default, str) and (
            not isinstance(bridge[key], str) or not bridge[key]
        ):
            raise ValueError(f"quest.bridge.{key} must be a nonempty string")
    if not 1 <= bridge["listen_port"] <= 65535:
        raise ValueError("listen_port must be 1..65535")
    if (
        any(
            bridge[k] <= 0
            for k in (
                "command_timeout_ms",
                "max_clock_uncertainty_ms",
                "max_clock_sync_age_ms",
                "client_receive_timeout_ms",
            )
        )
        or bridge["max_source_future_ms"] < 0
    ):
        raise ValueError("invalid bridge freshness limit")
    ns = bridge["namespace"]
    from teleop_bringup.configuration import full_topic

    if ns != "/":
        full_topic(ns)
    raw = data.get("recording", {})
    if set(raw) - set(RECORDING_DEFAULTS):
        raise ValueError("unknown recording setting")
    recording = dict(RECORDING_DEFAULTS, **raw)
    if recording["mode"] not in ("menu", "continuous"):
        raise ValueError("recording.mode must be menu/continuous")
    if not isinstance(recording["namespace"], str):
        raise ValueError("invalid recording namespace")
    if recording["namespace"] != "/":
        full_topic(recording["namespace"])
    import math

    debounce = recording["menu_debounce_seconds"]
    if (
        isinstance(debounce, bool)
        or not isinstance(debounce, (float, int))
        or not math.isfinite(debounce)
        or debounce < 0.35
    ):
        raise ValueError(
            "recording.menu_debounce_seconds must be finite and at least 0.35"
        )
    left = recording["left_controller_state_topic"]
    if not isinstance(left, str):
        raise ValueError("invalid Menu input topic")
    if left:
        full_topic(left)
    return pipeline, bridge, recording


def feature_document(data):
    if "pipeline" in data:
        unified_settings(data)
        return {
            key: value
            for key, value in data.items()
            if key in set(ALLOWED) | {"arm_settings"}
        }
    return data


def bridge_controller_topic(data, side):
    namespace = unified_settings(data)[1]["namespace"] if "pipeline" in data else "/"
    return namespace.rstrip("/") + "/quest_usb_bridge/" + side + "/controller_state"
