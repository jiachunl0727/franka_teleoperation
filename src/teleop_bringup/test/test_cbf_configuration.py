"""Activation/merge/resolution; detailed CBF schema is validated by the C++ tool."""

from pathlib import Path
from types import SimpleNamespace
import pytest
from teleop_bringup.cbf_configuration import merged_selection, prepare_cbf, resolve_file
from teleop_bringup.runtime_configuration import load_runtime
import yaml

SPHERE_GEOMETRY = (Path(__file__).resolve().parents[2] / "teleop_controllers" /
                   "test/data/release_braking/geometry.yaml")


def test_disabled_missing_section_and_path_do_no_validation(monkeypatch):
    monkeypatch.setattr(
        "teleop_bringup.cbf_configuration.subprocess.run",
        lambda *a, **k: pytest.fail("disabled validator called"),
    )
    arms = [SimpleNamespace(name="left"), SimpleNamespace(name="right")]
    assert prepare_cbf({}, arms, "mock")["left"] == {
        "cbf_enabled": False,
        "cbf_config_path": "",
    }
    assert not prepare_cbf(
        {"cbf": {"enabled": False, "config_file": "/missing"}}, arms, "hardware"
    )["right"]["cbf_enabled"]


def test_global_per_arm_precedence_and_real_validator():
    arms = [SimpleNamespace(name="left"), SimpleNamespace(name="right")]
    settings = {
        "cbf": {
            "enabled": False,
            "config_file": "package://cbf_safety_filter/config/single_panda/cbf.yaml",
        },
        "arm_settings": {"right": {"cbf": {"enabled": True}}},
    }
    prepared = prepare_cbf(settings, arms, "mock")
    assert not prepared["left"]["cbf_enabled"]
    assert prepared["right"]["cbf_enabled"]
    assert Path(prepared["right"]["cbf_config_path"]).is_absolute()
    assert prepared["right"]["cbf_commissioning_mode"] == "filtered"
    with pytest.raises(ValueError, match="requires cbf.hardware_mode"):
        prepare_cbf(settings, arms, "hardware")


def test_bad_selections_and_files(tmp_path):
    arms = [SimpleNamespace(name="arm")]
    path = tmp_path / "runtime.yaml"
    for value in (
        {"enabled": 1},
        {"enabled": "true"},
        {"config_file": 2},
        {"unknown": False},
    ):
        path.write_text(yaml.safe_dump({"cbf": value}))
        with pytest.raises(ValueError):
            load_runtime(path, arms)
    with pytest.raises(ValueError, match="requires"):
        merged_selection({"cbf": {"enabled": True}}, "arm")
    for value in (
        "relative.yaml",
        "/missing/cbf.yaml",
        "package://other/config.yaml",
        "package://cbf_safety_filter/../../escape.yaml",
    ):
        with pytest.raises(ValueError):
            resolve_file(value)
    bad = tmp_path / "bad.yaml"
    bad.write_text("version: 1\nunknown: true\n")
    with pytest.raises(ValueError, match="invalid CBF configuration"):
        prepare_cbf({"cbf": {"enabled": True, "config_file": str(bad)}}, arms, "mock")


def test_dual_profile_preflights_each_owner_geometry():
    arms = [
        SimpleNamespace(name="panda_" + side, parameters={"controller_id": side})
        for side in ("left", "right")
    ]
    settings = {"cbf": {
        "enabled": True,
        "config_file": "package://cbf_safety_filter/config/dual_panda/cbf.yaml",
        "hardware_mode": "filtered",
    }}
    prepared = prepare_cbf(settings, arms, "hardware")
    assert prepared["panda_left"]["cbf_config_path"] == prepared["panda_right"]["cbf_config_path"]
    arms[1].parameters["controller_id"] = "unknown"
    with pytest.raises(ValueError, match="controller selector"):
        prepare_cbf(settings, arms, "hardware")


@pytest.mark.parametrize("mode", ["shadow", "filtered"])
def test_explicit_single_arm_commissioning_routes_without_opening_hardware(
    tmp_path, mode
):
    source = Path(resolve_file("package://cbf_safety_filter/config/single_panda/cbf.yaml"))
    config = yaml.safe_load(source.read_text())
    config["geometry"]["file"] = str(source.parent / "geometry_panda_umi.yaml")
    path = tmp_path / "cbf.yaml"
    path.write_text(yaml.safe_dump(config))
    settings = {
        "cbf": {
            "enabled": True,
            "config_file": str(path),
            "hardware_mode": mode,
        }
    }
    prepared = prepare_cbf(settings, [SimpleNamespace(name="right")], "hardware")
    assert prepared["right"]["cbf_commissioning_mode"] == mode
    paired = prepare_cbf(
        settings,
        [SimpleNamespace(name="left"), SimpleNamespace(name="right")],
        "hardware",
    )
    assert all(value["cbf_commissioning_mode"] == mode for value in paired.values())
    settings["cbf"].pop("hardware_mode")
    with pytest.raises(ValueError, match="requires cbf.hardware_mode"):
        prepare_cbf(settings, [SimpleNamespace(name="right")], "hardware")


@pytest.mark.parametrize(
    "override",
    [{"enabled": False}, {"hardware_mode": "shadow"}],
)
def test_dual_hardware_requires_same_activation_and_output_mode(override):
    settings = {
        "cbf": {
            "enabled": True,
            "config_file": "package://cbf_safety_filter/config/single_panda/cbf.yaml",
            "hardware_mode": "filtered",
        },
        "arm_settings": {"right": {"cbf": override}},
    }
    with pytest.raises(ValueError, match="requires both arms enabled in the same output mode"):
        prepare_cbf(
            settings, [SimpleNamespace(name="left"), SimpleNamespace(name="right")], "hardware"
        )


@pytest.mark.parametrize(
    "value",
    [
        {"hardware_mode": "bypass"},
        {"hardware_mode": "blocked"},
        {"commissioning_confirmed": True},
    ],
)
def test_commissioning_schema_rejects_invalid_values(value):
    from teleop_bringup.cbf_configuration import validate_selection

    with pytest.raises(ValueError):
        validate_selection(value)


@pytest.mark.parametrize("change", ["budget", "diagnostics", "residuals", "interarm"])
def test_hardware_policy_preflight_precedes_child_creation(
    tmp_path, change
):
    source = Path(resolve_file("package://cbf_safety_filter/config/single_panda/cbf.yaml"))
    config = yaml.safe_load(source.read_text())
    config["geometry"]["file"] = str(source.parent / "geometry_panda_umi.yaml")
    if change == "budget":
        config["execution"]["max_callback_time_us"] = 1000
    elif change == "interarm":
        config["constraints"].setdefault("inter_arm_collision", {})["enabled"] = True
    elif change == "residuals":
        config["diagnostics"]["record_constraint_residuals"] = False
    else:
        config["diagnostics"]["enabled"] = False
    path = tmp_path / "cbf.yaml"
    path.write_text(yaml.safe_dump(config))
    expected = "requires two inter-arm participants" if change == "interarm" else "requires timing/residual diagnostics"
    with pytest.raises(ValueError, match=expected):
        prepare_cbf(
            {
                "cbf": {
                    "enabled": True,
                    "config_file": str(path),
                    "hardware_mode": "shadow",
                }
            },
            [SimpleNamespace(name="right")],
            "hardware",
        )


def test_dual_recovery_profile_requires_both_participants():
    arms = [SimpleNamespace(name="panda_" + side, parameters={"controller_id": side})
            for side in ("left", "right")]
    settings = {"cbf": {
        "enabled": True,
        "config_file": "package://cbf_safety_filter/config/dual_panda/cbf.yaml",
        "hardware_mode": "filtered",
    }}
    assert len(prepare_cbf(settings, arms, "mock")) == 2
    settings["arm_settings"] = {"panda_right": {"cbf": {"enabled": False}}}
    with pytest.raises(ValueError, match="snapshot recovery requires both participants"):
        prepare_cbf(settings, arms, "mock")
