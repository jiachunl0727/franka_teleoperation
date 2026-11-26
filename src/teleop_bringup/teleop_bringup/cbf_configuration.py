"""Bringup-only activation/path routing; numerical validation stays in C++."""

from pathlib import Path
import subprocess
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def load_cbf_document(path, active=()):
    """Read the same bounded profile overlay used by the native validator."""
    path = Path(path).resolve()
    if path in active or len(active) >= 8:
        raise ValueError("cyclic or excessive CBF profile inheritance")
    layer = yaml.safe_load(path.read_text())
    if not isinstance(layer, dict):
        raise ValueError("CBF profile must be a mapping")
    source = layer.pop("extends", None)
    base = load_cbf_document(path.parent / source, (*active, path)) if source else {}
    geometry = layer.get("geometry", {})
    if "file" in geometry:
        geometry["file"] = str((path.parent / geometry["file"]).resolve())
    if "files_by_controller" in geometry:
        geometry["files_by_controller"] = {k: str((path.parent / v).resolve())
                                             for k, v in geometry["files_by_controller"].items()}
    def merge(original, update):
        for key, value in update.items():
            if isinstance(value, dict) and isinstance(original.get(key), dict):
                merge(original[key], value)
            else:
                original[key] = value
        return original
    return merge(base, layer)


DEFAULTS = {
    "enabled": False,
    "config_file": "",
}
HARDWARE_MODES = ("shadow", "filtered")


def validate_selection(value):
    if not isinstance(value, dict) or set(value) - (set(DEFAULTS) | {"hardware_mode"}):
        raise ValueError("unknown cbf selection key")
    if "enabled" in value and not isinstance(value["enabled"], bool):
        raise ValueError("cbf.enabled must be true/false")
    if "config_file" in value and not isinstance(value["config_file"], str):
        raise ValueError("cbf.config_file must be a string")

    if "hardware_mode" in value and value["hardware_mode"] not in HARDWARE_MODES:
        raise ValueError("cbf.hardware_mode must be shadow or filtered")


def merged_selection(settings, arm_name):
    result = dict(DEFAULTS)
    result.update(settings.get("cbf", {}))
    result.update(settings.get("arm_settings", {}).get(arm_name, {}).get("cbf", {}))
    validate_selection(result)
    if result["enabled"] and not result["config_file"]:
        raise ValueError("enabled CBF requires cbf.config_file")
    return result


def resolve_file(value):
    if value.startswith("package://"):
        package, separator, relative = value[len("package://") :].partition("/")
        if not separator or package != "cbf_safety_filter":
            raise ValueError("CBF package URI must name cbf_safety_filter")
        relative_path = Path(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise ValueError("CBF package path escapes package share")
        # ament --symlink-install legitimately resolves package files into src.
        base = Path(get_package_share_directory(package))
        path = (base / relative_path).resolve()
    else:
        path = Path(value)
        if not path.is_absolute():
            raise ValueError("cbf.config_file must be absolute or package://")
    if not path.is_file():
        raise ValueError("CBF config file does not exist: " + str(path))
    return str(path.resolve())


def prepare_cbf(settings, arms, backend):
    selections = [merged_selection(settings, arm.name) for arm in arms]
    enabled = [selection for selection in selections if selection["enabled"]]
    if backend == "hardware" and enabled:
        if any("hardware_mode" not in selection for selection in enabled):
            raise ValueError("enabled hardware CBF requires cbf.hardware_mode: shadow or filtered")
        if len(arms) == 2 and (
            len(enabled) != 2 or enabled[0]["hardware_mode"] != enabled[1]["hardware_mode"]
        ):
            raise ValueError(
                "dual hardware CBF requires both arms enabled in the same output mode"
            )
    result = {}
    inter_arm_settings = []
    for arm, selection in zip(arms, selections):
        if not selection["enabled"]:
            result[arm.name] = {"cbf_enabled": False, "cbf_config_path": ""}
            continue
        path = resolve_file(selection["config_file"])
        validator = (
            Path(get_package_prefix("cbf_safety_filter"))
            / "lib/cbf_safety_filter/cbf_validate"
        )
        # Match the owner selector; never silently give both robots one geometry.
        controller_id = getattr(arm, "parameters", {}).get("controller_id", "")
        checked = subprocess.run(
            [str(validator), path] + ([controller_id] if controller_id else []),
            capture_output=True, text=True, timeout=30,
        )
        if checked.returncode:
            raise ValueError("invalid CBF configuration: " + checked.stderr.strip())
        config = load_cbf_document(path)
        inter_arm = (
            config.get("constraints", {}).get("inter_arm_collision", {}).get("enabled", False)
        )
        inter_arm_settings.append(inter_arm)

        if backend == "hardware":
            # Match owner policy before launch can create an owner or Hand action.
            execution, diagnostics = config.get("execution", {}), config.get(
                "diagnostics", {}
            )
            if (
                float(execution.get("max_callback_time_us", 800)) > 800
                or not diagnostics.get("enabled", True)
                or not diagnostics.get("record_computation_time", True)
                or not diagnostics.get("record_constraint_residuals", True)
            ):
                raise ValueError(
                    "hardware commissioning requires timing/residual diagnostics and callback budget <=800 us"
                )
            if inter_arm and len(arms) != 2:
                raise ValueError("hardware commissioning requires two inter-arm participants")
        result[arm.name] = {
            "cbf_enabled": True,
            "cbf_config_path": path,
            "cbf_commissioning_mode": selection.get("hardware_mode", "filtered"),
        }
    if any(inter_arm_settings) and (
        len(arms) != 2 or len(inter_arm_settings) != 2 or not all(inter_arm_settings)
    ):
        raise ValueError("inter-arm collision requires CBF enabled for both participants")
    return result
