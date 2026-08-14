"""Cabinet URDF generation for runtime re-spawn.

The launch file generates cabinet URDFs once (``xacro.process_file`` plus the
per-control button profiles read from ``cabinet_controls.yaml``) and spawns
them with ``spawn_entity.py``.  Scene switching deletes those cabinet entities
and must re-create them faithfully, so this module mirrors that generation as
pure, importable functions shared by the gateway.  It depends only on ``xacro``
and ``yaml``, both present in the sourced ROS environment.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping, Optional, Tuple

import yaml

try:
    import xacro
except ImportError:  # pragma: no cover - xacro is present in the ROS env
    xacro = None  # type: ignore[assignment]


class CabinetSpawnError(RuntimeError):
    """Raised when a cabinet URDF cannot be generated for re-spawn."""


_BUTTON_FIELDS = (
    "max_position",
    "spring_stiffness",
    "press_threshold",
    "default_force",
)


def read_button_profiles(
    controls_path: str | Path,
) -> Tuple[Optional[Mapping[str, float]], Mapping[str, Mapping[str, Any]]]:
    """Return ``(button_defaults, profiles)`` from ``cabinet_controls.yaml``.

    Mirrors ``_read_button_profiles`` in the launch file: defaults are shared
    across every button, profiles carry the effective per-control parameters
    keyed by control ID and include the physical ``joint_name``.
    """
    path = Path(controls_path).expanduser()
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise CabinetSpawnError(
            f"Could not read cabinet controls '{path}': {error}"
        ) from error

    try:
        parameters = document["/**"]["ros__parameters"]
        controls = parameters["controls"]
        control_ids = parameters["control_ids"]
    except (KeyError, TypeError) as error:
        raise CabinetSpawnError(
            "Cabinet controls must define control_ids and controls."
        ) from error
    if not isinstance(controls, dict) or not isinstance(control_ids, list):
        raise CabinetSpawnError(
            "Cabinet controls and control_ids must be defined."
        )

    button_ids = []
    for control_id in control_ids:
        spec = controls.get(control_id)
        if not isinstance(control_id, str) or not isinstance(spec, dict):
            raise CabinetSpawnError("Every control_id must reference a mapping.")
        if spec.get("type") == "button":
            button_ids.append(control_id)
    if not button_ids:
        return None, {}

    defaults = parameters.get("button_defaults")
    if not isinstance(defaults, dict):
        raise CabinetSpawnError(
            "button_defaults must be a mapping when buttons are configured."
        )
    default_profile = _validated_button_profile(defaults, "button_defaults")
    profiles: dict[str, Mapping[str, Any]] = {}
    for control_id in button_ids:
        spec = controls.get(control_id)
        effective = dict(default_profile)
        for field in effective:
            if field in spec:
                effective[field] = spec[field]
        profile = _validated_button_profile(effective, f"controls.{control_id}")
        joint_name = spec.get("joint_name")
        if not isinstance(joint_name, str) or not joint_name:
            raise CabinetSpawnError(
                f"controls.{control_id}.joint_name is required."
            )
        profile["joint_name"] = joint_name
        profiles[control_id] = profile
    return default_profile, profiles


def build_cabinet_urdf(
    cabinet_xacro_path: str | Path,
    name: str,
    button_defaults: Optional[Mapping[str, float]],
    button_profiles: Mapping[str, Mapping[str, Any]],
) -> str:
    """Generate one cabinet's URDF XML for a re-spawn."""
    if xacro is None:
        raise CabinetSpawnError("xacro is not importable in this environment.")
    xacro_path = Path(cabinet_xacro_path).expanduser()
    if not xacro_path.is_file():
        raise CabinetSpawnError(f"Cabinet Xacro does not exist: {xacro_path}")
    mappings: dict[str, str] = {"cabinet_name": name}
    if button_defaults is not None:
        mappings.update(
            {
                "button_max_travel": str(button_defaults["max_position"]),
                "button_spring_stiffness": str(
                    button_defaults["spring_stiffness"]
                ),
                "button_press_threshold": str(
                    button_defaults["press_threshold"]
                ),
            }
        )
    try:
        document = xacro.process_file(str(xacro_path), mappings=mappings)
    except Exception as error:  # noqa: BLE001
        raise CabinetSpawnError(
            f"Could not generate URDF for cabinet '{name}': {error}"
        ) from error
    _apply_button_profiles(document, button_profiles)
    return document.toxml()


def _validated_button_profile(source: Mapping[str, Any], label: str) -> dict[str, float]:
    profile: dict[str, float] = {}
    for field in _BUTTON_FIELDS:
        value = source.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CabinetSpawnError(f"{label}.{field} must be a number.")
        value = float(value)
        if not math.isfinite(value):
            raise CabinetSpawnError(f"{label}.{field} must be finite.")
        profile[field] = value

    max_position = profile["max_position"]
    stiffness = profile["spring_stiffness"]
    threshold = profile["press_threshold"]
    default_force = profile["default_force"]
    if max_position <= 0.0 or stiffness <= 0.0:
        raise CabinetSpawnError(
            f"{label} max_position and spring_stiffness must be positive."
        )
    if not 0.0 < threshold <= max_position:
        raise CabinetSpawnError(
            f"{label}.press_threshold must be in (0, max_position]."
        )
    if not 0.0 < default_force <= stiffness * max_position:
        raise CabinetSpawnError(
            f"{label}.default_force must be in the physical force range."
        )
    return profile


def _apply_button_profiles(document: Any, profiles: Mapping[str, Mapping[str, Any]]) -> None:
    joints = {
        element.getAttribute("name"): element
        for element in document.getElementsByTagName("joint")
    }
    plugin_controls = {}
    for element in document.getElementsByTagName("control"):
        if _element_text(element, "control_type") != "button":
            continue
        plugin_controls[_element_text(element, "control_id")] = element

    for control_id, profile in profiles.items():
        joint = joints.get(profile["joint_name"])
        plugin_control = plugin_controls.get(control_id)
        if joint is None or plugin_control is None:
            raise CabinetSpawnError(
                f"Generated cabinet has no physical button '{control_id}'."
            )
        limits = joint.getElementsByTagName("limit")
        if len(limits) != 1:
            raise CabinetSpawnError(
                f"Button '{control_id}' has no joint limit."
            )
        limits[0].setAttribute("upper", str(profile["max_position"]))
        _set_element_text(
            plugin_control, "spring_stiffness", profile["spring_stiffness"]
        )
        _set_element_text(
            plugin_control, "press_threshold", profile["press_threshold"]
        )
        _set_element_text(
            plugin_control, "release_threshold", 0.5 * profile["press_threshold"]
        )


def _element_text(element: Any, tag_name: str) -> str:
    children = element.getElementsByTagName(tag_name)
    if len(children) != 1 or children[0].firstChild is None:
        raise CabinetSpawnError(f"Generated cabinet is missing {tag_name}.")
    return children[0].firstChild.data.strip()


def _set_element_text(element: Any, tag_name: str, value: Any) -> None:
    children = element.getElementsByTagName(tag_name)
    if len(children) != 1 or children[0].firstChild is None:
        raise CabinetSpawnError(f"Generated cabinet is missing {tag_name}.")
    children[0].firstChild.data = str(value)
