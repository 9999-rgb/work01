"""Cross-file contract validation for robot and cabinet adapter profiles.

This module deliberately contains no ROS imports.  It verifies the portable
configuration boundary shared by launch, the Web task layer, and the C++
cabinet nodes before a simulation is started.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from .inventory import CabinetInventory
from .inventory import InventoryError
from .robot_adapter import RobotAdapterError
from .robot_adapter import load as load_robot_adapter


_CONTROL_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_CONTROL_TYPES = {"button", "knob", "switch", "door"}


class ProfileContractError(ValueError):
    """Raised when two otherwise valid profile files disagree."""


@dataclass(frozen=True)
class ProfileContractReport:
    """Summary returned after a complete adapter-profile validation."""

    cabinet_count: int
    control_count: int
    button_count: int
    knob_count: int
    switch_count: int
    door_count: int
    arm_joint_count: int
    gripper_joint_count: int
    planning_frame: str
    navigation_frame: str


def _load_yaml(path_value: str | Path, label: str) -> Mapping[str, Any]:
    path = Path(path_value).expanduser()
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise ProfileContractError(
            f"Cannot read {label} file {path}: {error}"
        ) from error
    except yaml.YAMLError as error:
        raise ProfileContractError(
            f"Invalid {label} YAML {path}: {error}"
        ) from error
    if not isinstance(document, Mapping):
        raise ProfileContractError(f"{label} file must contain a mapping.")
    return document


def _parameters(
    document: Mapping[str, Any],
    node_key: str,
    label: str,
) -> Mapping[str, Any]:
    node = document.get(node_key)
    if not isinstance(node, Mapping):
        raise ProfileContractError(
            f"{label} must define '{node_key}'."
        )
    parameters = node.get("ros__parameters")
    if not isinstance(parameters, Mapping):
        raise ProfileContractError(
            f"{label} '{node_key}' must contain ros__parameters."
        )
    return parameters


def _string(value: Any, field: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise ProfileContractError(f"{field} must be a string.")
    result = value.strip()
    if not result and not allow_empty:
        raise ProfileContractError(f"{field} must not be empty.")
    return result


def _unique_strings(value: Any, field: str) -> tuple[str, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
    ):
        raise ProfileContractError(f"{field} must be a sequence.")
    values = tuple(_string(entry, f"{field} entry") for entry in value)
    if len(set(values)) != len(values):
        raise ProfileContractError(f"{field} entries must be unique.")
    return values


def _vector(value: Any, field: str, size: int) -> None:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
        or len(value) != size
    ):
        raise ProfileContractError(
            f"{field} must contain exactly {size} numeric values."
        )
    for entry in value:
        if (
            isinstance(entry, bool)
            or not isinstance(entry, (int, float))
            or not math.isfinite(float(entry))
        ):
            raise ProfileContractError(
                f"{field} must contain exactly {size} numeric values."
            )


def _validate_frame_geometry(scene: Mapping[str, Any]) -> None:
    part_ids = _unique_strings(scene.get("frame_part_ids"), "frame_part_ids")
    if not part_ids:
        raise ProfileContractError("frame_part_ids must not be empty.")
    parts = scene.get("frame_parts")
    if not isinstance(parts, Mapping):
        raise ProfileContractError("frame_parts must be a mapping.")
    for part_id in part_ids:
        part = parts.get(part_id)
        if not isinstance(part, Mapping):
            raise ProfileContractError(
                f"frame_parts.{part_id} must be a mapping."
            )
        _vector(part.get("size"), f"frame_parts.{part_id}.size", 3)
        _vector(part.get("position"), f"frame_parts.{part_id}.position", 3)


def _validate_control_collision(
    scene: Mapping[str, Any],
    present_types: set[str],
) -> None:
    collision = scene.get("control_collision")
    if not isinstance(collision, Mapping):
        if present_types & {"button", "knob"}:
            raise ProfileContractError(
                "control_collision is required for button or knob controls."
            )
        return
    if "button" in present_types:
        _vector(collision.get("button_size"), "control_collision.button_size", 2)
        offset = collision.get("button_center_offset")
        if (
            isinstance(offset, bool)
            or not isinstance(offset, (int, float))
            or not math.isfinite(float(offset))
        ):
            raise ProfileContractError(
                "control_collision.button_center_offset must be numeric."
            )
    if "knob" in present_types:
        _vector(collision.get("knob_size"), "control_collision.knob_size", 2)
        offset = collision.get("knob_center_offset")
        if (
            isinstance(offset, bool)
            or not isinstance(offset, (int, float))
            or not math.isfinite(float(offset))
        ):
            raise ProfileContractError(
                "control_collision.knob_center_offset must be numeric."
            )


def _validate_door_geometry(
    scene: Mapping[str, Any],
    door_id: str | None,
) -> None:
    if door_id is None:
        return
    door = scene.get("door")
    if not isinstance(door, Mapping):
        raise ProfileContractError("door geometry is required for a door control.")
    configured_id = _string(door.get("control_id"), "door.control_id")
    if configured_id != door_id:
        raise ProfileContractError(
            f"door.control_id '{configured_id}' does not match catalog door "
            f"'{door_id}'."
        )
    for field in ("hinge_position", "axis", "panel_size", "panel_position"):
        _vector(door.get(field), f"door.{field}", 3)
    part_ids = _unique_strings(
        door.get("handle_part_ids"), "door.handle_part_ids"
    )
    if not part_ids:
        raise ProfileContractError("door.handle_part_ids must not be empty.")
    parts = door.get("handle_parts")
    if not isinstance(parts, Mapping):
        raise ProfileContractError("door.handle_parts must be a mapping.")
    for part_id in part_ids:
        part = parts.get(part_id)
        if not isinstance(part, Mapping):
            raise ProfileContractError(
                f"door.handle_parts.{part_id} must be a mapping."
            )
        _vector(part.get("size"), f"door.handle_parts.{part_id}.size", 3)
        _vector(
            part.get("position"),
            f"door.handle_parts.{part_id}.position",
            3,
        )


def _validate_switch_geometry(
    scene: Mapping[str, Any],
    switch_id: str | None,
    parent_id: str,
) -> None:
    if switch_id is None:
        return
    switch = scene.get("switch")
    if not isinstance(switch, Mapping):
        raise ProfileContractError(
            "switch geometry is required for a switch control."
        )
    configured_id = _string(switch.get("control_id"), "switch.control_id")
    if configured_id != switch_id:
        raise ProfileContractError(
            f"switch.control_id '{configured_id}' does not match catalog "
            f"switch '{switch_id}'."
        )
    configured_parent = _string(
        switch.get("parent_control_id", ""),
        "switch.parent_control_id",
        allow_empty=True,
    )
    if configured_parent != parent_id:
        raise ProfileContractError(
            "switch.parent_control_id does not match the shared catalog."
        )
    for field in ("pivot_position", "axis", "size", "center_offset"):
        _vector(switch.get(field), f"switch.{field}", 3)


def _validate_robot_control_overrides(
    adapter_document: Mapping[str, Any],
    control_ids: set[str],
) -> None:
    operator = _parameters(
        adapter_document,
        "/**/xczs_cabinet_button_operator",
        "robot adapter",
    )
    overrides = operator.get("controls", {})
    if not isinstance(overrides, Mapping):
        raise ProfileContractError("robot adapter controls must be a mapping.")
    unknown_overrides = set(overrides) - control_ids
    if unknown_overrides:
        raise ProfileContractError(
            "robot adapter controls contains unknown IDs: "
            + ", ".join(sorted(str(value) for value in unknown_overrides))
        )
    for control_id, override in overrides.items():
        if not isinstance(override, Mapping):
            raise ProfileContractError(
                f"robot adapter controls.{control_id} must be a mapping."
            )
        if "operable" in override and not isinstance(
            override["operable"], bool
        ):
            raise ProfileContractError(
                f"robot adapter controls.{control_id}.operable must be boolean."
            )
        if override.get("operable") is False:
            _string(
                override.get("unavailable_reason"),
                f"robot adapter controls.{control_id}.unavailable_reason",
            )

    unreachable = _unique_strings(
        operator.get("unreachable_control_ids", ()),
        "unreachable_control_ids",
    )
    unknown_unreachable = set(unreachable) - control_ids
    if unknown_unreachable:
        raise ProfileContractError(
            "unreachable_control_ids contains unknown IDs: "
            + ", ".join(sorted(unknown_unreachable))
        )
    # A shared unreachable policy and a per-control navigation station are
    # orthogonal: every submitted control still needs a robot-visible station
    # so the live planning-only validation can run.  Reject only a second
    # availability policy for an ID already covered by the shared list.
    policy_overlap = {
        control_id
        for control_id in set(unreachable) & set(overrides)
        if "operable" in overrides[control_id]
        or "unavailable_reason" in overrides[control_id]
    }
    if policy_overlap:
        raise ProfileContractError(
            "Controls must not define availability in both per-control "
            "overrides and unreachable_control_ids: "
            + ", ".join(sorted(policy_overlap))
        )
    if unreachable:
        _string(
            operator.get("unreachable_control_reason"),
            "unreachable_control_reason",
        )
    station_ids = {
        str(control_id)
        for control_id, override in overrides.items()
        if isinstance(override, Mapping)
        and isinstance(override.get("navigation_station"), Mapping)
    }
    missing_stations = control_ids - station_ids
    if missing_stations:
        raise ProfileContractError(
            "Every submitted control requires an explicit robot-adapter "
            "navigation_station: "
            + ", ".join(sorted(missing_stations))
        )


def validate_profile(
    *,
    robot_adapter_path: str | Path,
    instances_path: str | Path,
    controls_path: str | Path,
    scene_path: str | Path,
    pose_path: str | Path,
) -> ProfileContractReport:
    """Validate one complete robot + cabinet profile without starting ROS."""
    try:
        adapter = load_robot_adapter(robot_adapter_path)
        inventory = CabinetInventory.load(instances_path, scene_path)
    except (RobotAdapterError, InventoryError) as error:
        raise ProfileContractError(str(error)) from error

    adapter_document = _load_yaml(robot_adapter_path, "robot adapter")
    adapter_global = _parameters(adapter_document, "/**", "robot adapter")
    embedded_navigation = adapter_global.get("allow_embedded_navigation")
    if embedded_navigation is not False:
        raise ProfileContractError(
            "allow_embedded_navigation must be false; the task layer owns the "
            "single navigation-station calculation."
        )

    controls_document = _load_yaml(controls_path, "control catalog")
    controls_parameters = _parameters(
        controls_document, "/**", "control catalog"
    )
    ordered_ids = _unique_strings(
        controls_parameters.get("control_ids"), "control_ids"
    )
    if not ordered_ids:
        raise ProfileContractError("control_ids must not be empty.")
    for control_id in ordered_ids:
        if _CONTROL_ID_PATTERN.fullmatch(control_id) is None:
            raise ProfileContractError(
                f"Control ID '{control_id}' must match "
                "[a-z][a-z0-9_]{0,62}."
            )
    controls = controls_parameters.get("controls")
    if not isinstance(controls, Mapping):
        raise ProfileContractError("controls must be a mapping.")
    if set(controls) != set(ordered_ids):
        missing = sorted(set(ordered_ids) - set(controls))
        extra = sorted(set(controls) - set(ordered_ids))
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if extra:
            details.append("unknown " + ", ".join(str(value) for value in extra))
        raise ProfileContractError(
            "controls must match control_ids exactly (" + "; ".join(details) + ")."
        )

    type_by_id: dict[str, str] = {}
    parent_by_id: dict[str, str] = {}
    for control_id in ordered_ids:
        control = controls[control_id]
        if not isinstance(control, Mapping):
            raise ProfileContractError(
                f"controls.{control_id} must be a mapping."
            )
        control_type = _string(
            control.get("type"), f"controls.{control_id}.type"
        )
        if control_type not in _CONTROL_TYPES:
            raise ProfileContractError(
                f"No planning-scene adapter exists for control type "
                f"'{control_type}'."
            )
        type_by_id[control_id] = control_type
        parent_by_id[control_id] = _string(
            control.get("parent_control_id", ""),
            f"controls.{control_id}.parent_control_id",
            allow_empty=True,
        )

    door_ids = [key for key, value in type_by_id.items() if value == "door"]
    switch_ids = [
        key for key, value in type_by_id.items() if value == "switch"
    ]
    if len(door_ids) > 1:
        raise ProfileContractError(
            "The current planning-scene adapter supports at most one door."
        )
    if len(switch_ids) > 1:
        raise ProfileContractError(
            "The current planning-scene adapter supports at most one switch."
        )
    switch_parent = parent_by_id[switch_ids[0]] if switch_ids else ""
    if switch_parent:
        if type_by_id.get(switch_parent) != "door":
            raise ProfileContractError(
                f"Switch parent '{switch_parent}' must reference the catalog door."
            )
        if not door_ids or door_ids[0] != switch_parent:
            raise ProfileContractError(
                "The switch parent must match the configured scene door."
            )

    scene_document = _load_yaml(scene_path, "cabinet scene")
    scene = _parameters(
        scene_document,
        "/**/xczs_cabinet_planning_scene",
        "cabinet scene",
    )
    _validate_frame_geometry(scene)
    _validate_control_collision(scene, set(type_by_id.values()))
    _validate_door_geometry(scene, door_ids[0] if door_ids else None)
    _validate_switch_geometry(
        scene,
        switch_ids[0] if switch_ids else None,
        switch_parent,
    )

    pose_document = _load_yaml(pose_path, "cabinet pose")
    pose = _parameters(
        pose_document,
        "/**/xczs_cabinet_pose_authority",
        "cabinet pose",
    )
    pose_parent = _string(pose.get("parent_frame"), "parent_frame")
    if pose_parent != adapter.pose_parent_frame:
        raise ProfileContractError(
            f"cabinet pose parent_frame '{pose_parent}' does not match robot "
            f"pose_parent_frame '{adapter.pose_parent_frame}'."
        )

    for cabinet in inventory:
        station = inventory.station_for(cabinet.name)
        if station.frame_id != adapter.navigation_frame:
            raise ProfileContractError(
                f"Navigation station for '{cabinet.name}' uses frame "
                f"'{station.frame_id}', expected '{adapter.navigation_frame}'."
            )
        for control_id, control_station in (
            adapter.control_navigation_stations
        ):
            resolved = inventory.station_for(
                cabinet.name,
                control_station=control_station,
            )
            if resolved.frame_id != adapter.navigation_frame:
                raise ProfileContractError(
                    f"Navigation station for '{cabinet.name}/"
                    f"{control_id}' uses frame '{resolved.frame_id}', "
                    f"expected '{adapter.navigation_frame}'."
                )

    _validate_robot_control_overrides(adapter_document, set(ordered_ids))
    counts = {
        control_type: sum(
            value == control_type for value in type_by_id.values()
        )
        for control_type in _CONTROL_TYPES
    }
    return ProfileContractReport(
        cabinet_count=len(inventory),
        control_count=len(ordered_ids),
        button_count=counts["button"],
        knob_count=counts["knob"],
        switch_count=counts["switch"],
        door_count=counts["door"],
        arm_joint_count=len(adapter.arm_joint_names),
        gripper_joint_count=len(adapter.gripper_joint_names),
        planning_frame=adapter.planning_frame,
        navigation_frame=adapter.navigation_frame,
    )
