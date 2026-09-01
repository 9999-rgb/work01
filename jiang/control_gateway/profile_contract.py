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
_RELATIVE_TOPIC_PATTERN = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
_CONTROL_TYPES = {"button", "knob", "switch", "door", "slider", "drawer"}


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
    operable_control_ids: tuple[str, ...]
    joint_group_counts: tuple[tuple[str, int], ...]
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


def _relative_topic(value: Any, field: str) -> str:
    result = _string(value, field)
    if _RELATIVE_TOPIC_PATTERN.fullmatch(result) is None:
        raise ProfileContractError(
            f"{field} must be a non-empty relative ROS topic."
        )
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


def _finite_number(value: Any, field: str, *, allow_zero: bool) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or (float(value) < 0.0 if allow_zero else float(value) <= 0.0)
    ):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ProfileContractError(f"{field} must be a finite {qualifier} number.")
    return float(value)


def _worst_footprint_extent(
    points: Sequence[tuple[float, float]],
    yaw_offset: float,
    yaw_tolerance: float,
) -> float:
    extent = -math.inf
    for x_value, y_value in points:
        projection = lambda angle: (  # noqa: E731
            x_value * math.cos(angle) - y_value * math.sin(angle)
        )
        extent = max(
            extent,
            projection(yaw_offset - yaw_tolerance),
            projection(yaw_offset + yaw_tolerance),
        )
        radius = math.hypot(x_value, y_value)
        if yaw_tolerance >= math.pi:
            extent = max(extent, radius)
            continue
        maximizing_angle = math.atan2(-y_value, x_value)
        angle_from_center = math.atan2(
            math.sin(maximizing_angle - yaw_offset),
            math.cos(maximizing_angle - yaw_offset),
        )
        if abs(angle_from_center) <= yaw_tolerance + 1e-12:
            extent = max(extent, radius)
    return extent


def _validate_operable_station_clearance(
    operator: Mapping[str, Any],
    overrides: Mapping[str, Any],
    operable: tuple[str, ...],
) -> None:
    if not operable:
        return
    raw_footprint = operator.get("docking_base_footprint")
    if (
        not isinstance(raw_footprint, Sequence)
        or isinstance(raw_footprint, (str, bytes, bytearray))
        or len(raw_footprint) < 6
        or len(raw_footprint) % 2 != 0
    ):
        raise ProfileContractError(
            "docking_base_footprint must contain at least three flat [x, y] "
            "points when any control is operable."
        )
    _vector(raw_footprint, "docking_base_footprint", len(raw_footprint))
    footprint = tuple(
        (float(raw_footprint[index]), float(raw_footprint[index + 1]))
        for index in range(0, len(raw_footprint), 2)
    )
    padding = _finite_number(
        operator.get("docking_base_footprint_padding", 0.03),
        "docking_base_footprint_padding",
        allow_zero=False,
    )
    position_tolerance = _finite_number(
        operator.get("docking_position_tolerance", 0.015),
        "docking_position_tolerance",
        allow_zero=False,
    )
    yaw_tolerance = _finite_number(
        operator.get("docking_yaw_tolerance", 0.10),
        "docking_yaw_tolerance",
        allow_zero=False,
    )
    for control_id in operable:
        station = overrides[control_id]["navigation_station"]
        yaw_offset = station.get("base_yaw_offset", 0.0)
        if (
            isinstance(yaw_offset, bool)
            or not isinstance(yaw_offset, (int, float))
            or not math.isfinite(float(yaw_offset))
        ):
            raise ProfileContractError(
                f"controls.{control_id}.navigation_station.base_yaw_offset "
                "must be finite."
            )
        extent = _worst_footprint_extent(
            footprint, float(yaw_offset), yaw_tolerance
        )
        minimum_standoff = extent + padding + position_tolerance
        standoff = float(station["standoff"])
        if extent <= 0.0 or standoff + 1e-12 < minimum_standoff:
            raise ProfileContractError(
                f"Operable control '{control_id}' station standoff "
                f"{standoff:.6f} m is below the full docking footprint "
                f"safety envelope {minimum_standoff:.6f} m."
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
    adapter_parameters: Mapping[str, Any],
    controls: Mapping[str, Any],
    controls_parameters: Mapping[str, Any],
) -> tuple[str, ...]:
    control_ids = set(controls)
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
        if "operable" in override:
            raise ProfileContractError(
                "Per-control operable flags are not supported; physical "
                "capability has one authoritative operable_control_ids "
                "allowlist."
            )
        if "unavailable_reason" in override:
            _string(
                override.get("unavailable_reason"),
                f"robot adapter controls.{control_id}.unavailable_reason",
            )
        control = controls[control_id]
        assert isinstance(control, Mapping)
        if "tool_roll_offset" in override:
            field = f"robot adapter controls.{control_id}.tool_roll_offset"
            roll_offset = override.get("tool_roll_offset")
            if control.get("type") != "button":
                raise ProfileContractError(
                    f"{field} is only valid for button controls."
                )
            if (
                isinstance(roll_offset, bool)
                or not isinstance(roll_offset, (int, float))
                or not math.isfinite(float(roll_offset))
                or abs(float(roll_offset)) > math.pi
            ):
                raise ProfileContractError(
                    f"{field} must be finite and within [-pi, pi]."
                )
        has_roll_calibration = "tool_roll_offsets" in override
        has_partial_release = "detent_release_fraction" in override
        has_ready_joint_seed = "ready_joint_seed" in override
        if has_roll_calibration or has_partial_release:
            if control.get("type") != "knob":
                raise ProfileContractError(
                    f"robot adapter controls.{control_id} rotary tool "
                    "calibration is only valid for knob controls."
                )
            defaults = controls_parameters.get("knob_defaults", {})
            if not isinstance(defaults, Mapping):
                defaults = {}
            raw_state_ids = control.get(
                "state_ids", defaults.get("state_ids")
            )
            state_ids = _unique_strings(
                raw_state_ids,
                f"controls.{control_id}.state_ids",
            )
            if not state_ids:
                raise ProfileContractError(
                    f"controls.{control_id}.state_ids must not be empty."
                )
            if has_roll_calibration:
                field = (
                    f"robot adapter controls.{control_id}.tool_roll_offsets"
                )
                roll_offsets = override.get("tool_roll_offsets")
                _vector(roll_offsets, field, len(state_ids) ** 2)
                assert isinstance(roll_offsets, Sequence)
                if any(abs(float(value)) > math.pi for value in roll_offsets):
                    raise ProfileContractError(
                        f"{field} entries must be within [-pi, pi]."
                    )
            if has_partial_release:
                field = (
                    f"robot adapter controls.{control_id}."
                    "detent_release_fraction"
                )
                fraction = override.get("detent_release_fraction")
                if (
                    isinstance(fraction, bool)
                    or not isinstance(fraction, (int, float))
                    or not math.isfinite(float(fraction))
                    or float(fraction) <= 0.5
                    or float(fraction) > 1.0
                ):
                    raise ProfileContractError(
                        f"{field} must be a finite number in (0.5, 1.0]."
                    )
        if has_ready_joint_seed:
            field = (
                f"robot adapter controls.{control_id}.ready_joint_seed"
            )
            seed = override.get("ready_joint_seed")
            if not isinstance(seed, Mapping):
                raise ProfileContractError(f"{field} must be a mapping.")
            unknown_seed_fields = set(seed) - {
                "joint_names",
                "positions",
            }
            if unknown_seed_fields:
                raise ProfileContractError(
                    f"{field} contains unknown fields: "
                    + ", ".join(sorted(str(v) for v in unknown_seed_fields))
                )
            seed_names = _unique_strings(
                seed.get("joint_names"), f"{field}.joint_names"
            )
            seed_positions = seed.get("positions")
            _vector(seed_positions, f"{field}.positions", len(seed_names))

            # ready_joint_seed pins the free-space target pose onto one IK
            # branch (e.g. the r_arm_4<0 branch that keeps a button approach
            # continuous across the wrist-flip singularity), so it is valid
            # for any control whose tool_profile names an arm move_group --
            # buttons, knobs and switches.  The seed joints must be exactly
            # the variables of that control's own move_group.
            control_type = control.get("type")
            if control_type not in ("button", "knob", "switch"):
                raise ProfileContractError(
                    f"{field} is only valid for button, knob or switch "
                    "controls."
                )
            tool_profiles = operator.get("tool_profiles", {})
            manual_groups = adapter_parameters.get(
                "manual_joint_groups", {}
            )
            control_profile = (
                tool_profiles.get(control_type, {})
                if isinstance(tool_profiles, Mapping)
                else {}
            )
            move_group = (
                control_profile.get("move_group")
                if isinstance(control_profile, Mapping)
                else None
            )
            expected_names = (
                manual_groups.get(move_group)
                if isinstance(manual_groups, Mapping)
                and isinstance(move_group, str)
                else None
            )
            if (
                not isinstance(expected_names, Sequence)
                or isinstance(expected_names, (str, bytes))
                or set(seed_names) != set(expected_names)
                or len(seed_names) != len(expected_names)
            ):
                raise ProfileContractError(
                    f"{field}.joint_names must name every joint in the "
                    f"'{move_group}' MoveIt group exactly once."
                )

    for legacy_field in (
        "unreachable_control_ids",
        "unreachable_control_reason",
    ):
        if legacy_field in operator:
            raise ProfileContractError(
                f"{legacy_field} is fail-open and no longer supported; use "
                "operable_control_ids with inoperable_control_reason."
            )
    raw_operable = operator.get("operable_control_ids", ())
    if "operable_control_ids" in operator and raw_operable == []:
        raise ProfileContractError(
            "operable_control_ids must be omitted when empty; ROS 2 "
            "parameter files cannot infer the type of an empty sequence."
        )
    operable = _unique_strings(raw_operable, "operable_control_ids")
    unknown_operable = set(operable) - control_ids
    if unknown_operable:
        raise ProfileContractError(
            "operable_control_ids contains unknown IDs: "
            + ", ".join(sorted(unknown_operable))
        )
    reason_overlap = {
        control_id
        for control_id in set(operable) & set(overrides)
        if "unavailable_reason" in overrides[control_id]
    }
    if reason_overlap:
        raise ProfileContractError(
            "Operable controls must not declare unavailable_reason: "
            + ", ".join(sorted(reason_overlap))
        )
    if (
        "inoperable_control_reason" in operator
        or set(operable) != control_ids
    ):
        _string(
            operator.get("inoperable_control_reason"),
            "inoperable_control_reason",
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
    _validate_operable_station_clearance(operator, overrides, tuple(operable))
    return tuple(operable)


def validate_profile(
    *,
    robot_adapter_path: str | Path,
    instances_path: str | Path,
    controls_path: str | Path,
    scene_path: str | Path,
    pose_path: str | Path,
    toolset: str = "A",
) -> ProfileContractReport:
    """Validate one complete robot + cabinet profile without starting ROS."""
    try:
        adapter = load_robot_adapter(robot_adapter_path, toolset=toolset)
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

    operator = _parameters(
        adapter_document,
        "/**/xczs_cabinet_button_operator",
        "robot adapter",
    )
    scene_pose_topic = _relative_topic(
        scene.get("pose_valid_topic", "pose_valid"),
        "cabinet scene pose_valid_topic",
    )
    authority_pose_topic = _relative_topic(
        pose.get("validity_topic", "pose_valid"),
        "cabinet pose validity_topic",
    )
    operator_pose_topic = _relative_topic(
        operator.get("cabinet_pose_valid_topic", "pose_valid"),
        "robot adapter cabinet_pose_valid_topic",
    )
    pose_topics = {
        scene_pose_topic,
        authority_pose_topic,
        operator_pose_topic,
    }
    if len(pose_topics) != 1:
        raise ProfileContractError(
            "Cabinet pose-validity topics must match across cabinet_scene, "
            "cabinet_pose, and cabinet_robot_adapter."
        )
    if scene.get("require_pose_valid") is not True:
        raise ProfileContractError(
            "cabinet scene require_pose_valid must be true."
        )
    if operator.get("require_cabinet_pose_valid", True) is not True:
        raise ProfileContractError(
            "robot adapter require_cabinet_pose_valid must be true."
        )

    # 混合库存（共享 profile 的 cabinet 实例 + 夹具注册）中，夹具实例使用
    # 逐场景 scene_controls/ 三件套，本站校验（共享 adapter/controls/scene）
    # 只覆盖 kind == cabinet 的实例。若库存全为夹具（逐场景 profile），
    # 夹具实例照常校验其导航/控制站。
    has_cabinet_instances = any(
        instance.kind != "fixture" for instance in inventory
    )
    for cabinet in inventory:
        if cabinet.kind == "fixture" and has_cabinet_instances:
            continue
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

    operable_control_ids = _validate_robot_control_overrides(
        adapter_document, adapter_global, controls, controls_parameters
    )
    counts = {
        control_type: sum(
            value == control_type for value in type_by_id.values()
        )
        for control_type in _CONTROL_TYPES
    }
    # 共享 profile 只覆盖 kind == cabinet 的实例（夹具实例用逐场景
    # scene_controls/ 三件套，由 check_scene_config/check_adapter_contract
    # 校验），计数与之保持一致。
    cabinet_count = sum(
        1 for instance in inventory if instance.kind != "fixture"
    )
    return ProfileContractReport(
        cabinet_count=cabinet_count,
        control_count=len(ordered_ids),
        button_count=counts["button"],
        knob_count=counts["knob"],
        switch_count=counts["switch"],
        door_count=counts["door"],
        operable_control_ids=operable_control_ids,
        joint_group_counts=tuple(
            (group.name, len(group.joint_names))
            for group in adapter.controller_groups
        ),
        planning_frame=adapter.planning_frame,
        navigation_frame=adapter.navigation_frame,
    )
