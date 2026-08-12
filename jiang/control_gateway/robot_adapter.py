"""Typed robot-interface configuration shared by the Web control gateway."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple, Union

import yaml

from .inventory import InventoryError
from .inventory import NavigationStationSpec


_ABSOLUTE_ROS_NAME = re.compile(
    r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
_DEFAULT_NAVIGATION_FRAME = "map"
_LEGACY_DEFAULTS = {
    "planning_frame": "odom",
    "manual_linear_axis": "y",
    "navigation_velocity_yaw_offset": 1.57079632679,
    "navigation_action": "/navigate_to_pose",
    "navigation_mode_service": "/xczs/set_navigation_mode",
    "navigation_mode_topic": "/xczs/navigation_mode",
    "map_topic": "/map",
    "localization_pose_topic": "/amcl_pose",
    "manual_cmd_vel_topic": "/xczs/manual_cmd_vel",
    "navigation_cmd_vel_topic": "/cmd_vel",
    "base_output_topic": "/xczs/cmd_vel",
    "joint_trajectory_topic": "/xczs/joint_trajectory",
    "joint_state_topic": "/xczs/joint_states",
    "reset_base_pose": {
        "frame_id": "map",
        "x": 0.0,
        "y": 0.0,
        "yaw": math.pi / 2.0,
    },
    "reset_joint_tolerance": 0.02,
    "reset_joint_timeout_sec": 15.0,
    "reset_joint_duration_sec": 5.0,
    "arm_controller_topic": "/xczs/arm_controller/joint_trajectory",
    "arm_controller_status_topic": (
        "/xczs/arm_controller/follow_joint_trajectory/_action/status"
    ),
    "gripper_controller_topic": (
        "/xczs/gripper_controller/joint_trajectory"
    ),
    "gripper_controller_status_topic": (
        "/xczs/gripper_controller/follow_joint_trajectory/_action/status"
    ),
    "arm_joint_names": (
        "body_arm1",
        "arm1_arm2",
        "arm2_arm3",
        "arm3_arm4",
        "arm4_arm5",
        "arm5_end",
    ),
    "gripper_joint_names": (
        "end_worklink1",
        "end_worklink2",
    ),
    "manual_joint_limits": {
        "body_arm1": {"min_position": -2.8, "max_position": 2.8},
        "arm1_arm2": {"min_position": -2.8, "max_position": 2.8},
        "arm2_arm3": {"min_position": -2.8, "max_position": 2.8},
        "arm3_arm4": {"min_position": -2.8, "max_position": 2.8},
        "arm4_arm5": {"min_position": -2.8, "max_position": 2.8},
        "arm5_end": {"min_position": -2.8, "max_position": 2.8},
        "end_worklink1": {"min_position": 0.0, "max_position": 0.35},
        "end_worklink2": {"min_position": -0.35, "max_position": 0.0},
    },
}


class RobotAdapterError(RuntimeError):
    """Raised when the robot adapter file is missing or malformed."""


@dataclass(frozen=True)
class ManualJointConfig:
    """One ordered manual-control joint and its safe command range."""

    name: str
    group: str
    min_position: float
    max_position: float
    default_position: float
    open_position: Union[float, None]


@dataclass(frozen=True)
class ResetBasePoseConfig:
    """Validated navigation-frame pose used for a robot reset."""

    frame_id: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RobotAdapterConfig:
    """Immutable ROS/TF interface contract for one robot adapter."""

    planning_frame: str
    pose_parent_frame: str
    manual_linear_axis: str
    navigation_velocity_yaw_offset: float
    navigation_frame: str
    navigation_base_frame: str
    navigation_action: str
    navigation_mode_service: str
    navigation_mode_topic: str
    map_topic: str
    localization_pose_topic: str
    manual_cmd_vel_topic: str
    navigation_cmd_vel_topic: str
    base_output_topic: str
    joint_trajectory_topic: str
    joint_state_topic: str
    reset_base_pose: ResetBasePoseConfig
    reset_joint_tolerance: float
    reset_joint_timeout_sec: float
    reset_joint_duration_sec: float
    arm_controller_topic: str
    arm_controller_status_topic: str
    gripper_controller_topic: Union[str, None]
    gripper_controller_status_topic: Union[str, None]
    manual_joints: Tuple[ManualJointConfig, ...]
    control_navigation_stations: Tuple[
        Tuple[str, NavigationStationSpec], ...
    ]

    @property
    def manual_joint_names(self) -> Tuple[str, ...]:
        """Return the ordered joint names expected by the Web payload."""
        return tuple(joint.name for joint in self.manual_joints)

    @property
    def arm_joint_names(self) -> Tuple[str, ...]:
        """Return ordered manual joints owned by the arm controller."""
        return tuple(
            joint.name for joint in self.manual_joints if joint.group == "arm"
        )

    @property
    def gripper_joint_names(self) -> Tuple[str, ...]:
        """Return ordered manual joints owned by the gripper controller."""
        return tuple(
            joint.name
            for joint in self.manual_joints
            if joint.group == "gripper"
        )

    def control_navigation_station(
        self,
        control_id: str,
    ) -> Optional[NavigationStationSpec]:
        """Return a robot-specific station for one control, if configured."""
        for configured_id, station in self.control_navigation_stations:
            if configured_id == control_id:
                return station
        return None


def _relative_name(value: Any, field: str) -> str:
    if not isinstance(value, str):
        raise RobotAdapterError(f"{field} must be a non-empty relative name.")
    name = value.strip()
    if (
        not name
        or name.startswith("/")
        or name.endswith("/")
        or "//" in name
        or any(character.isspace() for character in name)
    ):
        raise RobotAdapterError(f"{field} must be a non-empty relative name.")
    return name


def _absolute_ros_name(value: Any, field: str) -> str:
    if not isinstance(value, str):
        raise RobotAdapterError(f"{field} must be an absolute ROS name.")
    name = value.strip()
    if _ABSOLUTE_ROS_NAME.fullmatch(name) is None:
        raise RobotAdapterError(f"{field} must be an absolute ROS name.")
    return name


def _joint_names(
    value: Any,
    field: str,
    *,
    allow_empty: bool,
) -> Tuple[str, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
        or (not value and not allow_empty)
    ):
        raise RobotAdapterError(
            f"{field} must be "
            + ("a sequence." if allow_empty else "a non-empty sequence.")
        )
    names = tuple(
        _relative_name(name, f"{field} entry") for name in value
    )
    if len(set(names)) != len(names):
        raise RobotAdapterError(f"{field} entries must be unique.")
    return names


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise RobotAdapterError(f"{field} must be a finite number.")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise RobotAdapterError(f"{field} must be a finite number.") from error
    if not math.isfinite(number):
        raise RobotAdapterError(f"{field} must be a finite number.")
    return number


def _positive_number(value: Any, field: str) -> float:
    number = _finite_number(value, field)
    if number <= 0.0:
        raise RobotAdapterError(f"{field} must be positive.")
    return number


def _reset_base_pose(
    value: Any,
    navigation_frame: str,
) -> ResetBasePoseConfig:
    field = "reset_base_pose"
    if not isinstance(value, Mapping):
        raise RobotAdapterError(f"{field} must be a mapping.")
    if any(not isinstance(name, str) for name in value):
        raise RobotAdapterError(f"{field} keys must be strings.")
    required = {"frame_id", "x", "y", "yaw"}
    configured = set(value)
    missing = sorted(required - configured)
    unknown = sorted(configured - required)
    if missing:
        raise RobotAdapterError(
            f"{field} is missing required fields: {', '.join(missing)}."
        )
    if unknown:
        raise RobotAdapterError(
            f"{field} contains unknown fields: {', '.join(unknown)}."
        )
    frame_id = _relative_name(value["frame_id"], f"{field}.frame_id")
    if frame_id != navigation_frame:
        raise RobotAdapterError(
            f"{field}.frame_id must match navigation_frame "
            f"'{navigation_frame}'."
        )
    return ResetBasePoseConfig(
        frame_id=frame_id,
        x=_finite_number(value["x"], f"{field}.x"),
        y=_finite_number(value["y"], f"{field}.y"),
        yaw=_finite_number(value["yaw"], f"{field}.yaw"),
    )


def _manual_joints(
    arm_names_value: Any,
    gripper_names_value: Any,
    limits_value: Any,
) -> Tuple[ManualJointConfig, ...]:
    arm_names = _joint_names(
        arm_names_value,
        "arm_joint_names",
        allow_empty=False,
    )
    gripper_names = _joint_names(
        gripper_names_value,
        "gripper_joint_names",
        allow_empty=True,
    )
    names = arm_names + gripper_names
    if len(set(names)) != len(names):
        raise RobotAdapterError(
            "arm_joint_names and gripper_joint_names must not overlap."
        )
    if not isinstance(limits_value, Mapping):
        raise RobotAdapterError("manual_joint_limits must be a mapping.")
    if any(not isinstance(name, str) for name in limits_value):
        raise RobotAdapterError(
            "manual_joint_limits keys must be manual joint names."
        )
    configured_names = set(limits_value)
    expected_names = set(names)
    if configured_names != expected_names:
        missing = sorted(expected_names - configured_names)
        extra = sorted(configured_names - expected_names)
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if extra:
            details.append("unknown " + ", ".join(extra))
        raise RobotAdapterError(
            "manual_joint_limits must match arm_joint_names plus "
            "gripper_joint_names exactly ("
            + "; ".join(details)
            + ")."
        )

    joints = []
    for name in names:
        limits = limits_value[name]
        if not isinstance(limits, Mapping):
            raise RobotAdapterError(
                f"manual_joint_limits.{name} must be a mapping."
            )
        if "min_position" not in limits or "max_position" not in limits:
            raise RobotAdapterError(
                f"manual_joint_limits.{name} requires min_position and "
                "max_position."
            )
        minimum = _finite_number(
            limits["min_position"],
            f"manual_joint_limits.{name}.min_position",
        )
        maximum = _finite_number(
            limits["max_position"],
            f"manual_joint_limits.{name}.max_position",
        )
        if minimum > maximum:
            raise RobotAdapterError(
                f"manual_joint_limits.{name} minimum exceeds maximum."
            )
        default = _finite_number(
            limits.get("default_position", 0.0),
            f"manual_joint_limits.{name}.default_position",
        )
        if default < minimum or default > maximum:
            raise RobotAdapterError(
                f"manual_joint_limits.{name}.default_position is outside "
                "its configured range."
            )
        open_position = None
        if "open_position" in limits:
            open_position = _finite_number(
                limits["open_position"],
                f"manual_joint_limits.{name}.open_position",
            )
            if open_position < minimum or open_position > maximum:
                raise RobotAdapterError(
                    f"manual_joint_limits.{name}.open_position is outside "
                    "its configured range."
                )
        group = "arm" if name in arm_names else "gripper"
        if group == "arm" and open_position is not None:
            raise RobotAdapterError(
                f"manual_joint_limits.{name}.open_position is only valid "
                "for gripper joints."
            )
        joints.append(
            ManualJointConfig(
                name=name,
                group=group,
                min_position=minimum,
                max_position=maximum,
                default_position=default,
                open_position=open_position,
            )
        )
    return tuple(joints)


def _parameters(
    document: Mapping[str, Any],
    path: Path,
) -> tuple[Mapping[str, Any], bool]:
    """Return the authoritative parameter mapping and whether it is legacy."""
    if "/**" in document:
        global_config = document["/**"]
        if not isinstance(global_config, Mapping):
            raise RobotAdapterError(
                f"Robot adapter {path} /** entry must contain a mapping."
            )
        parameters = global_config.get("ros__parameters")
        if not isinstance(parameters, Mapping):
            raise RobotAdapterError(
                f"Robot adapter {path} /** must contain ros__parameters."
            )
        return parameters, False

    matches = []
    for node_config in document.values():
        if not isinstance(node_config, Mapping):
            continue
        parameters = node_config.get("ros__parameters")
        if (
            isinstance(parameters, Mapping)
            and "navigation_base_frame" in parameters
        ):
            matches.append(parameters)
    if len(matches) != 1:
        raise RobotAdapterError(
            f"Robot adapter {path} must define /**.ros__parameters, or "
            "exactly one legacy ros__parameters.navigation_base_frame."
        )
    return matches[0], True


def _required(
    parameters: Mapping[str, Any],
    field: str,
    *,
    legacy: bool,
) -> Any:
    if field in parameters:
        return parameters[field]
    if legacy and field in _LEGACY_DEFAULTS:
        return _LEGACY_DEFAULTS[field]
    raise RobotAdapterError(f"Missing required robot adapter field: {field}.")


def _operator_parameters(
    document: Mapping[str, Any],
    path: Path,
) -> Optional[Mapping[str, Any]]:
    """Return the optional cabinet-operator section from an adapter file."""
    matches = []
    for node_name, node_config in document.items():
        if not isinstance(node_name, str) or not (
            node_name == "xczs_cabinet_button_operator"
            or node_name.endswith("/xczs_cabinet_button_operator")
        ):
            continue
        if not isinstance(node_config, Mapping):
            raise RobotAdapterError(
                f"Robot adapter {path} operator entry must be a mapping."
            )
        parameters = node_config.get("ros__parameters")
        if not isinstance(parameters, Mapping):
            raise RobotAdapterError(
                f"Robot adapter {path} operator entry must contain "
                "ros__parameters."
            )
        matches.append(parameters)
    if len(matches) > 1:
        raise RobotAdapterError(
            f"Robot adapter {path} defines multiple cabinet operator entries."
        )
    return matches[0] if matches else None


def _control_navigation_stations(
    document: Mapping[str, Any],
    path: Path,
) -> Tuple[Tuple[str, NavigationStationSpec], ...]:
    """Load full per-control station profiles from the operator adapter."""
    operator = _operator_parameters(document, path)
    if operator is None:
        return ()
    controls = operator.get("controls", {})
    if not isinstance(controls, Mapping):
        raise RobotAdapterError(
            f"Robot adapter {path} operator controls must be a mapping."
        )
    stations = []
    normalized_ids = set()
    for control_id, override in controls.items():
        if not isinstance(control_id, str) or not control_id.strip():
            raise RobotAdapterError(
                f"Robot adapter {path} control IDs must be non-empty strings."
            )
        normalized_id = control_id.strip()
        if normalized_id in normalized_ids:
            raise RobotAdapterError(
                f"Robot adapter {path} defines duplicate control ID after "
                f"trimming whitespace: {normalized_id}."
            )
        normalized_ids.add(normalized_id)
        if not isinstance(override, Mapping):
            raise RobotAdapterError(
                f"Robot adapter controls.{control_id} must be a mapping."
            )
        value = override.get("navigation_station")
        if value is None:
            continue
        try:
            station = NavigationStationSpec.from_mapping(
                value,
                context=f"controls.{control_id}.navigation_station",
            )
        except InventoryError as error:
            raise RobotAdapterError(str(error)) from error
        stations.append((normalized_id, station))
    return tuple(stations)


def load(path_value: Union[str, Path]) -> RobotAdapterConfig:
    """Load and strictly validate a cabinet robot adapter YAML file."""
    path = Path(path_value).expanduser().resolve()
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise RobotAdapterError(
            f"Cannot read robot adapter {path}: {error}"
        ) from error
    except yaml.YAMLError as error:
        raise RobotAdapterError(
            f"Invalid robot adapter YAML {path}: {error}"
        ) from error
    if not isinstance(document, Mapping):
        raise RobotAdapterError(
            f"Robot adapter {path} must contain a mapping."
        )

    parameters, legacy = _parameters(document, path)
    if not legacy and "manual_joint_names" in parameters:
        raise RobotAdapterError(
            "manual_joint_names is deprecated; use arm_joint_names and "
            "gripper_joint_names as the single ordered source."
        )
    planning_frame = _relative_name(
        _required(parameters, "planning_frame", legacy=legacy),
        "planning_frame",
    )
    navigation_frame = _relative_name(
        parameters.get("navigation_frame", _DEFAULT_NAVIGATION_FRAME),
        "navigation_frame",
    )
    manual_linear_axis_value = parameters.get(
        "manual_linear_axis",
        _LEGACY_DEFAULTS["manual_linear_axis"] if legacy else None,
    )
    if manual_linear_axis_value not in {"x", "y"}:
        raise RobotAdapterError("manual_linear_axis must be either x or y.")
    manual_joints = _manual_joints(
        _required(parameters, "arm_joint_names", legacy=legacy),
        _required(parameters, "gripper_joint_names", legacy=legacy),
        _required(parameters, "manual_joint_limits", legacy=legacy),
    )
    reset_base_pose = _reset_base_pose(
        _required(parameters, "reset_base_pose", legacy=legacy),
        navigation_frame,
    )
    reset_joint_tolerance = _positive_number(
        _required(parameters, "reset_joint_tolerance", legacy=legacy),
        "reset_joint_tolerance",
    )
    reset_joint_timeout_sec = _positive_number(
        _required(parameters, "reset_joint_timeout_sec", legacy=legacy),
        "reset_joint_timeout_sec",
    )
    reset_joint_duration_sec = _positive_number(
        _required(parameters, "reset_joint_duration_sec", legacy=legacy),
        "reset_joint_duration_sec",
    )
    if reset_joint_duration_sec > reset_joint_timeout_sec:
        raise RobotAdapterError(
            "reset_joint_duration_sec must not exceed "
            "reset_joint_timeout_sec."
        )
    has_gripper = any(joint.group == "gripper" for joint in manual_joints)
    gripper_topic = parameters.get("gripper_controller_topic")
    gripper_status_topic = parameters.get("gripper_controller_status_topic")
    if legacy:
        gripper_topic = gripper_topic or _LEGACY_DEFAULTS[
            "gripper_controller_topic"
        ]
        gripper_status_topic = gripper_status_topic or _LEGACY_DEFAULTS[
            "gripper_controller_status_topic"
        ]
    if has_gripper and gripper_topic is None:
        raise RobotAdapterError(
            "Missing required robot adapter field: gripper_controller_topic."
        )
    if has_gripper and gripper_status_topic is None:
        raise RobotAdapterError(
            "Missing required robot adapter field: "
            "gripper_controller_status_topic."
        )
    if gripper_topic is not None:
        gripper_topic = _absolute_ros_name(
            gripper_topic, "gripper_controller_topic"
        )
    if gripper_status_topic is not None:
        gripper_status_topic = _absolute_ros_name(
            gripper_status_topic,
            "gripper_controller_status_topic",
        )
    pose_parent_frame_raw = parameters.get("pose_parent_frame")
    pose_parent_frame = (
        _relative_name(pose_parent_frame_raw, "pose_parent_frame")
        if pose_parent_frame_raw
        else planning_frame
    )
    return RobotAdapterConfig(
        planning_frame=planning_frame,
        pose_parent_frame=pose_parent_frame,
        manual_linear_axis=manual_linear_axis_value,
        navigation_velocity_yaw_offset=_finite_number(
            _required(
                parameters,
                "navigation_velocity_yaw_offset",
                legacy=legacy,
            ),
            "navigation_velocity_yaw_offset",
        ),
        navigation_frame=navigation_frame,
        navigation_base_frame=_relative_name(
            _required(parameters, "navigation_base_frame", legacy=legacy),
            "navigation_base_frame",
        ),
        navigation_action=_absolute_ros_name(
            _required(parameters, "navigation_action", legacy=legacy),
            "navigation_action",
        ),
        navigation_mode_service=_absolute_ros_name(
            _required(parameters, "navigation_mode_service", legacy=legacy),
            "navigation_mode_service",
        ),
        navigation_mode_topic=_absolute_ros_name(
            _required(parameters, "navigation_mode_topic", legacy=legacy),
            "navigation_mode_topic",
        ),
        map_topic=_absolute_ros_name(
            _required(parameters, "map_topic", legacy=legacy),
            "map_topic",
        ),
        localization_pose_topic=_absolute_ros_name(
            _required(parameters, "localization_pose_topic", legacy=legacy),
            "localization_pose_topic",
        ),
        manual_cmd_vel_topic=_absolute_ros_name(
            _required(parameters, "manual_cmd_vel_topic", legacy=legacy),
            "manual_cmd_vel_topic",
        ),
        navigation_cmd_vel_topic=_absolute_ros_name(
            _required(parameters, "navigation_cmd_vel_topic", legacy=legacy),
            "navigation_cmd_vel_topic",
        ),
        base_output_topic=_absolute_ros_name(
            _required(parameters, "base_output_topic", legacy=legacy),
            "base_output_topic",
        ),
        joint_trajectory_topic=_absolute_ros_name(
            _required(parameters, "joint_trajectory_topic", legacy=legacy),
            "joint_trajectory_topic",
        ),
        joint_state_topic=_absolute_ros_name(
            _required(parameters, "joint_state_topic", legacy=legacy),
            "joint_state_topic",
        ),
        reset_base_pose=reset_base_pose,
        reset_joint_tolerance=reset_joint_tolerance,
        reset_joint_timeout_sec=reset_joint_timeout_sec,
        reset_joint_duration_sec=reset_joint_duration_sec,
        arm_controller_topic=_absolute_ros_name(
            _required(parameters, "arm_controller_topic", legacy=legacy),
            "arm_controller_topic",
        ),
        arm_controller_status_topic=_absolute_ros_name(
            _required(
                parameters,
                "arm_controller_status_topic",
                legacy=legacy,
            ),
            "arm_controller_status_topic",
        ),
        gripper_controller_topic=gripper_topic,
        gripper_controller_status_topic=gripper_status_topic,
        manual_joints=manual_joints,
        control_navigation_stations=_control_navigation_stations(
            document,
            path,
        ),
    )
