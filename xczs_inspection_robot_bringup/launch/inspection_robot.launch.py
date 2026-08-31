"""Start the XCZS simulation with a configurable cabinet inventory."""

import atexit
from functools import partial
import math
import os
from pathlib import Path
import re
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import yaml


BRINGUP_PACKAGE = "xczs_inspection_robot_bringup"
CONTROL_PACKAGE = "xczs_inspection_robot_control"
DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"
GAZEBO_PACKAGE = "xczs_inspection_robot_gazebo"
MOVEIT_CONFIG_PACKAGE = "xczs_inspection_robot_moveit_config"
NAV2_CONFIG_PACKAGE = "xczs_inspection_robot_nav2"
ROBOT_NAME = "xczs_inspection_robot"
XACRO_FILENAME = "xczs_inspection_robot.urdf.xacro"
CABINET_XACRO_FILENAME = "control_cabinet.urdf.xacro"
SCENE_FLOOR_ENTITY = "xczs_scene_floor"

# 末端工具套装 A / B(见 xczs_inspection_robot.urdf.xacro)。非 A/B 一律回退 A。
VALID_TOOLSETS = ("A", "B")
DEFAULT_TOOLSET = "A"
DEFAULT_GAZEBO_PLUGIN_INSTANCE_ID = "initial"
_GAZEBO_PLUGIN_INSTANCE_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,63}$")


def _normalize_toolset(value: str) -> str:
    """Normalize a toolset value to ``A`` or ``B`` (case-insensitive)."""
    value = (value or "").strip().upper()
    return value if value in VALID_TOOLSETS else DEFAULT_TOOLSET


def _toolset_substitution() -> PythonExpression:
    """Runtime substitution resolving the normalized toolset (A/B, uppercase)."""
    return PythonExpression(
        [
            "(str('",
            LaunchConfiguration("toolset"),
            "').strip().upper() if str('",
            LaunchConfiguration("toolset"),
            "').strip().upper() in ('A','B') else 'A')",
        ]
    )


def _gazebo_plugin_instance_id(context) -> str:
    """Read the validated per-child Gazebo plugin node suffix.

    The suffix is consumed only by the names of Gazebo ROS plugins in the
    robot's xacro.  Their namespaces and topic remappings remain unchanged,
    while a new robot child cannot collide with a plugin whose deletion is
    still completing inside Gazebo Classic.
    """
    raw_value = LaunchConfiguration("gazebo_plugin_instance_id").perform(
        context
    ).strip()
    if _GAZEBO_PLUGIN_INSTANCE_ID_PATTERN.fullmatch(raw_value) is None:
        raise RuntimeError(
            "gazebo_plugin_instance_id must match [a-z][a-z0-9_]{0,63}, "
            f"got {raw_value!r}."
        )
    return raw_value


_CABINET_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_GENERATED_DIRECTORIES = []
_BOOLEAN_LAUNCH_ARGUMENTS = (
    "gui",
    "gazebo",
    "robot_bringup",
    "use_sim_time",
    "paused",
    "teleop",
    "control_gui",
    "moveit",
    "moveit_rviz",
    "nav2",
    "nav2_rviz",
    "cabinet_bringup",
    "spawn_cabinet",
)
_INCLUDED_REQUIRED_RUNTIME_NODES = {
    ("nav2_map_server", "map_server"): "Nav2 map server",
    ("nav2_amcl", "amcl"): "Nav2 AMCL",
    ("nav2_controller", "controller_server"): "Nav2 controller server",
    ("nav2_smoother", "smoother_server"): "Nav2 smoother server",
    ("nav2_planner", "planner_server"): "Nav2 planner server",
    ("nav2_behaviors", "behavior_server"): "Nav2 behavior server",
    ("nav2_bt_navigator", "bt_navigator"): "Nav2 BT navigator",
    (
        "nav2_waypoint_follower",
        "waypoint_follower",
    ): "Nav2 waypoint follower",
    (
        "nav2_velocity_smoother",
        "velocity_smoother",
    ): "Nav2 velocity smoother",
    (
        "nav2_lifecycle_manager",
        "lifecycle_manager",
    ): "Nav2 lifecycle manager",
}


def _continue_or_shutdown_required_process(
    event,
    _context,
    *,
    process_label,
    success_actions,
):
    """Continue a startup chain only after a required process succeeds."""
    if event.returncode == 0:
        return success_actions
    reason = (
        f"Required startup process '{process_label}' exited with code "
        f"{event.returncode}; downstream nodes will not be started."
    )
    return [
        LogInfo(msg=reason),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


def _shutdown_on_required_runtime_exit(
    event,
    context,
    *,
    process_label,
):
    """Shut down when a required long-running process exits unexpectedly."""
    if context.is_shutdown:
        return []
    reason = (
        f"Required runtime process '{process_label}' exited with code "
        f"{event.returncode}; the robot stack is no longer operational."
    )
    return [
        LogInfo(msg=reason),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


def _required_runtime_handler(process, process_label):
    """Create an exit watchdog for a required long-running process."""
    return RegisterEventHandler(
        OnProcessExit(
            target_action=process,
            on_exit=partial(
                _shutdown_on_required_runtime_exit,
                process_label=process_label,
            ),
        )
    )


def _guard_required_runtime_nodes(labeled_nodes):
    """Register every watchdog before scheduling any guarded node."""
    labeled_nodes = list(labeled_nodes)
    return [
        _required_runtime_handler(process, label)
        for process, label in labeled_nodes
    ] + [process for process, _label in labeled_nodes]


def _included_required_runtime_node(action):
    """Match known required Nodes created by the upstream Nav2 include."""
    return isinstance(action, Node) and (
        action.node_package,
        action.node_executable,
    ) in _INCLUDED_REQUIRED_RUNTIME_NODES


def _shutdown_on_included_required_runtime_exit(event, context):
    key = (event.action.node_package, event.action.node_executable)
    return _shutdown_on_required_runtime_exit(
        event,
        context,
        process_label=_INCLUDED_REQUIRED_RUNTIME_NODES[key],
    )


def _launch_boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in {"true", "false"}:
        raise RuntimeError(f"{name} must be true or false, got {value!r}.")
    return value == "true"


def _launch_finite_number(context, name):
    raw_value = LaunchConfiguration(name).perform(context).strip()
    try:
        value = float(raw_value)
    except ValueError as error:
        raise RuntimeError(
            f"{name} must be a finite number, got {raw_value!r}."
        ) from error
    if not math.isfinite(value):
        raise RuntimeError(
            f"{name} must be a finite number, got {raw_value!r}."
        )
    return value


def _optional_launch_finite_number(context, name):
    """Read an optional finite launch number, preserving an empty default."""
    raw_value = str(context.launch_configurations.get(name, "")).strip()
    if not raw_value:
        return None
    try:
        value = float(raw_value)
    except ValueError as error:
        raise RuntimeError(
            f"{name} must be a finite number when supplied, got {raw_value!r}."
        ) from error
    if not math.isfinite(value):
        raise RuntimeError(
            f"{name} must be a finite number when supplied, got {raw_value!r}."
        )
    return value


def _robot_spawn_override(context):
    """Return an all-or-nothing runtime spawn-pose override.

    The toolset supervisor captures the current Gazebo root pose before it
    replaces the mutually exclusive A/B robot entity.  Requiring all four
    values prevents a partial override from silently mixing a saved x/y with
    a scene-default z/yaw.
    """
    values = {
        field: _optional_launch_finite_number(context, f"robot_spawn_{field}")
        for field in ("x", "y", "z", "yaw")
    }
    supplied = [field for field, value in values.items() if value is not None]
    if supplied and len(supplied) != len(values):
        missing = sorted(set(values) - set(supplied))
        raise RuntimeError(
            "robot_spawn_x/y/z/yaw must be supplied together; missing "
            + ", ".join(missing)
            + "."
        )
    return values if supplied else None


def _validate_launch_arguments(context):
    actions = []
    for name in _BOOLEAN_LAUNCH_ARGUMENTS:
        value = _launch_boolean(context, name)
        actions.append(
            SetLaunchConfiguration(name, "true" if value else "false")
        )
    _launch_finite_number(context, "spawn_z")
    _robot_spawn_override(context)
    actions.append(
        SetLaunchConfiguration(
            "gazebo_plugin_instance_id", _gazebo_plugin_instance_id(context)
        )
    )
    return actions


def _finite_number(instance, field, default=None):
    value = instance.get(field, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(
            f"Cabinet '{instance.get('name', '?')}' field '{field}' "
            "must be a finite number."
        )
    value = float(value)
    if not math.isfinite(value):
        raise RuntimeError(
            f"Cabinet '{instance.get('name', '?')}' field '{field}' "
            "must be finite."
        )
    return value


def _resolve_package_path(value):
    """Resolve a ``package://`` reference to an absolute filesystem path."""
    value = str(value).strip()
    if not value.startswith("package://"):
        return str(Path(value).expanduser())
    rest = value[len("package://"):]
    package, separator, relative = rest.partition("/")
    if not package or not separator or not relative:
        raise RuntimeError(f"Invalid package:// reference: {value!r}.")
    return str(Path(get_package_share_directory(package)) / relative)


def _read_scenes(path):
    """Read ``scenes.yaml`` into a name-keyed mapping of scene specs.

    This duplicates the shared ``control_gateway.scene_catalog`` contract in
    the launch process, which cannot import the Web gateway package.
    """
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"Could not read scene catalog '{path}': {error}") from error
    if not isinstance(document, dict) or not isinstance(
        document.get("scenes"), list
    ):
        raise RuntimeError("Scene catalog must contain a scenes list.")
    if not document["scenes"]:
        raise RuntimeError("Scene catalog must contain at least one scene.")

    scenes = {}
    for raw in document["scenes"]:
        if not isinstance(raw, dict):
            raise RuntimeError("Every scene must be a mapping.")
        name = raw.get("name")
        if not isinstance(name, str) or not _CABINET_NAME_PATTERN.fullmatch(name):
            raise RuntimeError("Scene name must match [a-z][a-z0-9_]{0,62}.")
        if name in scenes:
            raise RuntimeError(f"Duplicate scene name '{name}'.")
        spawn_cabinet = raw.get("spawn_cabinet")
        if not isinstance(spawn_cabinet, bool):
            raise RuntimeError(f"Scene '{name}' spawn_cabinet must be a boolean.")
        nav2_map = raw.get("nav2_map")
        if not isinstance(nav2_map, str) or not nav2_map.strip():
            raise RuntimeError(f"Scene '{name}' nav2_map must be a non-empty string.")

        model = raw.get("model")
        model_spec = None
        if model is not None:
            if not isinstance(model, dict):
                raise RuntimeError(f"Scene '{name}' model must be a mapping or null.")
            file = model.get("file")
            if not isinstance(file, str) or not file.strip():
                raise RuntimeError(f"Scene '{name}' model.file must be a non-empty string.")
            pose = model.get("pose")
            if not isinstance(pose, dict):
                raise RuntimeError(f"Scene '{name}' model.pose must be a mapping.")
            model_spec = {
                "file": file.strip(),
                "x": _scene_pose_number(pose, "x", name),
                "y": _scene_pose_number(pose, "y", name),
                "z": _scene_pose_number(pose, "z", name),
                "roll": _scene_pose_number(pose, "roll", name),
                "pitch": _scene_pose_number(pose, "pitch", name),
                "yaw": _scene_pose_number(pose, "yaw", name),
            }
        # The robot's initial pose in the map frame.  Used both to place the
        # Gazebo entity at launch and to re-localise AMCL, so the robot never
        # starts inside an obstacle or with a mismatched map pose.
        robot_spawn = raw.get("robot_spawn")
        robot_spawn_spec = None
        if robot_spawn is not None:
            if not isinstance(robot_spawn, dict):
                raise RuntimeError(f"Scene '{name}' robot_spawn must be a mapping or null.")
            robot_spawn_spec = {
                "x": _scene_pose_number(robot_spawn, "x", name),
                "y": _scene_pose_number(robot_spawn, "y", name),
                "z": _scene_pose_number(robot_spawn, "z", name),
                "yaw": _scene_pose_number(robot_spawn, "yaw", name),
            }
        scenes[name] = {
            "name": name,
            "spawn_cabinet": spawn_cabinet,
            "model": model_spec,
            "nav2_map": nav2_map.strip(),
            "robot_spawn": robot_spawn_spec,
        }
    return scenes


def _scene_pose_number(pose, field, scene_name):
    value = pose.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(
            f"Scene '{scene_name}' model.pose.{field} must be a finite number."
        )
    value = float(value)
    if not math.isfinite(value):
        raise RuntimeError(
            f"Scene '{scene_name}' model.pose.{field} must be finite."
        )
    return value


def _scene_spec(context):
    """Return the active scene spec resolved from launch arguments."""
    scenes_path = Path(
        LaunchConfiguration("scenes_config").perform(context)
    ).expanduser()
    scene_name = LaunchConfiguration("scene").perform(context).strip()
    scenes = _read_scenes(scenes_path)
    if scene_name not in scenes:
        raise RuntimeError(f"Unknown scene '{scene_name}' in {scenes_path}.")
    return scenes[scene_name]


def _read_instances(path):
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(
            f"Could not read cabinet inventory '{path}': {error}"
        ) from error
    if not isinstance(document, dict) or not isinstance(
        document.get("instances"), list
    ):
        raise RuntimeError("Cabinet inventory must contain an instances list.")
    if not document["instances"]:
        raise RuntimeError("Cabinet inventory must contain at least one instance.")

    names = set()
    instances = []
    for raw in document["instances"]:
        if not isinstance(raw, dict):
            raise RuntimeError("Every cabinet instance must be a mapping.")
        name = raw.get("name")
        if not isinstance(name, str) or not _CABINET_NAME_PATTERN.fullmatch(name):
            raise RuntimeError(
                "Cabinet name must match [a-z][a-z0-9_]{0,62}."
            )
        if name in names:
            raise RuntimeError(f"Duplicate cabinet instance name '{name}'.")
        names.add(name)
        kind = raw.get("kind", "cabinet")
        if kind not in ("cabinet", "fixture"):
            raise RuntimeError(
                f"Instance '{name}' kind must be 'cabinet' or 'fixture'."
            )
        associated_scene = raw.get("associated_scene", "cabinet_operation")
        if not isinstance(associated_scene, str) or not associated_scene.strip():
            raise RuntimeError(
                f"Instance '{name}' associated_scene must be a non-empty string."
            )
        is_fixture = kind == "fixture"
        numeric = {}
        for field in ("x", "y", "z", "roll", "yaw"):
            if field in raw:
                numeric[field] = _finite_number(raw, field)
            elif is_fixture:
                # 夹具随场景模型以单位位姿加载，位姿字段可省略（恒等）。
                numeric[field] = 0.0
            else:
                raise RuntimeError(f"Instance '{name}' is missing field '{field}'.")
        numeric["pitch"] = (
            _finite_number(raw, "pitch")
            if "pitch" in raw
            else 0.0
        )
        instance = {
            "name": name,
            "kind": kind,
            "associated_scene": associated_scene.strip(),
            **numeric,
        }
        for field in ("controls_config", "scene_config", "adapter_config"):
            raw_value = raw.get(field)
            if raw_value is None:
                # 缺省回退共享三件套（launch 参数 cabinet_controls 等）。
                instance[field] = None
            else:
                instance[field] = _resolve_package_path(raw_value)
        instances.append(instance)
    return instances


def _validated_button_profile(source, label):
    profile = {}
    for field in (
        "max_position",
        "spring_stiffness",
        "press_threshold",
        "default_force",
    ):
        value = source.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(f"{label}.{field} must be a number.")
        value = float(value)
        if not math.isfinite(value):
            raise RuntimeError(f"{label}.{field} must be finite.")
        profile[field] = value

    max_position = profile["max_position"]
    stiffness = profile["spring_stiffness"]
    threshold = profile["press_threshold"]
    default_force = profile["default_force"]
    if max_position <= 0.0 or stiffness <= 0.0:
        raise RuntimeError(
            f"{label} max_position and spring_stiffness must be positive."
        )
    if not 0.0 < threshold <= max_position:
        raise RuntimeError(
            f"{label}.press_threshold must be in (0, max_position]."
        )
    if not 0.0 < default_force <= stiffness * max_position:
        raise RuntimeError(
            f"{label}.default_force must be in the physical force range."
        )
    return profile


def _read_button_profiles(path):
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(
            f"Could not read cabinet controls '{path}': {error}"
        ) from error

    try:
        parameters = document["/**"]["ros__parameters"]
        controls = parameters["controls"]
        control_ids = parameters["control_ids"]
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            "Cabinet controls must define control_ids and controls."
        ) from error
    if not isinstance(controls, dict) or not isinstance(control_ids, list):
        raise RuntimeError("Cabinet controls and control_ids must be defined.")

    button_ids = []
    for control_id in control_ids:
        spec = controls.get(control_id)
        if not isinstance(control_id, str) or not isinstance(spec, dict):
            raise RuntimeError("Every control_id must reference a mapping.")
        if spec.get("type") == "button":
            button_ids.append(control_id)
    if not button_ids:
        return None, {}

    defaults = parameters.get("button_defaults")
    if not isinstance(defaults, dict):
        raise RuntimeError(
            "button_defaults must be a mapping when buttons are configured."
        )
    default_profile = _validated_button_profile(defaults, "button_defaults")
    profiles = {}
    for control_id in button_ids:
        spec = controls.get(control_id)
        effective = dict(default_profile)
        for field in effective:
            if field in spec:
                effective[field] = spec[field]
        profile = _validated_button_profile(
            effective,
            f"controls.{control_id}",
        )
        joint_name = spec.get("joint_name")
        if not isinstance(joint_name, str) or not joint_name:
            raise RuntimeError(f"controls.{control_id}.joint_name is required.")
        profile["joint_name"] = joint_name
        profiles[control_id] = profile
    return default_profile, profiles


def _read_robot_adapter_interfaces(path):
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        parameters = document["/**"]["ros__parameters"]
    except (OSError, yaml.YAMLError, KeyError, TypeError) as error:
        raise RuntimeError(
            f"Could not read robot adapter interfaces '{path}': {error}"
        ) from error
    if not isinstance(parameters, dict):
        raise RuntimeError(
            "Robot adapter /**.ros__parameters must be a mapping."
        )
    required = (
        "planning_frame",
        "navigation_frame",
        "joint_state_topic",
    )
    optional = ("pose_parent_frame",)
    interfaces = {}
    for field in required:
        value = parameters.get(field)
        if not isinstance(value, str) or not value.strip():
            raise RuntimeError(
                f"Robot adapter field '{field}' must be a non-empty string."
            )
        interfaces[field] = value.strip()
    if interfaces["planning_frame"].startswith("/"):
        raise RuntimeError("planning_frame must be a relative TF frame.")
    if interfaces["navigation_frame"].startswith("/"):
        raise RuntimeError("navigation_frame must be a relative TF frame.")
    if not interfaces["joint_state_topic"].startswith("/"):
        raise RuntimeError("joint_state_topic must be an absolute ROS name.")
    for field in optional:
        value = parameters.get(field)
        if isinstance(value, str) and value.strip():
            interfaces[field] = value.strip()
    return interfaces


def _configure_robot_adapter(context):
    adapter_path = Path(
        LaunchConfiguration("cabinet_robot_adapter").perform(context)
    ).expanduser()
    interfaces = _read_robot_adapter_interfaces(adapter_path)
    robot_bringup = _launch_boolean(context, "robot_bringup")
    if not robot_bringup:
        local_only_flags = (
            "teleop",
            "control_gui",
            "moveit_rviz",
            "nav2_rviz",
        )
        enabled = [
            name
            for name in local_only_flags
            if _launch_boolean(context, name)
        ]
        if enabled:
            raise RuntimeError(
                "robot_bringup=false means an external complete robot stack "
                "provides controllers, routers, MoveIt and Nav2; local-only "
                "features must be false: "
                + ", ".join(enabled)
            )
    actions = [
        SetLaunchConfiguration(
            "adapter_joint_state_topic",
            interfaces["joint_state_topic"],
        )
    ]
    if not robot_bringup:
        actions.append(
            LogInfo(
                msg=(
                    "Using an external complete robot stack; local robot "
                    "spawn, controllers, routers, MoveIt and Nav2 are skipped."
                )
            )
        )
    return actions


def _element_text(element, tag_name):
    children = element.getElementsByTagName(tag_name)
    if len(children) != 1 or children[0].firstChild is None:
        raise RuntimeError(f"Generated cabinet is missing {tag_name}.")
    return children[0].firstChild.data.strip()


def _set_element_text(element, tag_name, value):
    children = element.getElementsByTagName(tag_name)
    if len(children) != 1 or children[0].firstChild is None:
        raise RuntimeError(f"Generated cabinet is missing {tag_name}.")
    children[0].firstChild.data = str(value)


def _apply_button_profiles(document, profiles):
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
            raise RuntimeError(
                f"Generated cabinet has no physical button '{control_id}'."
            )
        limits = joint.getElementsByTagName("limit")
        if len(limits) != 1:
            raise RuntimeError(f"Button '{control_id}' has no joint limit.")
        limits[0].setAttribute("upper", str(profile["max_position"]))
        _set_element_text(
            plugin_control,
            "spring_stiffness",
            profile["spring_stiffness"],
        )
        _set_element_text(
            plugin_control,
            "press_threshold",
            profile["press_threshold"],
        )
        _set_element_text(
            plugin_control,
            "release_threshold",
            0.5 * profile["press_threshold"],
        )


def _cabinet_nodes(context, *, cabinet_xacro):
    cabinet_bringup = _launch_boolean(context, "cabinet_bringup")
    # 场景目录决定该场景是否布柜体；`spawn_cabinet` 参数作为全局开关
    # （外部机器人栈等场景可整体关闭柜体 spawn）。
    spawn_cabinet = (
        _scene_spec(context)["spawn_cabinet"]
        and _launch_boolean(context, "spawn_cabinet")
    )
    if spawn_cabinet and not cabinet_bringup:
        raise RuntimeError(
            "spawn_cabinet=true requires cabinet_bringup=true."
        )
    if not cabinet_bringup:
        return []
    moveit_enabled = _launch_boolean(context, "moveit")
    if spawn_cabinet:
        if hasattr(cabinet_xacro, "perform"):
            cabinet_xacro = cabinet_xacro.perform(context)
        cabinet_xacro = Path(str(cabinet_xacro)).expanduser()
        if not cabinet_xacro.is_file():
            raise RuntimeError(
                f"Cabinet Xacro does not exist: {cabinet_xacro}"
            )
    moveit_client_config = None
    if moveit_enabled:
        robot_name = LaunchConfiguration("robot_name").perform(context)
        robot_xacro = LaunchConfiguration("robot_xacro").perform(context)
        toolset = _normalize_toolset(
            LaunchConfiguration("toolset").perform(context)
        )
        gazebo_plugin_instance_id = _gazebo_plugin_instance_id(context)
        moveit_config_package = LaunchConfiguration(
            "moveit_config_package"
        ).perform(context)
        moveit_srdf = LaunchConfiguration("moveit_srdf").perform(context)
        moveit_kinematics = LaunchConfiguration(
            "moveit_kinematics"
        ).perform(context)
        moveit_joint_limits = LaunchConfiguration(
            "moveit_joint_limits"
        ).perform(context)
        moveit_controllers = LaunchConfiguration(
            "moveit_controllers"
        ).perform(context)
        moveit_client_config = (
            MoveItConfigsBuilder(
                robot_name,
                package_name=moveit_config_package,
            )
            .robot_description(
                file_path=robot_xacro,
                mappings={
                    "toolset": toolset,
                    "gazebo_plugin_instance_id": gazebo_plugin_instance_id,
                },
            )
            .robot_description_semantic(file_path=moveit_srdf)
            .robot_description_kinematics(file_path=moveit_kinematics)
            .joint_limits(file_path=moveit_joint_limits)
            .planning_pipelines(
                default_planning_pipeline="ompl",
                pipelines=["ompl"],
            )
            # Cabinet operators consume only the descriptions/limits below,
            # but ``to_moveit_configs`` otherwise performs an implicit
            # controller-file guess and emits a misleading startup warning.
            .trajectory_execution(
                file_path=moveit_controllers,
                moveit_manage_controllers=False,
            )
            .to_moveit_configs()
        )
    inventory_path = Path(
        LaunchConfiguration("cabinet_instances").perform(context)
    ).expanduser()
    instances = _read_instances(inventory_path)
    controls_config = LaunchConfiguration("cabinet_controls").perform(context)
    generated_directory = None
    button_defaults = None
    button_profiles = {}
    if spawn_cabinet:
        generated_directory = _make_generated_directory("xczs_cabinets_")
        button_defaults, button_profiles = _read_button_profiles(
            Path(controls_config).expanduser()
        )
    scene_config = LaunchConfiguration("cabinet_scene").perform(context)
    pose_config = LaunchConfiguration("cabinet_pose").perform(context)
    adapter_config = LaunchConfiguration("cabinet_robot_adapter").perform(context)
    adapter_interfaces = _read_robot_adapter_interfaces(
        Path(adapter_config).expanduser()
    )
    nodes = []
    grasp_topics = []
    for instance in instances:
        name = instance["name"]
        namespace = f"/xczs/cabinet/{name}"
        cabinet_frame = f"{name}_frame"
        # 夹具实例不 spawn（场景模型承载夹具，随场景切换而生）；三件套配置
        # 逐实例覆盖，缺省回退共享 launch 参数。
        is_fixture = instance.get("kind", "cabinet") == "fixture"
        instance_controls = (
            instance["controls_config"]
            if instance.get("controls_config") else controls_config
        )
        instance_scene = (
            instance["scene_config"]
            if instance.get("scene_config") else scene_config
        )
        instance_adapter = (
            instance["adapter_config"]
            if instance.get("adapter_config") else adapter_config
        )
        # 机器人接口契约（planning_frame/pose_parent_frame/joint_state_topic）
        # 按实例适配层解析：夹具实例自包含，柜体实例回退共享适配层。
        instance_adapter_interfaces = adapter_interfaces
        if instance.get("adapter_config"):
            instance_adapter_interfaces = _read_robot_adapter_interfaces(
                Path(instance["adapter_config"]).expanduser()
            )
        spawn_node = None
        if spawn_cabinet and not is_fixture:
            assert generated_directory is not None
            urdf_path = generated_directory / f"{name}.urdf"
            try:
                xacro_mappings = {"cabinet_name": name}
                if button_defaults is not None:
                    xacro_mappings.update(
                        {
                            "button_max_travel": str(
                                button_defaults["max_position"]
                            ),
                            "button_spring_stiffness": str(
                                button_defaults["spring_stiffness"]
                            ),
                            "button_press_threshold": str(
                                button_defaults["press_threshold"]
                            ),
                        }
                    )
                document = xacro.process_file(
                    str(cabinet_xacro),
                    mappings=xacro_mappings,
                )
                _apply_button_profiles(document, button_profiles)
                urdf_path.write_text(document.toxml(), encoding="utf-8")
            except Exception as error:
                raise RuntimeError(
                    f"Could not generate URDF for cabinet '{name}': {error}"
                ) from error

            spawn_node = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name=f"spawn_{name}",
                output="screen",
                prefix="/usr/bin/python3",
                arguments=[
                    "-entity", name,
                    "-file", str(urdf_path),
                    "-x", str(instance["x"]),
                    "-y", str(instance["y"]),
                    "-z", str(instance["z"]),
                    "-R", str(instance["roll"]),
                    "-P", str(instance["pitch"]),
                    "-Y", str(instance["yaw"]),
                ],
            )
        cabinet_runtime_nodes = [
            (
                Node(
                    package=CONTROL_PACKAGE,
                    executable="cabinet_pose_authority",
                    namespace=namespace,
                    name="xczs_cabinet_pose_authority",
                    output="screen",
                    parameters=[
                        pose_config,
                        {
                            "use_sim_time": ParameterValue(
                                LaunchConfiguration("use_sim_time"),
                                value_type=bool,
                            ),
                            "pose_source": LaunchConfiguration(
                                "cabinet_pose_source"
                            ),
                            "parent_frame": instance_adapter_interfaces.get(
                                "pose_parent_frame",
                                instance_adapter_interfaces["planning_frame"],
                            ),
                            "cabinet_frame": cabinet_frame,
                            "static_pose.x": instance["x"],
                            "static_pose.y": instance["y"],
                            "static_pose.z": instance["z"],
                            "static_pose.roll": instance["roll"],
                            "static_pose.pitch": instance["pitch"],
                            "static_pose.yaw": instance["yaw"],
                        },
                    ],
                ),
                f"cabinet pose authority '{name}'",
            )
        ]
        if moveit_enabled:
            cabinet_runtime_nodes.append(
                (
                    Node(
                        package=CONTROL_PACKAGE,
                        executable="cabinet_planning_scene",
                        namespace=namespace,
                        name="xczs_cabinet_planning_scene",
                        output="screen",
                        parameters=[
                            instance_controls,
                            instance_scene,
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                ),
                                "frame_id": instance_adapter_interfaces[
                                    "planning_frame"
                                ],
                                "cabinet_frame": cabinet_frame,
                                "collision_object_prefix": name,
                            },
                        ],
                    ),
                    f"cabinet planning scene '{name}'",
                )
            )
            cabinet_runtime_nodes.append(
                (
                    Node(
                        package=CONTROL_PACKAGE,
                        executable="cabinet_button_operator",
                        namespace=namespace,
                        name="xczs_cabinet_button_operator",
                        output="screen",
                        remappings=[
                            (
                                "joint_states",
                                instance_adapter_interfaces["joint_state_topic"],
                            )
                        ],
                        parameters=[
                            instance_controls,
                            instance_adapter,
                            moveit_client_config.robot_description,
                            moveit_client_config.robot_description_semantic,
                            moveit_client_config.robot_description_kinematics,
                            moveit_client_config.joint_limits,
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                ),
                                "cabinet_frame": cabinet_frame,
                                "toolset": toolset,
                            },
                        ],
                    ),
                    f"cabinet button operator '{name}'",
                )
            )
            grasp_topics.append(f"{namespace}/grasp_active")

        guarded_cabinet_runtime = _guard_required_runtime_nodes(
            cabinet_runtime_nodes
        )
        if spawn_node is None:
            nodes.extend(guarded_cabinet_runtime)
        else:
            # A spawn process can exit immediately.  Register its handler
            # before scheduling the process so a fast success or failure
            # cannot race the cabinet runtime-node dependency chain.
            nodes.extend(
                [
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=spawn_node,
                            on_exit=partial(
                                _continue_or_shutdown_required_process,
                                process_label=f"Gazebo cabinet spawn '{name}'",
                                success_actions=guarded_cabinet_runtime,
                            ),
                        )
                    ),
                    spawn_node,
                ]
            )

    if moveit_enabled:
        grasp_aggregator = Node(
            package=CONTROL_PACKAGE,
            executable="cabinet_grasp_aggregator",
            name="xczs_cabinet_grasp_aggregator",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    ),
                    "input_topics": grasp_topics,
                    "output_topic": "/xczs/cabinet/grasp_active",
                }
            ],
        )
        nodes.extend(
            _guard_required_runtime_nodes(
                [(grasp_aggregator, "cabinet grasp aggregator")]
            )
        )
    return nodes


def _scene_floor_nodes(context):
    """Spawn the initial scene's static floor model (if any)."""
    if not _launch_boolean(context, "gazebo"):
        return []
    model = _scene_spec(context).get("model")
    if model is None:
        return []
    model_path = _resolve_package_path(model["file"])
    if not Path(model_path).is_file():
        raise RuntimeError(f"Scene floor model does not exist: {model_path}")
    # Scene models are authored as xacro (root <sdf> so Gazebo keeps the full
    # meshes); spawn_entity.py has no xacro flag, so expand the source to SDF
    # now and hand it the concrete file.  The temp file outlives this call --
    # it must exist until the spawn node reads it -- and is removed at exit.
    if model_path.endswith(".xacro"):
        handle = tempfile.NamedTemporaryFile(
            mode="w", suffix=".sdf", prefix="xczs_scene_",
            delete=False, encoding="utf-8",
        )
        handle.write(xacro.process_file(model_path).documentElement.toxml())
        handle.close()
        atexit.register(partial(os.unlink, handle.name))
        model_path = handle.name
    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_scene_floor",
        output="screen",
        prefix="/usr/bin/python3",
        arguments=[
            "-entity", SCENE_FLOOR_ENTITY,
            "-file", model_path,
            "-x", str(model["x"]),
            "-y", str(model["y"]),
            "-z", str(model["z"]),
            "-R", str(model["roll"]),
            "-P", str(model["pitch"]),
            "-Y", str(model["yaw"]),
        ],
    )
    # The world-only owner must fail closed if its immutable scene floor is
    # absent.  Otherwise Gazebo would remain alive and the supervisor would
    # bring up a perfectly healthy robot in an incomplete scene.
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_node,
                on_exit=partial(
                    _continue_or_shutdown_required_process,
                    process_label="Gazebo scene floor spawn",
                    success_actions=[],
                ),
            )
        ),
        spawn_node,
    ]


def _nav2_nodes(context):
    """Include Nav2 with the map and AMCL initial pose from the active scene."""
    robot_bringup = _launch_boolean(context, "robot_bringup")
    nav2_enabled = _launch_boolean(context, "nav2")
    if not (robot_bringup and nav2_enabled):
        return []
    # ``nav2_map`` is an optional per-run override (empty by default); the
    # active scene's ``nav2_map`` is the single source of truth otherwise.
    nav2_map_override = LaunchConfiguration("nav2_map").perform(context).strip()
    nav2_map = _resolve_package_path(
        nav2_map_override or _scene_spec(context)["nav2_map"]
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                LaunchConfiguration("nav2_launch").perform(context)
            ),
            launch_arguments={
                "map": nav2_map,
                "nav2_params_file": _nav2_params_for_scene(context),
                "rviz": LaunchConfiguration("nav2_rviz").perform(context),
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ).perform(context),
            }.items(),
        )
    ]


def _nav2_params_for_scene(context):
    """Write a scene-specific copy of ``nav2_params.yaml``.

    AMCL's ``initial_pose`` is shared across scenes in the committed params
    file (it matches only the cabinet-operation origin spawn).  Rewrite it to
    the active scene's ``robot_spawn`` so AMCL never starts with a pose that
    contradicts where Gazebo actually placed the robot.
    """
    base_path = Path(
        LaunchConfiguration("nav2_params_file").perform(context)
    ).expanduser()
    try:
        params = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"Could not read nav2 params '{base_path}': {error}") from error
    if not isinstance(params, dict):
        raise RuntimeError(f"nav2 params '{base_path}' must be a mapping.")

    spawn = _active_robot_spawn(context)
    amcl_params = params.setdefault("amcl", {}).setdefault("ros__parameters", {})
    amcl_params["initial_pose"] = {
        "x": spawn["x"],
        "y": spawn["y"],
        "z": 0.0,
        "yaw": spawn["yaw"],
    }

    generated_directory = _make_generated_directory("xczs_nav2_params_")
    out_path = generated_directory / "nav2_params.yaml"
    out_path.write_text(
        yaml.safe_dump(params, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return str(out_path)


def _active_robot_spawn(context):
    """Return the active scene's robot_spawn, falling back to the origin pose.

    ``robot_spawn`` may be null (keep current pose), which only makes sense on
    scene *switch*; at initial launch there is no prior pose, so default to the
    historical origin spawn used by the cabinet-operation scene.

    顶层 ``spawn_z``（对应 shell 的 ``SPAWN_Z``）始终覆盖场景里的 ``z``，
    否则当场景显式指定了 ``robot_spawn`` 时，用户设置的覆盖高度会被静默忽略。
    """
    override = _robot_spawn_override(context)
    if override is not None:
        return override
    spawn = _scene_spec(context).get("robot_spawn")
    override_z = _launch_finite_number(context, "spawn_z")
    if spawn is None:
        return {
            "x": 0.0,
            "y": 0.0,
            "z": override_z,
            "yaw": math.pi / 2.0,
        }
    spawn = dict(spawn)
    spawn["z"] = override_z
    return spawn


def _robot_spawn_node(context, *, controllers=None):
    """Spawn the robot at the active scene's ``robot_spawn``.

    ``spawn_entity`` places the root ``body`` link, whose -Y is the physical
    forward direction.  ``base_link`` is a fixed -pi/2 child of ``body`` (see
    ``dual_arm_body.xacro``), so to point ``base_link`` at ``robot_spawn.yaw`` in
    the map frame the body must be rotated forward by pi/2 -- the same
    convention used by ``runner._teleport_robot`` on scene switch.

    When ``controllers`` is supplied, the returned action also chains the
    controller spawner behind the robot spawn.  ``OnProcessExit`` must target
    the concrete spawn ``Node`` created here rather than the wrapping
    ``OpaqueFunction``, so the event handler is built inside this function.
    """
    spawn = _active_robot_spawn(context)
    body_yaw = spawn["yaw"] + math.pi / 2.0
    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_xczs_inspection_robot",
        output="screen",
        prefix="/usr/bin/python3",
        arguments=[
            "-entity", LaunchConfiguration("robot_name"),
            "-topic", "robot_description",
            "-x", str(spawn["x"]),
            "-y", str(spawn["y"]),
            "-z", str(spawn["z"]),
            "-Y", str(body_yaw),
        ],
        condition=IfCondition(LaunchConfiguration("robot_bringup")),
    )
    if controllers is None:
        return [spawn_node]
    # Register the watchdog before the spawn process so a fast spawn exit
    # cannot race handler registration (same convention as _cabinet_nodes).
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_node,
                on_exit=partial(
                    _continue_or_shutdown_required_process,
                    process_label="Gazebo robot spawn",
                    success_actions=[controllers],
                ),
            )
        ),
        spawn_node,
    ]


def _tool_controller_spawner_node(context, *, success_actions):
    """Build the per-toolset tool controller spawner.

    Tool controllers exist only for the active toolset (controller_manager
    configures them from ``ros2_controllers_toolset_{A,B}.yaml``), so the
    spawner must only ever name the mounted set's controllers.
    """
    toolset = _normalize_toolset(
        LaunchConfiguration("toolset").perform(context)
    )
    tools = (
        ["three_cylinder_controller", "two_cylinder_controller"]
        if toolset == "A"
        else ["rotate_button_controller", "rocker_controller"]
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="xczs_tool_controller_spawner",
        output="screen",
        arguments=[
            *tools,
            "--controller-manager", "/xczs/controller_manager",
            "--controller-manager-timeout", "30",
            "--switch-timeout", "30",
            "--activate-as-group",
        ],
        condition=IfCondition(LaunchConfiguration("robot_bringup")),
    )
    # The tool controller set is part of the physical robot contract.  Do not
    # release the startup joint hold or expose MoveIt/Web command routes until
    # this spawner has completed successfully.  Register before scheduling the
    # process so a fast failure cannot race the event handler.
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawner,
                on_exit=partial(
                    _continue_or_shutdown_required_process,
                    process_label="ros2_control tool controller spawner",
                    success_actions=success_actions,
                ),
            )
        ),
        spawner,
    ]


def _cleanup_generated_directories():
    while _GENERATED_DIRECTORIES:
        shutil.rmtree(_GENERATED_DIRECTORIES.pop(), ignore_errors=True)


def _make_generated_directory(prefix):
    """Create launch artifacts under run_all's owned runtime directory.

    ``run_all.sh`` may have to terminate ``ros2 launch`` with SIGTERM, a path
    on which Python/launch shutdown callbacks are not guaranteed to finish.
    Nesting generated data below a parent owned by the shell lets its EXIT
    trap provide the final cleanup.  Direct ``ros2 launch`` remains supported
    and continues to use the system temporary directory plus OnShutdown and
    ``atexit`` cleanup.
    """
    runtime_root = os.environ.get("XCZS_LAUNCH_RUNTIME_DIRECTORY", "").strip()
    if runtime_root:
        root = Path(runtime_root).expanduser()
        if not root.is_absolute() or not root.is_dir():
            raise RuntimeError(
                "XCZS_LAUNCH_RUNTIME_DIRECTORY must be an existing absolute "
                f"directory: {runtime_root}"
            )
        generated_directory = Path(
            tempfile.mkdtemp(prefix=prefix, dir=str(root))
        )
    else:
        generated_directory = Path(tempfile.mkdtemp(prefix=prefix))
    _GENERATED_DIRECTORIES.append(generated_directory)
    return generated_directory


def _cleanup_generated_files(_context):
    _cleanup_generated_directories()
    return []


atexit.register(_cleanup_generated_directories)


def generate_launch_description() -> LaunchDescription:
    """Create the unified Gazebo, MoveIt, Nav2 and cabinet launch."""
    gazebo_ros_share = Path(get_package_share_directory("gazebo_ros"))
    control_share = Path(get_package_share_directory(CONTROL_PACKAGE))
    description_share = Path(get_package_share_directory(DESCRIPTION_PACKAGE))
    gazebo_share = Path(get_package_share_directory(GAZEBO_PACKAGE))
    moveit_share = Path(get_package_share_directory(MOVEIT_CONFIG_PACKAGE))
    nav2_share = Path(get_package_share_directory(NAV2_CONFIG_PACKAGE))
    control_config = control_share / "config" / "robot_control.yaml"
    xacro_file = description_share / "urdf" / XACRO_FILENAME
    cabinet_xacro = description_share / "urdf" / CABINET_XACRO_FILENAME

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                LaunchConfiguration("robot_xacro"),
                " toolset:=",
                _toolset_substitution(),
                " gazebo_plugin_instance_id:=",
                LaunchConfiguration("gazebo_plugin_instance_id"),
            ]
        ),
        value_type=str,
    )

    arguments = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("gazebo", default_value="true"),
        DeclareLaunchArgument("robot_bringup", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("teleop", default_value="false"),
        DeclareLaunchArgument("control_gui", default_value="true"),
        DeclareLaunchArgument("moveit", default_value="true"),
        DeclareLaunchArgument("moveit_rviz", default_value="false"),
        DeclareLaunchArgument("nav2", default_value="false"),
        DeclareLaunchArgument("nav2_rviz", default_value="false"),
        DeclareLaunchArgument(
            "nav2_map",
            default_value="",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=str(nav2_share / "config" / "nav2_params.yaml"),
        ),
        DeclareLaunchArgument(
            "world",
            default_value=str(
                gazebo_share / "worlds" / "inspection_robot.world"
            ),
        ),
        DeclareLaunchArgument("robot_name", default_value=ROBOT_NAME),
        DeclareLaunchArgument(
            "robot_xacro",
            default_value=str(xacro_file),
        ),
        DeclareLaunchArgument(
            "toolset",
            default_value=DEFAULT_TOOLSET,
            description=(
                "末端工具套装 A/B: A = 三电缸(右)+两电缸(左), "
                "B = 旋转按钮(右)+摇入摇出(左)。默认 A。"
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_plugin_instance_id",
            default_value=DEFAULT_GAZEBO_PLUGIN_INSTANCE_ID,
            description=(
                "当前机器人 child 的 Gazebo ROS 插件私有节点后缀；"
                "由末端热切换监督器生成。"
            ),
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value=MOVEIT_CONFIG_PACKAGE,
        ),
        DeclareLaunchArgument(
            "moveit_srdf",
            default_value=[
                "config/xczs_inspection_robot_toolset_",
                _toolset_substitution(),
                ".srdf",
            ],
        ),
        DeclareLaunchArgument(
            "moveit_kinematics",
            default_value="config/kinematics.yaml",
        ),
        DeclareLaunchArgument(
            "moveit_joint_limits",
            default_value=[
                "config/joint_limits_toolset_",
                _toolset_substitution(),
                ".yaml",
            ],
        ),
        DeclareLaunchArgument(
            "moveit_controllers",
            default_value=[
                "config/moveit_controllers_toolset_",
                _toolset_substitution(),
                ".yaml",
            ],
        ),
        DeclareLaunchArgument(
            "moveit_rviz_config",
            default_value=str(moveit_share / "config" / "moveit.rviz"),
        ),
        DeclareLaunchArgument(
            "moveit_launch",
            default_value=str(moveit_share / "launch" / "move_group.launch.py"),
        ),
        DeclareLaunchArgument(
            "nav2_launch",
            default_value=str(nav2_share / "launch" / "navigation.launch.py"),
        ),
        DeclareLaunchArgument(
            "robot_control",
            default_value=str(control_config),
        ),
        DeclareLaunchArgument(
            "cabinet_xacro",
            default_value=str(cabinet_xacro),
        ),
        DeclareLaunchArgument("spawn_z", default_value="0.515"),
        DeclareLaunchArgument("robot_spawn_x", default_value=""),
        DeclareLaunchArgument("robot_spawn_y", default_value=""),
        DeclareLaunchArgument("robot_spawn_z", default_value=""),
        DeclareLaunchArgument("robot_spawn_yaw", default_value=""),
        DeclareLaunchArgument("cabinet_bringup", default_value="true"),
        DeclareLaunchArgument("spawn_cabinet", default_value="true"),
        DeclareLaunchArgument("scene", default_value="cabinet_operation"),
        DeclareLaunchArgument(
            "scenes_config",
            default_value=str(control_share / "config" / "scenes.yaml"),
        ),
        DeclareLaunchArgument("cabinet_pose_source", default_value="static"),
        DeclareLaunchArgument(
            "cabinet_instances",
            default_value=str(control_share / "config" / "cabinet_instances.yaml"),
        ),
        DeclareLaunchArgument(
            "cabinet_controls",
            default_value=str(control_share / "config" / "cabinet_controls.yaml"),
        ),
        DeclareLaunchArgument(
            "cabinet_scene",
            default_value=str(control_share / "config" / "cabinet_scene.yaml"),
        ),
        DeclareLaunchArgument(
            "cabinet_pose",
            default_value=str(control_share / "config" / "cabinet_pose.yaml"),
        ),
        DeclareLaunchArgument(
            "cabinet_robot_adapter",
            default_value=str(
                control_share / "config" / "cabinet_robot_adapter.yaml"
            ),
        ),
    ]

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_ros_share / "launch" / "gzserver.launch.py")
        ),
        launch_arguments={
            "pause": LaunchConfiguration("paused"),
            "world": LaunchConfiguration("world"),
            "server_required": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("gazebo")),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_ros_share / "launch" / "gzclient.launch.py")
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("gazebo"),
                    "' == 'true' and '",
                    LaunchConfiguration("gui"),
                    "' == 'true'",
                ]
            )
        ),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("moveit_launch")
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("moveit_rviz"),
            "robot_name": LaunchConfiguration("robot_name"),
            "toolset": LaunchConfiguration("toolset"),
            "moveit_config_package": LaunchConfiguration(
                "moveit_config_package"
            ),
            "robot_xacro": LaunchConfiguration("robot_xacro"),
            "moveit_srdf": LaunchConfiguration("moveit_srdf"),
            "moveit_kinematics": LaunchConfiguration(
                "moveit_kinematics"
            ),
            "moveit_joint_limits": LaunchConfiguration(
                "moveit_joint_limits"
            ),
            "moveit_controllers": LaunchConfiguration(
                "moveit_controllers"
            ),
            "joint_state_topic": LaunchConfiguration(
                "adapter_joint_state_topic"
            ),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz_config": LaunchConfiguration("moveit_rviz_config"),
        }.items(),
        # ``moveit`` also tells cabinet clients that the external planning
        # stack is available.  Only include our local move_group when this
        # launch owns the robot bringup; external-stack mode must not create a
        # duplicate move_group with the same services and planning scene.
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("robot_bringup"),
                    "' == 'true' and '",
                    LaunchConfiguration("moveit"),
                    "' == 'true'",
                ]
            )
        ),
    )
    # Nav2 的地图由场景目录决定；作为 OpaqueFunction 包装以便在运行时读取
    # `scene`/`scenes_config` 并解析对应地图（默认场景等价于旧 inspection_map）。
    nav2 = OpaqueFunction(function=_nav2_nodes)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
            }
        ],
        remappings=[
            ("joint_states", LaunchConfiguration("adapter_joint_state_topic"))
        ],
        condition=IfCondition(LaunchConfiguration("robot_bringup")),
    )
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        name="xczs_controller_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "left_arm_controller",
            "right_arm_controller",
            "--controller-manager", "/xczs/controller_manager",
            "--controller-manager-timeout", "30",
            "--switch-timeout", "30",
            "--activate-as-group",
        ],
        condition=IfCondition(LaunchConfiguration("robot_bringup")),
    )
    spawn_robot = OpaqueFunction(
        function=_robot_spawn_node,
        kwargs={"controllers": controllers},
    )
    base_router = Node(
        package=CONTROL_PACKAGE,
        executable="base_command_router",
        name="xczs_base_command_router",
        output="screen",
        parameters=[
            LaunchConfiguration("robot_control"),
            LaunchConfiguration("cabinet_robot_adapter"),
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
                "navigation_enabled": ParameterValue(
                    LaunchConfiguration("nav2"), value_type=bool
                ),
            },
        ],
        # In world-preserving toolset mode the first launch owns only Gazebo;
        # the supervisor starts a separate robot child with this argument set
        # to true.  Without this condition the world owner would leave a
        # duplicate pair of command routers alive and both stacks would claim
        # the same ROS interfaces during every A/B replacement.
        condition=IfCondition(LaunchConfiguration("robot_bringup")),
    )
    trajectory_router = Node(
        package=CONTROL_PACKAGE,
        executable="legacy_trajectory_router",
        name="xczs_legacy_trajectory_router",
        output="screen",
        parameters=[
            LaunchConfiguration("robot_control"),
            LaunchConfiguration("cabinet_robot_adapter"),
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration("robot_bringup")),
    )
    keyboard = Node(
        package=CONTROL_PACKAGE,
        executable="keyboard_teleop",
        name="xczs_keyboard_teleop",
        output="screen",
        prefix="xfce4-terminal --disable-server --execute",
        parameters=[LaunchConfiguration("robot_control")],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )
    control_gui = Node(
        package=CONTROL_PACKAGE,
        executable="inspection_robot_gui",
        name="xczs_inspection_robot_gui",
        output="screen",
        parameters=[LaunchConfiguration("robot_control")],
        condition=IfCondition(LaunchConfiguration("control_gui")),
    )
    operation_lease_coordinator = Node(
        package=CONTROL_PACKAGE,
        executable="operation_lease_coordinator",
        name="xczs_operation_lease_coordinator",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("robot_bringup"),
                    "' == 'true' and '",
                    LaunchConfiguration("moveit"),
                    "' == 'true' and '",
                    LaunchConfiguration("cabinet_bringup"),
                    "' == 'true'",
                ]
            )
        ),
        parameters=[LaunchConfiguration("cabinet_robot_adapter")],
    )
    runtime_watchdogs = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=_included_required_runtime_node,
                on_exit=_shutdown_on_included_required_runtime_exit,
            )
        ),
        _required_runtime_handler(
            robot_state_publisher,
            "robot state publisher",
        ),
        _required_runtime_handler(
            base_router,
            "base command router",
        ),
        _required_runtime_handler(
            trajectory_router,
            "manual trajectory router",
        ),
        _required_runtime_handler(
            operation_lease_coordinator,
            "operation lease coordinator",
        ),
    ]
    # GazeboSystem applies the fortune-cat joint initial values while loading
    # the model.  This required process only verifies the measured state and
    # controller readiness; it sends no trajectory.  MoveIt and command
    # routers are deliberately held back until the spawn state is confirmed.
    initial_pose_verifier = ExecuteProcess(
        cmd=[
            "python3",
            str(
                Path(get_package_share_directory(BRINGUP_PACKAGE))
                / "scripts" / "verify_initial_pose.py"
            ),
            "--positions-file",
            str(
                Path(get_package_share_directory(DESCRIPTION_PACKAGE))
                / "config" / "initial_positions.yaml"
            ),
            "--joint-state-topic",
            LaunchConfiguration("adapter_joint_state_topic"),
            "--toolset",
            _toolset_substitution(),
        ],
        output="screen",
    )
    initial_pose_verifier_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=initial_pose_verifier,
            on_exit=partial(
                _continue_or_shutdown_required_process,
                process_label="spawn-time fortune-cat pose verification",
                success_actions=[
                    base_router,
                    trajectory_router,
                    keyboard,
                    control_gui,
                    move_group,
                    nav2,
                ],
            ),
        )
    )
    tool_controllers = OpaqueFunction(
        function=_tool_controller_spawner_node,
        kwargs={
            "success_actions": [
                initial_pose_verifier_handler,
                initial_pose_verifier,
            ],
        },
    )
    after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=controllers,
            on_exit=partial(
                _continue_or_shutdown_required_process,
                process_label="ros2_control controller spawner",
                success_actions=[
                    tool_controllers,
                ],
            ),
        )
    )
    cabinet_loader = OpaqueFunction(
        function=_cabinet_nodes,
        kwargs={
            "cabinet_xacro": LaunchConfiguration("cabinet_xacro"),
        },
    )
    scene_floor = OpaqueFunction(function=_scene_floor_nodes)
    adapter_configuration = OpaqueFunction(
        function=_configure_robot_adapter,
    )
    argument_validation = OpaqueFunction(
        function=_validate_launch_arguments,
    )
    cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_generated_files)])
    )

    return LaunchDescription(
        arguments
        + [
            # Register cleanup before any OpaqueFunction can create a temporary
            # cabinet/Nav2 directory. atexit above is a second line of defence
            # for normal interpreter exit before launch dispatches Shutdown.
            cleanup,
            argument_validation,
            adapter_configuration,
            # Register the required-process handlers before either short-lived
            # process can start.  In particular, a spawn failure may exit
            # immediately and must not race handler registration while cabinet
            # Xacros are being generated.  The robot-spawn -> controller chain
            # is returned by _robot_spawn_node itself so its OnProcessExit can
            # reference the concrete spawn Node.
            after_controllers,
            *runtime_watchdogs,
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_robot,
            scene_floor,
            operation_lease_coordinator,
            cabinet_loader,
        ]
    )
