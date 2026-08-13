"""Start the XCZS simulation with a configurable cabinet inventory."""

from functools import partial
import math
from pathlib import Path
import re
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
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


CONTROL_PACKAGE = "xczs_inspection_robot_control"
DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"
MOVEIT_CONFIG_PACKAGE = "xczs_inspection_robot_moveit_config"
NAV2_CONFIG_PACKAGE = "xczs_inspection_robot_nav2"
ROBOT_NAME = "xczs_inspection_robot"
XACRO_FILENAME = "xczs_inspection_robot.urdf.xacro"
CABINET_XACRO_FILENAME = "control_cabinet.urdf.xacro"
_CABINET_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_GENERATED_DIRECTORIES = []
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
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in {"true", "false"}:
        raise RuntimeError(f"{name} must be true or false.")
    return value == "true"


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
        instances.append(
            {
                "name": name,
                "x": _finite_number(raw, "x"),
                "y": _finite_number(raw, "y"),
                "z": _finite_number(raw, "z"),
                "roll": _finite_number(raw, "roll"),
                "pitch": _finite_number(raw, "pitch", 0.0),
                "yaw": _finite_number(raw, "yaw"),
            }
        )
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
    spawn_cabinet = _launch_boolean(context, "spawn_cabinet")
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
        moveit_client_config = (
            MoveItConfigsBuilder(
                robot_name,
                package_name=moveit_config_package,
            )
            .robot_description(file_path=robot_xacro)
            .robot_description_semantic(file_path=moveit_srdf)
            .robot_description_kinematics(file_path=moveit_kinematics)
            .joint_limits(file_path=moveit_joint_limits)
            .planning_pipelines(
                default_planning_pipeline="ompl",
                pipelines=["ompl"],
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
        generated_directory = Path(
            tempfile.mkdtemp(prefix="xczs_cabinets_")
        )
        _GENERATED_DIRECTORIES.append(generated_directory)
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
        spawn_node = None
        if spawn_cabinet:
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
                            "parent_frame": adapter_interfaces.get(
                                "pose_parent_frame",
                                adapter_interfaces["planning_frame"],
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
                            controls_config,
                            scene_config,
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                ),
                                "frame_id": adapter_interfaces[
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
                                adapter_interfaces["joint_state_topic"],
                            )
                        ],
                        parameters=[
                            controls_config,
                            adapter_config,
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


def _cleanup_generated_files(_context):
    while _GENERATED_DIRECTORIES:
        shutil.rmtree(_GENERATED_DIRECTORIES.pop(), ignore_errors=True)
    return []


def generate_launch_description() -> LaunchDescription:
    """Create the unified Gazebo, MoveIt, Nav2 and cabinet launch."""
    gazebo_ros_share = Path(get_package_share_directory("gazebo_ros"))
    control_share = Path(get_package_share_directory(CONTROL_PACKAGE))
    description_share = Path(get_package_share_directory(DESCRIPTION_PACKAGE))
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
            default_value=str(nav2_share / "maps" / "inspection_map.yaml"),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=str(nav2_share / "config" / "nav2_params.yaml"),
        ),
        DeclareLaunchArgument(
            "world",
            default_value=str(
                description_share / "worlds" / "inspection_robot.world"
            ),
        ),
        DeclareLaunchArgument("robot_name", default_value=ROBOT_NAME),
        DeclareLaunchArgument(
            "robot_xacro",
            default_value=str(xacro_file),
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value=MOVEIT_CONFIG_PACKAGE,
        ),
        DeclareLaunchArgument(
            "moveit_srdf",
            default_value="config/xczs_inspection_robot.srdf",
        ),
        DeclareLaunchArgument(
            "moveit_kinematics",
            default_value="config/kinematics.yaml",
        ),
        DeclareLaunchArgument(
            "moveit_joint_limits",
            default_value="config/joint_limits.yaml",
        ),
        DeclareLaunchArgument(
            "moveit_controllers",
            default_value="config/moveit_controllers.yaml",
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
        DeclareLaunchArgument("cabinet_bringup", default_value="true"),
        DeclareLaunchArgument("spawn_cabinet", default_value="true"),
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
        condition=IfCondition(LaunchConfiguration("moveit")),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("nav2_launch")
        ),
        launch_arguments={
            "map": LaunchConfiguration("nav2_map"),
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
            "rviz": LaunchConfiguration("nav2_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("nav2")),
    )
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
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_xczs_inspection_robot",
        output="screen",
        prefix="/usr/bin/python3",
        arguments=[
            "-entity", LaunchConfiguration("robot_name"),
            "-topic", "robot_description",
            "-z", LaunchConfiguration("spawn_z"),
        ],
        condition=IfCondition(LaunchConfiguration("robot_bringup")),
    )
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        name="xczs_controller_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster", "arm_controller", "gripper_controller",
            "--controller-manager", "/xczs/controller_manager",
            "--controller-manager-timeout", "30",
            "--switch-timeout", "30",
            "--activate-as-group",
        ],
        condition=IfCondition(LaunchConfiguration("robot_bringup")),
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
    after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=controllers,
            on_exit=partial(
                _continue_or_shutdown_required_process,
                process_label="ros2_control controller spawner",
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
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=partial(
                _continue_or_shutdown_required_process,
                process_label="Gazebo robot spawn",
                success_actions=[controllers],
            ),
        )
    )
    cabinet_loader = OpaqueFunction(
        function=_cabinet_nodes,
        kwargs={
            "cabinet_xacro": LaunchConfiguration("cabinet_xacro"),
        },
    )
    adapter_configuration = OpaqueFunction(
        function=_configure_robot_adapter,
    )
    cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_generated_files)])
    )

    return LaunchDescription(
        arguments
        + [
            adapter_configuration,
            # Register the required-process handlers before either short-lived
            # process can start.  In particular, a spawn failure may exit
            # immediately and must not race handler registration while cabinet
            # Xacros are being generated.
            start_controllers,
            after_controllers,
            *runtime_watchdogs,
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_robot,
            operation_lease_coordinator,
            cabinet_loader,
            cleanup,
        ]
    )
