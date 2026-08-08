"""Start the XCZS simulation with a configurable cabinet inventory."""

import math
from pathlib import Path
import re
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnShutdown
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
        defaults = parameters["button_defaults"]
        controls = parameters["controls"]
        control_ids = parameters["control_ids"]
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            "Cabinet controls must define /**.ros__parameters.button_defaults."
        ) from error
    if not isinstance(defaults, dict):
        raise RuntimeError("button_defaults must be a mapping.")
    if not isinstance(controls, dict) or not isinstance(control_ids, list):
        raise RuntimeError("Cabinet controls and control_ids must be defined.")

    default_profile = _validated_button_profile(
        defaults,
        "button_defaults",
    )
    profiles = {}
    for control_id in control_ids:
        spec = controls.get(control_id)
        if not isinstance(control_id, str) or not isinstance(spec, dict):
            raise RuntimeError("Every control_id must reference a mapping.")
        if spec.get("type") != "button":
            continue
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
    if not profiles:
        raise RuntimeError("Cabinet controls must contain at least one button.")
    return default_profile, profiles


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


def _cabinet_nodes(context, *, cabinet_xacro, moveit_client_config):
    inventory_path = Path(
        LaunchConfiguration("cabinet_instances").perform(context)
    ).expanduser()
    instances = _read_instances(inventory_path)
    generated_directory = Path(tempfile.mkdtemp(prefix="xczs_cabinets_"))
    _GENERATED_DIRECTORIES.append(generated_directory)

    controls_config = LaunchConfiguration("cabinet_controls").perform(context)
    button_defaults, button_profiles = _read_button_profiles(
        Path(controls_config).expanduser()
    )
    scene_config = LaunchConfiguration("cabinet_scene").perform(context)
    pose_config = LaunchConfiguration("cabinet_pose").perform(context)
    adapter_config = LaunchConfiguration("cabinet_robot_adapter").perform(
        context
    )
    nodes = []
    grasp_topics = []
    moveit_and_cabinet = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("moveit"),
                "' == 'true' and '",
                LaunchConfiguration("spawn_cabinet"),
                "' == 'true'",
            ]
        )
    )

    for instance in instances:
        name = instance["name"]
        namespace = f"/xczs/cabinet/{name}"
        cabinet_frame = f"{name}_frame"
        urdf_path = generated_directory / f"{name}.urdf"
        try:
            document = xacro.process_file(
                str(cabinet_xacro),
                mappings={
                    "cabinet_name": name,
                    "button_max_travel": str(
                        button_defaults["max_position"]
                    ),
                    "button_spring_stiffness": str(
                        button_defaults["spring_stiffness"]
                    ),
                    "button_press_threshold": str(
                        button_defaults["press_threshold"]
                    ),
                },
            )
            _apply_button_profiles(document, button_profiles)
            urdf_path.write_text(document.toxml(), encoding="utf-8")
        except Exception as error:
            raise RuntimeError(
                f"Could not generate URDF for cabinet '{name}': {error}"
            ) from error

        nodes.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name=f"spawn_{name}",
                output="screen",
                prefix="/usr/bin/python3",
                condition=IfCondition(LaunchConfiguration("spawn_cabinet")),
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
        )
        nodes.append(
            Node(
                package=CONTROL_PACKAGE,
                executable="cabinet_pose_authority",
                namespace=namespace,
                name="xczs_cabinet_pose_authority",
                output="screen",
                condition=IfCondition(LaunchConfiguration("spawn_cabinet")),
                parameters=[
                    pose_config,
                    {
                        "use_sim_time": True,
                        "pose_source": LaunchConfiguration(
                            "cabinet_pose_source"
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
            )
        )
        nodes.append(
            Node(
                package=CONTROL_PACKAGE,
                executable="cabinet_planning_scene",
                namespace=namespace,
                name="xczs_cabinet_planning_scene",
                output="screen",
                condition=moveit_and_cabinet,
                parameters=[
                    controls_config,
                    scene_config,
                    {
                        "use_sim_time": True,
                        "frame_id": "odom",
                        "cabinet_frame": cabinet_frame,
                        "collision_object_prefix": name,
                    },
                ],
            )
        )
        nodes.append(
            Node(
                package=CONTROL_PACKAGE,
                executable="cabinet_button_operator",
                namespace=namespace,
                name="xczs_cabinet_button_operator",
                output="screen",
                condition=moveit_and_cabinet,
                remappings=[("joint_states", "/xczs/joint_states")],
                parameters=[
                    controls_config,
                    adapter_config,
                    moveit_client_config.robot_description,
                    moveit_client_config.robot_description_semantic,
                    moveit_client_config.robot_description_kinematics,
                    {
                        "use_sim_time": True,
                        "planning_frame": "odom",
                        "navigation_frame": "map",
                        "cabinet_frame": cabinet_frame,
                    },
                ],
            )
        )
        grasp_topics.append(f"{namespace}/grasp_active")

    nodes.append(
        Node(
            package=CONTROL_PACKAGE,
            executable="cabinet_grasp_aggregator",
            name="xczs_cabinet_grasp_aggregator",
            output="screen",
            condition=IfCondition(LaunchConfiguration("spawn_cabinet")),
            parameters=[
                {
                    "use_sim_time": True,
                    "input_topics": grasp_topics,
                    "output_topic": "/xczs/cabinet/grasp_active",
                }
            ],
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
        Command([FindExecutable(name="xacro"), " ", str(xacro_file)]),
        value_type=str,
    )
    moveit_client_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_CONFIG_PACKAGE)
        .robot_description(file_path=str(xacro_file))
        .robot_description_semantic(
            file_path="config/xczs_inspection_robot.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl", pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    arguments = [
        DeclareLaunchArgument("gui", default_value="true"),
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
        DeclareLaunchArgument("spawn_z", default_value="0.515"),
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
            "world": str(description_share / "worlds" / "inspection_robot.world"),
        }.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_ros_share / "launch" / "gzclient.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_share / "launch" / "move_group.launch.py")
        ),
        launch_arguments={"rviz": LaunchConfiguration("moveit_rviz")}.items(),
        condition=IfCondition(LaunchConfiguration("moveit")),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / "launch" / "navigation.launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("nav2_map"),
            "rviz": LaunchConfiguration("nav2_rviz"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("nav2")),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        remappings=[("joint_states", "/xczs/joint_states")],
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_xczs_inspection_robot",
        output="screen",
        prefix="/usr/bin/python3",
        arguments=[
            "-entity", ROBOT_NAME,
            "-topic", "robot_description",
            "-z", LaunchConfiguration("spawn_z"),
        ],
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
    )
    base_router = Node(
        package=CONTROL_PACKAGE,
        executable="base_command_router",
        name="xczs_base_command_router",
        output="screen",
        parameters=[
            str(control_config),
            {
                "use_sim_time": True,
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
        parameters=[str(control_config), {"use_sim_time": True}],
    )
    keyboard = Node(
        package=CONTROL_PACKAGE,
        executable="keyboard_teleop",
        name="xczs_keyboard_teleop",
        output="screen",
        prefix="xfce4-terminal --disable-server --execute",
        parameters=[str(control_config)],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )
    control_gui = Node(
        package=CONTROL_PACKAGE,
        executable="inspection_robot_gui",
        name="xczs_inspection_robot_gui",
        output="screen",
        parameters=[str(control_config)],
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
                    LaunchConfiguration("spawn_cabinet"),
                    "' == 'true'",
                ]
            )
        ),
        parameters=[LaunchConfiguration("cabinet_robot_adapter")],
    )
    after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=controllers,
            on_exit=[
                base_router,
                trajectory_router,
                keyboard,
                control_gui,
                move_group,
                nav2,
            ],
        )
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[controllers])
    )
    cabinet_loader = OpaqueFunction(
        function=_cabinet_nodes,
        kwargs={
            "cabinet_xacro": cabinet_xacro,
            "moveit_client_config": moveit_client_config,
        },
    )
    cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_generated_files)])
    )

    return LaunchDescription(
        arguments
        + [
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_robot,
            operation_lease_coordinator,
            cabinet_loader,
            start_controllers,
            after_controllers,
            cleanup,
        ]
    )
