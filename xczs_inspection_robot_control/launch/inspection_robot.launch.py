"""Start the complete XCZS inspection robot Gazebo simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


CONTROL_PACKAGE = "xczs_inspection_robot_control"
DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"
MOVEIT_CONFIG_PACKAGE = "xczs_inspection_robot_moveit_config"
NAV2_CONFIG_PACKAGE = "xczs_inspection_robot_nav2"
ROBOT_NAME = "xczs_inspection_robot"
XACRO_FILENAME = "xczs_inspection_robot.urdf.xacro"
CABINET_NAME = "control_cabinet"
CABINET_URDF_FILENAME = "generated/control_cabinet.urdf"
CABINET_EXPORT_ROLL = "1.57079632679"


def generate_launch_description() -> LaunchDescription:
    """Create a unified Gazebo and robot-control launch description."""
    gazebo_ros_share = Path(get_package_share_directory("gazebo_ros"))
    control_share = Path(get_package_share_directory(CONTROL_PACKAGE))
    description_share = Path(
        get_package_share_directory(DESCRIPTION_PACKAGE)
    )
    moveit_share = Path(
        get_package_share_directory(MOVEIT_CONFIG_PACKAGE)
    )
    nav2_share = Path(
        get_package_share_directory(NAV2_CONFIG_PACKAGE)
    )
    control_config = control_share / "config" / "robot_control.yaml"
    cabinet_controls_config = (
        control_share / "config" / "cabinet_controls.yaml"
    )
    cabinet_robot_adapter_config = (
        control_share / "config" / "cabinet_robot_adapter.yaml"
    )
    cabinet_scene_config = (
        control_share / "config" / "cabinet_scene.yaml"
    )
    cabinet_pose_config = control_share / "config" / "cabinet_pose.yaml"
    xacro_file = description_share / "urdf" / XACRO_FILENAME
    cabinet_urdf = (
        description_share / "urdf" / CABINET_URDF_FILENAME
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(xacro_file),
            ]
        ),
        value_type=str,
    )
    # MoveGroupInterface otherwise falls back to the global transient
    # `robot_description` topic.  A workstation may have another description
    # publisher (for example a depth camera), so every cabinet action client
    # must receive the XCZS model explicitly and deterministically.
    moveit_client_config = (
        MoveItConfigsBuilder(
            ROBOT_NAME,
            package_name=MOVEIT_CONFIG_PACKAGE,
        )
        .robot_description(file_path=str(xacro_file))
        .robot_description_semantic(
            file_path="config/xczs_inspection_robot.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # Restrict the client-only configuration to the pipeline that exists
        # in this package.  Humble's builder otherwise auto-discovers Pilz and
        # tries to load a non-existent pilz_cartesian_limits.yaml file while
        # the launch description is being generated.
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .to_moveit_configs()
    )

    gui_argument = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start the Gazebo graphical client.",
    )
    paused_argument = DeclareLaunchArgument(
        "paused",
        default_value="false",
        description="Start Gazebo with physics paused.",
    )
    teleop_argument = DeclareLaunchArgument(
        "teleop",
        default_value="false",
        description="Open the keyboard controller in a separate terminal.",
    )
    control_gui_argument = DeclareLaunchArgument(
        "control_gui",
        default_value="true",
        description="Start the graphical robot controller.",
    )
    moveit_argument = DeclareLaunchArgument(
        "moveit",
        default_value="true",
        description="Start MoveIt 2 planning and trajectory execution.",
    )
    moveit_rviz_argument = DeclareLaunchArgument(
        "moveit_rviz",
        default_value="false",
        description="Start RViz with the MoveIt Motion Planning plugin.",
    )
    nav2_argument = DeclareLaunchArgument(
        "nav2",
        default_value="false",
        description="Start Nav2 localization and autonomous navigation.",
    )
    nav2_rviz_argument = DeclareLaunchArgument(
        "nav2_rviz",
        default_value="false",
        description="Start RViz with the Nav2 navigation panel.",
    )
    nav2_map_argument = DeclareLaunchArgument(
        "nav2_map",
        default_value=str(
            nav2_share / "maps" / "inspection_map.yaml"
        ),
        description="Occupancy map YAML used by Nav2 and AMCL.",
    )
    spawn_z_argument = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.515",
        description="Initial robot height above the Gazebo world origin.",
    )
    spawn_cabinet_argument = DeclareLaunchArgument(
        "spawn_cabinet",
        default_value="true",
        description="Spawn the control cabinet model in Gazebo.",
    )
    cabinet_x_argument = DeclareLaunchArgument(
        "cabinet_x",
        default_value="2.0",
        description="Control cabinet X position in the Gazebo world.",
    )
    cabinet_y_argument = DeclareLaunchArgument(
        "cabinet_y",
        default_value="0.33",
        description="Control cabinet Y position in the Gazebo world.",
    )
    cabinet_z_argument = DeclareLaunchArgument(
        "cabinet_z",
        default_value="0.0",
        description="Control cabinet Z position in the Gazebo world.",
    )
    cabinet_yaw_argument = DeclareLaunchArgument(
        "cabinet_yaw",
        default_value="-1.57079632679",
        description="Control cabinet yaw angle in radians.",
    )
    cabinet_pose_source_argument = DeclareLaunchArgument(
        "cabinet_pose_source",
        default_value="static",
        description=(
            "Cabinet pose adapter: static or topic. Topic mode never falls "
            "back to the spawn coordinates."
        ),
    )
    cabinet_pose_topic_argument = DeclareLaunchArgument(
        "cabinet_pose_topic",
        default_value="/xczs/cabinet/pose_measurement",
        description="PoseWithCovarianceStamped input used in topic mode.",
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_ros_share / "launch" / "gzserver.launch.py")
        ),
        launch_arguments={
            "pause": LaunchConfiguration("paused"),
            "world": str(
                description_share / "worlds" / "inspection_robot.world"
            ),
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
        launch_arguments={
            "rviz": LaunchConfiguration("moveit_rviz"),
        }.items(),
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
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        remappings=[("joint_states", "/xczs/joint_states")],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_xczs_inspection_robot",
        output="screen",
        # Avoid a Conda Python shadowing the ROS 2 system Python.
        prefix="/usr/bin/python3",
        arguments=[
            "-entity",
            ROBOT_NAME,
            "-topic",
            "robot_description",
            "-z",
            LaunchConfiguration("spawn_z"),
        ],
    )
    spawn_cabinet = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_control_cabinet",
        output="screen",
        # Avoid a Conda Python shadowing the ROS 2 system Python.
        prefix="/usr/bin/python3",
        condition=IfCondition(LaunchConfiguration("spawn_cabinet")),
        arguments=[
            "-entity",
            CABINET_NAME,
            "-file",
            str(cabinet_urdf),
            "-x",
            LaunchConfiguration("cabinet_x"),
            "-y",
            LaunchConfiguration("cabinet_y"),
            "-z",
            LaunchConfiguration("cabinet_z"),
            "-R",
            CABINET_EXPORT_ROLL,
            "-Y",
            LaunchConfiguration("cabinet_yaw"),
        ],
    )
    cabinet_pose_authority = Node(
        package=CONTROL_PACKAGE,
        executable="cabinet_pose_authority",
        name="xczs_cabinet_pose_authority",
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_cabinet")),
        parameters=[
            str(cabinet_pose_config),
            {
                "use_sim_time": True,
                "pose_source": LaunchConfiguration("cabinet_pose_source"),
                "measurement_topic": LaunchConfiguration(
                    "cabinet_pose_topic"
                ),
                "static_pose.x": ParameterValue(
                    LaunchConfiguration("cabinet_x"), value_type=float
                ),
                "static_pose.y": ParameterValue(
                    LaunchConfiguration("cabinet_y"), value_type=float
                ),
                "static_pose.z": ParameterValue(
                    LaunchConfiguration("cabinet_z"), value_type=float
                ),
                "static_pose.roll": float(CABINET_EXPORT_ROLL),
                "static_pose.pitch": 0.0,
                "static_pose.yaw": ParameterValue(
                    LaunchConfiguration("cabinet_yaw"), value_type=float
                ),
            },
        ],
    )

    keyboard_teleop = Node(
        package=CONTROL_PACKAGE,
        executable="keyboard_teleop",
        name="xczs_keyboard_teleop",
        output="screen",
        prefix="xfce4-terminal --disable-server --execute",
        parameters=[str(control_config)],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )
    gui_controller = Node(
        package=CONTROL_PACKAGE,
        executable="inspection_robot_gui",
        name="xczs_inspection_robot_gui",
        output="screen",
        parameters=[str(control_config)],
        condition=IfCondition(LaunchConfiguration("control_gui")),
    )
    ros2_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="xczs_controller_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "gripper_controller",
            "--controller-manager",
            "/xczs/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--switch-timeout",
            "30",
            "--activate-as-group",
        ],
    )
    legacy_trajectory_router = Node(
        package=CONTROL_PACKAGE,
        executable="legacy_trajectory_router",
        name="xczs_legacy_trajectory_router",
        output="screen",
        parameters=[
            str(control_config),
            {"use_sim_time": True},
        ],
    )
    base_command_router = Node(
        package=CONTROL_PACKAGE,
        executable="base_command_router",
        name="xczs_base_command_router",
        output="screen",
        parameters=[
            str(control_config),
            {
                "use_sim_time": True,
                "navigation_enabled": ParameterValue(
                    LaunchConfiguration("nav2"),
                    value_type=bool,
                ),
            },
        ],
    )
    cabinet_planning_scene = Node(
        package=CONTROL_PACKAGE,
        executable="cabinet_planning_scene",
        name="xczs_cabinet_planning_scene",
        output="screen",
        parameters=[
            str(cabinet_controls_config),
            str(cabinet_scene_config),
            {
                "use_sim_time": True,
                "frame_id": "odom",
            }
        ],
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
    )
    cabinet_button_operator = Node(
        package=CONTROL_PACKAGE,
        executable="cabinet_button_operator",
        name="xczs_cabinet_button_operator",
        output="screen",
        remappings=[("joint_states", "/xczs/joint_states")],
        parameters=[
            str(cabinet_controls_config),
            str(cabinet_robot_adapter_config),
            moveit_client_config.robot_description,
            moveit_client_config.robot_description_semantic,
            moveit_client_config.robot_description_kinematics,
            {
                "use_sim_time": True,
                "planning_frame": "odom",
                "navigation_frame": "map",
            }
        ],
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
    )
    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[ros2_control_spawner],
        )
    )
    start_control_interfaces_after_ros2_control = RegisterEventHandler(
        OnProcessExit(
            target_action=ros2_control_spawner,
            on_exit=[
                base_command_router,
                legacy_trajectory_router,
                keyboard_teleop,
                gui_controller,
                move_group,
                cabinet_planning_scene,
                cabinet_button_operator,
                nav2,
            ],
        )
    )

    return LaunchDescription(
        [
            gui_argument,
            paused_argument,
            teleop_argument,
            control_gui_argument,
            moveit_argument,
            moveit_rviz_argument,
            nav2_argument,
            nav2_rviz_argument,
            nav2_map_argument,
            spawn_z_argument,
            spawn_cabinet_argument,
            cabinet_x_argument,
            cabinet_y_argument,
            cabinet_z_argument,
            cabinet_yaw_argument,
            cabinet_pose_source_argument,
            cabinet_pose_topic_argument,
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_robot,
            spawn_cabinet,
            cabinet_pose_authority,
            start_controllers_after_spawn,
            start_control_interfaces_after_ros2_control,
        ]
    )
