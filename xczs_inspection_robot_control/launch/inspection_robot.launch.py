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
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CONTROL_PACKAGE = "xczs_inspection_robot_control"
DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"
ROBOT_NAME = "xczs_inspection_robot"
XACRO_FILENAME = "xczs_inspection_robot.urdf.xacro"
CABINET_NAME = "control_cabinet"
CABINET_URDF_FILENAME = "control_cabinet.urdf"
CABINET_EXPORT_ROLL = "1.57079632679"


def generate_launch_description() -> LaunchDescription:
    """Create a unified Gazebo and robot-control launch description."""
    gazebo_ros_share = Path(get_package_share_directory("gazebo_ros"))
    control_share = Path(get_package_share_directory(CONTROL_PACKAGE))
    description_share = Path(
        get_package_share_directory(DESCRIPTION_PACKAGE)
    )
    control_config = control_share / "config" / "robot_control.yaml"
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
    task_scheduler_argument = DeclareLaunchArgument(
        "task_scheduler",
        default_value="false",
        description="Start the autonomous task scheduler (A→B→C→D pipeline).",
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
    task_scheduler_node = Node(
        package=CONTROL_PACKAGE,
        executable="task_scheduler",
        name="xczs_task_scheduler",
        output="screen",
        parameters=[
            str(control_config),
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("task_scheduler")),
    )
    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[keyboard_teleop, gui_controller, task_scheduler_node],
        )
    )

    return LaunchDescription(
        [
            gui_argument,
            paused_argument,
            teleop_argument,
            control_gui_argument,
            task_scheduler_argument,
            spawn_z_argument,
            spawn_cabinet_argument,
            cabinet_x_argument,
            cabinet_y_argument,
            cabinet_z_argument,
            cabinet_yaw_argument,
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_robot,
            spawn_cabinet,
            start_controllers_after_spawn,
        ]
    )
