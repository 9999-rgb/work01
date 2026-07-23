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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CONTROL_PACKAGE = "xczs_inspection_robot_control"
DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"
ROBOT_NAME = "xczs_inspection_robot"
URDF_FILENAME = "xczs_inspection_robot.urdf"


def generate_launch_description() -> LaunchDescription:
    """Create a unified Gazebo and keyboard-control launch description."""
    gazebo_ros_share = Path(get_package_share_directory("gazebo_ros"))
    description_share = Path(
        get_package_share_directory(DESCRIPTION_PACKAGE)
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
        default_value="true",
        description="Open the keyboard controller in a separate terminal.",
    )
    spawn_z_argument = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.515",
        description="Initial robot height above the Gazebo world origin.",
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
            "-file",
            str(description_share / "urdf" / URDF_FILENAME),
            "-z",
            LaunchConfiguration("spawn_z"),
        ],
    )

    keyboard_teleop = Node(
        package=CONTROL_PACKAGE,
        executable="keyboard_teleop",
        name="xczs_keyboard_teleop",
        output="screen",
        prefix="xfce4-terminal --disable-server --execute",
        condition=IfCondition(LaunchConfiguration("teleop")),
    )
    start_teleop_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[keyboard_teleop],
        )
    )

    return LaunchDescription(
        [
            gui_argument,
            paused_argument,
            teleop_argument,
            spawn_z_argument,
            gazebo_server,
            gazebo_client,
            spawn_robot,
            start_teleop_after_spawn,
        ]
    )
