"""Launch Gazebo Classic and spawn the XCZS inspection robot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "xczs_inspection_robot_description"
URDF_FILENAME = "0.XCZS_巡操机器人.urdf"


def generate_launch_description() -> LaunchDescription:
    """Create the Gazebo launch description for the inspection robot."""
    gazebo_ros_share = Path(get_package_share_directory("gazebo_ros"))
    robot_share = Path(get_package_share_directory(PACKAGE_NAME))

    gui_argument = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start the Gazebo graphical client.",
    )
    paused_argument = DeclareLaunchArgument(
        "paused",
        default_value="true",
        description="Start Gazebo with physics paused for model inspection.",
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_ros_share / "launch" / "gzserver.launch.py")
        ),
        launch_arguments={"pause": LaunchConfiguration("paused")}.items(),
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
        arguments=[
            "-entity",
            "xczs_inspection_robot",
            "-file",
            str(robot_share / "urdf" / URDF_FILENAME),
        ],
    )

    return LaunchDescription(
        [
            gui_argument,
            paused_argument,
            gazebo_server,
            gazebo_client,
            spawn_robot,
        ]
    )
