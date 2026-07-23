"""Launch Gazebo Classic and spawn the CBA robot model."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the Gazebo launch description for CBA."""
    gazebo_ros_share = Path(get_package_share_directory("gazebo_ros"))
    cba_share = Path(get_package_share_directory("cba"))

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

    spawn_cba = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_cba",
        output="screen",
        arguments=[
            "-entity",
            "cba",
            "-file",
            str(cba_share / "urdf" / "cba.urdf"),
        ],
    )

    return LaunchDescription(
        [
            gui_argument,
            paused_argument,
            gazebo_server,
            gazebo_client,
            spawn_cba,
        ]
    )
