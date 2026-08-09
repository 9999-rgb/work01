"""Start Nav2 localization, planning, control and optional RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NAV2_CONFIG_PACKAGE = "xczs_inspection_robot_nav2"


def generate_launch_description() -> LaunchDescription:
    """Build the XCZS Nav2 launch description."""
    config_share = Path(
        get_package_share_directory(NAV2_CONFIG_PACKAGE)
    )
    nav2_bringup_share = Path(
        get_package_share_directory("nav2_bringup")
    )

    default_map = config_share / "maps" / "inspection_map.yaml"
    default_params = config_share / "config" / "nav2_params.yaml"
    default_rviz = (
        nav2_bringup_share / "rviz" / "nav2_default_view.rviz"
    )

    map_argument = DeclareLaunchArgument(
        "map",
        default_value=str(default_map),
        description="Absolute path to the occupancy map YAML file.",
    )
    params_argument = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=str(default_params),
        description="Absolute path to the Nav2 parameter file.",
    )
    autostart_argument = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically activate all Nav2 lifecycle nodes.",
    )
    rviz_argument = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Start RViz with the Nav2 panel.",
    )
    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the simulation clock for all Nav2 nodes.",
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_bringup_share / "launch" / "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("nav2_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("autostart"),
            "slam": "False",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )
    nav2_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        output="log",
        arguments=["-d", str(default_rviz)],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            map_argument,
            params_argument,
            autostart_argument,
            rviz_argument,
            use_sim_time_argument,
            nav2_bringup,
            nav2_rviz,
        ]
    )
