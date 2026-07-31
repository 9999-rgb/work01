"""Start MoveIt 2 move_group and the optional Motion Planning RViz view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


MOVEIT_CONFIG_PACKAGE = "xczs_inspection_robot_moveit_config"
DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"


def generate_launch_description() -> LaunchDescription:
    """Build the XCZS MoveIt configuration and launch its planning nodes."""
    description_share = Path(
        get_package_share_directory(DESCRIPTION_PACKAGE)
    )
    moveit_share = Path(
        get_package_share_directory(MOVEIT_CONFIG_PACKAGE)
    )
    robot_xacro = (
        description_share / "urdf" / "xczs_inspection_robot.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "xczs_inspection_robot",
            package_name=MOVEIT_CONFIG_PACKAGE,
        )
        .robot_description(file_path=str(robot_xacro))
        .robot_description_semantic(
            file_path="config/xczs_inspection_robot.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    rviz_argument = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Start RViz with the MoveIt Motion Planning plugin.",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        remappings=[("joint_states", "/xczs/joint_states")],
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "allow_trajectory_execution": True,
                "monitor_dynamics": False,
            },
        ],
    )
    moveit_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="log",
        arguments=["-d", str(moveit_share / "config" / "moveit.rviz")],
        remappings=[("joint_states", "/xczs/joint_states")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            rviz_argument,
            move_group,
            moveit_rviz,
        ]
    )
