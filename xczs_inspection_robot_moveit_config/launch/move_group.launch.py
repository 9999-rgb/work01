"""Start MoveIt 2 move_group and the optional Motion Planning RViz view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


MOVEIT_CONFIG_PACKAGE = "xczs_inspection_robot_moveit_config"
DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"


def _launch_setup(context):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    moveit_config_package = LaunchConfiguration(
        "moveit_config_package"
    ).perform(context)
    robot_xacro = LaunchConfiguration("robot_xacro").perform(context)
    moveit_srdf = LaunchConfiguration("moveit_srdf").perform(context)
    moveit_kinematics = LaunchConfiguration("moveit_kinematics").perform(
        context
    )
    moveit_joint_limits = LaunchConfiguration(
        "moveit_joint_limits"
    ).perform(context)
    moveit_controllers = LaunchConfiguration("moveit_controllers").perform(
        context
    )
    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(
        context
    )
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower()
        == "true"
    )
    moveit_share = Path(
        get_package_share_directory(moveit_config_package)
    )
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    if not rviz_config:
        rviz_config = str(moveit_share / "config" / "moveit.rviz")
    moveit_config = (
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
        .trajectory_execution(
            file_path=moveit_controllers,
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        remappings=[("joint_states", joint_state_topic)],
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
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
        arguments=["-d", rviz_config],
        remappings=[("joint_states", joint_state_topic)],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return [move_group, moveit_rviz]


def generate_launch_description() -> LaunchDescription:
    """Build MoveIt from an explicit robot-profile contract."""
    description_share = Path(
        get_package_share_directory(DESCRIPTION_PACKAGE)
    )
    moveit_share = Path(
        get_package_share_directory(MOVEIT_CONFIG_PACKAGE)
    )
    arguments = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="xczs_inspection_robot",
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value=MOVEIT_CONFIG_PACKAGE,
        ),
        DeclareLaunchArgument(
            "robot_xacro",
            default_value=str(
                description_share
                / "urdf"
                / "xczs_inspection_robot.urdf.xacro"
            ),
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
            "joint_state_topic",
            default_value="/xczs/joint_states",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_share / "config" / "moveit.rviz"),
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz with the MoveIt Motion Planning plugin.",
        ),
    ]
    return LaunchDescription(
        arguments + [OpaqueFunction(function=_launch_setup)]
    )
