"""Start MoveIt 2 move_group and the optional Motion Planning RViz view."""

from functools import partial
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


MOVEIT_CONFIG_PACKAGE = "xczs_inspection_robot_moveit_config"
DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"

VALID_TOOLSETS = ("A", "B")
DEFAULT_TOOLSET = "A"


def _toolset_substitution() -> PythonExpression:
    """Runtime substitution resolving the normalized toolset (A/B, uppercase)."""
    return PythonExpression(
        [
            "(str('",
            LaunchConfiguration("toolset"),
            "').strip().upper() if str('",
            LaunchConfiguration("toolset"),
            "').strip().upper() in ('A','B') else 'A')",
        ]
    )


def _shutdown_on_required_runtime_exit(
    event,
    context,
    *,
    process_label,
):
    """Shut down this included launch if a required runtime exits."""
    if context.is_shutdown:
        return []
    reason = (
        f"Required runtime process '{process_label}' exited with code "
        f"{event.returncode}; MoveIt is no longer operational."
    )
    return [
        LogInfo(msg=reason),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


def _required_runtime_handler(process, process_label):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=process,
            on_exit=partial(
                _shutdown_on_required_runtime_exit,
                process_label=process_label,
            ),
        )
    )


def _launch_boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in {"true", "false"}:
        raise RuntimeError(f"{name} must be true or false, got {value!r}")
    return value == "true"


def _launch_setup(context):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    moveit_config_package = LaunchConfiguration(
        "moveit_config_package"
    ).perform(context)
    robot_xacro = LaunchConfiguration("robot_xacro").perform(context)
    toolset = (
        LaunchConfiguration("toolset").perform(context).strip().upper()
        or DEFAULT_TOOLSET
    )
    if toolset not in VALID_TOOLSETS:
        toolset = DEFAULT_TOOLSET
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
    use_sim_time = _launch_boolean(context, "use_sim_time")
    rviz_enabled = _launch_boolean(context, "rviz")
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
        .robot_description(
            file_path=robot_xacro,
            mappings={"toolset": toolset},
        )
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
        output="screen",
        remappings=[("joint_states", joint_state_topic)],
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "allow_trajectory_execution": True,
                "monitor_dynamics": False,
                # The arm's collision meshes pack to ~3 mm between non-adjacent
                # links (r_arm_4/r_arm_6) in the folded fortune-cat idle pose.
                # MoveIt's default 10 mm robot self-padding flags that gap as a
                # "collision" and blocks every plan from the idle pose, even
                # though Gazebo ODE (padding 0) runs it contact-free.  Shrink
                # robot self-padding to 1 mm so the folded pose plans, while
                # object/attached padding keeps the 10 mm cabinet clearance.
                "robot_description_planning": {
                    "default_robot_padding": 0.001,
                    "default_robot_padding_scale": 1.0,
                    "default_object_padding": 0.01,
                    "default_attached_padding": 0.01,
                },
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
        condition=IfCondition("true" if rviz_enabled else "false"),
    )

    return [
        _required_runtime_handler(move_group, "MoveIt move_group"),
        move_group,
        moveit_rviz,
    ]


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
            "toolset",
            default_value=DEFAULT_TOOLSET,
            description=(
                "末端工具套装 A/B: A = 三电缸(右)+两电缸(左), "
                "B = 旋转按钮(右)+摇入摇出(左)。默认 A。"
            ),
        ),
        DeclareLaunchArgument(
            "moveit_srdf",
            default_value=[
                "config/xczs_inspection_robot_toolset_",
                _toolset_substitution(),
                ".srdf",
            ],
        ),
        DeclareLaunchArgument(
            "moveit_kinematics",
            default_value="config/kinematics.yaml",
        ),
        DeclareLaunchArgument(
            "moveit_joint_limits",
            default_value=[
                "config/joint_limits_toolset_",
                _toolset_substitution(),
                ".yaml",
            ],
        ),
        DeclareLaunchArgument(
            "moveit_controllers",
            default_value=[
                "config/moveit_controllers_toolset_",
                _toolset_substitution(),
                ".yaml",
            ],
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
