"""Unit tests for required-process startup chaining."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, LogInfo
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "inspection_robot.launch.py"
    )
    spec = spec_from_file_location("inspection_robot_launch", launch_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeMoveItConfigsBuilder:
    def robot_description(self, **_kwargs):
        return self

    def robot_description_semantic(self, **_kwargs):
        return self

    def robot_description_kinematics(self, **_kwargs):
        return self

    def joint_limits(self, **_kwargs):
        return self

    def planning_pipelines(self, **_kwargs):
        return self

    def to_moveit_configs(self):
        return SimpleNamespace(
            robot_description={},
            robot_description_semantic={},
            robot_description_kinematics={},
            joint_limits={},
        )


def _build_cabinet_actions(module, tmp_path, *, spawn_cabinet):
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "cabinet_bringup": "true",
            "spawn_cabinet": str(spawn_cabinet).lower(),
            "moveit": "true",
            "robot_name": "test_robot",
            "robot_xacro": "robot.xacro",
            "moveit_config_package": "test_moveit_config",
            "moveit_srdf": "test.srdf",
            "moveit_kinematics": "kinematics.yaml",
            "moveit_joint_limits": "joint_limits.yaml",
            "cabinet_instances": "instances.yaml",
            "cabinet_controls": "controls.yaml",
            "cabinet_scene": "scene.yaml",
            "cabinet_pose": "pose.yaml",
            "cabinet_robot_adapter": "adapter.yaml",
            "cabinet_pose_source": "static",
            "use_sim_time": "true",
        }
    )
    cabinet_xacro = tmp_path / "cabinet.urdf.xacro"
    cabinet_xacro.write_text("<robot/>", encoding="utf-8")
    generated_directory = tmp_path / "generated"
    generated_directory.mkdir()
    document = SimpleNamespace(
        getElementsByTagName=lambda _name: [],
        toxml=lambda: "<robot/>",
    )
    instance = {
        "name": "cabinet_a",
        "x": 1.0,
        "y": 2.0,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    interfaces = {
        "planning_frame": "world",
        "navigation_frame": "map",
        "joint_state_topic": "/joint_states",
    }
    with (
        patch.object(module, "_read_instances", return_value=[instance]),
        patch.object(
            module,
            "_read_button_profiles",
            return_value=(None, {}),
        ),
        patch.object(
            module,
            "_read_robot_adapter_interfaces",
            return_value=interfaces,
        ),
        patch.object(
            module,
            "MoveItConfigsBuilder",
            return_value=_FakeMoveItConfigsBuilder(),
        ),
        patch.object(module.xacro, "process_file", return_value=document),
        patch.object(
            module.tempfile,
            "mkdtemp",
            return_value=str(generated_directory),
        ),
    ):
        return module._cabinet_nodes(
            context,
            cabinet_xacro=cabinet_xacro,
        )


def _on_process_exit_target(handler):
    return handler.event_handler._OnActionEventBase__action_matcher


def _on_process_exit_callback(handler):
    return handler.event_handler._OnActionEventBase__on_event


def _node_executables(actions):
    return [
        action.node_executable
        for action in actions
        if isinstance(action, Node)
    ]


def _assert_nodes_have_earlier_watchdogs(actions, expected_executables):
    guarded_executables = []
    for index, action in enumerate(actions):
        if not isinstance(action, Node):
            continue
        earlier_targets = [
            _on_process_exit_target(candidate)
            for candidate in actions[:index]
            if isinstance(candidate, RegisterEventHandler)
            and isinstance(candidate.event_handler, OnProcessExit)
        ]
        if action in earlier_targets:
            guarded_executables.append(action.node_executable)
    assert guarded_executables == expected_executables


def test_required_process_success_starts_only_configured_downstream_actions():
    module = _load_launch_module()
    downstream = [object(), object()]

    result = module._continue_or_shutdown_required_process(
        SimpleNamespace(returncode=0),
        None,
        process_label="test process",
        success_actions=downstream,
    )

    assert result is downstream


def test_required_process_failure_logs_and_requests_shutdown():
    module = _load_launch_module()
    downstream = [object()]

    result = module._continue_or_shutdown_required_process(
        SimpleNamespace(returncode=7),
        None,
        process_label="test process",
        success_actions=downstream,
    )

    assert len(result) == 2
    assert isinstance(result[0], LogInfo)
    assert isinstance(result[1], EmitEvent)
    assert isinstance(result[1].event, Shutdown)
    assert "test process" in result[1].event.reason
    assert "code 7" in result[1].event.reason
    assert all(action not in result for action in downstream)


def test_required_process_handlers_are_registered_before_nodes(tmp_path):
    module = _load_launch_module()
    with patch.object(
        module,
        "get_package_share_directory",
        return_value=str(tmp_path),
    ):
        description = module.generate_launch_description()

    entities = list(description.entities)
    required_handler_indexes = [
        index
        for index, entity in enumerate(entities)
        if isinstance(entity, RegisterEventHandler)
        and isinstance(entity.event_handler, OnProcessExit)
    ]
    node_indexes = [
        index
        for index, entity in enumerate(entities)
        if isinstance(entity, Node)
    ]

    assert len(required_handler_indexes) == 7
    assert node_indexes
    assert max(required_handler_indexes) < min(node_indexes)

    targets = [
        _on_process_exit_target(entities[index])
        for index in required_handler_indexes
    ]
    assert sorted(
        target.node_executable
        for target in targets
        if isinstance(target, Node)
    ) == sorted(
        [
            "spawner",
            "spawn_entity.py",
            "robot_state_publisher",
            "base_command_router",
            "legacy_trajectory_router",
            "operation_lease_coordinator",
        ]
    )
    assert module._included_required_runtime_node in targets

    gazebo_server = next(
        entity
        for entity in entities
        if isinstance(entity, IncludeLaunchDescription)
        and "server_required" in dict(entity.launch_arguments)
    )
    assert dict(gazebo_server.launch_arguments)["server_required"] == "true"


def test_runtime_exit_requests_shutdown_even_for_clean_exit():
    module = _load_launch_module()
    context = LaunchContext()

    result = module._shutdown_on_required_runtime_exit(
        SimpleNamespace(returncode=0),
        context,
        process_label="test runtime",
    )

    assert len(result) == 2
    assert isinstance(result[0], LogInfo)
    assert isinstance(result[1], EmitEvent)
    assert isinstance(result[1].event, Shutdown)
    assert "test runtime" in result[1].event.reason
    assert "code 0" in result[1].event.reason


def test_runtime_exit_during_launch_shutdown_is_ignored():
    module = _load_launch_module()
    context = LaunchContext()
    context._set_is_shutdown(True)

    result = module._shutdown_on_required_runtime_exit(
        SimpleNamespace(returncode=-15),
        context,
        process_label="test runtime",
    )

    assert result == []


def test_conditioned_off_required_node_does_not_trigger_watchdog():
    module = _load_launch_module()
    node = Node(
        package="package_that_must_not_be_resolved",
        executable="disabled_node",
        condition=IfCondition("false"),
    )
    with patch.object(
        module,
        "_shutdown_on_required_runtime_exit",
        wraps=module._shutdown_on_required_runtime_exit,
    ) as shutdown_callback:
        handler = module._required_runtime_handler(node, "disabled node")
        launch_service = LaunchService()
        launch_service.include_launch_description(
            LaunchDescription([handler, node])
        )

        assert launch_service.run() == 0
        shutdown_callback.assert_not_called()


def test_required_runtime_watchdog_observes_real_process_exit():
    module = _load_launch_module()
    process = ExecuteProcess(
        cmd=["/usr/bin/python3", "-c", "raise SystemExit(7)"],
    )
    with patch.object(
        module,
        "_shutdown_on_required_runtime_exit",
        wraps=module._shutdown_on_required_runtime_exit,
    ) as shutdown_callback:
        handler = module._required_runtime_handler(process, "test process")
        launch_service = LaunchService()
        launch_service.include_launch_description(
            LaunchDescription([handler, process])
        )

        assert launch_service.run() == 0
        shutdown_callback.assert_called_once()
        event = shutdown_callback.call_args.args[0]
        assert event.returncode == 7
        assert (
            shutdown_callback.call_args.kwargs["process_label"]
            == "test process"
        )


def test_included_nav2_nodes_are_watchdog_targets():
    module = _load_launch_module()
    expected = {
        ("nav2_map_server", "map_server"),
        ("nav2_amcl", "amcl"),
        ("nav2_controller", "controller_server"),
        ("nav2_smoother", "smoother_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower"),
        ("nav2_velocity_smoother", "velocity_smoother"),
        ("nav2_lifecycle_manager", "lifecycle_manager"),
    }

    for package, executable in expected:
        node = Node(package=package, executable=executable)
        assert module._included_required_runtime_node(node)

    assert not module._included_required_runtime_node(
        Node(package="rviz2", executable="rviz2")
    )
    assert not module._included_required_runtime_node(
        Node(package="gazebo_ros", executable="spawn_entity.py")
    )
    assert not module._included_required_runtime_node(
        Node(package="moveit_ros_move_group", executable="move_group")
    )


def test_included_runtime_exit_resolves_label_and_requests_shutdown():
    module = _load_launch_module()
    node = Node(
        package="nav2_planner",
        executable="planner_server",
    )

    result = module._shutdown_on_included_required_runtime_exit(
        SimpleNamespace(action=node, returncode=6),
        LaunchContext(),
    )

    assert isinstance(result[1], EmitEvent)
    assert isinstance(result[1].event, Shutdown)
    assert "Nav2 planner server" in result[1].event.reason
    assert "code 6" in result[1].event.reason


def test_cabinet_spawn_success_starts_runtime_nodes_after_registered_handler(
    tmp_path,
):
    module = _load_launch_module()

    actions = _build_cabinet_actions(
        module,
        tmp_path,
        spawn_cabinet=True,
    )

    assert isinstance(actions[0], RegisterEventHandler)
    assert isinstance(actions[0].event_handler, OnProcessExit)
    assert isinstance(actions[1], Node)
    assert actions[1].node_executable == "spawn_entity.py"
    assert _on_process_exit_target(actions[0]) is actions[1]
    assert _node_executables(actions) == [
        "spawn_entity.py",
        "cabinet_grasp_aggregator",
    ]
    _assert_nodes_have_earlier_watchdogs(
        actions,
        ["spawn_entity.py", "cabinet_grasp_aggregator"],
    )

    success_actions = _on_process_exit_callback(actions[0])(
        SimpleNamespace(returncode=0),
        None,
    )
    assert _node_executables(success_actions) == [
        "cabinet_pose_authority",
        "cabinet_planning_scene",
        "cabinet_button_operator",
    ]
    _assert_nodes_have_earlier_watchdogs(
        success_actions,
        [
            "cabinet_pose_authority",
            "cabinet_planning_scene",
            "cabinet_button_operator",
        ],
    )


def test_cabinet_spawn_failure_shuts_down_without_runtime_nodes(tmp_path):
    module = _load_launch_module()
    actions = _build_cabinet_actions(
        module,
        tmp_path,
        spawn_cabinet=True,
    )

    failure_actions = _on_process_exit_callback(actions[0])(
        SimpleNamespace(returncode=9),
        None,
    )

    assert len(failure_actions) == 2
    assert isinstance(failure_actions[0], LogInfo)
    assert isinstance(failure_actions[1], EmitEvent)
    assert isinstance(failure_actions[1].event, Shutdown)
    assert (
        "Gazebo cabinet spawn 'cabinet_a'"
        in failure_actions[1].event.reason
    )
    assert "code 9" in failure_actions[1].event.reason
    assert not any(isinstance(action, Node) for action in failure_actions)


def test_cabinet_runtime_nodes_start_directly_when_spawn_is_disabled(tmp_path):
    module = _load_launch_module()

    actions = _build_cabinet_actions(
        module,
        tmp_path,
        spawn_cabinet=False,
    )

    assert _node_executables(actions) == [
        "cabinet_pose_authority",
        "cabinet_planning_scene",
        "cabinet_button_operator",
        "cabinet_grasp_aggregator",
    ]
    _assert_nodes_have_earlier_watchdogs(
        actions,
        [
            "cabinet_pose_authority",
            "cabinet_planning_scene",
            "cabinet_button_operator",
            "cabinet_grasp_aggregator",
        ],
    )
