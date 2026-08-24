"""Unit tests for required-process startup chaining."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, LogInfo
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnShutdown
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

    def trajectory_execution(self, **_kwargs):
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
            "scene": "test_scene",
            "scenes_config": "scenes.yaml",
            "moveit": "true",
            "robot_name": "test_robot",
            "robot_xacro": "robot.xacro",
            "moveit_config_package": "test_moveit_config",
            "moveit_srdf": "test.srdf",
            "moveit_kinematics": "kinematics.yaml",
            "moveit_joint_limits": "joint_limits.yaml",
            "moveit_controllers": "controllers.yaml",
            "cabinet_instances": "instances.yaml",
            "cabinet_controls": "controls.yaml",
            "cabinet_scene": "scene.yaml",
            "cabinet_pose": "pose.yaml",
            "cabinet_robot_adapter": "adapter.yaml",
            "cabinet_pose_source": "static",
            "use_sim_time": "true",
            "toolset": "A",
            "gazebo_plugin_instance_id": "initial",
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
    # The scene's ``spawn_cabinet`` stays true so the global ``spawn_cabinet``
    # launch argument (the ``spawn_cabinet`` param) alone controls whether the
    # cabinet is spawned at runtime.
    fake_scenes = {
        "test_scene": {
            "name": "test_scene",
            "spawn_cabinet": True,
            "model": None,
            "nav2_map": "package://test/map.yaml",
            "robot_spawn": None,
        }
    }
    with (
        patch.object(module, "_read_instances", return_value=[instance]),
        patch.object(module, "_read_scenes", return_value=fake_scenes),
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


def _launch_argument_context(module):
    context = LaunchContext()
    context.launch_configurations.update(
        {name: "false" for name in module._BOOLEAN_LAUNCH_ARGUMENTS}
    )
    context.launch_configurations["spawn_z"] = "0.515"
    context.launch_configurations["gazebo_plugin_instance_id"] = "initial"
    return context


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


def test_every_boolean_launch_argument_is_validated():
    module = _load_launch_module()
    for name in module._BOOLEAN_LAUNCH_ARGUMENTS:
        context = _launch_argument_context(module)
        context.launch_configurations[name] = "truthy"
        with pytest.raises(RuntimeError, match=name):
            module._validate_launch_arguments(context)


def test_boolean_launch_arguments_are_normalized_before_use():
    module = _load_launch_module()
    context = _launch_argument_context(module)
    for name in module._BOOLEAN_LAUNCH_ARGUMENTS:
        context.launch_configurations[name] = " TRUE "

    actions = module._validate_launch_arguments(context)
    for action in actions:
        action.execute(context)
    assert all(
        context.launch_configurations[name] == "true"
        for name in module._BOOLEAN_LAUNCH_ARGUMENTS
    )


def test_gazebo_plugin_instance_id_is_validated_before_xacro_execution():
    module = _load_launch_module()
    context = _launch_argument_context(module)
    context.launch_configurations["gazebo_plugin_instance_id"] = " robot_b_12 "

    actions = module._validate_launch_arguments(context)
    for action in actions:
        action.execute(context)

    assert context.launch_configurations["gazebo_plugin_instance_id"] == "robot_b_12"

    context.launch_configurations["gazebo_plugin_instance_id"] = "unsafe-id"
    with pytest.raises(RuntimeError, match="gazebo_plugin_instance_id"):
        module._validate_launch_arguments(context)


def test_robot_gazebo_plugin_names_share_the_runtime_instance_id_contract():
    root = Path(__file__).resolve().parents[2]
    description_root = root / "xczs_inspection_robot_description" / "urdf"
    root_xacro = (description_root / "xczs_inspection_robot.urdf.xacro").read_text(
        encoding="utf-8"
    )
    assert '<xacro:arg name="gazebo_plugin_instance_id" default="initial" />' in root_xacro

    expected_plugins = {
        "components/sensors.xacro": (
            "xczs_arm_camera_plugin",
            "xczs_body_lidar_plugin",
        ),
        "components/gazebo_plugins.xacro": (
            "xczs_planar_move",
            "xczs_planar_stabilizer",
        ),
        "components/ros2_control.xacro": ("xczs_ros2_control",),
    }
    for relative_path, plugin_names in expected_plugins.items():
        source = (description_root / relative_path).read_text(encoding="utf-8")
        for plugin_name in plugin_names:
            assert f'{plugin_name}_$(arg gazebo_plugin_instance_id)' in source


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "not-a-number"])
def test_spawn_z_rejects_nonfinite_or_non_numeric_values(value):
    module = _load_launch_module()
    context = _launch_argument_context(module)
    context.launch_configurations["spawn_z"] = value

    with pytest.raises(RuntimeError, match="spawn_z must be a finite number"):
        module._validate_launch_arguments(context)


@pytest.mark.parametrize("value", ["0", "-0.125", "+5.15e-1"])
def test_spawn_z_accepts_finite_decimal_and_scientific_values(value):
    module = _load_launch_module()
    context = _launch_argument_context(module)
    context.launch_configurations["spawn_z"] = value

    assert module._validate_launch_arguments(context)


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


def test_spawn_pose_is_verified_without_delayed_motion(tmp_path):
    module = _load_launch_module()
    with patch.object(
        module,
        "get_package_share_directory",
        return_value=str(tmp_path),
    ):
        entities = list(module.generate_launch_description().entities)

    controller_handler = next(
        entity
        for entity in entities
        if isinstance(entity, RegisterEventHandler)
        and isinstance(entity.event_handler, OnProcessExit)
        and isinstance(_on_process_exit_target(entity), Node)
        and _on_process_exit_target(entity).node_executable == "spawner"
    )
    downstream = _on_process_exit_callback(controller_handler)(
        SimpleNamespace(returncode=0),
        None,
    )

    assert len(downstream) == 1
    assert isinstance(downstream[0], OpaqueFunction)
    tool_context = LaunchContext()
    tool_context.launch_configurations.update(
        toolset="A",
        robot_bringup="true",
    )
    tool_downstream = downstream[0].execute(tool_context)
    assert isinstance(tool_downstream[0], RegisterEventHandler)
    tool_spawner = tool_downstream[1]
    assert isinstance(tool_spawner, Node)
    assert tool_spawner.node_executable == "spawner"
    assert tool_downstream.index(tool_downstream[0]) < tool_downstream.index(
        tool_spawner
    )

    tool_failure_actions = _on_process_exit_callback(tool_downstream[0])(
        SimpleNamespace(returncode=1),
        None,
    )
    assert isinstance(tool_failure_actions[1], EmitEvent)
    assert isinstance(tool_failure_actions[1].event, Shutdown)
    assert "tool controller spawner" in tool_failure_actions[1].event.reason

    downstream = _on_process_exit_callback(tool_downstream[0])(
        SimpleNamespace(returncode=0),
        None,
    )

    verifier = next(
        action for action in downstream if isinstance(action, ExecuteProcess)
    )
    command_text = " ".join(
        substitution.text
        for part in verifier.cmd
        for substitution in part
        if hasattr(substitution, "text")
    )
    assert "verify_initial_pose.py" in command_text
    assert "set_initial_pose.py" not in command_text

    verifier_handler = next(
        action
        for action in downstream
        if isinstance(action, RegisterEventHandler)
        and _on_process_exit_target(action) is verifier
    )
    assert downstream.index(verifier_handler) < downstream.index(verifier)

    failure_actions = _on_process_exit_callback(verifier_handler)(
        SimpleNamespace(returncode=1),
        None,
    )
    assert isinstance(failure_actions[1], EmitEvent)
    assert isinstance(failure_actions[1].event, Shutdown)
    assert "spawn-time fortune-cat pose verification" in failure_actions[1].event.reason
    assert "code 1" in failure_actions[1].event.reason

    verified_actions = _on_process_exit_callback(verifier_handler)(
        SimpleNamespace(returncode=0),
        None,
    )
    assert "base_command_router" in _node_executables(verified_actions)
    assert "legacy_trajectory_router" in _node_executables(verified_actions)
    assert any(
        isinstance(action, IncludeLaunchDescription)
        for action in verified_actions
    )


def test_shutdown_handler_removes_all_generated_directories(tmp_path):
    module = _load_launch_module()
    with patch.object(
        module,
        "get_package_share_directory",
        return_value=str(tmp_path),
    ):
        entities = list(module.generate_launch_description().entities)

    cleanup_index, cleanup = next(
        (index, entity)
        for index, entity in enumerate(entities)
        if isinstance(entity, RegisterEventHandler)
        and isinstance(entity.event_handler, OnShutdown)
    )
    opaque_indexes = [
        index
        for index, entity in enumerate(entities)
        if isinstance(entity, OpaqueFunction)
    ]
    assert opaque_indexes
    assert cleanup_index < min(opaque_indexes)

    generated = [
        tmp_path / "xczs_cabinets_test",
        tmp_path / "xczs_nav2_params_test",
    ]
    for directory in generated:
        directory.mkdir()
        (directory / "generated.file").write_text("test", encoding="utf-8")
    module._GENERATED_DIRECTORIES.extend(generated)

    launch_service = LaunchService()
    launch_service.include_launch_description(
        LaunchDescription(
            [cleanup, EmitEvent(event=Shutdown(reason="test shutdown"))]
        )
    )
    assert launch_service.run() == 0
    assert not module._GENERATED_DIRECTORIES
    assert all(not directory.exists() for directory in generated)


def test_generated_directories_use_run_all_owned_runtime_root(
    tmp_path, monkeypatch
):
    module = _load_launch_module()
    runtime_root = tmp_path / "xczs_runtime_test"
    runtime_root.mkdir()
    monkeypatch.setenv(
        "XCZS_LAUNCH_RUNTIME_DIRECTORY", str(runtime_root)
    )

    cabinet_directory = module._make_generated_directory("xczs_cabinets_")
    nav2_directory = module._make_generated_directory("xczs_nav2_params_")

    assert cabinet_directory.parent == runtime_root
    assert nav2_directory.parent == runtime_root
    assert module._GENERATED_DIRECTORIES == [
        cabinet_directory,
        nav2_directory,
    ]

    module._cleanup_generated_directories()
    assert runtime_root.is_dir()
    assert not cabinet_directory.exists()
    assert not nav2_directory.exists()


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

    assert len(required_handler_indexes) == 6
    assert node_indexes
    assert max(required_handler_indexes) < min(node_indexes)

    targets = [
        _on_process_exit_target(entities[index])
        for index in required_handler_indexes
    ]
    # The robot spawn (spawn_entity.py) handler is now returned by
    # ``_robot_spawn_node`` inside an OpaqueFunction, so it is not a top-level
    # watchdog target; the remaining six top-level handlers are listed here.
    assert sorted(
        target.node_executable
        for target in targets
        if isinstance(target, Node)
    ) == sorted(
        [
            "spawner",
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


def test_external_stack_never_includes_local_moveit_or_nav2(tmp_path):
    module = _load_launch_module()
    with patch.object(
        module,
        "get_package_share_directory",
        return_value=str(tmp_path),
    ):
        entities = list(module.generate_launch_description().entities)

    controller_handler = next(
        entity
        for entity in entities
        if isinstance(entity, RegisterEventHandler)
        and isinstance(entity.event_handler, OnProcessExit)
        and isinstance(_on_process_exit_target(entity), Node)
        and _on_process_exit_target(entity).node_executable == "spawner"
    )
    downstream = _on_process_exit_callback(controller_handler)(
        SimpleNamespace(returncode=0),
        None,
    )
    tool_context = LaunchContext()
    tool_context.launch_configurations.update(
        toolset="A",
        robot_bringup="true",
    )
    tool_downstream = downstream[0].execute(tool_context)
    downstream = _on_process_exit_callback(tool_downstream[0])(
        SimpleNamespace(returncode=0),
        None,
    )
    verifier = next(
        action for action in downstream if isinstance(action, ExecuteProcess)
    )
    verifier_handler = next(
        action
        for action in downstream
        if isinstance(action, RegisterEventHandler)
        and _on_process_exit_target(action) is verifier
    )
    downstream = _on_process_exit_callback(verifier_handler)(
        SimpleNamespace(returncode=0),
        None,
    )
    # move_group is the only direct include; Nav2 is wrapped in an
    # OpaqueFunction so its map can be resolved from the active scene at
    # launch time.
    move_group = next(
        action
        for action in downstream
        if isinstance(action, IncludeLaunchDescription)
    )
    nav2 = next(
        action
        for action in downstream
        if isinstance(action, OpaqueFunction)
    )

    external = LaunchContext()
    external.launch_configurations.update(
        robot_bringup="false",
        moveit="true",
        nav2="true",
    )
    # External-stack mode must neither include our local move_group nor emit
    # any local Nav2 include.
    assert not move_group.condition.evaluate(external)
    assert nav2.execute(external) == []

    local = LaunchContext()
    local.launch_configurations.update(
        robot_bringup="true",
        moveit="true",
        nav2="true",
    )
    assert move_group.condition.evaluate(local)


def test_world_only_mode_does_not_start_duplicate_command_or_lease_nodes(
    tmp_path,
):
    """The persistent Gazebo owner must not compete with its robot child."""
    module = _load_launch_module()
    with patch.object(
        module,
        "get_package_share_directory",
        return_value=str(tmp_path),
    ):
        entities = list(module.generate_launch_description().entities)

    controller_handler = next(
        entity
        for entity in entities
        if isinstance(entity, RegisterEventHandler)
        and isinstance(entity.event_handler, OnProcessExit)
        and isinstance(_on_process_exit_target(entity), Node)
        and _on_process_exit_target(entity).node_executable == "spawner"
    )
    tool_start = _on_process_exit_callback(controller_handler)(
        SimpleNamespace(returncode=0),
        None,
    )
    tool_context = LaunchContext()
    tool_context.launch_configurations.update(toolset="A", robot_bringup="true")
    tool_actions = tool_start[0].execute(tool_context)
    verifier_actions = _on_process_exit_callback(tool_actions[0])(
        SimpleNamespace(returncode=0),
        None,
    )
    verifier = next(
        action for action in verifier_actions if isinstance(action, ExecuteProcess)
    )
    verifier_handler = next(
        action
        for action in verifier_actions
        if isinstance(action, RegisterEventHandler)
        and _on_process_exit_target(action) is verifier
    )
    verified_actions = _on_process_exit_callback(verifier_handler)(
        SimpleNamespace(returncode=0),
        None,
    )
    nodes = {
        entity.node_executable: entity
        for entity in verified_actions
        if isinstance(entity, Node)
        and entity.node_executable in {
            "base_command_router",
            "legacy_trajectory_router",
        }
    }
    assert set(nodes) == {"base_command_router", "legacy_trajectory_router"}
    lease = next(
        entity
        for entity in entities
        if isinstance(entity, Node)
        and entity.node_executable == "operation_lease_coordinator"
    )

    world_only = LaunchContext()
    world_only.launch_configurations.update(
        robot_bringup="false",
        moveit="false",
        cabinet_bringup="false",
    )
    assert not nodes["base_command_router"].condition.evaluate(world_only)
    assert not nodes["legacy_trajectory_router"].condition.evaluate(world_only)
    assert not lease.condition.evaluate(world_only)

    robot_child = LaunchContext()
    robot_child.launch_configurations.update(
        robot_bringup="true",
        moveit="true",
        cabinet_bringup="true",
    )
    assert nodes["base_command_router"].condition.evaluate(robot_child)
    assert nodes["legacy_trajectory_router"].condition.evaluate(robot_child)
    assert lease.condition.evaluate(robot_child)


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


def test_scene_floor_spawn_failure_stops_the_world_launch(tmp_path):
    module = _load_launch_module()
    floor = tmp_path / "floor.sdf"
    floor.write_text("<sdf version='1.6'/>", encoding="utf-8")
    context = LaunchContext()
    context.launch_configurations["gazebo"] = "true"
    scene = {
        "model": {
            "file": str(floor),
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
    }
    with patch.object(module, "_scene_spec", return_value=scene):
        actions = module._scene_floor_nodes(context)

    assert isinstance(actions[0], RegisterEventHandler)
    assert isinstance(actions[1], Node)
    assert actions[1].node_executable == "spawn_entity.py"
    assert _on_process_exit_target(actions[0]) is actions[1]
    failure_actions = _on_process_exit_callback(actions[0])(
        SimpleNamespace(returncode=4),
        None,
    )
    assert isinstance(failure_actions[1], EmitEvent)
    assert isinstance(failure_actions[1].event, Shutdown)
    assert "Gazebo scene floor spawn" in failure_actions[1].event.reason
