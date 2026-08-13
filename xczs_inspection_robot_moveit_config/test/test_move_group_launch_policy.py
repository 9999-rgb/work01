"""Tests for the MoveIt required-process launch watchdog."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "move_group.launch.py"
    )
    spec = spec_from_file_location("move_group_launch", launch_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeMoveItConfig:
    robot_description = {}
    robot_description_semantic = {}
    robot_description_kinematics = {}
    planning_pipelines = {}
    joint_limits = {}

    def to_dict(self):
        return {}


class _FakeMoveItConfigsBuilder:
    def __getattr__(self, _name):
        return lambda **_kwargs: self

    def to_moveit_configs(self):
        return _FakeMoveItConfig()


def _launch_context():
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "robot_name": "test_robot",
            "moveit_config_package": "test_moveit_config",
            "robot_xacro": "robot.xacro",
            "moveit_srdf": "robot.srdf",
            "moveit_kinematics": "kinematics.yaml",
            "moveit_joint_limits": "joint_limits.yaml",
            "moveit_controllers": "controllers.yaml",
            "joint_state_topic": "/joint_states",
            "use_sim_time": "true",
            "rviz_config": "moveit.rviz",
            "rviz": "false",
        }
    )
    return context


def _on_process_exit_target(handler):
    return handler.event_handler._OnActionEventBase__action_matcher


def _on_process_exit_callback(handler):
    return handler.event_handler._OnActionEventBase__on_event


def test_move_group_watchdog_is_registered_before_required_node(tmp_path):
    module = _load_launch_module()
    with (
        patch.object(
            module,
            "get_package_share_directory",
            return_value=str(tmp_path),
        ),
        patch.object(
            module,
            "MoveItConfigsBuilder",
            return_value=_FakeMoveItConfigsBuilder(),
        ),
    ):
        actions = module._launch_setup(_launch_context())

    assert isinstance(actions[0], RegisterEventHandler)
    assert isinstance(actions[0].event_handler, OnProcessExit)
    assert isinstance(actions[1], Node)
    assert actions[1].node_executable == "move_group"
    assert _on_process_exit_target(actions[0]) is actions[1]
    assert isinstance(actions[2], Node)
    assert actions[2].node_executable == "rviz2"


def test_move_group_exit_requests_shutdown_while_launch_is_active(tmp_path):
    module = _load_launch_module()
    with (
        patch.object(
            module,
            "get_package_share_directory",
            return_value=str(tmp_path),
        ),
        patch.object(
            module,
            "MoveItConfigsBuilder",
            return_value=_FakeMoveItConfigsBuilder(),
        ),
    ):
        actions = module._launch_setup(_launch_context())

    result = _on_process_exit_callback(actions[0])(
        SimpleNamespace(returncode=4),
        LaunchContext(),
    )

    assert len(result) == 2
    assert isinstance(result[0], LogInfo)
    assert isinstance(result[1], EmitEvent)
    assert isinstance(result[1].event, Shutdown)
    assert "MoveIt move_group" in result[1].event.reason
    assert "code 4" in result[1].event.reason


def test_move_group_exit_during_shutdown_does_not_emit_again():
    module = _load_launch_module()
    context = LaunchContext()
    context._set_is_shutdown(True)

    result = module._shutdown_on_required_runtime_exit(
        SimpleNamespace(returncode=-15),
        context,
        process_label="MoveIt move_group",
    )

    assert result == []
