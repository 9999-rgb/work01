"""Unit tests for required-process startup chaining."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from launch.actions import EmitEvent, LogInfo
from launch.actions import RegisterEventHandler
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
        index for index, entity in enumerate(entities) if isinstance(entity, Node)
    ]

    assert len(required_handler_indexes) == 2
    assert node_indexes
    assert max(required_handler_indexes) < min(node_indexes)
