"""Tests for strict Nav2 launch boolean handling."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
from launch import LaunchContext


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "navigation.launch.py"
    )
    spec = spec_from_file_location("navigation_launch", launch_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _launch_context(module):
    context = LaunchContext()
    context.launch_configurations.update(
        {name: "false" for name in module._BOOLEAN_LAUNCH_ARGUMENTS}
    )
    return context


def test_every_nav2_boolean_argument_is_validated():
    module = _load_launch_module()
    for name in module._BOOLEAN_LAUNCH_ARGUMENTS:
        context = _launch_context(module)
        context.launch_configurations[name] = "truthy"
        with pytest.raises(RuntimeError, match=name):
            module._validate_launch_arguments(context)


def test_nav2_boolean_arguments_are_normalized_before_forwarding():
    module = _load_launch_module()
    context = _launch_context(module)
    for name in module._BOOLEAN_LAUNCH_ARGUMENTS:
        context.launch_configurations[name] = " TRUE "

    actions = module._validate_launch_arguments(context)
    for action in actions:
        action.execute(context)

    assert all(
        context.launch_configurations[name] == "true"
        for name in module._BOOLEAN_LAUNCH_ARGUMENTS
    )
