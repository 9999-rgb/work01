"""ROS 2 camera and lidar streaming support for the XCZS web panel."""

from typing import Any

__all__ = [
    "SensorRosRuntime",
    "SensorStreamState",
]


def __getattr__(name: str) -> Any:
    if name == "SensorRosRuntime":
        from .ros_node import SensorRosRuntime

        return SensorRosRuntime
    if name == "SensorStreamState":
        from .state import SensorStreamState

        return SensorStreamState
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
