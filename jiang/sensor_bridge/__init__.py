"""ROS 2 camera and lidar streaming support for the XCZS web panel.

The public runtime imports stay lazy so CLI/configuration tooling can inspect
the standalone server on machines without a sourced ROS 2 environment.
"""

from typing import Any

__all__ = [
    "SensorRosRuntime",
    "SensorStreamState",
    "create_sensor_app",
]


def __getattr__(name: str) -> Any:
    if name == "SensorRosRuntime":
        from .ros_node import SensorRosRuntime

        return SensorRosRuntime
    if name == "SensorStreamState":
        from .state import SensorStreamState

        return SensorStreamState
    if name == "create_sensor_app":
        from .web_server import create_sensor_app

        return create_sensor_app
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
