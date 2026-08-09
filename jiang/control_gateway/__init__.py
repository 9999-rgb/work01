"""Browser control gateway modules for the XCZS inspection robot.

The package keeps its ROS-backed server import lazy so configuration tooling
can validate profiles on machines that do not have a sourced ROS workspace.
"""

from typing import Any


__all__ = ["ControlServer"]


def __getattr__(name: str) -> Any:
    if name == "ControlServer":
        from .runner import ControlServer

        return ControlServer
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
