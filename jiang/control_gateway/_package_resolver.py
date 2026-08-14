"""Lazy ``package://`` URI resolution, isolated so pure modules stay ROS-free.

:mod:`control_gateway.scene_catalog` and other pure modules never import ROS
packages at module scope.  This tiny module defers the ``ament_index_python``
import until a ``package://`` reference is actually resolved, and keeps the
resolution in one place that tests can monkeypatch without touching ROS.
"""

from __future__ import annotations

from pathlib import Path


class PackageResolutionError(ValueError):
    """Raised when a ``package://`` URI cannot be resolved to a path."""


def resolve_package_uri(uri: str) -> Path:
    """Return the filesystem path behind ``package://`` or an ordinary path."""
    if not isinstance(uri, str) or not uri.strip():
        raise PackageResolutionError("package URI must be a non-empty string.")
    value = uri.strip()
    if not value.startswith("package://"):
        return Path(value).expanduser()
    rest = value[len("package://") :]
    package, separator, relative = rest.partition("/")
    if not package or not separator or not relative:
        raise PackageResolutionError(f"Invalid package:// URI: {uri!r}.")
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as error:  # pragma: no cover - only outside ROS
        raise PackageResolutionError(
            "Resolving package:// URIs requires ament_index_python, which is "
            "only available in a sourced ROS environment."
        ) from error
    try:
        share = get_package_share_directory(package)
    except Exception as error:  # noqa: BLE001 - package may be missing
        raise PackageResolutionError(
            f"Unknown ROS package in {uri!r}: {package}"
        ) from error
    return Path(share) / relative
