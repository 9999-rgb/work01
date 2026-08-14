"""Validated scene catalog shared by the Web gateway and the launch file.

A scene describes one simulated environment the robot can operate in: whether
the cabinet cluster is present, an optional static floor model, the Nav2 map
to serve, and the robot's initial pose.  The module is deliberately free of
ROS imports so launch tooling, unit tests and the gateway can all validate the
same ``scenes.yaml`` without sourcing a workspace.  ``package://`` references
are resolved lazily through :func:`resolve_package_uri` for callers that run
inside a sourced ROS environment.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import yaml

from . import _package_resolver

_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_MODEL_POSE_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
_ROBOT_SPAWN_FIELDS = ("x", "y", "z", "yaw")


class SceneError(ValueError):
    """Base error for invalid scene configuration."""


class SceneNotFoundError(SceneError):
    """Raised when a requested scene name is absent from the catalog."""

    def __init__(self, name: str) -> None:
        super().__init__(f"Unknown scene: {name}")
        self.name = name


@dataclass(frozen=True)
class SceneModel:
    """A static scene floor spawned as a single Gazebo entity."""

    urdf: str
    pose: Tuple[float, float, float, float, float, float]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "urdf": self.urdf,
            "pose": dict(zip(_MODEL_POSE_FIELDS, self.pose)),
        }


@dataclass(frozen=True)
class RobotSpawn:
    """Robot initial pose: base position and map-frame yaw."""

    x: float
    y: float
    z: float
    yaw: float

    def to_dict(self) -> Dict[str, Any]:
        return {"x": self.x, "y": self.y, "z": self.z, "yaw": self.yaw}


@dataclass(frozen=True)
class SceneSpec:
    """One named scene with its spawn geometry, map and robot pose."""

    name: str
    spawn_cabinet: bool
    model: Optional[SceneModel]
    nav2_map: str
    robot_spawn: Optional[RobotSpawn]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "spawn_cabinet": self.spawn_cabinet,
            "model": self.model.to_dict() if self.model is not None else None,
            "nav2_map": self.nav2_map,
            "robot_spawn": (
                self.robot_spawn.to_dict()
                if self.robot_spawn is not None
                else None
            ),
        }


class SceneCatalog:
    """Immutable, strictly validated scene list keyed by name."""

    def __init__(self, scenes: Sequence[SceneSpec]) -> None:
        by_name: Dict[str, SceneSpec] = {}
        ordered: list[SceneSpec] = []
        for scene in scenes:
            if scene.name in by_name:
                raise SceneError(f"Duplicate scene name: {scene.name}")
            by_name[scene.name] = scene
            ordered.append(scene)
        if not ordered:
            raise SceneError("Scene catalog must contain at least one scene.")
        self._scenes = tuple(ordered)
        self._by_name = by_name

    @classmethod
    def load(cls, path: str | Path) -> "SceneCatalog":
        document = _load_yaml(path)
        return cls(_parse_scenes(document))

    def __len__(self) -> int:
        return len(self._scenes)

    def __iter__(self):  # type: ignore[no-untyped-def]
        return iter(self._scenes)

    @property
    def names(self) -> Tuple[str, ...]:
        return tuple(scene.name for scene in self._scenes)

    def get(self, name: str) -> SceneSpec:
        try:
            return self._by_name[name]
        except (KeyError, TypeError) as error:
            raise SceneNotFoundError(str(name)) from error

    def list_scenes(self) -> list[Dict[str, Any]]:
        return [scene.to_dict() for scene in self._scenes]


def load_scene_catalog(path: str | Path) -> SceneCatalog:
    """Convenience wrapper used by command-line entry points."""
    return SceneCatalog.load(path)


def resolve_package_uri(uri: str) -> Path:
    """Resolve a ``package://`` reference to a filesystem path.

    Non-``package://`` values are returned as filesystem paths.  ``package://``
    resolution needs the ROS environment and is performed through the injectable
    :mod:`control_gateway._package_resolver` helper so pure callers (tests,
    launch validation) can substitute it without importing ``ament_index``.
    """
    return _package_resolver.resolve_package_uri(uri)


def _load_yaml(path_value: str | Path) -> Mapping[str, Any]:
    path = Path(path_value).expanduser()
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise SceneError(f"Cannot read scene catalog {path}: {error}") from error
    except yaml.YAMLError as error:
        raise SceneError(f"Invalid scene catalog YAML {path}: {error}") from error
    if not isinstance(document, Mapping):
        raise SceneError(
            f"Scene catalog {path} must contain a mapping at its root."
        )
    return document


def _parse_scenes(document: Mapping[str, Any]) -> list[SceneSpec]:
    unknown_root = set(document) - {"scenes"}
    if unknown_root:
        raise SceneError(
            "scenes.yaml has unknown root fields: "
            + ", ".join(sorted(str(field) for field in unknown_root))
        )
    values = document.get("scenes")
    if not isinstance(values, list) or not values:
        raise SceneError("scenes must be a non-empty list.")
    return [_parse_scene(value, index) for index, value in enumerate(values)]


def _parse_scene(value: Any, index: int) -> SceneSpec:
    context = f"scenes[{index}]"
    if not isinstance(value, Mapping):
        raise SceneError(f"{context} must be a mapping.")
    allowed = {"name", "spawn_cabinet", "model", "nav2_map", "robot_spawn"}
    unknown = set(value) - allowed
    if unknown:
        raise SceneError(
            f"{context} has unknown fields: "
            + ", ".join(sorted(str(field) for field in unknown))
        )
    name = value.get("name")
    if not isinstance(name, str) or _NAME_PATTERN.fullmatch(name) is None:
        raise SceneError(
            f"{context}.name must match {_NAME_PATTERN.pattern}."
        )
    spawn_cabinet = value.get("spawn_cabinet")
    if not isinstance(spawn_cabinet, bool):
        raise SceneError(f"{context}.spawn_cabinet must be a boolean.")
    model = _parse_model(value.get("model"), f"{context}.model")
    nav2_map = value.get("nav2_map")
    if not isinstance(nav2_map, str) or not nav2_map.strip():
        raise SceneError(f"{context}.nav2_map must be a non-empty string.")
    robot_spawn = _parse_robot_spawn(
        value.get("robot_spawn"), f"{context}.robot_spawn"
    )
    return SceneSpec(
        name=name,
        spawn_cabinet=spawn_cabinet,
        model=model,
        nav2_map=nav2_map.strip(),
        robot_spawn=robot_spawn,
    )


def _parse_model(value: Any, context: str) -> Optional[SceneModel]:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise SceneError(f"{context} must be a mapping or null.")
    allowed = {"urdf", "pose"}
    unknown = set(value) - allowed
    if unknown:
        raise SceneError(
            f"{context} has unknown fields: "
            + ", ".join(sorted(str(field) for field in unknown))
        )
    urdf = value.get("urdf")
    if not isinstance(urdf, str) or not urdf.strip():
        raise SceneError(f"{context}.urdf must be a non-empty string.")
    pose = _parse_pose(value.get("pose"), f"{context}.pose", _MODEL_POSE_FIELDS)
    return SceneModel(urdf=urdf.strip(), pose=pose)


def _parse_robot_spawn(value: Any, context: str) -> Optional[RobotSpawn]:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise SceneError(f"{context} must be a mapping or null.")
    unknown = set(value) - set(_ROBOT_SPAWN_FIELDS)
    if unknown:
        raise SceneError(
            f"{context} has unknown fields: "
            + ", ".join(sorted(str(field) for field in unknown))
        )
    x = _finite_number(value.get("x"), f"{context}.x")
    y = _finite_number(value.get("y"), f"{context}.y")
    z = _finite_number(value.get("z"), f"{context}.z")
    yaw = _finite_number(value.get("yaw"), f"{context}.yaw")
    return RobotSpawn(x=x, y=y, z=z, yaw=yaw)


def _parse_pose(
    value: Any,
    context: str,
    fields: Sequence[str],
) -> Tuple[float, float, float, float, float, float]:
    if not isinstance(value, Mapping):
        raise SceneError(f"{context} must be a mapping.")
    unknown = set(value) - set(fields)
    if unknown:
        raise SceneError(
            f"{context} has unknown fields: "
            + ", ".join(sorted(str(field) for field in unknown))
        )
    missing = [field for field in fields if field not in value]
    if missing:
        raise SceneError(
            f"{context} is missing fields: " + ", ".join(missing)
        )
    return tuple(
        _finite_number(value[field], f"{context}.{field}")  # type: ignore[arg-type]
        for field in fields
    )  # type: ignore[return-value]


def _is_finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _finite_number(value: Any, label: str) -> float:
    if not _is_finite_number(value):
        raise SceneError(f"{label} must be a finite number.")
    return float(value)
