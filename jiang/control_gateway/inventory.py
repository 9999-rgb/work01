"""Validated cabinet inventory and navigation-station geometry.

The gateway deliberately keeps this module independent from ROS so the same
inventory can be validated by launch files, HTTP tooling, and unit tests.  A
cabinet instance supplies its world pose while ``cabinet_scene.yaml`` supplies
the common station geometry expressed in the cabinet frame.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Sequence

import yaml


_CABINET_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_RELATIVE_TOPIC_PATTERN = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
_POSE_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


class InventoryError(ValueError):
    """Base error for invalid inventory data or station geometry."""


class CabinetNotFoundError(InventoryError):
    """Raised when a requested cabinet is absent from the inventory."""

    def __init__(self, name: str) -> None:
        super().__init__(f"Unknown cabinet: {name}")
        self.name = name


class NavigationStationOutOfBoundsError(InventoryError):
    """Raised when a computed station does not fit inside the current map."""

    def __init__(self, cabinet: str, x: float, y: float) -> None:
        super().__init__(
            f"Navigation station for {cabinet} is outside the map: "
            f"x={x:.3f}, y={y:.3f}"
        )
        self.cabinet = cabinet
        self.x = x
        self.y = y


@dataclass(frozen=True)
class MapBounds:
    """Axis-aligned map bounds accepted by :meth:`station_for`.

    ``from_grid`` mirrors the fields of ``nav_msgs/OccupancyGrid.info`` without
    importing ROS messages.  Rotated occupancy grids can instead supply a
    custom ``contains(x, y, margin=...)`` object or predicate.
    """

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def __post_init__(self) -> None:
        values = (self.min_x, self.min_y, self.max_x, self.max_y)
        if not all(_is_finite_number(value) for value in values):
            raise InventoryError("Map bounds must contain finite numbers.")
        if self.max_x <= self.min_x or self.max_y <= self.min_y:
            raise InventoryError("Map bounds must have positive width and height.")

    @classmethod
    def from_grid(
        cls,
        *,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> "MapBounds":
        """Build bounds from occupancy-grid metadata."""
        if isinstance(width, bool) or not isinstance(width, int) or width <= 0:
            raise InventoryError("Map width must be a positive integer.")
        if isinstance(height, bool) or not isinstance(height, int) or height <= 0:
            raise InventoryError("Map height must be a positive integer.")
        resolution = _finite_number(resolution, "map resolution")
        if resolution <= 0.0:
            raise InventoryError("Map resolution must be positive.")
        origin_x = _finite_number(origin_x, "map origin_x")
        origin_y = _finite_number(origin_y, "map origin_y")
        return cls(
            min_x=origin_x,
            min_y=origin_y,
            max_x=origin_x + width * resolution,
            max_y=origin_y + height * resolution,
        )

    def contains(self, x: float, y: float, *, margin: float = 0.0) -> bool:
        """Return whether a point is inside the map after an inset margin."""
        margin = _finite_number(margin, "map margin")
        if margin < 0.0:
            raise InventoryError("Map margin must not be negative.")
        return (
            self.min_x + margin <= x <= self.max_x - margin
            and self.min_y + margin <= y <= self.max_y - margin
        )


@dataclass(frozen=True)
class OccupancyGridBoundary:
    """World boundary of a possibly rotated ROS OccupancyGrid."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def __post_init__(self) -> None:
        if isinstance(self.width, bool) or not isinstance(self.width, int) or self.width <= 0:
            raise InventoryError("Map width must be a positive integer.")
        if (
            isinstance(self.height, bool)
            or not isinstance(self.height, int)
            or self.height <= 0
        ):
            raise InventoryError("Map height must be a positive integer.")
        for label, value in (
            ("map resolution", self.resolution),
            ("map origin_x", self.origin_x),
            ("map origin_y", self.origin_y),
            ("map origin_yaw", self.origin_yaw),
        ):
            _finite_number(value, label)
        if self.resolution <= 0.0:
            raise InventoryError("Map resolution must be positive.")

    def contains(self, x: float, y: float, *, margin: float = 0.0) -> bool:
        """Test a world point after inverse-rotating it into grid coordinates."""
        x = _finite_number(x, "map query x")
        y = _finite_number(y, "map query y")
        margin = _finite_number(margin, "map margin")
        if margin < 0.0:
            raise InventoryError("Map margin must not be negative.")
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        return (
            margin <= local_x <= self.width * self.resolution - margin
            and margin <= local_y <= self.height * self.resolution - margin
        )


@dataclass(frozen=True)
class CabinetInstance:
    """One named instance of the shared cabinet model."""

    name: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    station_override: Optional[Mapping[str, Any]] = None

    @property
    def namespace(self) -> str:
        return f"/xczs/cabinet/{self.name}"

    @property
    def frame_id(self) -> str:
        return f"{self.name}_frame"

    def to_dict(self) -> Dict[str, Any]:
        """Return the stable JSON representation exposed by ``/cabinets``."""
        return {
            "name": self.name,
            "namespace": self.namespace,
            "frame_id": self.frame_id,
            "pose": {field: getattr(self, field) for field in _POSE_FIELDS},
        }


@dataclass(frozen=True)
class NavigationStationSpec:
    """Common operation-station geometry in the cabinet-local frame."""

    local_anchor: tuple[float, float, float]
    outward_axis: tuple[float, float, float]
    standoff: float
    base_yaw_offset: float = 0.0
    frame_id: str = "map"

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        context: str = "navigation_station",
    ) -> "NavigationStationSpec":
        if not isinstance(value, Mapping):
            raise InventoryError(f"{context} must be a mapping.")
        allowed = {
            "local_anchor",
            "outward_axis",
            "standoff",
            "base_yaw_offset",
            "frame_id",
        }
        unknown = set(value) - allowed
        if unknown:
            raise InventoryError(
                f"{context} has unknown fields: "
                + ", ".join(sorted(str(field) for field in unknown))
            )
        anchor = _vector3(value.get("local_anchor"), f"{context}.local_anchor")
        axis = _vector3(value.get("outward_axis"), f"{context}.outward_axis")
        norm = math.sqrt(sum(component * component for component in axis))
        if norm <= 1.0e-12:
            raise InventoryError(f"{context}.outward_axis must not be zero.")
        axis = tuple(component / norm for component in axis)
        standoff = _finite_number(value.get("standoff"), f"{context}.standoff")
        if standoff <= 0.0:
            raise InventoryError(f"{context}.standoff must be positive.")
        yaw_offset = _finite_number(
            value.get("base_yaw_offset", 0.0),
            f"{context}.base_yaw_offset",
        )
        frame_id = value.get("frame_id", "map")
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise InventoryError(f"{context}.frame_id must be a non-empty string.")
        if any(character.isspace() for character in frame_id):
            raise InventoryError(f"{context}.frame_id must not contain whitespace.")
        return cls(
            local_anchor=anchor,
            outward_axis=axis,
            standoff=standoff,
            base_yaw_offset=yaw_offset,
            frame_id=frame_id.strip(),
        )

    def with_override(
        self,
        override: Optional[Mapping[str, Any]],
        *,
        cabinet: str,
    ) -> "NavigationStationSpec":
        """Apply an optional per-instance parameter-package override."""
        if override is None:
            return self
        if not isinstance(override, Mapping):
            raise InventoryError(
                f"instances[{cabinet}].navigation_station must be a mapping."
            )
        allowed = {
            "local_anchor",
            "outward_axis",
            "standoff",
            "base_yaw_offset",
            "frame_id",
        }
        unknown = set(override) - allowed
        if unknown:
            raise InventoryError(
                f"instances[{cabinet}].navigation_station has unknown fields: "
                + ", ".join(sorted(str(field) for field in unknown))
            )
        merged: Dict[str, Any] = {
            "local_anchor": list(self.local_anchor),
            "outward_axis": list(self.outward_axis),
            "standoff": self.standoff,
            "base_yaw_offset": self.base_yaw_offset,
            "frame_id": self.frame_id,
        }
        merged.update(override)
        return NavigationStationSpec.from_mapping(
            merged,
            context=f"instances[{cabinet}].navigation_station",
        )


@dataclass(frozen=True)
class NavigationStation:
    """Computed Nav2 goal for a cabinet operation station."""

    cabinet: str
    frame_id: str
    x: float
    y: float
    z: float
    yaw: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "cabinet": self.cabinet,
            "frame_id": self.frame_id,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
        }


BoundaryPredicate = Callable[[float, float], bool]


class CabinetInventory:
    """Immutable cabinet lookup built from the two shared YAML files."""

    def __init__(
        self,
        instances: Iterable[CabinetInstance],
        station_spec: NavigationStationSpec,
        pose_valid_topic: str = "pose_valid",
    ) -> None:
        by_name: Dict[str, CabinetInstance] = {}
        ordered = []
        for instance in instances:
            if instance.name in by_name:
                raise InventoryError(f"Duplicate cabinet name: {instance.name}")
            by_name[instance.name] = instance
            ordered.append(instance)
        if not ordered:
            raise InventoryError(
                "Cabinet inventory must contain at least one instance."
            )
        self._instances = tuple(ordered)
        self._by_name = by_name
        self._station_spec = station_spec
        if (
            not isinstance(pose_valid_topic, str)
            or _RELATIVE_TOPIC_PATTERN.fullmatch(pose_valid_topic.strip()) is None
        ):
            raise InventoryError(
                "pose_valid_topic must be a non-empty relative ROS topic."
            )
        self._pose_valid_topic = pose_valid_topic.strip()

    @classmethod
    def load(
        cls,
        instances_path: str | Path,
        scene_path: str | Path,
    ) -> "CabinetInventory":
        """Load and strictly validate an instance list and scene parameters."""
        instance_document = _load_yaml(instances_path)
        scene_document = _load_yaml(scene_path)
        instances = _parse_instances(instance_document)
        station_parameters = _find_navigation_station(scene_document)
        station_spec = NavigationStationSpec.from_mapping(station_parameters)
        pose_valid_topic = _find_pose_valid_topic(scene_document)
        return cls(instances, station_spec, pose_valid_topic)

    def __len__(self) -> int:
        return len(self._instances)

    def __iter__(self):  # type: ignore[no-untyped-def]
        return iter(self._instances)

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(instance.name for instance in self._instances)

    def get(self, name: str) -> CabinetInstance:
        try:
            return self._by_name[name]
        except (KeyError, TypeError) as error:
            raise CabinetNotFoundError(str(name)) from error

    def pose_valid_topic_for(self, name: str) -> str:
        """Return the authoritative, instance-scoped pose-validity topic."""
        instance = self.get(name)
        return f"{instance.namespace}/{self._pose_valid_topic}"

    def list_cabinets(self, *, include_station: bool = True) -> list[Dict[str, Any]]:
        cabinets = []
        for instance in self._instances:
            cabinet = instance.to_dict()
            if include_station:
                cabinet["navigation_station"] = self.station_for(
                    instance.name
                ).to_dict()
            cabinets.append(cabinet)
        return cabinets

    def station_spec_for(
        self,
        name: str,
        *,
        control_station: Optional[NavigationStationSpec] = None,
    ) -> NavigationStationSpec:
        """Return the final cabinet-local station geometry for one request.

        Instance overrides are applied to the common scene geometry first.  A
        robot-adapter control station, when supplied, is already a complete
        specification and therefore replaces that result.  Exposing this
        resolved value lets ROS-backed callers apply the latest TF transform
        without duplicating inventory precedence rules.
        """
        instance = self.get(name)
        spec = self._station_spec.with_override(
            instance.station_override,
            cabinet=instance.name,
        )
        if control_station is None:
            return spec
        if not isinstance(control_station, NavigationStationSpec):
            raise InventoryError(
                "control_station must be a NavigationStationSpec."
            )
        return control_station

    @staticmethod
    def validate_station_bounds(
        station: NavigationStation,
        *,
        boundary: Optional[Any] = None,
        margin: float = 0.0,
    ) -> NavigationStation:
        """Validate a precomputed station against a live map boundary."""
        if not isinstance(station, NavigationStation):
            raise InventoryError("station must be a NavigationStation.")
        if not all(
            _is_finite_number(value)
            for value in (station.x, station.y, station.z, station.yaw)
        ):
            raise InventoryError(
                "Navigation station must contain only finite coordinates."
            )
        if boundary is not None and not _boundary_contains(
            boundary,
            station.x,
            station.y,
            margin,
        ):
            raise NavigationStationOutOfBoundsError(
                station.cabinet,
                station.x,
                station.y,
            )
        return station

    def station_for(
        self,
        name: str,
        *,
        control_station: Optional[NavigationStationSpec] = None,
        boundary: Optional[Any] = None,
        margin: float = 0.0,
    ) -> NavigationStation:
        """Compute a station and optionally validate it against a live map.

        ``control_station`` is a full robot-adapter override for the selected
        control.  It replaces the common/per-instance station geometry while
        retaining the cabinet instance's world transform.

        ``boundary`` may be a ``MapBounds``/object with a
        ``contains(x, y, margin=...)`` method, or a simple ``(x, y) -> bool``
        predicate.  This keeps occupancy-grid policy in the ROS adapter.
        """
        instance = self.get(name)
        spec = self.station_spec_for(
            name,
            control_station=control_station,
        )
        rotation = _rpy_rotation(instance.roll, instance.pitch, instance.yaw)
        local_position = tuple(
            spec.local_anchor[index] + spec.outward_axis[index] * spec.standoff
            for index in range(3)
        )
        rotated_position = _rotate(rotation, local_position)
        world_axis = _rotate(rotation, spec.outward_axis)
        horizontal_norm = math.hypot(world_axis[0], world_axis[1])
        if horizontal_norm <= 1.0e-9:
            raise InventoryError(
                f"Navigation outward axis for {name} has no horizontal component."
            )
        # The base faces back along the outward normal, toward the cabinet.
        yaw = _normalize_angle(
            math.atan2(-world_axis[1], -world_axis[0])
            + spec.base_yaw_offset
        )
        station = NavigationStation(
            cabinet=instance.name,
            frame_id=spec.frame_id,
            x=instance.x + rotated_position[0],
            y=instance.y + rotated_position[1],
            z=instance.z + rotated_position[2],
            yaw=yaw,
        )
        return self.validate_station_bounds(
            station,
            boundary=boundary,
            margin=margin,
        )


def load_inventory(
    instances_path: str | Path,
    scene_path: str | Path,
) -> CabinetInventory:
    """Convenience wrapper used by command-line entry points."""
    return CabinetInventory.load(instances_path, scene_path)


def _load_yaml(path_value: str | Path) -> Mapping[str, Any]:
    path = Path(path_value).expanduser()
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise InventoryError(f"Cannot read YAML file {path}: {error}") from error
    except yaml.YAMLError as error:
        raise InventoryError(f"Invalid YAML file {path}: {error}") from error
    if not isinstance(document, Mapping):
        raise InventoryError(f"YAML file {path} must contain a mapping at its root.")
    return document


def _parse_instances(document: Mapping[str, Any]) -> list[CabinetInstance]:
    unknown_root = set(document) - {"instances"}
    if unknown_root:
        raise InventoryError(
            "cabinet_instances.yaml has unknown root fields: "
            + ", ".join(sorted(str(field) for field in unknown_root))
        )
    values = document.get("instances")
    if not isinstance(values, list) or not values:
        raise InventoryError("instances must be a non-empty list.")
    instances = []
    names = set()
    allowed = {
        "name",
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
        "navigation_station",
    }
    for index, value in enumerate(values):
        context = f"instances[{index}]"
        if not isinstance(value, Mapping):
            raise InventoryError(f"{context} must be a mapping.")
        unknown = set(value) - allowed
        if unknown:
            raise InventoryError(
                f"{context} has unknown fields: "
                + ", ".join(sorted(str(field) for field in unknown))
            )
        name = value.get("name")
        if not isinstance(name, str) or not _CABINET_NAME_PATTERN.fullmatch(name):
            raise InventoryError(
                f"{context}.name must match {_CABINET_NAME_PATTERN.pattern}."
            )
        if name in names:
            raise InventoryError(f"Duplicate cabinet name: {name}")
        names.add(name)
        numeric: Dict[str, float] = {}
        for field in ("x", "y", "z", "roll", "yaw"):
            if field not in value:
                raise InventoryError(f"{context}.{field} is required.")
            numeric[field] = _finite_number(value[field], f"{context}.{field}")
        numeric["pitch"] = _finite_number(
            value.get("pitch", 0.0),
            f"{context}.pitch",
        )
        station_override = value.get("navigation_station")
        if station_override is not None and not isinstance(
            station_override, Mapping
        ):
            raise InventoryError(f"{context}.navigation_station must be a mapping.")
        instances.append(
            CabinetInstance(
                name=name,
                station_override=(
                    dict(station_override) if station_override is not None else None
                ),
                **numeric,
            )
        )
    return instances


def _find_navigation_station(document: Mapping[str, Any]) -> Mapping[str, Any]:
    direct = document.get("navigation_station")
    if direct is not None:
        if not isinstance(direct, Mapping):
            raise InventoryError("navigation_station must be a mapping.")
        return direct
    direct_parameters = document.get("ros__parameters")
    if isinstance(direct_parameters, Mapping):
        station = direct_parameters.get("navigation_station")
        if station is not None:
            if not isinstance(station, Mapping):
                raise InventoryError("navigation_station must be a mapping.")
            return station
    matches = []
    for node_name, node_config in document.items():
        if not isinstance(node_config, Mapping):
            continue
        parameters = node_config.get("ros__parameters")
        if not isinstance(parameters, Mapping):
            continue
        station = parameters.get("navigation_station")
        if station is not None:
            if not isinstance(station, Mapping):
                raise InventoryError(
                    f"{node_name}.ros__parameters.navigation_station "
                    "must be a mapping."
                )
            matches.append(station)
    if not matches:
        raise InventoryError(
            "cabinet_scene.yaml does not define ros__parameters.navigation_station."
        )
    if len(matches) > 1:
        raise InventoryError(
            "cabinet_scene.yaml defines navigation_station for multiple nodes."
        )
    return matches[0]


def _find_pose_valid_topic(document: Mapping[str, Any]) -> str:
    """Find the pose-validity topic beside the scene navigation contract."""
    parameter_candidates = []
    direct_topic = document.get("pose_valid_topic")
    direct_parameters = document.get("ros__parameters")
    if isinstance(direct_parameters, Mapping):
        parameter_candidates.append(direct_parameters)
    for node_config in document.values():
        if not isinstance(node_config, Mapping):
            continue
        parameters = node_config.get("ros__parameters")
        if (
            isinstance(parameters, Mapping)
            and "navigation_station" in parameters
        ):
            parameter_candidates.append(parameters)

    raw_values = ([] if direct_topic is None else [direct_topic]) + [
        parameters["pose_valid_topic"]
        for parameters in parameter_candidates
        if "pose_valid_topic" in parameters
    ]
    if any(not isinstance(value, str) for value in raw_values):
        raise InventoryError("pose_valid_topic must be a string.")
    values = {value.strip() for value in raw_values}
    if not values:
        # Compatibility for older standalone profiles.
        return "pose_valid"
    if len(values) != 1:
        raise InventoryError(
            "cabinet_scene.yaml defines conflicting pose_valid_topic values."
        )
    value = next(iter(values))
    if _RELATIVE_TOPIC_PATTERN.fullmatch(value) is None:
        raise InventoryError(
            "pose_valid_topic must be a non-empty relative ROS topic."
        )
    return value


def _is_finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _finite_number(value: Any, label: str) -> float:
    if not _is_finite_number(value):
        raise InventoryError(f"{label} must be a finite number.")
    return float(value)


def _vector3(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise InventoryError(f"{label} must contain exactly three numbers.")
    if len(value) != 3:
        raise InventoryError(f"{label} must contain exactly three numbers.")
    return tuple(
        _finite_number(component, f"{label}[{index}]")
        for index, component in enumerate(value)
    )  # type: ignore[return-value]


def _rpy_rotation(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[tuple[float, float, float], ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Standard ROS fixed-axis roll/pitch/yaw: Rz(yaw) * Ry(pitch) * Rx(roll).
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _rotate(
    matrix: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        sum(row[index] * vector[index] for index in range(3))
        for row in matrix
    )  # type: ignore[return-value]


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _boundary_contains(
    boundary: Any,
    x: float,
    y: float,
    margin: float,
) -> bool:
    margin = _finite_number(margin, "map margin")
    if margin < 0.0:
        raise InventoryError("Map margin must not be negative.")
    contains = getattr(boundary, "contains", None)
    if callable(contains):
        try:
            result = contains(x, y, margin=margin)
        except TypeError:
            if margin != 0.0:
                raise InventoryError(
                    "Custom map boundary does not accept a margin."
                ) from None
            result = contains(x, y)
    elif callable(boundary):
        if margin != 0.0:
            raise InventoryError(
                "A map predicate cannot be used with a non-zero margin."
            )
        result = boundary(x, y)
    else:
        raise InventoryError(
            "Map boundary must be callable or expose contains(x, y, margin=...)."
        )
    if not isinstance(result, bool):
        raise InventoryError("Map boundary result must be a boolean.")
    return result
