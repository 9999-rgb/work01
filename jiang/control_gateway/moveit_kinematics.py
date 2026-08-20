"""Validate that configured MoveIt kinematics plugins are discoverable.

MoveIt loads solver classes through pluginlib.  A syntactically valid
``kinematics.yaml`` can therefore start ``move_group`` without either arm
having an IK solver when the selected plugin package is absent.  This module
reads the same ament plugin-resource index used by pluginlib so startup can
fail before advertising a falsely ready control stack.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Iterable, Mapping, Tuple
import xml.etree.ElementTree as ET

import yaml


_PLUGIN_RESOURCE_TYPE = "moveit_core__pluginlib__plugin"


class KinematicsConfigError(ValueError):
    """Raised when a kinematics configuration cannot be used at runtime."""


@dataclass(frozen=True)
class KinematicsPluginReport:
    """Validated MoveIt group-to-solver assignments."""

    group_solvers: Tuple[Tuple[str, str], ...]
    available_classes: Tuple[str, ...]


def _nonempty_string(value: object, *, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise KinematicsConfigError(f"{context} must be a non-empty string.")
    return value.strip()


def configured_solver_classes(path: str | Path) -> Tuple[Tuple[str, str], ...]:
    """Return all configured ``(group, plugin class)`` pairs."""

    config_path = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise KinematicsConfigError(
            f"Could not read MoveIt kinematics config '{config_path}': {error}"
        ) from error
    if not isinstance(document, Mapping) or not document:
        raise KinematicsConfigError(
            f"MoveIt kinematics config '{config_path}' must be a non-empty mapping."
        )

    assignments = []
    for raw_group, raw_parameters in document.items():
        group = _nonempty_string(raw_group, context="MoveIt kinematics group")
        if not isinstance(raw_parameters, Mapping):
            raise KinematicsConfigError(
                f"MoveIt kinematics group '{group}' must be a mapping."
            )
        solver = _nonempty_string(
            raw_parameters.get("kinematics_solver"),
            context=f"{group}.kinematics_solver",
        )
        assignments.append((group, solver))
    return tuple(sorted(assignments))


def _ament_prefixes(prefixes: Iterable[str | Path] | None) -> Tuple[Path, ...]:
    raw_prefixes: Iterable[str | Path]
    if prefixes is None:
        raw_prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    else:
        raw_prefixes = prefixes
    result = []
    seen = set()
    for raw_prefix in raw_prefixes:
        value = str(raw_prefix).strip()
        if not value:
            continue
        prefix = Path(value).expanduser().resolve()
        if prefix not in seen:
            seen.add(prefix)
            result.append(prefix)
    return tuple(result)


def discover_moveit_kinematics_plugins(
    prefixes: Iterable[str | Path] | None = None,
) -> Tuple[str, ...]:
    """Discover ``kinematics::KinematicsBase`` classes from ament indexes."""

    classes = set()
    for prefix in _ament_prefixes(prefixes):
        resource_directory = (
            prefix
            / "share"
            / "ament_index"
            / "resource_index"
            / _PLUGIN_RESOURCE_TYPE
        )
        if not resource_directory.is_dir():
            continue
        for resource in sorted(resource_directory.iterdir()):
            if not resource.is_file():
                continue
            try:
                descriptions = resource.read_text(encoding="utf-8").splitlines()
            except OSError:
                continue
            for raw_description in descriptions:
                value = raw_description.strip()
                if not value:
                    continue
                description = Path(value)
                if not description.is_absolute():
                    description = prefix / description
                try:
                    root = ET.parse(description).getroot()
                except (OSError, ET.ParseError):
                    continue
                for plugin_class in root.iter("class"):
                    if plugin_class.get("base_class_type") != (
                        "kinematics::KinematicsBase"
                    ):
                        continue
                    name = plugin_class.get("name", "").strip()
                    if name:
                        classes.add(name)
    return tuple(sorted(classes))


def validate_kinematics_plugins(
    path: str | Path,
    *,
    prefixes: Iterable[str | Path] | None = None,
) -> KinematicsPluginReport:
    """Require every solver in ``path`` to be declared in the ament index."""

    assignments = configured_solver_classes(path)
    available = discover_moveit_kinematics_plugins(prefixes)
    missing = sorted(
        {solver for _, solver in assignments} - set(available)
    )
    if missing:
        searched = _ament_prefixes(prefixes)
        search_summary = ", ".join(str(prefix) for prefix in searched) or (
            "AMENT_PREFIX_PATH is empty"
        )
        raise KinematicsConfigError(
            "MoveIt kinematics plugin class(es) are not discoverable: "
            + ", ".join(missing)
            + f" (searched: {search_summary})."
        )
    return KinematicsPluginReport(assignments, available)
