"""Validated asset manifests for importable scene and cabinet packages.

A manifest is the single entry point of a self-contained asset directory.  It
declares the asset kind (``scene`` / ``cabinet``), a unique ``name``, a
``version``, and the role -> relative-path mapping for the files bundled in the
directory.  The module is deliberately free of ROS imports so launch tooling,
unit tests and the Web gateway can all validate a manifest without sourcing a
workspace, mirroring :mod:`control_gateway.scene_catalog`.

An imported asset is *data*: the gateway never interprets its contents beyond
the schema below.  Cross-file semantic validation (cabinet profile contract,
scene map existence) is delegated to ``profile_contract.validate_profile`` and
``scripts/check_scene_config`` at import time, not re-implemented here.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import yaml

_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
# Strict semantic version: no leading zeros (matching common semver tooling).
_VERSION_PATTERN = re.compile(
    r"^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$"
)
_ASSET_KINDS = frozenset({"scene", "cabinet"})

#: File roles allowed per asset kind.  Adding a role here does not change any
#: loader: roles are consumed by the asset-library selection layer, which maps
#: each role to the existing ``CABINET_*_PATH`` / ``SCENES_CONFIG`` pointers.
_SCENE_FILE_ROLES = frozenset({"scenes", "instances"})
_CABINET_FILE_ROLES = frozenset(
    {"controls", "scene", "pose", "adapter", "xacro", "instances"}
)
_FILE_ROLES_BY_KIND: Dict[str, frozenset] = {
    "scene": _SCENE_FILE_ROLES,
    "cabinet": _CABINET_FILE_ROLES,
}
#: Roles that must be present for an asset of a given kind.
_REQUIRED_ROLES_BY_KIND: Dict[str, frozenset] = {
    "scene": frozenset({"scenes"}),
    "cabinet": frozenset({"controls", "scene", "pose", "adapter", "xacro"}),
}

_REFERENCE_FIELDS = frozenset({"cabinet"})


class ManifestError(ValueError):
    """Base error for an invalid or unreadable asset manifest."""


@dataclass(frozen=True)
class AssetReferences:
    """Asset-level composition references.

    ``cabinet`` is optional.  A scene asset may pin the cabinet asset it was
    calibrated for; when the reference is absent the selection layer falls back
    to the built-in cabinet, so a scene-only deployment does not need to name
    it.
    """

    cabinet: Optional[str] = None

    def to_dict(self) -> Dict[str, Optional[str]]:
        return {"cabinet": self.cabinet}


@dataclass(frozen=True)
class AssetManifest:
    """One validated asset manifest.

    ``files`` maps a role (e.g. ``scenes`` for a scene asset, ``controls`` for
    a cabinet asset) to a path relative to the asset directory.  All paths are
    validated to stay inside the asset directory (no absolute paths, no ``..``
    segments) so the library can copy and reference them safely.
    """

    kind: str
    name: str
    version: str
    description: str
    files: Mapping[str, str]
    references: AssetReferences

    def file_path(self, root: str | Path, role: str) -> Path:
        """Return the absolute path of a role inside an asset root."""
        if role not in self.files:
            raise ManifestError(
                f"Asset {self.kind}/{self.name} has no file role {role!r}."
            )
        return Path(root).expanduser() / self.files[role]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "name": self.name,
            "version": self.version,
            "description": self.description,
            "files": dict(self.files),
            "references": self.references.to_dict(),
        }


def load_manifest(path: str | Path) -> AssetManifest:
    """Load and strictly validate a ``manifest.yaml`` at ``path``."""
    manifest_path = Path(path).expanduser()
    document = _load_yaml(manifest_path)
    kind = _required_string(document, "kind", manifest_path)
    if kind not in _ASSET_KINDS:
        raise ManifestError(
            f"{manifest_path}: kind must be one of "
            + ", ".join(sorted(_ASSET_KINDS))
            + f", got {kind!r}."
        )
    name = _required_string(document, "name", manifest_path)
    if _NAME_PATTERN.fullmatch(name) is None:
        raise ManifestError(
            f"{manifest_path}: name must match {_NAME_PATTERN.pattern}, "
            f"got {name!r}."
        )
    version = _required_string(document, "version", manifest_path)
    if _VERSION_PATTERN.fullmatch(version) is None:
        raise ManifestError(
            f"{manifest_path}: version must match {_VERSION_PATTERN.pattern} "
            f"(semantic version), got {version!r}."
        )
    description = document.get("description", "")
    if not isinstance(description, str):
        raise ManifestError(f"{manifest_path}: description must be a string.")
    files = _parse_files(document.get("files"), manifest_path, kind)
    references = _parse_references(
        document.get("references"), manifest_path
    )
    manifest = AssetManifest(
        kind=kind,
        name=name,
        version=version,
        description=description,
        files=files,
        references=references,
    )
    _validate_against_root(manifest, manifest_path.parent)
    return manifest


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise ManifestError(f"Cannot read manifest {path}: {error}") from error
    except yaml.YAMLError as error:
        raise ManifestError(f"Invalid manifest YAML {path}: {error}") from error
    if not isinstance(document, Mapping):
        raise ManifestError(f"Manifest {path} must contain a mapping at its root.")
    return document


def _required_string(
    document: Mapping[str, Any], field: str, path: Path
) -> str:
    value = document.get(field)
    if not isinstance(value, str) or not value.strip():
        raise ManifestError(f"{path}: {field} must be a non-empty string.")
    return value.strip()


def _parse_files(value: Any, path: Path, kind: str) -> Mapping[str, str]:
    if not isinstance(value, Mapping):
        raise ManifestError(f"{path}: files must be a mapping.")
    allowed = _FILE_ROLES_BY_KIND[kind]
    unknown = set(value) - allowed
    if unknown:
        raise ManifestError(
            f"{path}: files has unknown roles for kind {kind!r}: "
            + ", ".join(sorted(str(role) for role in unknown))
        )
    files: Dict[str, str] = {}
    for role in allowed:
        raw = value.get(role)
        if raw is None:
            continue
        if not isinstance(raw, str) or not raw.strip():
            raise ManifestError(f"{path}: files.{role} must be a non-empty string.")
        relative = Path(raw.strip())
        if relative.is_absolute() or ".." in relative.parts:
            raise ManifestError(
                f"{path}: files.{role} must be a relative path inside the "
                f"asset directory, got {raw.strip()!r}."
            )
        files[role] = relative.as_posix()
    missing = _REQUIRED_ROLES_BY_KIND[kind] - set(files)
    if missing:
        raise ManifestError(
            f"{path}: kind {kind!r} requires file roles: "
            + ", ".join(sorted(missing))
        )
    return files


def _parse_references(value: Any, path: Path) -> AssetReferences:
    if value is None:
        return AssetReferences()
    if not isinstance(value, Mapping):
        raise ManifestError(f"{path}: references must be a mapping or null.")
    unknown = set(value) - _REFERENCE_FIELDS
    if unknown:
        raise ManifestError(
            f"{path}: references has unknown fields: "
            + ", ".join(sorted(str(field) for field in unknown))
        )
    cabinet = _optional_name(value.get("cabinet"), "references.cabinet", path)
    return AssetReferences(cabinet=cabinet)


def _optional_name(value: Any, label: str, path: Path) -> Optional[str]:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ManifestError(f"{path}: {label} must be a non-empty string or null.")
    name = value.strip()
    if _NAME_PATTERN.fullmatch(name) is None:
        raise ManifestError(
            f"{path}: {label} must match {_NAME_PATTERN.pattern}, got {name!r}."
        )
    return name


def _validate_against_root(manifest: AssetManifest, root: Path) -> None:
    """Every declared file must actually exist inside the asset root."""
    for role, relative in manifest.files.items():
        target = root / relative
        if not target.is_file():
            raise ManifestError(
                f"{root}: declared file {role!r} ({relative}) does not exist."
            )
