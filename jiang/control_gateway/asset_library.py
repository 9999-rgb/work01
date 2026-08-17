"""Asset library: import, catalog and selection persistence.

The library is the on-disk home of imported scene/cabinet assets.  Import
copies an asset directory into ``<root>/<kind>/<name>/``, normalizes
self-contained references (relative ``nav2_map`` / ``model.file`` paths are
rewritten to absolute paths inside the library), records the entry in
``assets_catalog.yaml``, and — when a semantic validator is supplied — runs it
before persisting.

Selection is the "what runs now" state persisted in ``selection.yaml``.  The
selection layer never re-implements loading: :meth:`AssetLibrary.selection_to_env`
maps the selected assets onto the existing ``CABINET_*_PATH`` / ``SCENES_CONFIG``
/ ``SCENE`` environment pointers, which the launch and gateway already consume.
Semantic validation of the files themselves stays in the existing checkers
(``profile_contract`` / ``check_scene_config``), invoked through the injectable
``validate`` hook so tests and Web calls can substitute it.
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional

import yaml

from .asset_manifest import AssetManifest, load_manifest

CATALOG_FILENAME = "assets_catalog.yaml"
SELECTION_FILENAME = "selection.yaml"
MANIFEST_FILENAME = "manifest.yaml"

#: Directories never copied into the library.
_SKIP_PARTS = frozenset({".git", ".svn", "__pycache__", ".hg"})
_URI_PREFIXES = ("package://", "model://", "file://")
_REFERENCE_FIELDS = ("cabinet", "gripper_variant")


class AssetLibraryError(ValueError):
    """Base error for asset library operations."""


class AssetNotFoundError(AssetLibraryError):
    """Raised when a requested asset is absent from the catalog."""

    def __init__(self, kind: str, name: str) -> None:
        super().__init__(f"Unknown {kind} asset: {name}")
        self.kind = kind
        self.name = name


class AssetExistsError(AssetLibraryError):
    """Raised when importing an asset that already exists (no ``force``)."""

    def __init__(self, kind: str, name: str, version: str) -> None:
        super().__init__(
            f"{kind} asset {name!r} is already imported "
            f"(version {version}). Use force=True to replace it."
        )
        self.kind = kind
        self.name = name
        self.version = version


@dataclass(frozen=True)
class AssetRecord:
    """One entry in the asset catalog."""

    kind: str
    name: str
    version: str
    description: str
    path: str  # relative to the library root, e.g. "scene/my_scene"
    files: Mapping[str, str]
    references: Mapping[str, Optional[str]]
    imported_at: str
    validated: bool

    def to_dict(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "name": self.name,
            "version": self.version,
            "description": self.description,
            "path": self.path,
            "files": dict(self.files),
            "references": dict(self.references),
            "imported_at": self.imported_at,
            "validated": self.validated,
        }


@dataclass(frozen=True)
class AssetSelection:
    """The currently selected run composition.

    ``scene`` and ``cabinet`` name imported assets (by asset name).  For a scene
    asset the asset name equals the primary scene name inside its ``scenes.yaml``
    (enforced at import).  ``gripper_variant`` names one of the fixed robot
    gripper variants and is consumed by the launch layer.
    """

    scene: Optional[str] = None
    cabinet: Optional[str] = None
    gripper_variant: Optional[str] = None

    def to_dict(self) -> Dict[str, Optional[str]]:
        return {
            "scene": self.scene,
            "cabinet": self.cabinet,
            "gripper_variant": self.gripper_variant,
        }


class AssetLibrary:
    """Filesystem-backed asset library with catalog and selection state."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser()

    # ── catalog ─────────────────────────────────────────────────────────

    def catalog_path(self) -> Path:
        return self.root / CATALOG_FILENAME

    def load_catalog(self) -> List[AssetRecord]:
        """Return all imported assets, oldest first."""
        path = self.catalog_path()
        if not path.is_file():
            return []
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            raise AssetLibraryError(
                f"Cannot read asset catalog {path}: {error}"
            ) from error
        if not isinstance(document, Mapping):
            raise AssetLibraryError(
                f"Asset catalog {path} must contain a mapping at its root."
            )
        values = document.get("assets", [])
        if not isinstance(values, list):
            raise AssetLibraryError(f"Asset catalog {path}.assets must be a list.")
        records: List[AssetRecord] = []
        for value in values:
            if not isinstance(value, Mapping):
                raise AssetLibraryError(
                    f"Asset catalog {path} contains a non-mapping entry."
                )
            try:
                records.append(_record_from_mapping(value))
            except (TypeError, KeyError, ValueError) as error:
                raise AssetLibraryError(
                    f"Asset catalog {path} has an invalid entry: {error}"
                ) from error
        return records

    def find(self, kind: str, name: str) -> AssetRecord:
        for record in self.load_catalog():
            if record.kind == kind and record.name == name:
                return record
        raise AssetNotFoundError(kind, name)

    def asset_root(self, record: AssetRecord) -> Path:
        return self.root / record.path

    def asset_manifest(self, record: AssetRecord) -> AssetManifest:
        return load_manifest(self.asset_root(record) / MANIFEST_FILENAME)

    def _save_catalog(self, records: List[AssetRecord]) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        path = self.catalog_path()
        path.write_text(
            yaml.safe_dump(
                {"assets": [record.to_dict() for record in records]},
                sort_keys=False,
                allow_unicode=True,
            ),
            encoding="utf-8",
        )

    # ── import ──────────────────────────────────────────────────────────

    def import_asset(
        self,
        source_dir: str | Path,
        *,
        validate: Optional[Callable[[AssetManifest, Path], None]] = None,
        force: bool = False,
    ) -> AssetRecord:
        """Import an asset directory into the library.

        ``validate`` is an injectable semantic checker ``(manifest, copied_root)
        -> None`` that raises on failure; when omitted only the manifest schema
        (and declared-file existence) is checked.  ``force=True`` overwrites an
        existing entry of the same kind+name.
        """
        source = Path(source_dir).expanduser()
        manifest_path = source / MANIFEST_FILENAME
        if not manifest_path.is_file():
            raise AssetLibraryError(
                f"{source} is not an asset: missing {MANIFEST_FILENAME}."
            )
        manifest = load_manifest(manifest_path)
        existing = [
            record
            for record in self.load_catalog()
            if record.kind == manifest.kind and record.name == manifest.name
        ]
        if existing and not force:
            record = existing[0]
            raise AssetExistsError(record.kind, record.name, record.version)

        destination = self.root / manifest.kind / manifest.name
        try:
            _copy_asset_tree(source, destination)
            _normalize_scene_refs(manifest, destination)

            validated = False
            if validate is not None:
                validate(manifest, destination)
                validated = True
        except Exception as error:
            # Never leave a partially imported directory behind: a failed
            # import must be indistinguishable from no import at all.
            shutil.rmtree(destination, ignore_errors=True)
            if isinstance(error, AssetLibraryError):
                raise
            raise AssetLibraryError(
                f"Asset {manifest.kind}/{manifest.name} failed validation: "
                f"{error}"
            ) from error

        record = AssetRecord(
            kind=manifest.kind,
            name=manifest.name,
            version=manifest.version,
            description=manifest.description,
            path=manifest.kind + "/" + manifest.name,
            files=dict(manifest.files),
            references=manifest.references.to_dict(),
            imported_at=_utc_now(),
            validated=validated,
        )
        remaining = [
            entry for entry in self.load_catalog()
            if not (entry.kind == record.kind and entry.name == record.name)
        ]
        self._save_catalog(remaining + [record])
        return record

    def remove_asset(self, kind: str, name: str) -> AssetRecord:
        """Delete an asset and clear it from the selection if selected.

        Raises :class:`AssetNotFoundError` when the asset is not in the catalog.
        The directory is removed and the catalog entry dropped in one step; the
        selection's matching field (``scene`` / ``cabinet``) is reset to ``None``
        so a later ``selection_to_env`` does not point at a deleted asset.
        """
        record = self.find(kind, name)
        shutil.rmtree(self.asset_root(record), ignore_errors=True)
        remaining = [
            entry for entry in self.load_catalog()
            if not (entry.kind == kind and entry.name == name)
        ]
        self._save_catalog(remaining)

        selection = self.load_selection()
        changed = False
        if kind == "scene" and selection.scene == name:
            selection = AssetSelection(
                scene=None,
                cabinet=selection.cabinet,
                gripper_variant=selection.gripper_variant,
            )
            changed = True
        elif kind == "cabinet" and selection.cabinet == name:
            selection = AssetSelection(
                scene=selection.scene,
                cabinet=None,
                gripper_variant=selection.gripper_variant,
            )
            changed = True
        if changed:
            self.save_selection(selection)
        return record

    # ── selection ───────────────────────────────────────────────────────

    def selection_path(self) -> Path:
        return self.root / SELECTION_FILENAME

    def load_selection(self) -> AssetSelection:
        path = self.selection_path()
        if not path.is_file():
            return AssetSelection()
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            raise AssetLibraryError(
                f"Cannot read selection {path}: {error}"
            ) from error
        if document is None:
            return AssetSelection()
        if not isinstance(document, Mapping):
            raise AssetLibraryError(
                f"Selection {path} must contain a mapping at its root."
            )
        return AssetSelection(
            scene=_optional_selection_name(document.get("scene"), "scene", path),
            cabinet=_optional_selection_name(
                document.get("cabinet"), "cabinet", path
            ),
            gripper_variant=_optional_selection_name(
                document.get("gripper_variant"), "gripper_variant", path
            ),
        )

    def save_selection(self, selection: AssetSelection) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        path = self.selection_path()
        path.write_text(
            yaml.safe_dump(
                selection.to_dict(), sort_keys=False, allow_unicode=True
            ),
            encoding="utf-8",
        )

    def selection_to_env(
        self, selection: Optional[AssetSelection] = None
    ) -> Dict[str, str]:
        """Map a selection onto the existing env-var / launch pointers.

        Only the fields that are set produce an override, so the caller can
        merge the result over its current environment without losing values the
        user set explicitly on the command line.
        """
        selection = selection if selection is not None else self.load_selection()
        env: Dict[str, str] = {}
        if selection.scene:
            env.update(self._scene_env(selection.scene))
        if selection.cabinet:
            env.update(self._cabinet_env(selection.cabinet))
        if selection.gripper_variant:
            env["GRIPPER_VARIANT"] = selection.gripper_variant
        return env

    def _scene_env(self, name: str) -> Dict[str, str]:
        record = self.find("scene", name)
        manifest = self.asset_manifest(record)
        root = self.asset_root(record)
        env = {
            "SCENES_CONFIG": str(manifest.file_path(root, "scenes")),
            "SCENE": name,
        }
        if "instances" in manifest.files:
            env["CABINET_INSTANCES_PATH"] = str(
                manifest.file_path(root, "instances")
            )
        return env

    def _cabinet_env(self, name: str) -> Dict[str, str]:
        record = self.find("cabinet", name)
        manifest = self.asset_manifest(record)
        root = self.asset_root(record)
        return {
            "CABINET_CONTROLS_PATH": str(manifest.file_path(root, "controls")),
            "CABINET_SCENE_PATH": str(manifest.file_path(root, "scene")),
            "CABINET_POSE_PATH": str(manifest.file_path(root, "pose")),
            "CABINET_ROBOT_ADAPTER_PATH": str(
                manifest.file_path(root, "adapter")
            ),
            "CABINET_XACRO_PATH": str(manifest.file_path(root, "xacro")),
        }


def default_library_root() -> Path:
    """Return the default library root (``jiang/data/assets``)."""
    return Path(__file__).resolve().parents[1] / "data" / "assets"


# ── internal helpers ──────────────────────────────────────────────────────


def _copy_asset_tree(source: Path, destination: Path) -> None:
    """Copy an asset directory wholesale (minus VCS / cache dirs).

    The whole tree is copied, not just the roles declared in ``manifest.files``:
    supporting data referenced by the scenes / controls files (Nav2 maps, PGM,
    meshes, SDF models) lives outside the role mapping and must travel with the
    asset.  The declared roles stay the contract the selection layer consumes.
    """
    destination.mkdir(parents=True, exist_ok=True)
    for entry in source.rglob("*"):
        relative = entry.relative_to(source)
        if any(part in _SKIP_PARTS for part in relative.parts):
            continue
        if entry.is_dir():
            (destination / relative).mkdir(parents=True, exist_ok=True)
        elif entry.is_file():
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(entry, target)


def _normalize_scene_refs(manifest: AssetManifest, root: Path) -> None:
    """Rewrite relative ``nav2_map`` / ``model.file`` refs to absolute paths.

    Only applies to scene assets.  ``package://`` / ``model://`` / ``file://``
    references are left untouched because they are resolved by the ROS package
    index or Gazebo's model:// mechanism; a plain relative or absolute path is
    made absolute inside the asset root so the existing launch/gateway
    resolution (which expects an absolute path or a package URI) keeps working.
    """
    if manifest.kind != "scene":
        return
    scenes_path = manifest.file_path(root, "scenes")
    try:
        document = yaml.safe_load(scenes_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise AssetLibraryError(
            f"Scene asset {manifest.name}: cannot read scenes.yaml: {error}"
        ) from error
    if not isinstance(document, Mapping) or not isinstance(
        document.get("scenes"), list
    ):
        raise AssetLibraryError(
            f"Scene asset {manifest.name}: scenes.yaml must contain a scenes list."
        )
    names = []
    for scene in document["scenes"]:
        if not isinstance(scene, Mapping):
            raise AssetLibraryError(
                f"Scene asset {manifest.name}: scenes entry must be a mapping."
            )
        scene_name = scene.get("name")
        if not isinstance(scene_name, str):
            raise AssetLibraryError(
                f"Scene asset {manifest.name}: scene name must be a string."
            )
        names.append(scene_name)
        nav2_map = scene.get("nav2_map")
        if isinstance(nav2_map, str) and nav2_map.strip():
            scene["nav2_map"] = _absolute_reference(root, nav2_map.strip())
        model = scene.get("model")
        if isinstance(model, Mapping):
            file = model.get("file")
            if isinstance(file, str) and file.strip():
                model["file"] = _absolute_reference(root, file.strip())
    if manifest.name not in names:
        raise AssetLibraryError(
            f"Scene asset {manifest.name}: scenes.yaml must contain a scene "
            f"named exactly like the asset (found {names!r})."
        )
    scenes_path.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _absolute_reference(root: Path, reference: str) -> str:
    if reference.startswith(_URI_PREFIXES):
        return reference
    path = Path(reference).expanduser()
    if path.is_absolute():
        return str(path)
    return str((root / path).resolve())


def _record_from_mapping(value: Mapping[str, Any]) -> AssetRecord:
    kind = value["kind"]
    name = value["name"]
    files = value["files"]
    if not isinstance(files, Mapping):
        raise ValueError(f"asset {kind}/{name}: files must be a mapping")
    references = value.get("references", {})
    if not isinstance(references, Mapping):
        raise ValueError(f"asset {kind}/{name}: references must be a mapping")
    return AssetRecord(
        kind=kind,
        name=name,
        version=value["version"],
        description=value.get("description", ""),
        path=value["path"],
        files={str(role): str(relative) for role, relative in files.items()},
        references={
            str(field): references.get(field) for field in _REFERENCE_FIELDS
        },
        imported_at=value.get("imported_at", ""),
        validated=bool(value.get("validated", False)),
    )


def _optional_selection_name(value: Any, label: str, path: Path) -> Optional[str]:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise AssetLibraryError(
            f"Selection {path}: {label} must be a non-empty string or null."
        )
    return value.strip()


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()
