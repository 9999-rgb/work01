"""Asset library: import, catalog and selection persistence.

The library is the on-disk home of imported scene/cabinet assets.  Import
copies an asset directory into ``<root>/<kind>/<name>/``, normalizes
self-contained references (relative ``nav2_map`` / ``model.file`` paths are
rewritten to absolute paths inside the library), records the entry in the
catalog, and — when a semantic validator is supplied — runs it before
persisting.

Selection is the "what runs now" state.  The selection layer never
re-implements loading: :meth:`AssetLibrary.selection_to_env` maps the selected
assets onto the existing ``CABINET_*_PATH`` / ``SCENES_CONFIG`` / ``SCENE``
environment pointers, which the launch and gateway already consume.  Semantic
validation of the files themselves stays in the existing checkers
(``profile_contract`` / ``check_scene_config``), invoked through the injectable
``validate`` hook so tests and Web calls can substitute it.

Catalog and selection are persisted through an injectable :class:`AssetStore`.
The reference implementation is :class:`YamlAssetStore` (the original
``assets_catalog.yaml`` / ``selection.yaml`` files, kept for the pure-library
unit tests); production deployments inject a SQLite-backed store
(``app.assets.store.SqlAssetStore``) that reuses the app's SQLAlchemy stack and
stores the same records in ``assets`` / ``selection`` tables.
"""

from __future__ import annotations

import os
import shutil
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Protocol

import yaml

from .asset_manifest import AssetManifest, load_manifest

CATALOG_FILENAME = "assets_catalog.yaml"
SELECTION_FILENAME = "selection.yaml"
MANIFEST_FILENAME = "manifest.yaml"

#: Directories never copied into the library.
_SKIP_PARTS = frozenset({".git", ".svn", "__pycache__", ".hg"})
_URI_PREFIXES = ("package://", "model://", "file://")
_REFERENCE_FIELDS = ("cabinet",)


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
    (enforced at import).  ``toolset`` selects the end-effector tool set
    (``"A"``: 三电缸右 + 两电缸左, ``"B"``: 旋转按钮右 + 摇入摇出左).  The
    persisted value is consumed by the next manual startup; it never changes
    the already loaded Gazebo robot model at runtime.
    """

    scene: Optional[str] = None
    cabinet: Optional[str] = None
    toolset: Optional[str] = None

    def to_dict(self) -> Dict[str, Optional[str]]:
        return {
            "scene": self.scene,
            "cabinet": self.cabinet,
            "toolset": self.toolset,
        }


class AssetStore(Protocol):
    """Persistence contract for the asset catalog and run selection.

    :class:`AssetLibrary` owns the filesystem side (copying/removing asset
    directories, manifest loading, selection -> env mapping) and delegates the
    catalog/selection *index* to this store.  ``put_asset`` replaces any
    existing entry of the same ``kind`` + ``name`` (upsert).
    """

    def list_assets(self) -> List[AssetRecord]:
        """Return all imported assets, oldest first."""

    def get_asset(self, kind: str, name: str) -> AssetRecord:
        """Return one entry, raising :class:`AssetNotFoundError` if absent."""

    def put_asset(self, record: AssetRecord) -> None:
        """Insert or replace the entry keyed by ``record.kind`` + ``record.name``."""

    def delete_asset(self, kind: str, name: str) -> None:
        """Remove the entry; no-op when absent."""

    def load_selection(self) -> AssetSelection:
        """Return the persisted selection (empty when unset)."""

    def save_selection(self, selection: AssetSelection) -> None:
        """Persist the selection."""


class YamlAssetStore:
    """Filesystem (YAML) catalog + selection store — reference implementation.

    Keeps the original ``assets_catalog.yaml`` / ``selection.yaml`` shape so the
    pure-library unit tests exercise the import/selection workflow without a
    database.  Production paths inject ``app.assets.store.SqlAssetStore``.
    """

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser()

    def catalog_path(self) -> Path:
        return self.root / CATALOG_FILENAME

    def selection_path(self) -> Path:
        return self.root / SELECTION_FILENAME

    def list_assets(self) -> List[AssetRecord]:
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

    def get_asset(self, kind: str, name: str) -> AssetRecord:
        for record in self.list_assets():
            if record.kind == kind and record.name == name:
                return record
        raise AssetNotFoundError(kind, name)

    def put_asset(self, record: AssetRecord) -> None:
        remaining = [
            entry
            for entry in self.list_assets()
            if not (entry.kind == record.kind and entry.name == record.name)
        ]
        self._save_catalog(remaining + [record])

    def delete_asset(self, kind: str, name: str) -> None:
        remaining = [
            entry
            for entry in self.list_assets()
            if not (entry.kind == kind and entry.name == name)
        ]
        self._save_catalog(remaining)

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
            toolset=_optional_selection_name(
                document.get("toolset"), "toolset", path
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


class AssetLibrary:
    """Filesystem-backed asset library with pluggable catalog/selection store.

    The asset directories always live under ``root/<kind>/<name>/``; the
    catalog and selection index is delegated to ``store`` (defaulting to
    :class:`YamlAssetStore`, overridden in production by a SQLite store).
    """

    def __init__(self, root: str | Path, store: Optional[AssetStore] = None) -> None:
        self.root = Path(root).expanduser()
        self._store: AssetStore = store if store is not None else YamlAssetStore(self.root)

    # ── catalog (delegated to the store) ────────────────────────────────

    def load_catalog(self) -> List[AssetRecord]:
        """Return all imported assets, oldest first."""
        return self._store.list_assets()

    def find(self, kind: str, name: str) -> AssetRecord:
        return self._store.get_asset(kind, name)

    def asset_root(self, record: AssetRecord) -> Path:
        try:
            relative = Path(record.path)
            expected_parts = (record.kind, record.name)
        except TypeError as error:
            raise AssetLibraryError(
                f"Asset catalog entry has an invalid path: {record.path!r}."
            ) from error
        if relative.is_absolute() or relative.parts != expected_parts:
            expected_path = f"{record.kind}/{record.name}"
            raise AssetLibraryError(
                f"Asset catalog entry {record.kind}/{record.name} has invalid "
                f"path {record.path!r}; expected "
                f"{expected_path!r}."
            )
        root = self.root.resolve()
        expected = root / relative
        try:
            candidate = expected.resolve()
            candidate.relative_to(root)
        except (OSError, RuntimeError, ValueError) as error:
            raise AssetLibraryError(
                f"Asset catalog entry {record.kind}/{record.name} path "
                f"{record.path!r} cannot be resolved safely: {error}."
            ) from error
        if candidate != expected:
            # Imported asset directories are real directories.  Reject a
            # canonical-looking catalog entry redirected by a symlink even if
            # its target remains inside the library: removal must never delete
            # the root itself or a different asset tree.
            raise AssetLibraryError(
                f"Asset catalog entry {record.kind}/{record.name} resolves "
                f"away from its canonical directory: {candidate}."
            )
        return candidate

    def asset_manifest(self, record: AssetRecord) -> AssetManifest:
        return load_manifest(self.asset_root(record) / MANIFEST_FILENAME)

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
        destination_parent = destination.parent
        destination_parent.mkdir(parents=True, exist_ok=True)
        if destination.exists() and not existing:
            # A catalog-less directory may be an interrupted legacy import or
            # an operator's manual data.  Never silently overlay or delete it.
            raise AssetLibraryError(
                f"Asset destination {destination} exists without a catalog entry. "
                "Resolve it explicitly before importing."
            )
        if existing and not destination.is_dir():
            raise AssetLibraryError(
                f"Asset catalog entry {manifest.kind}/{manifest.name} points to "
                f"a missing or non-directory destination: {destination}."
            )

        # Copy, reference normalization, and semantic validation occur in an
        # unreferenced sibling directory.  In particular, a failed --force
        # upload can no longer erase the previously working asset directory.
        staging = Path(
            tempfile.mkdtemp(
                prefix=f".{manifest.name}.staging-",
                dir=destination_parent,
            )
        )
        backup: Path | None = None
        promoted = False
        try:
            _copy_asset_tree(source, staging)
            # Semantic validation runs against the unreferenced staging tree,
            # so refs must first resolve inside staging (the final destination
            # is not promoted yet).  They are re-pointed at the stable
            # destination right before the atomic promotion below.
            _normalize_scene_refs(
                manifest,
                staging,
                reference_root=staging,
            )

            validated = False
            if validate is not None:
                validate(manifest, staging)
                validated = True
            _normalize_scene_refs(
                manifest,
                staging,
                reference_root=destination,
            )
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

            if destination.exists():
                backup = Path(
                    tempfile.mkdtemp(
                        prefix=f".{manifest.name}.backup-",
                        dir=destination_parent,
                    )
                )
                # ``mkdtemp`` creates an empty directory; replacing it with
                # the old asset yields a same-filesystem rename, so both the
                # old tree and staging tree remain recoverable until the store
                # commit has completed.
                backup.rmdir()
                os.replace(destination, backup)
            os.replace(staging, destination)
            promoted = True
            try:
                self._store.put_asset(record)
            except Exception as error:
                _restore_import_destination(destination, backup)
                promoted = False
                _restore_catalog_record(
                    self._store,
                    existing[0] if existing else None,
                    kind=manifest.kind,
                    name=manifest.name,
                )
                raise AssetLibraryError(
                    f"Asset {manifest.kind}/{manifest.name} could not be indexed: "
                    f"{error}"
                ) from error

            if backup is not None:
                shutil.rmtree(backup)
                backup = None
            return record
        except Exception as error:
            if promoted:
                _restore_import_destination(destination, backup)
                _restore_catalog_record(
                    self._store,
                    existing[0] if existing else None,
                    kind=manifest.kind,
                    name=manifest.name,
                )
            if isinstance(error, AssetLibraryError):
                raise
            raise AssetLibraryError(
                f"Asset {manifest.kind}/{manifest.name} failed validation: "
                f"{error}"
            ) from error
        finally:
            # Staging is either still an unvalidated copy or has been moved to
            # ``destination``.  This cleanup never targets the live asset.
            if staging.exists():
                shutil.rmtree(staging, ignore_errors=True)
            if backup is not None and backup.exists() and not destination.exists():
                # Best-effort last line of defence if an unexpected exception
                # interrupted rollback before the original tree was restored.
                os.replace(backup, destination)

    def remove_asset(self, kind: str, name: str) -> AssetRecord:
        """Delete an asset and clear it from the selection if selected.

        Raises :class:`AssetNotFoundError` when the asset is not in the catalog.
        The directory is removed and the catalog entry dropped in one step; the
        selection's matching field (``scene`` / ``cabinet``) is reset to ``None``
        so a later ``selection_to_env`` does not point at a deleted asset.
        """
        record = self.find(kind, name)
        shutil.rmtree(self.asset_root(record), ignore_errors=True)
        self._store.delete_asset(kind, name)

        selection = self.load_selection()
        changed = False
        if kind == "scene" and selection.scene == name:
            selection = AssetSelection(
                scene=None,
                cabinet=selection.cabinet,
                toolset=selection.toolset,
            )
            changed = True
        elif kind == "cabinet" and selection.cabinet == name:
            selection = AssetSelection(
                scene=selection.scene,
                cabinet=None,
                toolset=selection.toolset,
            )
            changed = True
        if changed:
            self.save_selection(selection)
        return record

    # ── selection (delegated to the store) ──────────────────────────────

    def load_selection(self) -> AssetSelection:
        return self._store.load_selection()

    def save_selection(self, selection: AssetSelection) -> None:
        self._store.save_selection(selection)

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
        if selection.toolset:
            env["TOOLSET"] = selection.toolset.upper()
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


def _restore_import_destination(destination: Path, backup: Path | None) -> None:
    """Restore the old asset tree after a failed index write.

    Both paths are constructed by :meth:`AssetLibrary.import_asset` below the
    library's validated kind/name directory.  The helper intentionally never
    accepts an externally supplied path or glob.
    """
    if destination.exists():
        shutil.rmtree(destination)
    if backup is not None and backup.exists():
        os.replace(backup, destination)


def _restore_catalog_record(
    store: AssetStore,
    previous: AssetRecord | None,
    *,
    kind: str,
    name: str,
) -> None:
    """Best-effort restore of the catalog after a failed replacement write."""
    if previous is None:
        store.delete_asset(kind, name)
        return
    store.put_asset(previous)


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


def _normalize_scene_refs(
    manifest: AssetManifest,
    root: Path,
    *,
    reference_root: Path | None = None,
) -> None:
    """Rewrite relative ``nav2_map`` / ``model.file`` refs to absolute paths.

    Only applies to scene assets.  ``package://`` / ``model://`` / ``file://``
    references are left untouched because they are resolved by the ROS package
    index or Gazebo's model:// mechanism; a plain relative or absolute path is
    made absolute inside the asset root so the existing launch/gateway
    resolution (which expects an absolute path or a package URI) keeps working.
    """
    if manifest.kind != "scene":
        return
    # Files are edited in a staging tree during import, but relative map/model
    # references must point at the stable final asset directory after its
    # atomic promotion.  Legacy direct callers retain the previous ``root``
    # behavior by omitting this override.
    reference_root = reference_root if reference_root is not None else root
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
            scene["nav2_map"] = _absolute_reference(
                root,
                reference_root,
                nav2_map.strip(),
            )
        model = scene.get("model")
        if isinstance(model, Mapping):
            file = model.get("file")
            if isinstance(file, str) and file.strip():
                model["file"] = _absolute_reference(
                    root,
                    reference_root,
                    file.strip(),
                )
    if manifest.name not in names:
        raise AssetLibraryError(
            f"Scene asset {manifest.name}: scenes.yaml must contain a scene "
            f"named exactly like the asset (found {names!r})."
        )
    scenes_path.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _absolute_reference(root: Path, reference_root: Path, reference: str) -> str:
    """Root a plain reference inside ``reference_root``.

    ``package://`` / ``model://`` / ``file://`` URIs and plain absolute paths
    outside ``root`` are left untouched (they are already self-contained or
    resolved by the host machinery).  A plain absolute path that already points
    inside ``root`` is re-rooted onto ``reference_root`` — this re-points a
    staging-tree reference at the stable destination right before the atomic
    promotion.
    """
    if reference.startswith(_URI_PREFIXES):
        return reference
    path = Path(reference).expanduser()
    if path.is_absolute():
        try:
            # ``mkdtemp(dir=...)`` preserves a relative ``dir`` while the first
            # pass emits an absolute path.  Canonicalize only our trusted root
            # so the second pass can rebase it without resolving an unrelated
            # external path (which may itself be a broken/cyclic symlink).
            relative = path.relative_to(root.resolve())
        except ValueError:
            return str(path)
        return str((reference_root / relative).resolve())
    return str((reference_root / path).resolve())


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
