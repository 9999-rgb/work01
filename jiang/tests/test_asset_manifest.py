"""Unit tests for the asset manifest schema (pure, no ROS).

The manifest is the single entry point of an importable asset directory.  These
tests pin the schema contract: kind/name/version rules, the role -> relative
path mapping, reference fields, and the "declared files exist" check — the
validation the import flow runs before anything is copied into the library.
"""

from __future__ import annotations

import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import Any, Dict

import yaml


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
# ``control_gateway.__init__`` exposes the ROS-backed server; load this pure
# module without a sourced ROS workspace.
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.asset_manifest import (  # noqa: E402
    AssetManifest,
    ManifestError,
    load_manifest,
)


def _manifest(**overrides: Any) -> Dict[str, Any]:
    value: Dict[str, Any] = {
        "kind": "scene",
        "name": "scene_a",
        "version": "1.0.0",
        "description": "A test scene asset.",
        "files": {"scenes": "scenes.yaml"},
        "references": None,
    }
    value.update(overrides)
    return value


def _write_asset(
    directory: Path, document: Dict[str, Any], *, declared: bool = True
) -> Path:
    """Write a manifest and its declared files; return the manifest path."""
    files = document.get("files", {}) if isinstance(document, dict) else {}
    if isinstance(files, dict) and declared:
        for relative in files.values():
            if not isinstance(relative, str):
                continue
            # Never write outside the temp directory: absolute and parent
            # paths are rejected by the schema, so we only fixture safe ones.
            candidate = Path(relative)
            if candidate.is_absolute() or ".." in candidate.parts:
                continue
            target = directory / candidate
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text("# fixture\n", encoding="utf-8")
    path = directory / "manifest.yaml"
    path.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )
    return path


class AssetManifestTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _load(self, document: Dict[str, Any]) -> AssetManifest:
        path = _write_asset(self.directory, document)
        return load_manifest(path)

    # ── happy paths ──────────────────────────────────────────────────────

    def test_loads_scene_manifest(self) -> None:
        manifest = self._load(_manifest())

        self.assertEqual("scene", manifest.kind)
        self.assertEqual("scene_a", manifest.name)
        self.assertEqual("1.0.0", manifest.version)
        self.assertEqual("A test scene asset.", manifest.description)
        self.assertEqual({"scenes": "scenes.yaml"}, dict(manifest.files))
        self.assertIsNone(manifest.references.cabinet)
        self.assertEqual(
            self.directory / "scenes.yaml", manifest.file_path(self.directory, "scenes")
        )

    def test_loads_cabinet_manifest_with_all_required_roles(self) -> None:
        manifest = self._load(
            _manifest(
                kind="cabinet",
                name="control_cabinet",
                files={
                    "controls": "cabinet_controls.yaml",
                    "scene": "cabinet_scene.yaml",
                    "pose": "cabinet_pose.yaml",
                    "adapter": "cabinet_robot_adapter.yaml",
                    "xacro": "control_cabinet.urdf.xacro",
                },
                references={"cabinet": None},
            )
        )
        self.assertEqual("cabinet", manifest.kind)
        self.assertIsNone(manifest.references.cabinet)
        self.assertEqual(5, len(manifest.files))

    def test_scene_asset_may_declare_instances(self) -> None:
        manifest = self._load(
            _manifest(files={"scenes": "scenes.yaml", "instances": "instances.yaml"})
        )
        self.assertIn("instances", manifest.files)

    def test_missing_manifest_file_raises(self) -> None:
        with self.assertRaises(ManifestError):
            load_manifest(self.directory / "nope.yaml")

    def test_rejects_invalid_kind(self) -> None:
        for kind in ("robot", "gripper", "SCENE", ""):
            with self.subTest(kind=kind):
                with self.assertRaises(ManifestError):
                    self._load(_manifest(kind=kind))

    def test_rejects_invalid_name(self) -> None:
        for name in ("Bad-Case", "9starts_with_digit", "has space", "", "a" * 64):
            with self.subTest(name=name):
                with self.assertRaises(ManifestError):
                    self._load(_manifest(name=name))

    def test_rejects_invalid_version(self) -> None:
        for version in ("1.0", "v1.0.0", "1.0.0-beta", "", "01.0.0"):
            with self.subTest(version=version):
                with self.assertRaises(ManifestError):
                    self._load(_manifest(version=version))

    def test_rejects_missing_required_roles(self) -> None:
        for files in ({"controls": "c.yaml"}, {}, None):
            with self.subTest(files=files):
                with self.assertRaises(ManifestError):
                    self._load(_manifest(files=files))

    def test_rejects_unknown_role(self) -> None:
        with self.assertRaises(ManifestError):
            self._load(
                _manifest(files={"scenes": "scenes.yaml", "gadget": "x.yaml"})
            )

    def test_rejects_absolute_and_parent_paths(self) -> None:
        for files in (
            {"scenes": "/etc/scenes.yaml"},
            {"scenes": "../scenes.yaml"},
            {"scenes": "a/../../scenes.yaml"},
        ):
            with self.subTest(files=files):
                with self.assertRaises(ManifestError):
                    self._load(_manifest(files=files))

    def test_rejects_declared_file_that_does_not_exist(self) -> None:
        document = _manifest(files={"scenes": "missing.yaml"})
        _write_asset(self.directory, document, declared=False)
        with self.assertRaises(ManifestError):
            load_manifest(self.directory / "manifest.yaml")

    def test_rejects_invalid_files_mapping(self) -> None:
        for files in ("scenes.yaml", ["scenes.yaml"], {"scenes": 42}):
            with self.subTest(files=files):
                with self.assertRaises(ManifestError):
                    self._load(_manifest(files=files))

    def test_rejects_invalid_references(self) -> None:
        for references in (
            {"gadget": "x"},
            {"cabinet": "Bad-Case"},
            {"cabinet": ""},
            "not-a-mapping",
        ):
            with self.subTest(references=references):
                with self.assertRaises(ManifestError):
                    self._load(_manifest(references=references))

    def test_rejects_non_mapping_root_and_non_string_fields(self) -> None:
        for document in (
            ["not-a-mapping"],
            {"kind": 1, "name": "scene_a", "version": "1.0.0", "files": {}},
            _manifest(description=42),
            _manifest(version=None),
        ):
            with self.subTest(document=document):
                with self.assertRaises(ManifestError):
                    self._load(document)


if __name__ == "__main__":
    unittest.main()
