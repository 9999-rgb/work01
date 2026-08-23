"""Unit tests for the asset library (pure, no ROS).

The library owns the on-disk side of import/selection: copying an asset into
``<root>/<kind>/<name>/``, normalizing self-contained scene references to
absolute paths, recording entries in ``assets_catalog.yaml``, and mapping the
selected assets onto the existing ``CABINET_*_PATH`` / ``SCENES_CONFIG`` env
pointers that the launch and gateway already consume.
"""

from __future__ import annotations

import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import Any, Dict
from unittest.mock import patch

import yaml


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.asset_library import (  # noqa: E402
    AssetLibrary,
    AssetLibraryError,
    AssetNotFoundError,
    AssetRecord,
    AssetSelection,
)
from control_gateway.asset_manifest import ManifestError  # noqa: E402


def _scene_document(name: str, nav2_map: str = "maps/a.yaml") -> Dict[str, Any]:
    return {
        "scenes": [
            {
                "name": name,
                "spawn_cabinet": True,
                "model": {
                    "file": "worlds/floor.sdf",
                    "pose": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                },
                "nav2_map": nav2_map,
                "robot_spawn": {"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0},
            }
        ]
    }


def _write_scene_asset(source: Path, name: str = "scene_a") -> Path:
    """Create a self-contained scene asset directory and return it."""
    asset = source / name
    (asset / "maps").mkdir(parents=True)
    (asset / "worlds").mkdir(parents=True)
    (asset / "meshes").mkdir(parents=True)
    (asset / "maps" / "a.yaml").write_text("image: a.pgm\n", encoding="utf-8")
    (asset / "maps" / "a.pgm").write_bytes(b"\x00")
    (asset / "worlds" / "floor.sdf").write_text(
        "<sdf></sdf>", encoding="utf-8"
    )
    # Supporting data outside the role mapping must travel with the asset.
    (asset / "meshes" / "panel.stl").write_bytes(b"stl")
    (asset / "scenes.yaml").write_text(
        yaml.safe_dump(_scene_document(name), sort_keys=False), encoding="utf-8"
    )
    (asset / "instances.yaml").write_text(
        "cabinets: []\n", encoding="utf-8"
    )
    (asset / "manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "kind": "scene",
                "name": name,
                "version": "1.0.0",
                "description": "Test scene.",
                "files": {
                    "scenes": "scenes.yaml",
                    "instances": "instances.yaml",
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return asset


def _write_cabinet_asset(source: Path, name: str = "control_cabinet") -> Path:
    asset = source / name
    asset.mkdir(parents=True)
    roles = {
        "controls": "cabinet_controls.yaml",
        "scene": "cabinet_scene.yaml",
        "pose": "cabinet_pose.yaml",
        "adapter": "cabinet_robot_adapter.yaml",
        "xacro": "control_cabinet.urdf.xacro",
    }
    for relative in roles.values():
        (asset / relative).write_text("# fixture\n", encoding="utf-8")
    (asset / "manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "kind": "cabinet",
                "name": name,
                "version": "1.0.0",
                "files": roles,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return asset


class AssetLibraryTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)
        self.library = AssetLibrary(self.directory / "assets")

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    # ── import ──────────────────────────────────────────────────────────

    def test_import_copies_whole_tree_and_normalizes_refs(self) -> None:
        source = _write_scene_asset(self.directory / "source")

        record = self.library.import_asset(source)

        self.assertEqual("scene", record.kind)
        self.assertEqual("scene_a", record.name)
        self.assertEqual("1.0.0", record.version)
        self.assertFalse(record.validated)

        catalog = self.library.load_catalog()
        self.assertEqual(1, len(catalog))
        self.assertEqual(record, catalog[0])

        root = self.library.root / "scene" / "scene_a"
        self.assertTrue((root / "manifest.yaml").is_file())
        # Supporting data outside the role mapping was copied too.
        self.assertTrue((root / "meshes" / "panel.stl").is_file())
        self.assertTrue((root / "maps" / "a.pgm").is_file())

        scenes = yaml.safe_load((root / "scenes.yaml").read_text())
        scene = scenes["scenes"][0]
        self.assertEqual(str((root / "maps" / "a.yaml").resolve()), scene["nav2_map"])
        self.assertEqual(str((root / "worlds" / "floor.sdf").resolve()), scene["model"]["file"])

    def test_import_keeps_package_and_model_uris(self) -> None:
        source = _write_scene_asset(self.directory / "source")
        (source / "scenes.yaml").write_text(
            yaml.safe_dump(
                _scene_document(
                    "scene_a", nav2_map="package://some_pkg/maps/a.yaml"
                ),
                sort_keys=False,
            ),
            encoding="utf-8",
        )

        self.library.import_asset(source)
        root = self.library.root / "scene" / "scene_a"
        scenes = yaml.safe_load((root / "scenes.yaml").read_text())
        self.assertEqual(
            "package://some_pkg/maps/a.yaml", scenes["scenes"][0]["nav2_map"]
        )

    def test_import_with_validation_sets_validated_flag(self) -> None:
        source = _write_scene_asset(self.directory / "source")

        def validate(manifest, root) -> None:
            self.assertEqual("scene_a", manifest.name)
            self.assertTrue((root / "scenes.yaml").is_file())

        record = self.library.import_asset(source, validate=validate)
        self.assertTrue(record.validated)

    def test_import_validation_failure_leaves_no_trace(self) -> None:
        source = _write_scene_asset(self.directory / "source")

        def validate(manifest, root) -> None:
            raise ValueError("map file missing")

        with self.assertRaises(AssetLibraryError):
            self.library.import_asset(source, validate=validate)
        self.assertFalse((self.library.root / "scene" / "scene_a").exists())
        self.assertEqual([], self.library.load_catalog())

    def test_import_duplicate_rejected_then_force_replaces(self) -> None:
        source = _write_scene_asset(self.directory / "source", name="scene_a")
        self.library.import_asset(source)

        source2 = _write_scene_asset(self.directory / "source2", name="scene_a")
        (source2 / "manifest.yaml").write_text(
            (source2 / "manifest.yaml").read_text().replace("1.0.0", "2.0.0"),
            encoding="utf-8",
        )
        with self.assertRaises(AssetLibraryError):
            self.library.import_asset(source2)

        record = self.library.import_asset(source2, force=True)
        self.assertEqual("2.0.0", record.version)
        catalog = self.library.load_catalog()
        self.assertEqual(1, len(catalog))
        self.assertEqual("2.0.0", catalog[0].version)

    def test_force_validation_failure_preserves_the_previous_asset_tree(self) -> None:
        source = _write_scene_asset(self.directory / "source", name="scene_a")
        (source / "old-only.txt").write_text("keep", encoding="utf-8")
        self.library.import_asset(source)

        replacement = _write_scene_asset(
            self.directory / "replacement", name="scene_a"
        )
        (replacement / "manifest.yaml").write_text(
            (replacement / "manifest.yaml").read_text(encoding="utf-8").replace(
                "1.0.0", "2.0.0"
            ),
            encoding="utf-8",
        )

        with self.assertRaises(AssetLibraryError):
            self.library.import_asset(
                replacement,
                force=True,
                validate=lambda _manifest, _root: (_ for _ in ()).throw(
                    ValueError("reject replacement")
                ),
            )

        installed = self.library.root / "scene" / "scene_a"
        self.assertEqual("keep", (installed / "old-only.txt").read_text())
        self.assertEqual("1.0.0", self.library.find("scene", "scene_a").version)

    def test_force_replace_drops_files_that_are_not_in_the_new_asset(self) -> None:
        source = _write_scene_asset(self.directory / "source", name="scene_a")
        (source / "obsolete.txt").write_text("old", encoding="utf-8")
        self.library.import_asset(source)

        replacement = _write_scene_asset(
            self.directory / "replacement", name="scene_a"
        )
        (replacement / "manifest.yaml").write_text(
            (replacement / "manifest.yaml").read_text(encoding="utf-8").replace(
                "1.0.0", "2.0.0"
            ),
            encoding="utf-8",
        )
        self.library.import_asset(replacement, force=True)

        self.assertFalse(
            (self.library.root / "scene" / "scene_a" / "obsolete.txt").exists()
        )

    def test_force_index_failure_restores_previous_tree_and_catalog(self) -> None:
        source = _write_scene_asset(self.directory / "source", name="scene_a")
        (source / "old-only.txt").write_text("keep", encoding="utf-8")
        self.library.import_asset(source)
        replacement = _write_scene_asset(
            self.directory / "replacement", name="scene_a"
        )
        (replacement / "manifest.yaml").write_text(
            (replacement / "manifest.yaml").read_text(encoding="utf-8").replace(
                "1.0.0", "2.0.0"
            ),
            encoding="utf-8",
        )

        with patch.object(
            self.library._store,
            "put_asset",
            side_effect=[OSError("catalog unavailable"), None],
        ):
            with self.assertRaises(AssetLibraryError, msg="catalog unavailable"):
                self.library.import_asset(replacement, force=True)

        installed = self.library.root / "scene" / "scene_a"
        self.assertEqual("keep", (installed / "old-only.txt").read_text())
        self.assertEqual("1.0.0", self.library.find("scene", "scene_a").version)

    def test_import_rejects_non_asset_and_invalid_manifest(self) -> None:
        plain = self.directory / "plain"
        plain.mkdir()
        (plain / "random.txt").write_text("hi", encoding="utf-8")
        with self.assertRaises(AssetLibraryError):
            self.library.import_asset(plain)

        asset = _write_scene_asset(self.directory / "source")
        (asset / "manifest.yaml").write_text(
            "kind: scene\nname: Bad-Case\n", encoding="utf-8"
        )
        with self.assertRaises(ManifestError):
            self.library.import_asset(asset)

    def test_import_scene_without_matching_scene_name(self) -> None:
        asset = _write_scene_asset(self.directory / "source", name="scene_a")
        (asset / "scenes.yaml").write_text(
            yaml.safe_dump(_scene_document("other_scene"), sort_keys=False),
            encoding="utf-8",
        )
        with self.assertRaises(AssetLibraryError):
            self.library.import_asset(asset)
        self.assertFalse((self.library.root / "scene" / "scene_a").exists())
        self.assertEqual([], self.library.load_catalog())

    def test_import_cabinet_asset(self) -> None:
        source = _write_cabinet_asset(self.directory / "source")

        record = self.library.import_asset(source)

        self.assertEqual("cabinet", record.kind)
        root = self.library.root / "cabinet" / "control_cabinet"
        self.assertTrue((root / "control_cabinet.urdf.xacro").is_file())

    # ── catalog ─────────────────────────────────────────────────────────

    def test_load_catalog_empty_when_missing(self) -> None:
        self.assertEqual([], self.library.load_catalog())

    def test_find_unknown_raises(self) -> None:
        with self.assertRaises(AssetNotFoundError):
            self.library.find("scene", "missing")

    def test_catalog_rejects_bad_entries(self) -> None:
        self.library.root.mkdir(parents=True)
        (self.library.root / "assets_catalog.yaml").write_text(
            "assets: [{kind: scene, name: 1}]\n", encoding="utf-8"
        )
        with self.assertRaises(AssetLibraryError):
            self.library.load_catalog()

    # ── selection ───────────────────────────────────────────────────────

    def test_selection_roundtrip(self) -> None:
        selection = AssetSelection(scene="scene_a")
        self.library.save_selection(selection)
        loaded = self.library.load_selection()
        self.assertEqual(selection, loaded)

    def test_selection_roundtrip_preserves_toolset(self) -> None:
        # YamlAssetStore 是默认/参考 store：save->load 往返必须保留 toolset，
        # 否则 selection_to_env 不会输出 TOOLSET，下一次启动会静默回退默认套装。
        selection = AssetSelection(scene="scene_a", toolset="B")
        self.library.save_selection(selection)
        loaded = self.library.load_selection()
        self.assertEqual(selection, loaded)

    def test_selection_to_env_maps_toolset(self) -> None:
        env = self.library.selection_to_env(AssetSelection(toolset="b"))
        self.assertEqual("B", env["TOOLSET"])

    def test_load_selection_defaults_empty(self) -> None:
        self.assertEqual(AssetSelection(), self.library.load_selection())

    def test_load_selection_rejects_bad_types(self) -> None:
        self.library.root.mkdir(parents=True)
        # YAML int, empty string and whitespace-only string are all rejected;
        # schema accepts a non-empty string or null.
        for text in ("scene: 42\n", "scene: ''\n", "scene: '  '\n"):
            with self.subTest(text=text):
                (self.library.root / "selection.yaml").write_text(
                    text, encoding="utf-8"
                )
                with self.assertRaises(AssetLibraryError):
                    self.library.load_selection()

    def test_selection_to_env_maps_scene(self) -> None:
        source = _write_scene_asset(self.directory / "source")
        self.library.import_asset(source)

        env = self.library.selection_to_env(AssetSelection(scene="scene_a"))

        root = self.library.root / "scene" / "scene_a"
        self.assertEqual(str(root / "scenes.yaml"), env["SCENES_CONFIG"])
        self.assertEqual("scene_a", env["SCENE"])
        self.assertEqual(str(root / "instances.yaml"), env["CABINET_INSTANCES_PATH"])

    def test_selection_to_env_maps_cabinet(self) -> None:
        source = _write_cabinet_asset(self.directory / "source")
        self.library.import_asset(source)

        env = self.library.selection_to_env(AssetSelection(cabinet="control_cabinet"))

        root = self.library.root / "cabinet" / "control_cabinet"
        self.assertEqual(str(root / "cabinet_controls.yaml"), env["CABINET_CONTROLS_PATH"])
        self.assertEqual(str(root / "cabinet_scene.yaml"), env["CABINET_SCENE_PATH"])
        self.assertEqual(str(root / "cabinet_pose.yaml"), env["CABINET_POSE_PATH"])
        self.assertEqual(
            str(root / "cabinet_robot_adapter.yaml"),
            env["CABINET_ROBOT_ADAPTER_PATH"],
        )
        self.assertEqual(
            str(root / "control_cabinet.urdf.xacro"), env["CABINET_XACRO_PATH"]
        )

    def test_selection_to_env_reads_persisted(self) -> None:
        source = _write_scene_asset(self.directory / "source")
        self.library.import_asset(source)
        self.library.save_selection(AssetSelection(scene="scene_a"))

        env = self.library.selection_to_env()

        self.assertEqual("scene_a", env["SCENE"])

    def test_selection_to_env_unknown_asset_raises(self) -> None:
        with self.assertRaises(AssetNotFoundError):
            self.library.selection_to_env(AssetSelection(scene="missing"))

    def test_selection_to_env_absent_fields_produce_no_overrides(self) -> None:
        env = self.library.selection_to_env(AssetSelection())
        self.assertEqual({}, env)


if __name__ == "__main__":
    unittest.main()
