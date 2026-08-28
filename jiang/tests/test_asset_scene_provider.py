"""Unit tests for the asset-scene provider (pure, no ROS).

The provider maps imported scene assets onto validated SceneSpecs so the Web
gateway can list / switch to user-imported scenes without restarting.  It must
reuse the import-time normalization (relative ``nav2_map`` / ``model.file``
become absolute inside the library, ``package://`` URIs are preserved) and must
skip a corrupt asset without taking the rest of the catalog down.
"""

from __future__ import annotations

import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import Dict

import yaml


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.asset_library import AssetLibrary  # noqa: E402
from control_gateway.asset_scene_provider import (  # noqa: E402
    AssetSceneProvider,
)
from control_gateway.scene_catalog import SceneSpec  # noqa: E402


def _write_scene_asset(root: Path, name: str, *, nav2_map: str) -> Path:
    """Write a self-contained scene asset directory (not yet imported)."""
    source = root / "src" / name
    maps = source / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    (maps / "map.yaml").write_text(
        "image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    (maps / "map.pgm").write_bytes(b"P5\n2 2\n255\n\x00\x00\x00\x00")
    (source / "manifest.yaml").write_text(
        "\n".join(
            [
                "kind: scene",
                f"name: {name}",
                "version: 1.0.0",
                "description: test scene asset",
                "files:",
                "  scenes: scenes.yaml",
                "references:",
                "  cabinet: null",
            ]
        ),
        encoding="utf-8",
    )
    (source / "scenes.yaml").write_text(
        yaml.safe_dump(
            {
                "scenes": [
                    {
                        "name": name,
                        "spawn_cabinet": True,
                        "model": {
                            "file": "package://xczs_inspection_robot_description/urdf/scenes/floor.sdf",
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
                        "robot_spawn": {"x": 1.0, "y": 2.0, "z": 0.5, "yaw": 0.3},
                    }
                ]
            },
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return source


class AssetSceneProviderTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.temporary = Path(self._temporary_directory.name)
        self.assets_root = self.temporary / "assets"

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _library(self) -> AssetLibrary:
        return AssetLibrary(self.assets_root)

    def test_empty_library_and_none_provide_no_scenes(self) -> None:
        self.assertEqual(AssetSceneProvider().scene_asset_names(), [])
        self.assertIsNone(AssetSceneProvider().resolve_scene("anything"))
        self.assertEqual(
            AssetSceneProvider(self._library()).scene_asset_names(), []
        )
        self.assertEqual(
            list(AssetSceneProvider(self._library()).iter_scene_specs()), []
        )

    def test_resolve_scene_reuses_import_normalization(self) -> None:
        library = self._library()
        source = _write_scene_asset(
            self.temporary, "my_scene", nav2_map="maps/map.yaml"
        )
        library.import_asset(source, validate=None)

        provider = AssetSceneProvider(library)
        self.assertEqual(provider.scene_asset_names(), ["my_scene"])
        spec = provider.resolve_scene("my_scene")
        self.assertIsInstance(spec, SceneSpec)
        assert spec is not None
        self.assertEqual(spec.name, "my_scene")
        self.assertTrue(spec.spawn_cabinet)
        # 相对 nav2_map 已由导入归一化为资产库内绝对路径。
        self.assertTrue(Path(spec.nav2_map).is_absolute())
        self.assertTrue(
            Path(spec.nav2_map).is_relative_to(self.assets_root.resolve())
        )
        self.assertEqual(spec.robot_spawn.yaw, 0.3)  # type: ignore[union-attr]
        # package:// model.file 原样保留。
        self.assertEqual(
            spec.model.file,  # type: ignore[union-attr]
            "package://xczs_inspection_robot_description/urdf/scenes/floor.sdf",
        )
        self.assertEqual(
            [s.name for s in provider.iter_scene_specs()], ["my_scene"]
        )

    def test_unknown_scene_resolves_none(self) -> None:
        library = self._library()
        source = _write_scene_asset(
            self.temporary, "my_scene", nav2_map="maps/map.yaml"
        )
        library.import_asset(source, validate=None)
        self.assertIsNone(AssetSceneProvider(library).resolve_scene("missing"))

    def test_corrupt_scene_asset_is_skipped_not_fatal(self) -> None:
        library = self._library()
        good = _write_scene_asset(self.temporary, "good_scene", nav2_map="maps/map.yaml")
        library.import_asset(good, validate=None)
        # 场景条目带未知字段：导入归一化不检查字段，但 SceneCatalog 严格解析器
        # 会拒绝 —— 这正是「坏资产被跳过、不拖垮其余资产」的场景。
        bad_source = self.temporary / "src" / "bad_scene"
        (bad_source / "maps").mkdir(parents=True, exist_ok=True)
        (bad_source / "maps" / "map.yaml").write_text(
            "image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
            encoding="utf-8",
        )
        (bad_source / "manifest.yaml").write_text(
            "\n".join(
                [
                    "kind: scene",
                    "name: bad_scene",
                    "version: 1.0.0",
                    "description: corrupt scene",
                    "files:",
                    "  scenes: scenes.yaml",
                    "references:",
                    "  cabinet: null",
                ]
            ),
            encoding="utf-8",
        )
        (bad_source / "scenes.yaml").write_text(
            yaml.safe_dump(
                {
                    "scenes": [
                        {
                            "name": "bad_scene",
                            "spawn_cabinet": True,
                            "model": None,
                            "nav2_map": "maps/map.yaml",
                            "robot_spawn": {"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0},
                            "extra_field": "not a scene field",
                        }
                    ]
                },
                sort_keys=False,
                allow_unicode=True,
            ),
            encoding="utf-8",
        )
        library.import_asset(bad_source, validate=None)

        provider = AssetSceneProvider(library)
        # 坏资产在 catalog 中仍存在，但解析为 None 且不抛异常。
        self.assertIn("bad_scene", provider.scene_asset_names())
        self.assertIsNone(provider.resolve_scene("bad_scene"))
        self.assertEqual(
            [s.name for s in provider.iter_scene_specs()], ["good_scene"]
        )


if __name__ == "__main__":
    unittest.main()
