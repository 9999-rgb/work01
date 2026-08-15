"""Unit tests for the scene catalog contract (pure, no ROS).

The catalog is the single source of truth for scene switching: name/geometry/
map/robot-pose fields, strict validation, and the on-disk ``scenes.yaml`` that
drives the Web gateway, the launch file and Nav2 map switching.
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
WORKSPACE = JIANG_DIR.parent
sys.path.insert(0, str(JIANG_DIR))
# ``control_gateway.__init__`` exposes the ROS-backed server; load this pure
# module without a sourced ROS workspace.
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.scene_catalog import SceneCatalog  # noqa: E402
from control_gateway.scene_catalog import SceneError  # noqa: E402
from control_gateway.scene_catalog import SceneModel  # noqa: E402
from control_gateway.scene_catalog import SceneNotFoundError  # noqa: E402
from control_gateway.scene_catalog import resolve_package_uri  # noqa: E402


def _scene(**overrides: Any) -> Dict[str, Any]:
    value: Dict[str, Any] = {
        "name": "scene_a",
        "spawn_cabinet": True,
        "model": None,
        "nav2_map": "package://some_pkg/maps/a.yaml",
        "robot_spawn": {"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0},
    }
    value.update(overrides)
    return value


class SceneCatalogTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _load(self, scenes: list[Dict[str, Any]]) -> SceneCatalog:
        path = self.directory / "scenes.yaml"
        path.write_text(
            yaml.safe_dump({"scenes": scenes}, sort_keys=False),
            encoding="utf-8",
        )
        return SceneCatalog.load(path)

    def test_parses_fields_and_preserves_order(self) -> None:
        catalog = self._load(
            [
                _scene(),
                _scene(
                    name="scene_b",
                    spawn_cabinet=False,
                    model={
                        "file": "package://pkg/floor.sdf",
                        "pose": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                            "roll": 0.0,
                            "pitch": 0.0,
                            "yaw": 0.5,
                        },
                    },
                    robot_spawn=None,
                ),
            ]
        )

        self.assertEqual(("scene_a", "scene_b"), catalog.names)
        self.assertEqual(2, len(catalog))

        scene_a = catalog.get("scene_a")
        self.assertTrue(scene_a.spawn_cabinet)
        self.assertIsNone(scene_a.model)
        self.assertIsNotNone(scene_a.robot_spawn)
        self.assertAlmostEqual(0.5, scene_a.robot_spawn.z)

        scene_b = catalog.get("scene_b")
        self.assertFalse(scene_b.spawn_cabinet)
        self.assertIsInstance(scene_b.model, SceneModel)
        self.assertEqual(
            {"x", "y", "z", "roll", "pitch", "yaw"},
            set(scene_b.model.to_dict()["pose"]),
        )
        self.assertAlmostEqual(0.5, scene_b.model.pose[5])
        self.assertIsNone(scene_b.robot_spawn)

    def test_list_scenes_matches_to_dict(self) -> None:
        catalog = self._load([_scene()])
        listed = catalog.list_scenes()
        self.assertEqual(1, len(listed))
        self.assertEqual("scene_a", listed[0]["name"])
        self.assertIsNone(listed[0]["model"])
        self.assertEqual(0.0, listed[0]["robot_spawn"]["x"])

    def test_unknown_name_raises_not_found(self) -> None:
        catalog = self._load([_scene()])
        with self.assertRaises(SceneNotFoundError):
            catalog.get("missing")
        with self.assertRaises(SceneNotFoundError):
            catalog.get("")

    def test_rejects_invalid_documents(self) -> None:
        invalid_documents: list[list[Dict[str, Any]]] = [
            [],
            [_scene(name="Bad-Case")],
            [_scene(name="9starts_with_digit")],
            [_scene(), _scene()],  # duplicate name
            [_scene(spawn_cabinet="yes")],
            [_scene(spawn_cabinet=None)],
            [_scene(nav2_map="")],
            [_scene(nav2_map=42)],
            [_scene(extra="unexpected")],
            [_scene(robot_spawn={"x": float("nan"), "y": 0.0, "z": 0.0, "yaw": 0.0})],
            [_scene(robot_spawn={"x": 0.0, "y": 0.0, "z": 0.0})],  # missing yaw
            [
                _scene(
                    model={
                        "file": "package://p/floor.sdf",
                        "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0},
                    }
                )
            ],  # model pose missing yaw
            [_scene(model={"file": ""})],
            [_scene(model="not-a-mapping")],
            [_scene(robot_spawn=[])],
        ]
        for scenes in invalid_documents:
            with self.subTest(scenes=scenes):
                with self.assertRaises(SceneError):
                    self._load(scenes)

    def test_rejects_non_mapping_or_non_list_root(self) -> None:
        for document in (
            {"scenes": "not-a-list"},
            {"scenes": None},
            ["not-a-mapping"],
            {"scenes": [], "other": True},
        ):
            with self.subTest(document=document):
                path = self.directory / "bad.yaml"
                path.write_text(
                    yaml.safe_dump(document, sort_keys=False),
                    encoding="utf-8",
                )
                with self.assertRaises(SceneError):
                    SceneCatalog.load(path)

    def test_resolves_plain_filesystem_paths_without_ros(self) -> None:
        self.assertEqual(
            Path("/tmp/floor.sdf"),
            resolve_package_uri("/tmp/floor.sdf"),
        )

    def test_on_disk_scenes_yaml_contract(self) -> None:
        config = (
            WORKSPACE
            / "xczs_inspection_robot_control"
            / "config"
            / "scenes.yaml"
        )
        catalog = SceneCatalog.load(config)
        self.assertEqual(
            ("cabinet_operation", "electrical_mezzanine", "generator_plant"),
            catalog.names,
        )

        cabinet = catalog.get("cabinet_operation")
        self.assertTrue(cabinet.spawn_cabinet)
        self.assertIsNone(cabinet.model)

        for name in ("electrical_mezzanine", "generator_plant"):
            scene = catalog.get(name)
            self.assertFalse(scene.spawn_cabinet)
            self.assertIsNotNone(scene.model)
            self.assertIsNotNone(scene.robot_spawn)
            for uri in (scene.model.file, scene.nav2_map):
                self.assertTrue(
                    uri.startswith("package://"),
                    f"{name} reference is not a package:// URI: {uri!r}",
                )
                package, _sep, relative = uri[len("package://") :].partition("/")
                target = WORKSPACE / package / relative
                self.assertTrue(target.exists(), f"missing {target}")


if __name__ == "__main__":
    unittest.main()
