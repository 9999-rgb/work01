"""Focused pure-Python tests for ``scripts/validate/check_scene_config``.

The checker is exercised through its real CLI so these tests cover catalog,
map-image and Nav2-parameter parsing together without importing ROS.
"""

from __future__ import annotations

import copy
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

import yaml
from PIL import Image


WORKSPACE = Path(__file__).resolve().parents[2]
CHECKER = WORKSPACE / "scripts" / "validate" / "check_scene_config"

DEFAULT_MAP = {
    "image": "map.pgm",
    "mode": "trinary",
    "resolution": 0.1,
    "origin": [0.0, 0.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.25,
}


class SceneConfigValidationTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _write_fixture(
        self,
        directory: Path,
        *,
        map_document: dict[str, Any] | None = None,
        spawn: dict[str, float] | None = None,
        image_size: tuple[int, int] = (80, 80),
        image_value: int = 255,
    ) -> tuple[Path, Path, Path]:
        directory.mkdir(parents=True)
        map_path = directory / "map.yaml"
        map_path.write_text(
            yaml.safe_dump(map_document or DEFAULT_MAP, sort_keys=False),
            encoding="utf-8",
        )
        image_path = directory / "map.pgm"
        Image.new("L", image_size, image_value).save(image_path)

        scenes_path = directory / "scenes.yaml"
        scenes_path.write_text(
            yaml.safe_dump(
                {
                    "scenes": [
                        {
                            "name": "test_scene",
                            "spawn_cabinet": True,
                            "model": None,
                            "nav2_map": str(map_path),
                            "robot_spawn": spawn
                            or {"x": 4.0, "y": 4.0, "z": 0.515, "yaw": 0.0},
                        }
                    ]
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )

        nav2_path = directory / "nav2_params.yaml"
        nav2_path.write_text(
            yaml.safe_dump(
                {
                    "global_costmap": {
                        "global_costmap": {
                            "ros__parameters": {
                                "footprint": [
                                    [0.8, 0.3],
                                    [0.8, -0.3],
                                    [-0.2, -0.3],
                                    [-0.2, 0.3],
                                ],
                                "footprint_padding": 0.1,
                            }
                        }
                    }
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return scenes_path, nav2_path, image_path

    def _run(self, scenes_path: Path, nav2_path: Path) -> subprocess.CompletedProcess:
        return subprocess.run(
            [
                sys.executable,
                str(CHECKER),
                "--scenes",
                str(scenes_path),
                "--nav2-params",
                str(nav2_path),
            ],
            cwd=str(WORKSPACE),
            capture_output=True,
            text=True,
            check=False,
        )

    def test_rejects_invalid_required_map_metadata(self) -> None:
        cases = {
            "non_positive_resolution": ({"resolution": 0.0}, "resolution"),
            "non_finite_resolution": ({"resolution": math.inf}, "resolution"),
            "wrong_origin_length": ({"origin": [0.0, 0.0]}, "origin"),
            "non_finite_origin": ({"origin": [0.0, math.nan, 0.0]}, "origin"),
            "invalid_negate": ({"negate": 2}, "negate"),
            "non_finite_free": ({"free_thresh": math.nan}, "free_thresh"),
            "non_finite_occupied": (
                {"occupied_thresh": math.inf},
                "occupied_thresh",
            ),
            "unordered_thresholds": (
                {"free_thresh": 0.7, "occupied_thresh": 0.6},
                "thresholds must satisfy",
            ),
            "out_of_range_threshold": (
                {"free_thresh": -0.1},
                "thresholds must satisfy",
            ),
        }
        for name, (overrides, expected) in cases.items():
            with self.subTest(name=name):
                map_document = copy.deepcopy(DEFAULT_MAP)
                map_document.update(overrides)
                scenes, nav2, _image = self._write_fixture(
                    self.directory / name, map_document=map_document
                )
                result = self._run(scenes, nav2)
                self.assertEqual(1, result.returncode, result.stdout)
                self.assertIn(expected, result.stderr)

    def test_rejects_an_image_that_exists_but_cannot_be_decoded(self) -> None:
        scenes, nav2, image = self._write_fixture(self.directory / "bad_image")
        image.write_bytes(b"not an image")

        result = self._run(scenes, nav2)

        self.assertEqual(1, result.returncode, result.stdout)
        self.assertIn("cannot decode map image", result.stderr)

    def test_rejects_image_references_nav2_humble_cannot_resolve(self) -> None:
        for image_reference in ("package://demo/map.pgm", "~/maps/map.pgm"):
            with self.subTest(image_reference=image_reference):
                map_document = copy.deepcopy(DEFAULT_MAP)
                map_document["image"] = image_reference
                scenes, nav2, _image = self._write_fixture(
                    self.directory / image_reference.split("/")[0],
                    map_document=map_document,
                )
                result = self._run(scenes, nav2)
                self.assertEqual(1, result.returncode, result.stdout)
                self.assertIn("does not resolve", result.stderr)

    def test_trinary_alpha_matches_nav2_humble_occupancy_semantics(self) -> None:
        map_document = copy.deepcopy(DEFAULT_MAP)
        map_document["image"] = "map.png"
        # Nav2 accepts YAML booleans for negate; keep that valid form covered.
        map_document["negate"] = False
        scenes, nav2, _image = self._write_fixture(
            self.directory / "trinary_alpha", map_document=map_document
        )
        Image.new("RGBA", (80, 80), (255, 255, 255, 0)).save(
            self.directory / "trinary_alpha" / "map.png"
        )

        result = self._run(scenes, nav2)

        self.assertEqual(1, result.returncode, result.stdout)
        self.assertIn("covers unknown map cell", result.stderr)

    def test_raw_mode_rounds_grayscale_like_nav2_humble(self) -> None:
        map_document = copy.deepcopy(DEFAULT_MAP)
        map_document.update({"image": "map.png", "mode": "raw"})
        scenes, nav2, _image = self._write_fixture(
            self.directory / "raw_truncation", map_document=map_document
        )
        # Mean grayscale is 2/3. Nav2 rounds it to occupancy value 1, so it is
        # not a free cell and the spawn footprint must be rejected.
        Image.new("RGB", (80, 80), (0, 0, 2)).save(
            self.directory / "raw_truncation" / "map.png"
        )

        result = self._run(scenes, nav2)

        self.assertEqual(1, result.returncode, result.stdout)
        self.assertIn("covers occupied map cell", result.stderr)

    def test_rejects_rotated_footprint_outside_map(self) -> None:
        scenes, nav2, _image = self._write_fixture(
            self.directory / "out_of_bounds",
            spawn={
                "x": 1.0,
                "y": 1.3,
                "z": 0.515,
                "yaw": math.pi / 2.0,
            },
            image_size=(20, 20),
        )

        result = self._run(scenes, nav2)

        self.assertEqual(1, result.returncode, result.stdout)
        self.assertIn("padded footprint is outside the map", result.stderr)

    def test_rejects_footprint_exactly_on_half_open_upper_map_bound(self) -> None:
        for name, spawn in (
            (
                "right_edge",
                {"x": 1.1, "y": 1.0, "z": 0.515, "yaw": 0.0},
            ),
            (
                "top_edge",
                {"x": 1.0, "y": 1.6, "z": 0.515, "yaw": 0.0},
            ),
        ):
            with self.subTest(name=name):
                scenes, nav2, _image = self._write_fixture(
                    self.directory / name,
                    spawn=spawn,
                    image_size=(20, 20),
                )
                result = self._run(scenes, nav2)
                self.assertEqual(1, result.returncode, result.stdout)
                self.assertIn(
                    "padded footprint is outside the map",
                    result.stderr,
                )

    def test_zero_footprint_components_follow_nav2_sign0_padding(self) -> None:
        scenes, nav2, _image = self._write_fixture(
            self.directory / "zero_component_padding",
            spawn={"x": 1.95, "y": 1.95, "z": 0.515, "yaw": 0.0},
            image_size=(20, 20),
        )
        nav2.write_text(
            yaml.safe_dump(
                {
                    "global_costmap": {
                        "global_costmap": {
                            "ros__parameters": {
                                "footprint": [
                                    [0.0, 0.0],
                                    [-0.5, 0.0],
                                    [-0.5, -0.5],
                                    [0.0, -0.5],
                                ],
                                "footprint_padding": 0.1,
                            }
                        }
                    }
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )

        result = self._run(scenes, nav2)

        self.assertEqual(0, result.returncode, result.stderr)

    def test_rejects_obstacle_tangent_to_footprint_edge_like_nav2(self) -> None:
        map_document = copy.deepcopy(DEFAULT_MAP)
        map_document["resolution"] = 1.0
        scenes, nav2, image_path = self._write_fixture(
            self.directory / "edge_tangent",
            map_document=map_document,
            spawn={"x": 0.0, "y": 0.0, "z": 0.515, "yaw": 0.0},
            image_size=(4, 4),
        )
        nav2.write_text(
            yaml.safe_dump(
                {
                    "global_costmap": {
                        "global_costmap": {
                            "ros__parameters": {
                                "footprint": [
                                    [0.0, 1.0],
                                    [1.0, 1.0],
                                    [1.0, 2.0],
                                    [0.0, 2.0],
                                ],
                                "footprint_padding": 0.0,
                            }
                        }
                    }
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        with Image.open(image_path) as source:
            image = source.copy()
        # Cell (1, 1) only touches the footprint's x=1 edge. Nav2 maps that
        # edge to this cell and checks it with Bresenham lineCost().
        image.putpixel((1, 2), 0)
        image.save(image_path)

        result = self._run(scenes, nav2)

        self.assertEqual(1, result.returncode, result.stdout)
        self.assertIn("footprint edge covers occupied", result.stderr)

    def test_rejects_occupied_and_unknown_cells_under_footprint(self) -> None:
        for name, value, expected in (
            ("occupied", 0, "covers occupied map cell"),
            ("unknown", 128, "covers unknown map cell"),
        ):
            with self.subTest(name=name):
                scenes, nav2, image_path = self._write_fixture(
                    self.directory / name
                )
                with Image.open(image_path) as source:
                    image = source.copy()
                # Map grid (40, 40) maps to image row 79 - 40 and lies inside
                # the asymmetric footprint at robot_spawn (4.0, 4.0).
                image.putpixel((40, 39), value)
                image.save(image_path)

                result = self._run(scenes, nav2)
                self.assertEqual(1, result.returncode, result.stdout)
                self.assertIn(expected, result.stderr)

    def test_current_three_scene_catalog_passes(self) -> None:
        result = subprocess.run(
            [sys.executable, str(CHECKER)],
            cwd=str(WORKSPACE),
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "scenes=cabinet_operation, electrical_mezzanine, generator_plant",
            result.stdout,
        )


if __name__ == "__main__":
    unittest.main()
