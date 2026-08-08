"""Unit tests for cabinet inventory validation and station geometry."""

from __future__ import annotations

import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import Any, Dict

import yaml


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
# ``control_gateway.__init__`` intentionally exposes the ROS-backed server.
# Load this pure module without requiring a sourced ROS workspace.
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.inventory import CabinetInventory  # noqa: E402
from control_gateway.inventory import CabinetNotFoundError  # noqa: E402
from control_gateway.inventory import InventoryError  # noqa: E402
from control_gateway.inventory import MapBounds  # noqa: E402
from control_gateway.inventory import OccupancyGridBoundary  # noqa: E402
from control_gateway.inventory import (  # noqa: E402
    NavigationStationOutOfBoundsError,
)


class CabinetInventoryTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _load(
        self,
        instances: Dict[str, Any],
        station: Dict[str, Any] | None = None,
    ) -> CabinetInventory:
        instances_path = self.directory / "cabinet_instances.yaml"
        scene_path = self.directory / "cabinet_scene.yaml"
        instances_path.write_text(
            yaml.safe_dump(instances, sort_keys=False),
            encoding="utf-8",
        )
        station = station or {
            "local_anchor": [1.0, 2.0, 0.0],
            "outward_axis": [0.0, 2.0, 0.0],
            "standoff": 2.0,
            "base_yaw_offset": 0.0,
            "frame_id": "map",
        }
        scene_path.write_text(
            yaml.safe_dump(
                {
                    "/**/xczs_cabinet_planning_scene": {
                        "ros__parameters": {
                            "frame_parts": {},
                            "navigation_station": station,
                        }
                    }
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return CabinetInventory.load(instances_path, scene_path)

    @staticmethod
    def _instance(name: str = "cabinet_a", **overrides: Any) -> Dict[str, Any]:
        value: Dict[str, Any] = {
            "name": name,
            "x": 10.0,
            "y": 20.0,
            "z": 0.0,
            "roll": 0.0,
            "yaw": 0.0,
        }
        value.update(overrides)
        return value

    def test_loads_namespaces_and_computes_station(self) -> None:
        inventory = self._load(
            {
                "instances": [
                    self._instance(),
                    self._instance("cabinet_b", x=30.0, pitch=0.0),
                ]
            }
        )

        self.assertEqual(("cabinet_a", "cabinet_b"), inventory.names)
        self.assertEqual(
            "/xczs/cabinet/cabinet_a",
            inventory.get("cabinet_a").namespace,
        )
        self.assertEqual("cabinet_a_frame", inventory.get("cabinet_a").frame_id)
        station = inventory.station_for("cabinet_a")
        self.assertAlmostEqual(11.0, station.x)
        self.assertAlmostEqual(24.0, station.y)
        self.assertAlmostEqual(-math.pi / 2.0, station.yaw)
        listed = inventory.list_cabinets()
        self.assertEqual("cabinet_a", listed[0]["name"])
        self.assertEqual("map", listed[0]["navigation_station"]["frame_id"])

    def test_uses_full_rpy_rotation_and_instance_override(self) -> None:
        inventory = self._load(
            {
                "instances": [
                    self._instance(
                        roll=math.pi / 2.0,
                        navigation_station={
                            "local_anchor": [0.0, 0.0, 0.0],
                            "outward_axis": [0.0, 0.0, 1.0],
                            "standoff": 1.5,
                            "base_yaw_offset": 0.25,
                            "frame_id": "odom",
                        },
                    )
                ]
            }
        )

        station = inventory.station_for("cabinet_a")
        self.assertAlmostEqual(10.0, station.x)
        self.assertAlmostEqual(18.5, station.y)
        self.assertAlmostEqual(0.0, station.z, places=8)
        self.assertAlmostEqual(math.pi / 2.0 + 0.25, station.yaw)
        self.assertEqual("odom", station.frame_id)

    def test_validates_unknown_name_and_map_bounds(self) -> None:
        inventory = self._load({"instances": [self._instance()]})

        with self.assertRaises(CabinetNotFoundError):
            inventory.station_for("missing")
        station = inventory.station_for(
            "cabinet_a",
            boundary=MapBounds(0.0, 0.0, 20.0, 30.0),
            margin=1.0,
        )
        self.assertEqual("cabinet_a", station.cabinet)
        with self.assertRaises(NavigationStationOutOfBoundsError):
            inventory.station_for(
                "cabinet_a",
                boundary=MapBounds(0.0, 0.0, 10.0, 10.0),
            )
        self.assertEqual(
            "cabinet_a",
            inventory.station_for(
                "cabinet_a",
                boundary=lambda x, y: x == 11.0 and y == 24.0,
            ).cabinet,
        )

    def test_rejects_duplicate_invalid_and_nonfinite_instances(self) -> None:
        invalid_documents = [
            {
                "instances": [
                    self._instance(),
                    self._instance(),
                ]
            },
            {"instances": [self._instance("cabinet-with-dash")]},
            {"instances": [self._instance(x=float("nan"))]},
            {"instances": [self._instance(extra="unexpected")]},
            {"instances": []},
        ]
        for document in invalid_documents:
            with self.subTest(document=document):
                with self.assertRaises(InventoryError):
                    self._load(document)

    def test_requires_valid_navigation_station(self) -> None:
        invalid_stations = [
            {
                "local_anchor": [0.0, 0.0],
                "outward_axis": [1.0, 0.0, 0.0],
                "standoff": 1.0,
            },
            {
                "local_anchor": [0.0, 0.0, 0.0],
                "outward_axis": [0.0, 0.0, 0.0],
                "standoff": 1.0,
            },
            {
                "local_anchor": [0.0, 0.0, 0.0],
                "outward_axis": [1.0, 0.0, 0.0],
                "standoff": 0.0,
            },
        ]
        for station in invalid_stations:
            with self.subTest(station=station):
                with self.assertRaises(InventoryError):
                    self._load(
                        {"instances": [self._instance()]},
                        station,
                    )

    def test_map_bounds_factory_validates_grid_metadata(self) -> None:
        bounds = MapBounds.from_grid(
            width=100,
            height=50,
            resolution=0.1,
            origin_x=-5.0,
            origin_y=-2.5,
        )
        self.assertTrue(bounds.contains(0.0, 0.0))
        self.assertFalse(bounds.contains(5.1, 0.0))
        with self.assertRaises(InventoryError):
            MapBounds.from_grid(
                width=0,
                height=50,
                resolution=0.1,
                origin_x=0.0,
                origin_y=0.0,
            )

    def test_rotated_occupancy_grid_boundary_preserves_margin(self) -> None:
        boundary = OccupancyGridBoundary(
            width=20,
            height=10,
            resolution=0.1,
            origin_x=3.0,
            origin_y=-2.0,
            origin_yaw=math.pi / 2.0,
        )
        # Grid-local (1.0, 0.5) rotated into world coordinates.
        self.assertTrue(boundary.contains(2.5, -1.0, margin=0.1))
        # Grid-local x=0.05 is inside the raw grid but violates the margin.
        self.assertFalse(boundary.contains(2.5, -1.95, margin=0.1))


if __name__ == "__main__":
    unittest.main()
