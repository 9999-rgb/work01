"""Scene-switch orchestration tests (ROS-gated, like test_task_runner).

These tests exercise the runtime switch ordering through ``ControlServer``
without a live Gazebo/Nav2 stack: a bare ``object.__new__`` server is wired
with fake Gazebo/Nav2 clients and a real ``SceneCatalog``, then the switch is
driven through the public ``switch_scene`` method.  The fake clients record
calls so ordering, teleport math and rollback are asserted deterministically.

Requires a sourced ROS workspace (``source install/setup.bash``) because
``control_gateway.runner`` imports ``rclpy``.
"""

from __future__ import annotations

import math
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Optional
from unittest.mock import patch


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway import runner as runner_module  # noqa: E402
from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.runner import ControlServer  # noqa: E402
from control_gateway.scene_catalog import RobotSpawn  # noqa: E402
from control_gateway.scene_catalog import SceneCatalog  # noqa: E402
from control_gateway.scene_catalog import SceneModel  # noqa: E402
from control_gateway.scene_catalog import SceneSpec  # noqa: E402


class _FakeGazeboClient:
    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []

    def delete_entity(
        self, name: str, ignore_missing: bool = False, timeout_sec: Any = None
    ) -> None:
        self.calls.append(("delete", name))

    def spawn_entity(
        self,
        name: str,
        xml: str,
        pose: Any = None,
        reference_frame: str = "world",
        timeout_sec: Any = None,
    ) -> None:
        self.calls.append(("spawn", name, pose))

    def set_entity_state(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        reference_frame: str = "world",
        timeout_sec: Any = None,
    ) -> None:
        self.calls.append(("teleport", name, x, y, z, yaw))


class _FakeNode:
    def __init__(self, fail_load_map: bool = False) -> None:
        self.loaded_maps: list[str] = []
        self.initial_poses: list[tuple[float, float, float, str]] = []
        self.fail_load_map = fail_load_map

    def navigation_snapshot(self) -> dict[str, Any]:
        return {"state": "idle", "retiring_goals": 0}

    def load_map(self, map_url: str, timeout_sec: float = 20.0) -> None:
        if self.fail_load_map:
            raise ControlRequestError("map load failed", 503)
        self.loaded_maps.append(map_url)

    def publish_initial_pose(
        self, x: float, y: float, yaw: float, frame_id: str = "map"
    ) -> None:
        self.initial_poses.append((x, y, yaw, frame_id))


class _Cabinet:
    def __init__(self, name: str) -> None:
        self.name = name
        self.x = 2.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


def _spec(
    name: str,
    spawn_cabinet: bool,
    model: Optional[SceneModel] = None,
    nav2_map: str = "/tmp/maps/map.yaml",
    robot_spawn: Optional[RobotSpawn] = None,
) -> SceneSpec:
    return SceneSpec(
        name=name,
        spawn_cabinet=spawn_cabinet,
        model=model,
        nav2_map=nav2_map,
        robot_spawn=robot_spawn,
    )


class SceneSwitchTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)
        self.floor_urdf = self.directory / "floor.urdf"
        self.floor_urdf.write_text(
            '<robot name="floor"><link name="ground"/></robot>',
            encoding="utf-8",
        )
        self.floor_model = SceneModel(
            urdf=str(self.floor_urdf),
            pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _server(
        self,
        catalog: SceneCatalog,
        *,
        inventory: tuple[_Cabinet, ...] = (),
        active: Optional[str] = None,
        fail_load_map: bool = False,
    ) -> tuple[ControlServer, _FakeGazeboClient, _FakeNode]:
        server = object.__new__(ControlServer)
        server._scene_catalog = catalog
        server._gazebo_client = _FakeGazeboClient()
        server._node = _FakeNode(fail_load_map=fail_load_map)
        server._inventory = inventory
        server._robot_adapter = SimpleNamespace(navigation_frame="map")
        server._robot_entity_name = "xczs_inspection_robot"
        server._cabinet_xacro_path = "/nonexistent/cabinet.xacro"
        server._cabinet_controls_path = "/nonexistent/controls.yaml"
        server._scene_switch_lock = threading.Lock()
        server._active_scene = active if active is not None else catalog.names[0]
        server._request_condition = threading.Condition()
        server._stopping = False
        server._active_requests = 0
        server._task_manager = None
        server._cabinet_clients = {}
        return server, server._gazebo_client, server._node

    def test_switch_to_model_scene_orders_geometry_map_pose(self) -> None:
        cabinet = _spec("cabinet_operation", True, robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963))
        plant = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            nav2_map="/tmp/maps/plant.yaml",
            robot_spawn=RobotSpawn(2.0, 3.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([cabinet, plant])
        server, gazebo, node = self._server(
            catalog,
            inventory=(_Cabinet("cabinet_a"), _Cabinet("cabinet_b"), _Cabinet("cabinet_c")),
            active="cabinet_operation",
        )

        result = server.switch_scene("generator_plant")

        self.assertEqual("switched", result["status"])
        self.assertEqual("generator_plant", result["scene"])
        self.assertEqual("cabinet_operation", result["previous"])
        self.assertEqual("generator_plant", server._active_scene)

        # Order: load map, then delete 3 cabinets, delete-then-spawn the scene
        # floor, teleport.
        kinds = [call[0] for call in gazebo.calls]
        self.assertEqual(
            ["delete", "delete", "delete", "delete", "spawn", "teleport"],
            kinds,
        )
        self.assertEqual(
            ["cabinet_a", "cabinet_b", "cabinet_c"],
            [call[1] for call in gazebo.calls[:3]],
        )
        self.assertEqual("xczs_scene_floor", gazebo.calls[3][1])
        self.assertEqual("xczs_scene_floor", gazebo.calls[4][1])

        self.assertEqual(
            ["/tmp/maps/plant.yaml"],
            node.loaded_maps,
        )
        # base_link is a fixed +pi/2 child of the root body link, so the body
        # yaw is the requested map-frame yaw minus that offset.
        teleport = gazebo.calls[5]
        self.assertEqual("xczs_inspection_robot", teleport[1])
        self.assertAlmostEqual(2.0, teleport[2])
        self.assertAlmostEqual(3.0, teleport[3])
        self.assertAlmostEqual(0.0 - math.pi / 2.0, teleport[5])
        self.assertEqual([(2.0, 3.0, 0.0, "map")], node.initial_poses)

    def test_switching_back_spawns_cabinets_and_deletes_floor(self) -> None:
        cabinet = _spec("cabinet_operation", True, robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963))
        plant = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            robot_spawn=RobotSpawn(2.0, 2.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([cabinet, plant])
        inventory = (_Cabinet("cabinet_a"), _Cabinet("cabinet_b"))
        server, gazebo, _node = self._server(
            catalog, inventory=inventory, active="generator_plant"
        )

        with patch.object(
            runner_module, "read_button_profiles", return_value=({}, {})
        ), patch.object(
            runner_module, "build_cabinet_urdf", return_value="<robot/>"
        ):
            result = server.switch_scene("cabinet_operation")

        self.assertEqual("switched", result["status"])
        self.assertEqual("cabinet_operation", server._active_scene)

        kinds = [call[0] for call in gazebo.calls]
        # delete old floor, then per-cabinet delete-then-spawn reconciliation,
        # then teleport the robot to the cabinet scene's initial pose.
        self.assertEqual(
            ["delete", "delete", "spawn", "delete", "spawn", "teleport"],
            kinds,
        )
        self.assertEqual("xczs_scene_floor", gazebo.calls[0][1])
        spawned = [call[1] for call in gazebo.calls if call[0] == "spawn"]
        self.assertEqual(["cabinet_a", "cabinet_b"], spawned)

    def test_same_scene_is_idempotent(self) -> None:
        cabinet = _spec("cabinet_operation", True)
        catalog = SceneCatalog([cabinet])
        server, gazebo, node = self._server(catalog, active="cabinet_operation")

        result = server.switch_scene("cabinet_operation")

        self.assertEqual("unchanged", result["status"])
        self.assertEqual("cabinet_operation", result["scene"])
        self.assertEqual([], gazebo.calls)
        self.assertEqual([], node.loaded_maps)

    def test_unknown_scene_rejects(self) -> None:
        catalog = SceneCatalog([_spec("cabinet_operation", True)])
        server, _gazebo, _node = self._server(catalog)

        with self.assertRaises(ControlRequestError) as raised:
            server.switch_scene("missing_scene")
        self.assertEqual(404, raised.exception.status)

    def test_invalid_scene_name_rejects(self) -> None:
        catalog = SceneCatalog([_spec("cabinet_operation", True)])
        server, _gazebo, _node = self._server(catalog)

        with self.assertRaises(ControlRequestError) as raised:
            server.switch_scene("   ")
        self.assertEqual(400, raised.exception.status)

    def test_failed_map_load_keeps_previous_scene(self) -> None:
        cabinet = _spec("cabinet_operation", True)
        plant = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            robot_spawn=RobotSpawn(2.0, 2.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([cabinet, plant])
        server, gazebo, _node = self._server(
            catalog, active="cabinet_operation", fail_load_map=True
        )

        with self.assertRaises(ControlRequestError) as raised:
            server.switch_scene("generator_plant")
        self.assertEqual(503, raised.exception.status)
        # The active scene only advances after the whole switch succeeds.
        self.assertEqual("cabinet_operation", server._active_scene)
        # The map is loaded before any geometry change, so a rejected map leaves
        # the world untouched rather than half-switched.
        self.assertEqual([], gazebo.calls)

    def test_scenes_and_active_scene_snapshots(self) -> None:
        catalog = SceneCatalog(
            [
                _spec("cabinet_operation", True),
                _spec("generator_plant", False, model=self.floor_model),
            ]
        )
        server, _gazebo, _node = self._server(catalog, active="cabinet_operation")

        scenes = server.scenes()
        self.assertEqual("cabinet_operation", scenes["active"])
        self.assertEqual(
            ["cabinet_operation", "generator_plant"],
            [entry["name"] for entry in scenes["scenes"]],
        )

        active = server.active_scene()
        self.assertEqual("cabinet_operation", active["name"])
        self.assertTrue(active["spawn_cabinet"])


if __name__ == "__main__":
    unittest.main()
