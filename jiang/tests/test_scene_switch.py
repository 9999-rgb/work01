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
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Optional
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
        self.last_entity_pose = {"x": x, "y": y, "yaw": yaw, "frame_id": "map"}


_JOINT_NAMES = (
    "body_arm1",
    "arm1_arm2",
    "arm2_arm3",
    "arm3_arm4",
    "arm4_arm5",
    "arm5_end",
    "end_worklink1",
    "end_worklink2",
)

_HOME_POSITIONS = {
    "body_arm1": 0.0,
    "arm1_arm2": -math.pi / 2.0,
    "arm2_arm3": 0.0,
    "arm3_arm4": 0.0,
    "arm4_arm5": 0.0,
    "arm5_end": 0.0,
    "end_worklink1": 0.0,
    "end_worklink2": 0.0,
}


class _FakeNode:
    def __init__(self, fail_load_map: bool = False) -> None:
        self.loaded_maps: list[str] = []
        self.initial_poses: list[tuple[float, float, float, str]] = []
        self.planar_odom_waits: list[tuple[float, float, float]] = []
        self._gazebo_client: Optional[_FakeGazeboClient] = None
        self.fail_load_map = fail_load_map
        self.joint_state_available = True
        self.joint_positions: dict[str, float] = dict(_HOME_POSITIONS)
        self.joint_state_received_monotonic: Optional[float] = 0.0
        self.joint_targets: list[list[float]] = []

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

    def wait_for_planar_odom(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        tolerance_xy: float = 0.06,
        tolerance_yaw: float = 0.15,
        timeout_sec: float = 5.0,
    ) -> bool:
        # 测试 fixture 的遥移是瞬时的；记录请求并直接判定到位即可。
        self.planar_odom_waits.append((x, y, yaw))
        return True

    def localized_pose(self) -> Optional[Dict[str, Any]]:
        # 遥移经 gazebo client 写入的是 body 位姿；AMCL 上报的是 base_link
        # 位姿（base_link 是 body 的固定 -pi/2 子关节），因此 mock 在返回前
        # 把 body yaw 回转 pi/2，与真实 /amcl_pose 等价。
        pose = getattr(self._gazebo_client, "last_entity_pose", None)
        if pose is None:
            return None
        return {
            "x": pose["x"],
            "y": pose["y"],
            "yaw": pose["yaw"] - math.pi / 2.0,
            "frame_id": pose["frame_id"],
        }

    def set_joint_target(
        self,
        positions: list[float],
        duration_sec: float = 0.5,
        *,
        lower_limit_margin: dict[str, float] | None = None,
    ) -> list[float]:
        self.joint_targets.append(list(positions))
        self.joint_positions = {
            name: position for name, position in zip(_JOINT_NAMES, positions)
        }
        self.joint_state_received_monotonic = time.monotonic()
        return list(positions)

    def robot_joint_state_snapshot(self) -> dict[str, Any]:
        return {
            "available": self.joint_state_available,
            "positions": dict(self.joint_positions),
            "received_monotonic": self.joint_state_received_monotonic,
        }


class _Cabinet:
    def __init__(self, name: str, *, kind: str = "cabinet") -> None:
        self.name = name
        self.kind = kind
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


class _FakeProvider:
    """Stand-in for AssetSceneProvider in runner-level tests."""

    def __init__(self, specs: dict[str, SceneSpec]) -> None:
        self._specs = dict(specs)

    def resolve_scene(self, name: str) -> Optional[SceneSpec]:
        return self._specs.get(name)

    def iter_scene_specs(self):
        return iter(self._specs.values())


class SceneSwitchTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)
        self.floor_file = self.directory / "floor.sdf"
        self.floor_file.write_text(
            '<sdf version="1.6"><model name="floor"/></sdf>',
            encoding="utf-8",
        )
        self.floor_model = SceneModel(
            file=str(self.floor_file),
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
        asset_scene_provider: Any = None,
    ) -> tuple[ControlServer, _FakeGazeboClient, _FakeNode]:
        server = object.__new__(ControlServer)
        server._scene_catalog = catalog
        server._scene_specs = {scene.name: scene for scene in catalog}
        server._asset_scene_provider = asset_scene_provider
        server._gazebo_client = _FakeGazeboClient()
        server._node = _FakeNode(fail_load_map=fail_load_map)
        # 让节点能看到 gazebo client 写入的遥移位姿（见 localized_pose）。
        server._node._gazebo_client = server._gazebo_client
        server._inventory = inventory
        server._robot_adapter = SimpleNamespace(
            navigation_frame="map",
            manual_joints=tuple(
                SimpleNamespace(
                    name=name,
                    default_position=_HOME_POSITIONS[name],
                )
                for name in _JOINT_NAMES
            ),
            reset_joint_tolerance=0.02,
            reset_joint_timeout_sec=0.5,
            reset_joint_duration_sec=0.2,
        )
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
        cabinet = _spec(
            "cabinet_operation",
            True,
            robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963),
        )
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

        # Order: load map, then delete 3 cabinets, teleport the robot into the
        # cleared world, then delete-then-spawn the incoming scene floor.  The
        # teleport precedes the spawn so the incoming structure cannot collide
        # with the robot mid-home (reverse-switch homing timeout).
        kinds = [call[0] for call in gazebo.calls]
        self.assertEqual(
            ["delete", "delete", "delete", "teleport", "delete", "spawn"],
            kinds,
        )
        self.assertEqual(
            ["cabinet_a", "cabinet_b", "cabinet_c"],
            [call[1] for call in gazebo.calls[:3]],
        )
        self.assertEqual("xczs_inspection_robot", gazebo.calls[3][1])
        self.assertEqual("xczs_scene_floor", gazebo.calls[4][1])
        self.assertEqual("xczs_scene_floor", gazebo.calls[5][1])

        self.assertEqual(
            ["/tmp/maps/plant.yaml"],
            node.loaded_maps,
        )
        # base_link is a fixed -pi/2 child of the root body link, so the body
        # yaw is the requested map-frame yaw plus pi/2.
        teleport = gazebo.calls[3]
        self.assertEqual("xczs_inspection_robot", teleport[1])
        self.assertAlmostEqual(2.0, teleport[2])
        self.assertAlmostEqual(3.0, teleport[3])
        self.assertAlmostEqual(0.0 + math.pi / 2.0, teleport[5])
        self.assertEqual([(2.0, 3.0, 0.0, "map")], node.initial_poses)

    def test_switch_homes_arm_before_teleport(self) -> None:
        cabinet = _spec(
            "cabinet_operation",
            True,
            robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963),
        )
        plant = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            robot_spawn=RobotSpawn(2.0, 3.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([cabinet, plant])
        server, gazebo, node = self._server(catalog, active="cabinet_operation")
        # Leave the arm off its home pose so the switch must home it first.
        node.joint_positions["arm1_arm2"] = 0.0

        result = server.switch_scene("generator_plant")

        self.assertEqual("switched", result["status"])
        # The switch commanded the arm back to adapter defaults (arm1_arm2 is
        # the second joint) before touching geometry.
        self.assertEqual(1, len(node.joint_targets))
        self.assertAlmostEqual(-math.pi / 2.0, node.joint_targets[0][1])
        teleports = [call for call in gazebo.calls if call[0] == "teleport"]
        self.assertEqual(1, len(teleports))
        self.assertEqual("xczs_inspection_robot", teleports[0][1])

    def test_switching_back_spawns_cabinets_and_deletes_floor(self) -> None:
        cabinet = _spec(
            "cabinet_operation",
            True,
            robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963),
        )
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
        # delete old floor, teleport into the cleared world, then per-cabinet
        # delete-then-spawn reconciliation (teleport precedes the spawn so the
        # incoming cabinets cannot collide with the robot mid-home).
        self.assertEqual(
            ["delete", "teleport", "delete", "spawn", "delete", "spawn"],
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

    def test_reimport_changing_active_scene_content_triggers_reconcile(
        self,
    ) -> None:
        """A re-import that changes the active scene's definition must not be
        masked by the name-only idempotency short-circuit."""
        from control_gateway.runner import _scene_spec_signature

        plant_v1 = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            nav2_map="/tmp/maps/plant.yaml",
            robot_spawn=RobotSpawn(2.0, 3.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([plant_v1])
        server, gazebo, node = self._server(
            catalog, active="generator_plant"
        )
        # Mimic a real server that booted the world to this spec.
        server._applied_scene_signatures = {
            "generator_plant": _scene_spec_signature(plant_v1),
        }

        # Same name, same content -> historical short-circuit.
        first = server.switch_scene("generator_plant")
        self.assertEqual("unchanged", first["status"])
        self.assertEqual([], gazebo.calls)

        # A force re-import replaces the resolved spec with a new definition
        # (different map and pose) under the same scene name.
        plant_v2 = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            nav2_map="/tmp/maps/plant_v2.yaml",
            robot_spawn=RobotSpawn(5.0, 6.0, 0.515, 0.0),
        )
        server._scene_specs["generator_plant"] = plant_v2

        second = server.switch_scene("generator_plant")
        self.assertEqual("reconciled", second["status"])
        self.assertEqual("generator_plant", second["scene"])
        # The new map is actually loaded and the new pose teleported to.
        self.assertEqual(["/tmp/maps/plant_v2.yaml"], node.loaded_maps)
        self.assertEqual(
            (5.0, 6.0, 0.0, "map"),
            node.initial_poses[-1],
        )
        # The new definition is now the recorded baseline.
        self.assertEqual(
            server._applied_scene_signatures["generator_plant"],
            _scene_spec_signature(plant_v2),
        )
        # And a further re-switch to the now-applied spec is a no-op again.
        third = server.switch_scene("generator_plant")
        self.assertEqual("unchanged", third["status"])

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
            catalog,
            active="cabinet_operation",
            fail_load_map=True,
            # 真实系统库存恒有 cabinet_operation 实例；空库存会让
            # _require_cabinet_scene 的实例门控误判为“场景无夹具”。
            inventory=(_Cabinet("cabinet_a"),),
        )

        with self.assertRaises(ControlRequestError) as raised:
            server.switch_scene("generator_plant")
        self.assertEqual(503, raised.exception.status)
        # The active scene only advances after the whole switch succeeds.
        self.assertEqual("cabinet_operation", server._active_scene)
        # The map is loaded before any geometry change, so a rejected map leaves
        # the world untouched rather than half-switched.
        self.assertEqual([], gazebo.calls)
        # Because the failure happened before any mutation, the world is still
        # consistent with the recorded scene: cabinet work must NOT be locked
        # behind a false reconciliation-pending flag.  (The gate reads the
        # pending flag, so exercising it directly verifies cabinet work is open.)
        self.assertFalse(
            getattr(server, "_scene_reconcile_pending", False)
        )
        self.assertFalse(server.scenes()["reconcile_pending"])
        server._require_cabinet_scene("cabinet operation")

    def test_partial_failure_marks_reconcile_pending_and_reconciles(self) -> None:
        cabinet = _spec(
            "cabinet_operation",
            True,
            robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963),
        )
        plant = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            nav2_map="/tmp/maps/plant.yaml",
            robot_spawn=RobotSpawn(2.0, 2.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([cabinet, plant])
        inventory = (_Cabinet("cabinet_a"),)
        server, gazebo, _node = self._server(
            catalog, inventory=inventory, active="cabinet_operation"
        )

        # Simulate a mid-switch failure during the incoming floor spawn: the
        # world is on plant's map with the cabinet deleted and the robot
        # teleported, while _active_scene still names the cabinet scene.
        with patch.object(
            runner_module, "read_button_profiles", return_value=({}, {})
        ), patch.object(runner_module, "build_cabinet_urdf", return_value="<robot/>"), patch.object(
            ControlServer,
            "_spawn_scene_floor",
            side_effect=ControlRequestError("spawn timeout", 503),
        ):
            with self.assertRaises(ControlRequestError) as raised:
                server.switch_scene("generator_plant")
        self.assertEqual(503, raised.exception.status)
        self.assertEqual("cabinet_operation", server._active_scene)
        self.assertTrue(server._scene_reconcile_pending)
        self.assertTrue(server.scenes()["reconcile_pending"])

        # The failure is mid-switch: geometry was touched (cabinet deleted,
        # robot teleported) while the map already flipped to plant's.
        kinds = [call[0] for call in gazebo.calls]
        self.assertEqual(["delete", "teleport"], kinds)

        # Cabinet-scoped work is refused until a successful switch reconciles.
        with self.assertRaises(ControlRequestError) as refused:
            server.submit_operation_task(
                "cabinet_a", "box_8_button_1", "press", None, None, None
            )
        self.assertEqual(409, refused.exception.status)
        self.assertIn("reconcile", str(refused.exception))

        # Re-issuing the switch to the *recorded* scene must reconcile instead
        # of short-circuiting to "unchanged".
        with patch.object(
            runner_module, "read_button_profiles", return_value=({}, {})
        ), patch.object(
            runner_module, "build_cabinet_urdf", return_value="<robot/>"
        ):
            result = server.switch_scene("cabinet_operation")
        self.assertEqual("reconciled", result["status"])
        self.assertFalse(server._scene_reconcile_pending)
        self.assertFalse(server.scenes()["reconcile_pending"])
        self.assertEqual("cabinet_operation", server._active_scene)
        # The reconciliation re-ran the full apply (map, home, teleport,
        # delete-then-spawn the cabinets) and converged.
        self.assertEqual(
            ["/tmp/maps/plant.yaml", "/tmp/maps/map.yaml"],
            _node.loaded_maps,
        )

        # After reconciliation the idempotency short-circuit is armed again.
        again = server.switch_scene("cabinet_operation")
        self.assertEqual("unchanged", again["status"])

    def test_clean_switch_clears_reconcile_pending(self) -> None:
        cabinet = _spec(
            "cabinet_operation",
            True,
            robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963),
        )
        plant = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            nav2_map="/tmp/maps/plant.yaml",
            robot_spawn=RobotSpawn(2.0, 2.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([cabinet, plant])

        # A mid-switch failure (mutation already started) leaves the flag set;
        # a later successful switch to a different scene clears it.
        server, gazebo, node = self._server(
            catalog, active="cabinet_operation"
        )
        with patch.object(
            runner_module, "read_button_profiles", return_value=({}, {})
        ), patch.object(runner_module, "build_cabinet_urdf", return_value="<robot/>"), patch.object(
            ControlServer,
            "_spawn_scene_floor",
            side_effect=ControlRequestError("spawn timeout", 503),
        ):
            with self.assertRaises(ControlRequestError):
                server.switch_scene("generator_plant")
        self.assertTrue(server._scene_reconcile_pending)
        self.assertTrue(server.scenes()["reconcile_pending"])

        result = server.switch_scene("generator_plant")
        self.assertEqual("switched", result["status"])
        self.assertFalse(server._scene_reconcile_pending)
        self.assertEqual("generator_plant", server._active_scene)

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

    def test_scenes_lists_deleted_active_asset_scene(self) -> None:
        """删除仍处于活动状态的资产场景后，/scenes 必须继续列出它。

        资产 spec 在切换时被缓存进 ``_scene_specs``（供回切协调），删除资产
        后 provider 不再产出它；若 /scenes 漏掉活动场景，Web 端无法高亮当前
        场景，切换按钮还会把默认目标错指到列表首个场景。
        """
        cabinet = _spec(
            "cabinet_operation",
            True,
            robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963),
        )
        asset_mezzanine = _spec(
            "electrical_mezzanine",
            False,
            model=self.floor_model,
            nav2_map="/tmp/assets/maps/mezzanine.yaml",
            robot_spawn=RobotSpawn(1.0, 1.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([cabinet])
        provider = _FakeProvider({"electrical_mezzanine": asset_mezzanine})
        server, _gazebo, _node = self._server(
            catalog,
            inventory=(_Cabinet("cabinet_a"),),
            active="cabinet_operation",
            asset_scene_provider=provider,
        )
        result = server.switch_scene("electrical_mezzanine")
        self.assertEqual("switched", result["status"])
        self.assertEqual("electrical_mezzanine", server._active_scene)
        # 模拟删除资产：provider 不再产出该场景，但 _scene_specs 缓存仍在。
        provider._specs.clear()

        scenes = server.scenes()
        self.assertEqual("electrical_mezzanine", scenes["active"])
        by_name = {entry["name"]: entry for entry in scenes["scenes"]}
        self.assertEqual(
            {"cabinet_operation", "electrical_mezzanine"},
            set(by_name),
        )
        self.assertEqual("asset", by_name["electrical_mezzanine"]["source"])
        # active_scene() 仍可解析（走缓存），回切协调路径不中断。
        self.assertEqual("electrical_mezzanine", server.active_scene()["name"])

    def test_cabinet_operations_rejected_in_plant_scene(self) -> None:
        catalog = SceneCatalog(
            [
                _spec("cabinet_operation", True),
                _spec("generator_plant", False, model=self.floor_model),
            ]
        )
        server, _gazebo, _node = self._server(catalog, active="generator_plant")

        for operation in (
            lambda: server.submit_operation_task(
                "cabinet_a", "box_8_button_1", "press", None, None, None
            ),
            lambda: server.submit_reset_task("cabinet_a"),
            lambda: server.press_cabinet_button("box_8_button_1", False),
            lambda: server.submit_navigation_task("cabinet_a"),
        ):
            with self.assertRaises(ControlRequestError) as caught:
                operation()
            self.assertEqual(409, caught.exception.status)

        # The cabinet scene still admits cabinet work: the call gets past the
        # scene gate and fails at cabinet-client lookup instead (404, not 409).
        inventory = SimpleNamespace(
            get=lambda name: _Cabinet(name), names=["cabinet_a"]
        )
        cabinet_server, _gazebo2, _node2 = self._server(
            catalog, inventory=inventory, active="cabinet_operation"
        )
        with self.assertRaises(ControlRequestError) as caught2:
            cabinet_server.submit_operation_task(
                "cabinet_a", "box_8_button_1", "press", None, None, None
            )
        self.assertEqual(404, caught2.exception.status)


    def test_scenes_merges_asset_scenes_with_source_tag_and_dedupe(self) -> None:
        catalog = SceneCatalog(
            [
                _spec("cabinet_operation", True),
                _spec("generator_plant", False, model=self.floor_model),
            ]
        )
        asset_mezzanine = _spec(
            "electrical_mezzanine",
            False,
            model=self.floor_model,
            nav2_map="/tmp/assets/maps/mezzanine.yaml",
        )
        asset_cabinet = _spec(
            "cabinet_operation", True, nav2_map="/tmp/assets/maps/cabinet.yaml"
        )
        provider = _FakeProvider(
            {
                "electrical_mezzanine": asset_mezzanine,
                "cabinet_operation": asset_cabinet,
            }
        )
        server, _gazebo, _node = self._server(
            catalog, active="cabinet_operation", asset_scene_provider=provider
        )

        scenes = server.scenes()
        by_name = {entry["name"]: entry for entry in scenes["scenes"]}
        self.assertEqual(
            {"cabinet_operation", "generator_plant", "electrical_mezzanine"},
            set(by_name),
        )
        # 内置场景带 builtin 标记。
        self.assertEqual("builtin", by_name["generator_plant"]["source"])
        # 同名冲突时资产胜出：source 为 asset，nav2_map 用资产版本。
        self.assertEqual("asset", by_name["cabinet_operation"]["source"])
        self.assertEqual(
            "/tmp/assets/maps/cabinet.yaml",
            by_name["cabinet_operation"]["nav2_map"],
        )
        self.assertEqual("asset", by_name["electrical_mezzanine"]["source"])

    def test_switch_to_asset_scene_resolves_caches_and_switches_back(self) -> None:
        cabinet = _spec(
            "cabinet_operation",
            True,
            robot_spawn=RobotSpawn(0.0, 0.0, 0.515, 1.5707963),
        )
        plant = _spec(
            "generator_plant",
            False,
            model=self.floor_model,
            nav2_map="/tmp/maps/plant.yaml",
            robot_spawn=RobotSpawn(2.0, 3.0, 0.515, 0.0),
        )
        asset_mezzanine = _spec(
            "electrical_mezzanine",
            False,
            model=self.floor_model,
            nav2_map="/tmp/assets/maps/mezzanine.yaml",
            robot_spawn=RobotSpawn(1.0, 1.0, 0.515, 0.0),
        )
        catalog = SceneCatalog([cabinet, plant])
        provider = _FakeProvider({"electrical_mezzanine": asset_mezzanine})
        server, gazebo, node = self._server(
            catalog,
            inventory=(_Cabinet("cabinet_a"),),
            active="cabinet_operation",
            asset_scene_provider=provider,
        )

        result = server.switch_scene("electrical_mezzanine")
        self.assertEqual("switched", result["status"])
        self.assertEqual("electrical_mezzanine", result["scene"])
        self.assertEqual("cabinet_operation", result["previous"])
        self.assertEqual("electrical_mezzanine", server._active_scene)
        # 资产场景已缓存进注册表，previous 查询与回切均命中。
        self.assertIn("electrical_mezzanine", server._scene_specs)

        back = server.switch_scene("generator_plant")
        self.assertEqual("switched", back["status"])
        self.assertEqual("generator_plant", server._active_scene)

        again = server.switch_scene("electrical_mezzanine")
        self.assertEqual("switched", again["status"])
        self.assertEqual("electrical_mezzanine", server._active_scene)

    def test_switch_to_unknown_still_404_when_neither_catalog_nor_asset(
        self,
    ) -> None:
        catalog = SceneCatalog([_spec("cabinet_operation", True)])
        provider = _FakeProvider(
            {"electrical_mezzanine": _spec("electrical_mezzanine", False)}
        )
        server, _gazebo, _node = self._server(
            catalog, asset_scene_provider=provider
        )

        with self.assertRaises(ControlRequestError) as raised:
            server.switch_scene("missing_scene")
        self.assertEqual(404, raised.exception.status)

    def test_active_scene_works_for_asset_scene(self) -> None:
        catalog = SceneCatalog([_spec("cabinet_operation", True)])
        asset_mezzanine = _spec(
            "electrical_mezzanine",
            False,
            model=self.floor_model,
            nav2_map="/tmp/assets/maps/mezzanine.yaml",
        )
        provider = _FakeProvider({"electrical_mezzanine": asset_mezzanine})
        server, _gazebo, _node = self._server(
            catalog, active="cabinet_operation", asset_scene_provider=provider
        )
        server._active_scene = "electrical_mezzanine"

        active = server.active_scene()
        self.assertEqual("electrical_mezzanine", active["name"])
        self.assertFalse(active["spawn_cabinet"])


if __name__ == "__main__":
    unittest.main()
