"""传感器缺帧和异常断开时的 HTTP/WebSocket 合同。"""

from __future__ import annotations

import asyncio
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

from starlette.websockets import WebSocketDisconnect


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))


class SensorFailureContractTest(unittest.TestCase):
    def test_health_is_degraded_until_both_streams_are_ready(self) -> None:
        from sensor_bridge.state import SensorStreamState

        state = SensorStreamState()
        self.assertEqual(state.health()["status"], "degraded")

        state.update_camera(b"jpeg", {"frame_id": "camera"})
        camera_only = state.health()
        self.assertEqual(camera_only["status"], "degraded")
        self.assertTrue(camera_only["camera"]["ready"])
        self.assertFalse(camera_only["lidar"]["ready"])

        state.update_lidar({"ranges": [1.0], "sample_count": 1})
        self.assertEqual(state.health()["status"], "ok")

    def test_stale_sensor_samples_are_not_reported_as_ready(self) -> None:
        from sensor_bridge.state import SensorStreamState

        state = SensorStreamState()
        with patch("sensor_bridge.state.time.monotonic", return_value=10.0):
            state.update_camera(b"jpeg", {"frame_id": "camera"})
            state.update_lidar({"ranges": [1.0], "sample_count": 1})
        with patch("sensor_bridge.state.time.monotonic", return_value=100.0):
            health = state.health()

        self.assertEqual(health["status"], "degraded")
        self.assertFalse(health["camera"]["ready"])
        self.assertTrue(health["camera"]["stale"])
        self.assertFalse(health["lidar"]["ready"])
        self.assertTrue(health["lidar"]["stale"])

    def test_stale_threshold_is_configurable_and_uses_monotonic_time(self) -> None:
        from sensor_bridge.state import SensorStreamState

        state = SensorStreamState(stale_after_seconds=2.0)
        with patch("sensor_bridge.state.time.monotonic", return_value=0.0):
            state.update_camera(b"jpeg", {"frame_id": "camera"})
        with patch("sensor_bridge.state.time.monotonic", return_value=2.0):
            self.assertTrue(state.health()["camera"]["ready"])
        with patch("sensor_bridge.state.time.monotonic", return_value=2.01):
            self.assertFalse(state.health()["camera"]["ready"])

    def test_fastapi_camera_endpoint_rejects_a_stale_last_frame(self) -> None:
        from fastapi.testclient import TestClient

        from app.main import create_app
        from sensor_bridge.state import SensorStreamState

        state = SensorStreamState()
        with patch("sensor_bridge.state.time.monotonic", return_value=10.0):
            state.update_camera(b"jpeg", {"frame_id": "camera"})
        app = create_app(
            sensor_state=state,
            enable_db=False,
            auth_enabled=False,
        )
        with (
            patch("sensor_bridge.state.time.monotonic", return_value=100.0),
            TestClient(app) as client,
        ):
            response = client.get("/camera.jpg")
        self.assertEqual(response.status_code, 503)

    def test_fastapi_camera_stream_rejects_missing_initial_frame(self) -> None:
        from fastapi.testclient import TestClient

        from app.main import create_app
        from sensor_bridge.state import SensorStreamState

        app = create_app(
            sensor_state=SensorStreamState(),
            enable_db=False,
            auth_enabled=False,
        )
        client = TestClient(app)

        health = client.get("/sensors/health")
        self.assertEqual(health.status_code, 200)
        self.assertEqual(health.json()["status"], "degraded")
        stream = client.get("/camera.mjpg")
        self.assertEqual(stream.status_code, 503)
        self.assertEqual(stream.json(), {"error": "相机帧不可用。"})

    def test_abrupt_websocket_disconnect_has_idempotent_cleanup(self) -> None:
        from app.sensors.router import lidar_websocket
        from sensor_bridge.state import SensorStreamState

        state = SensorStreamState()
        state.update_lidar({"ranges": [1.0], "sample_count": 1})
        websocket = MagicMock()
        websocket.app = types.SimpleNamespace(
            state=types.SimpleNamespace(
                allowed_origins=frozenset(),
                auth_enabled=False,
                sensor_state=state,
            )
        )
        websocket.headers = {
            "host": "127.0.0.1:8090",
            "origin": "http://127.0.0.1:8090",
        }
        websocket.query_params = {}
        websocket.accept = AsyncMock()
        websocket.send_text = AsyncMock()
        websocket.receive = AsyncMock(
            side_effect=WebSocketDisconnect(code=1006)
        )
        websocket.close = AsyncMock(
            side_effect=WebSocketDisconnect(code=1006)
        )

        asyncio.run(lidar_websocket(websocket))

        websocket.accept.assert_awaited_once_with()
        websocket.send_text.assert_awaited_once_with(
            json.dumps(
                {"ranges": [1.0], "sample_count": 1},
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":"),
            )
        )
        websocket.close.assert_awaited_once_with()

    def test_websocket_ignores_binary_client_messages_and_still_cleans_up(
        self,
    ) -> None:
        from app.sensors.router import lidar_websocket
        from sensor_bridge.state import SensorStreamState

        state = SensorStreamState()
        state.update_lidar({"ranges": [1.0], "sample_count": 1})
        websocket = MagicMock()
        websocket.app = types.SimpleNamespace(
            state=types.SimpleNamespace(
                allowed_origins=frozenset(),
                auth_enabled=False,
                sensor_state=state,
            )
        )
        websocket.headers = {
            "host": "127.0.0.1:8090",
            "origin": "http://127.0.0.1:8090",
        }
        websocket.query_params = {}
        websocket.accept = AsyncMock()
        websocket.send_text = AsyncMock()
        websocket.receive = AsyncMock(
            side_effect=[
                {"type": "websocket.receive", "bytes": b"probe"},
                {"type": "websocket.disconnect", "code": 1000},
            ]
        )
        websocket.close = AsyncMock()

        asyncio.run(lidar_websocket(websocket))

        self.assertEqual(websocket.receive.await_count, 2)
        websocket.close.assert_awaited_once_with()

    def test_websocket_closes_when_the_last_lidar_scan_becomes_stale(self) -> None:
        from app.sensors.router import lidar_websocket
        from sensor_bridge.state import SensorStreamState

        state = SensorStreamState(stale_after_seconds=1.0)
        with patch("sensor_bridge.state.time.monotonic", return_value=10.0):
            state.update_lidar({"ranges": [1.0], "sample_count": 1})
        websocket = MagicMock()
        websocket.app = types.SimpleNamespace(
            state=types.SimpleNamespace(
                allowed_origins=frozenset(),
                auth_enabled=False,
                sensor_state=state,
            )
        )
        websocket.headers = {
            "host": "127.0.0.1:8090",
            "origin": "http://127.0.0.1:8090",
        }
        websocket.query_params = {}
        websocket.accept = AsyncMock()
        websocket.send_text = AsyncMock()
        websocket.receive = AsyncMock()
        websocket.close = AsyncMock()

        with patch("sensor_bridge.state.time.monotonic", return_value=12.0):
            asyncio.run(lidar_websocket(websocket))

        websocket.close.assert_awaited_once_with(
            code=1013,
            reason="LiDAR 扫描已过期。",
        )
        websocket.receive.assert_not_awaited()

    def test_stale_websocket_close_tolerates_a_disappearing_peer(self) -> None:
        from app.sensors.router import lidar_websocket

        state = MagicMock()
        state.fresh_lidar_snapshot.return_value = (1, None, None)
        state.health.return_value = {"lidar": {"stale": True}}
        websocket = MagicMock()
        websocket.app = types.SimpleNamespace(
            state=types.SimpleNamespace(
                allowed_origins=frozenset(),
                auth_enabled=False,
                sensor_state=state,
            )
        )
        websocket.headers = {
            "host": "127.0.0.1:8090",
            "origin": "http://127.0.0.1:8090",
        }
        websocket.query_params = {}
        websocket.accept = AsyncMock()
        websocket.close = AsyncMock(
            side_effect=RuntimeError("peer already closed")
        )
        websocket.receive = AsyncMock()

        asyncio.run(lidar_websocket(websocket))

        websocket.close.assert_awaited_once_with(
            code=1013,
            reason="LiDAR 扫描已过期。",
        )
        websocket.receive.assert_not_awaited()

    def test_auth_expiry_close_tolerates_a_disappearing_peer(self) -> None:
        from app.sensors.router import lidar_websocket

        authorization = MagicMock()
        authorization.is_valid = AsyncMock(return_value=False)
        websocket = MagicMock()
        websocket.app = types.SimpleNamespace(
            state=types.SimpleNamespace(sensor_state=MagicMock())
        )
        websocket.accept = AsyncMock()
        websocket.close = AsyncMock(
            side_effect=RuntimeError("peer already closed")
        )

        with patch(
            "app.sensors.router._authorize_websocket",
            new=AsyncMock(return_value=authorization),
        ):
            asyncio.run(lidar_websocket(websocket))

        websocket.close.assert_awaited_once_with(
            code=4401,
            reason="认证已失效。",
        )


if __name__ == "__main__":
    unittest.main()
