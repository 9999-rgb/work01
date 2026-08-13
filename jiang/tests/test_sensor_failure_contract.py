"""传感器缺帧和异常断开时的 HTTP/WebSocket 合同。"""

from __future__ import annotations

import asyncio
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

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

    def test_standalone_camera_stream_rejects_missing_initial_frame(self) -> None:
        from sensor_bridge.state import SensorStreamState
        from sensor_bridge.web_server import SensorWebHandlers

        handlers = SensorWebHandlers(
            state=SensorStreamState(),
            stream_fps=10.0,
            lidar_fps=10.0,
        )
        response = asyncio.run(handlers.camera_mjpeg(MagicMock()))
        self.assertEqual(response.status, 503)

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
        websocket.receive_text = AsyncMock(
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


if __name__ == "__main__":
    unittest.main()
