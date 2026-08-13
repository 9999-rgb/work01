"""Contracts for standalone Web helpers and isolated Zenoh clients."""

from __future__ import annotations

import argparse
import io
import sys
import types
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest.mock import MagicMock, patch


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))


class _Config:
    def __init__(self) -> None:
        self.inserts: list[tuple[str, str]] = []

    def insert_json5(self, key: str, value: str) -> None:
        self.inserts.append((key, value))


class WebAuxiliaryContractTest(unittest.TestCase):
    def test_standalone_http_servers_default_to_loopback(self) -> None:
        from sensor_bridge.runner import _build_argument_parser as sensor_parser
        from sse_bridge import _build_argument_parser as sse_parser

        self.assertEqual(sensor_parser().parse_args([]).host, "127.0.0.1")
        self.assertEqual(sse_parser().parse_args([]).host, "127.0.0.1")
        self.assertEqual(
            sse_parser().parse_args(["--host", "192.0.2.10"]).host,
            "192.0.2.10",
        )

    def test_proxy_legacy_control_is_disabled_by_default(self) -> None:
        from run_xczs_proxy import _build_argument_parser

        self.assertEqual(
            _build_argument_parser().parse_args([]).control_port,
            0,
        )

    def test_legacy_control_is_loopback_only_and_warns(self) -> None:
        from run_xczs_proxy import _start_legacy_control_server

        server = MagicMock()
        control_gateway = types.ModuleType("control_gateway")
        control_gateway.ControlServer = MagicMock(return_value=server)
        warning = io.StringIO()
        with (
            patch.dict(sys.modules, {"control_gateway": control_gateway}),
            redirect_stderr(warning),
        ):
            returned = _start_legacy_control_server(18090)

        self.assertIs(returned, server)
        control_gateway.ControlServer.assert_called_once_with(
            host="127.0.0.1",
            port=18090,
        )
        server.start.assert_called_once_with()
        self.assertIn("unauthenticated", warning.getvalue())
        self.assertIn("loopback", warning.getvalue())

    def test_legacy_control_start_failure_stops_constructed_server(self) -> None:
        from run_xczs_proxy import _start_legacy_control_server

        server = MagicMock()
        server.start.side_effect = OSError("address in use")
        control_gateway = types.ModuleType("control_gateway")
        control_gateway.ControlServer = MagicMock(return_value=server)
        with (
            patch.dict(sys.modules, {"control_gateway": control_gateway}),
            redirect_stderr(io.StringIO()),
            self.assertRaisesRegex(OSError, "address in use"),
        ):
            _start_legacy_control_server(18090)
        server.stop.assert_called_once_with()

    def test_proxy_connection_failure_closes_runner_then_control_server(
        self,
    ) -> None:
        import run_xczs_proxy

        parser = MagicMock()
        parser.parse_args.return_value = argparse.Namespace(
            host="127.0.0.1",
            port=17447,
            control_port=18090,
            list=False,
        )
        events: list[str] = []
        control = MagicMock()
        control.stop.side_effect = lambda: events.append("control.stop")
        runner = MagicMock()
        runner.connect.side_effect = RuntimeError("router unavailable")
        runner.close.side_effect = lambda: events.append("runner.close")

        with (
            patch.object(run_xczs_proxy, "_build_argument_parser", return_value=parser),
            patch.object(run_xczs_proxy, "register_standard_types"),
            patch.object(run_xczs_proxy, "register_xczs_topics"),
            patch.object(
                run_xczs_proxy,
                "_start_legacy_control_server",
                return_value=control,
            ),
            patch("zenoh_proxy.runner.ProxyRunner", return_value=runner),
            self.assertRaisesRegex(RuntimeError, "router unavailable"),
        ):
            run_xczs_proxy.main()

        self.assertEqual(events, ["runner.close", "control.stop"])

    def test_shared_key_normalizer_accepts_only_exact_ros_topics(self) -> None:
        from zenoh_key import normalize_ros_key

        self.assertEqual(
            normalize_ros_key("xczs/joint_states/json"),
            "xczs/joint_states",
        )
        for key in ("", "/xczs/odom", "xczs//odom", "xczs/*", "xczs/../odom"):
            with self.subTest(key=key), self.assertRaises(ValueError):
                normalize_ros_key(key)

    def test_zenoh_source_is_client_only_without_multicast_scouting(self) -> None:
        import sse_bridge

        config = _Config()
        session = MagicMock()
        with (
            patch.object(sse_bridge.zenoh, "Config", return_value=config),
            patch.object(sse_bridge.zenoh, "open", return_value=session),
        ):
            source = sse_bridge.ZenohSource("tcp/127.0.0.1:17447")
            source.close()

        self.assertEqual(
            config.inserts,
            [
                ("mode", '"client"'),
                ("scouting/multicast/enabled", "false"),
                ("connect/endpoints", '["tcp/127.0.0.1:17447"]'),
            ],
        )
        session.close.assert_called_once_with()

    def test_sse_bind_failure_closes_zenoh_source(self) -> None:
        import sse_bridge

        parser = MagicMock()
        parser.parse_args.return_value = argparse.Namespace(
            host="127.0.0.1",
            port=18001,
            zenoh="tcp/127.0.0.1:17447",
        )
        source = MagicMock()
        with (
            patch.object(sse_bridge, "_build_argument_parser", return_value=parser),
            patch.object(sse_bridge, "ZenohSource", return_value=source),
            patch.object(
                sse_bridge,
                "ThreadingHTTPServer",
                side_effect=OSError("address in use"),
            ),
            redirect_stderr(io.StringIO()),
            self.assertRaisesRegex(OSError, "address in use"),
        ):
            sse_bridge.main()
        source.close.assert_called_once_with()
        self.assertIsNone(sse_bridge._zenoh_source)

    def test_sensor_app_setup_failure_stops_started_ros_runtime(self) -> None:
        import sensor_bridge.runner as sensor_runner

        parser = MagicMock()
        parser.parse_args.return_value = argparse.Namespace(
            host="127.0.0.1",
            port=18003,
            camera_topic="/camera",
            lidar_topic="/scan",
            jpeg_quality=80,
            camera_fps=10.0,
            lidar_fps=10.0,
        )
        runtime = MagicMock()
        runtime_builder = MagicMock()
        runtime_builder.return_value.start.return_value = runtime
        ros_node = types.ModuleType("sensor_bridge.ros_node")
        ros_node.SensorRosRuntime = runtime_builder
        state_module = types.ModuleType("sensor_bridge.state")
        state_module.SensorStreamState = MagicMock
        web_server = types.ModuleType("sensor_bridge.web_server")
        web_server.create_sensor_app = MagicMock(
            side_effect=RuntimeError("app setup failed")
        )
        with (
            patch.object(sensor_runner, "_build_argument_parser", return_value=parser),
            patch.dict(
                sys.modules,
                {
                    "sensor_bridge.ros_node": ros_node,
                    "sensor_bridge.state": state_module,
                    "sensor_bridge.web_server": web_server,
                },
            ),
            self.assertRaisesRegex(RuntimeError, "app setup failed"),
        ):
            sensor_runner.main()
        runtime.stop.assert_called_once_with()

    def test_proxy_runner_is_client_only_without_multicast_scouting(self) -> None:
        import zenoh
        from zenoh_proxy import runner as runner_module

        config = _Config()
        session = MagicMock()
        subscriptions = MagicMock()
        with (
            patch.object(zenoh, "Config", return_value=config),
            patch.object(zenoh, "open", return_value=session),
            patch.object(
                runner_module,
                "SubscriptionManager",
                return_value=subscriptions,
            ),
        ):
            runner = runner_module.ProxyRunner(host="127.0.0.1", port=17447)
            runner.connect()
            runner.close()

        self.assertEqual(
            config.inserts,
            [
                ("mode", '"client"'),
                ("scouting/multicast/enabled", "false"),
                ("connect/endpoints", '["tcp/127.0.0.1:17447"]'),
            ],
        )
        subscriptions.close.assert_called_once_with()
        session.close.assert_called_once_with()

    def test_proxy_runner_closes_session_if_manager_setup_fails(self) -> None:
        import zenoh
        from zenoh_proxy import runner as runner_module

        config = _Config()
        session = MagicMock()
        with (
            patch.object(zenoh, "Config", return_value=config),
            patch.object(zenoh, "open", return_value=session),
            patch.object(
                runner_module,
                "SubscriptionManager",
                side_effect=RuntimeError("manager setup failed"),
            ),
        ):
            runner = runner_module.ProxyRunner()
            with self.assertRaisesRegex(RuntimeError, "manager setup failed"):
                runner.connect()

        session.close.assert_called_once_with()
        self.assertIsNone(runner.session)

    def test_proxy_runner_closes_session_if_subscription_cleanup_fails(self) -> None:
        from zenoh_proxy import runner as runner_module

        runner = runner_module.ProxyRunner()
        subscriptions = MagicMock()
        subscriptions.close.side_effect = RuntimeError("cleanup failed")
        session = MagicMock()
        runner._sub_mgr = subscriptions
        runner._session = session

        with self.assertRaisesRegex(RuntimeError, "cleanup failed"):
            runner.close()
        session.close.assert_called_once_with()
        self.assertIsNone(runner.session)
        self.assertIsNone(runner._sub_mgr)


if __name__ == "__main__":
    unittest.main()
