"""Contracts for standalone Web helpers and isolated Zenoh clients."""

from __future__ import annotations

import argparse
import io
import sys
import threading
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
    def test_standalone_sse_server_defaults_to_loopback(self) -> None:
        from sse_bridge import _build_argument_parser as sse_parser

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

    def test_zenoh_source_releases_lock_before_undeclaring(self) -> None:
        import sse_bridge

        config = _Config()
        session = MagicMock()
        lock_available: list[bool] = []
        lock_threads: list[threading.Thread] = []
        source = None

        class _Subscriber:
            def undeclare(self) -> None:
                assert source is not None
                acquired = threading.Event()

                def try_lock() -> None:
                    source._lock.acquire()
                    acquired.set()
                    source._lock.release()

                worker = threading.Thread(target=try_lock)
                lock_threads.append(worker)
                worker.start()
                lock_available.append(acquired.wait(timeout=0.2))

        session.declare_subscriber.side_effect = (
            lambda *_args, **_kwargs: _Subscriber()
        )
        with (
            patch.object(sse_bridge.zenoh, "Config", return_value=config),
            patch.object(sse_bridge.zenoh, "open", return_value=session),
        ):
            source = sse_bridge.ZenohSource("tcp/127.0.0.1:17447")

            def callback(*_args) -> None:
                return None

            source.add_listener("xczs/odom", callback)
            source.remove_listener("xczs/odom", callback)
            source.add_listener("xczs/joint_states", callback)
            source.close()
            source.close()

        for worker in lock_threads:
            worker.join(timeout=1.0)
            self.assertFalse(worker.is_alive())
        self.assertEqual(lock_available, [True, True])
        session.close.assert_called_once_with()

    def test_zenoh_source_replays_last_sample_to_late_listener(self) -> None:
        import sse_bridge

        config = _Config()
        session = MagicMock()
        with (
            patch.object(sse_bridge.zenoh, "Config", return_value=config),
            patch.object(sse_bridge.zenoh, "open", return_value=session),
        ):
            source = sse_bridge.ZenohSource("tcp/127.0.0.1:17447")

            seen: list[tuple[str, str]] = []

            # First listener establishes the subscription and seeds the cache
            # exactly as _make_callback would after a decoded sample.
            source.add_listener("xczs/odom", lambda *a: seen.append(a))
            source._last["xczs/odom"] = ("xczs/odom", '{"x":1}')

            # A second listener joins the already-live key; it must be replayed
            # the cached last sample rather than waiting for the next publish.
            source.add_listener("xczs/odom", lambda *a: seen.append(a))

            source.close()

        # The replay is only for the late listener, not the original one.
        self.assertEqual(seen, [("xczs/odom", '{"x":1}')])
        session.declare_subscriber.assert_called_once()

    def test_zenoh_source_does_not_replay_when_no_sample_seen(self) -> None:
        import sse_bridge

        config = _Config()
        session = MagicMock()
        with (
            patch.object(sse_bridge.zenoh, "Config", return_value=config),
            patch.object(sse_bridge.zenoh, "open", return_value=session),
        ):
            source = sse_bridge.ZenohSource("tcp/127.0.0.1:17447")
            seen: list[tuple[str, str]] = []
            source.add_listener("xczs/odom", lambda *a: seen.append(a))
            source.close()

        self.assertEqual(seen, [])
        session.declare_subscriber.assert_called_once()

    def test_topic_globstar_matches_zero_or_more_segments(self) -> None:
        from zenoh_proxy.registry import TopicRegistry

        registry = TopicRegistry()
        message_type = type("Message", (), {})
        middle = registry.register("robot/**/pose", message_type)
        suffix = registry.register("diagnostics/**", message_type)
        prefix = registry.register("**/status", message_type)

        for registration, matching in (
            (middle, ("robot/pose", "robot/arm/pose", "robot/a/b/pose")),
            (suffix, ("diagnostics", "diagnostics/cpu")),
            (prefix, ("status", "robot/status")),
        ):
            for topic in matching:
                with self.subTest(pattern=registration.pattern, topic=topic):
                    self.assertIsNotNone(registration.regex.fullmatch(topic))

        for registration, nonmatching in (
            (middle, ("robot//pose", "robot/arm/state", "/robot/pose")),
            (suffix, ("diagnostic", "diagnostics/", "x/diagnostics")),
            (prefix, ("status/", "/status", "robot/status/extra")),
        ):
            for topic in nonmatching:
                with self.subTest(pattern=registration.pattern, topic=topic):
                    self.assertIsNone(registration.regex.fullmatch(topic))

        only = registry.register("**", message_type)
        adjacent = registry.register("root/**/**/leaf", message_type)
        for topic in ("", "one", "one/two"):
            self.assertIsNotNone(only.regex.fullmatch(topic))
        for topic in ("root/leaf", "root/a/leaf", "root/a/b/leaf"):
            self.assertIsNotNone(adjacent.regex.fullmatch(topic))
        for topic in ("one/", "/one", "root//leaf"):
            self.assertIsNone(only.regex.fullmatch(topic))
            self.assertIsNone(adjacent.regex.fullmatch(topic))

    def test_proxy_close_stops_an_external_spin_loop(self) -> None:
        from zenoh_proxy.runner import ProxyRunner

        runner = ProxyRunner()
        runner._running = True
        runner.close()
        self.assertFalse(runner.is_running)

    def test_proxy_rejects_spin_before_connect_completes(self) -> None:
        from zenoh_proxy.runner import ProxyRunner

        runner = ProxyRunner()
        with self.assertRaisesRegex(RuntimeError, "before connect"):
            runner.spin()
        runner._connecting = True
        with self.assertRaisesRegex(RuntimeError, "before connect"):
            runner.spin()

    def test_proxy_background_spin_can_be_stopped_and_joined(self) -> None:
        import time

        from zenoh_proxy.runner import ProxyRunner

        runner = ProxyRunner()
        runner._session = MagicMock()
        worker = threading.Thread(target=runner.spin, daemon=True)
        worker.start()
        deadline = time.monotonic() + 1.0
        while not runner.is_running and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertTrue(runner.is_running)
        runner.close()
        worker.join(timeout=2.0)
        self.assertFalse(worker.is_alive())
        self.assertFalse(runner.is_running)

    def test_proxy_rejects_two_concurrent_spin_owners(self) -> None:
        import time

        from zenoh_proxy.runner import ProxyRunner

        runner = ProxyRunner()
        runner._session = MagicMock()
        start = threading.Barrier(3)
        errors: list[BaseException] = []

        def spin() -> None:
            start.wait()
            try:
                runner.spin()
            except BaseException as error:
                errors.append(error)

        workers = [threading.Thread(target=spin, daemon=True) for _ in range(2)]
        for worker in workers:
            worker.start()
        start.wait()
        deadline = time.monotonic() + 1.0
        while len(errors) < 1 and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertEqual(len(errors), 1)
        self.assertRegex(str(errors[0]), "already spinning")
        self.assertTrue(runner.is_running)
        runner.close()
        for worker in workers:
            worker.join(timeout=2.0)
            self.assertFalse(worker.is_alive())

    def test_proxy_rejects_reconnect_until_stopped_spin_has_exited(self) -> None:
        from zenoh_proxy import runner as runner_module

        runner = runner_module.ProxyRunner()
        runner._session = MagicMock()
        sleeping = threading.Event()
        release_sleep = threading.Event()

        def controlled_sleep(_seconds: float) -> None:
            sleeping.set()
            release_sleep.wait(timeout=2.0)

        with patch.object(runner_module.time, "sleep", controlled_sleep):
            worker = threading.Thread(target=runner.spin, daemon=True)
            worker.start()
            self.assertTrue(sleeping.wait(timeout=1.0))
            self.assertTrue(runner.is_running)

            runner.close()
            with self.assertRaisesRegex(RuntimeError, "previous spin loop"):
                runner.connect()

            release_sleep.set()
            worker.join(timeout=2.0)
            self.assertFalse(worker.is_alive())

    def test_proxy_subscriptions_are_serialized_with_close(self) -> None:
        from zenoh_proxy.runner import ProxyRunner

        for method_name, manager_method, args in (
            ("subscribe", "subscribe_multiple", (["xczs/odom"],)),
            (
                "subscribe_all_registered",
                "subscribe_all_registered",
                (),
            ),
        ):
            with self.subTest(method=method_name):
                runner = ProxyRunner()
                manager = MagicMock()
                session = MagicMock()
                entered = threading.Event()
                release = threading.Event()
                close_finished = threading.Event()
                events: list[str] = []

                def blocked_subscribe(*_args) -> None:
                    events.append("subscribe.start")
                    entered.set()
                    self.assertTrue(release.wait(timeout=2.0))
                    events.append("subscribe.end")

                getattr(manager, manager_method).side_effect = blocked_subscribe
                manager.close.side_effect = lambda: events.append("close")
                runner._sub_mgr = manager
                runner._session = session

                subscriber = threading.Thread(
                    target=lambda: getattr(runner, method_name)(*args),
                    daemon=True,
                )
                closer = threading.Thread(
                    target=lambda: (runner.close(), close_finished.set()),
                    daemon=True,
                )
                subscriber.start()
                self.assertTrue(entered.wait(timeout=1.0))
                closer.start()
                self.assertFalse(close_finished.wait(timeout=0.05))
                manager.close.assert_not_called()

                release.set()
                subscriber.join(timeout=2.0)
                closer.join(timeout=2.0)
                self.assertFalse(subscriber.is_alive())
                self.assertFalse(closer.is_alive())
                self.assertEqual(
                    events,
                    ["subscribe.start", "subscribe.end", "close"],
                )

    def test_proxy_spin_print_failure_still_cleans_claimed_generation(self) -> None:
        import builtins

        from zenoh_proxy import runner as runner_module

        runner = runner_module.ProxyRunner()
        runner._sub_mgr = MagicMock()
        runner._session = MagicMock()
        real_print = builtins.print
        print_calls = 0

        def fail_first_print(*args, **kwargs) -> None:
            nonlocal print_calls
            print_calls += 1
            if print_calls == 1:
                raise RuntimeError("startup print failed")
            real_print(*args, **kwargs)

        with (
            patch("builtins.print", side_effect=fail_first_print),
            self.assertRaisesRegex(RuntimeError, "startup print failed"),
        ):
            runner.spin()

        self.assertFalse(runner.is_running)
        self.assertFalse(runner._spin_active)
        self.assertIsNone(runner.session)
        self.assertIsNone(runner._sub_mgr)

    def test_proxy_spin_signal_setup_failures_clean_and_restore(self) -> None:
        from zenoh_proxy import runner as runner_module

        for failure_point in ("getsignal", "signal"):
            with self.subTest(failure_point=failure_point):
                runner = runner_module.ProxyRunner()
                manager = MagicMock()
                session = MagicMock()
                runner._sub_mgr = manager
                runner._session = session
                original_handler = object()
                signal_calls: list[object] = []

                def install_or_restore(_sig, handler) -> None:
                    signal_calls.append(handler)
                    if len(signal_calls) == 1:
                        raise RuntimeError("signal install failed")

                getsignal = (
                    MagicMock(side_effect=RuntimeError("getsignal failed"))
                    if failure_point == "getsignal"
                    else MagicMock(return_value=original_handler)
                )
                signal_side_effect = (
                    MagicMock()
                    if failure_point == "getsignal"
                    else install_or_restore
                )
                with (
                    patch.object(
                        runner_module.signal,
                        "getsignal",
                        getsignal,
                    ),
                    patch.object(
                        runner_module.signal,
                        "signal",
                        side_effect=signal_side_effect,
                    ),
                    self.assertRaisesRegex(
                        RuntimeError,
                        f"{failure_point} failed|signal install failed",
                    ),
                ):
                    runner.spin()

                manager.close.assert_called_once_with()
                session.close.assert_called_once_with()
                self.assertFalse(runner.is_running)
                self.assertFalse(runner._spin_active)
                if failure_point == "getsignal":
                    self.assertEqual(signal_calls, [])
                else:
                    self.assertEqual(len(signal_calls), 2)
                    self.assertIs(signal_calls[-1], original_handler)

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

    def test_sse_subscription_failure_returns_503_before_200(self) -> None:
        import sse_bridge

        source = MagicMock()
        source.add_listener.side_effect = RuntimeError("router unavailable")
        handler = object.__new__(sse_bridge.SSEHandler)
        handler.path = "/xczs/odom"
        handler.send_error = MagicMock()
        handler.send_response = MagicMock()

        with patch.object(sse_bridge, "_zenoh_source", source):
            handler.do_GET()

        handler.send_error.assert_called_once_with(
            503,
            "Zenoh subscription is unavailable",
        )
        handler.send_response.assert_not_called()
        source.remove_listener.assert_not_called()

    def test_sse_response_and_cleanup_use_the_registered_local_source(
        self,
    ) -> None:
        import sse_bridge

        source = MagicMock()
        replacement = MagicMock()
        events: list[str] = []
        handler = object.__new__(sse_bridge.SSEHandler)
        handler.path = "/xczs/odom"
        handler.send_header = MagicMock()
        handler.end_headers = MagicMock()
        handler.wfile = MagicMock()
        handler.wfile.write.side_effect = BrokenPipeError

        def add_listener(_key, callback) -> None:
            events.append("add")
            sse_bridge._zenoh_source = replacement
            callback("xczs/odom", '{"position": 1}')

        def send_response(status) -> None:
            events.append(f"response:{status}")

        source.add_listener.side_effect = add_listener
        source.remove_listener.side_effect = (
            lambda *_args: events.append("remove")
        )
        handler.send_response = MagicMock(side_effect=send_response)

        with patch.object(sse_bridge, "_zenoh_source", source):
            handler.do_GET()

        self.assertEqual(events, ["add", "response:200", "remove"])
        source.remove_listener.assert_called_once()
        replacement.remove_listener.assert_not_called()

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

    def test_proxy_runner_can_retry_after_configuration_failure(self) -> None:
        import zenoh
        from zenoh_proxy import runner as runner_module

        config = _Config()
        session = MagicMock()
        subscriptions = MagicMock()
        with (
            patch.object(
                zenoh,
                "Config",
                side_effect=[RuntimeError("bad config"), config],
            ),
            patch.object(zenoh, "open", return_value=session),
            patch.object(
                runner_module,
                "SubscriptionManager",
                return_value=subscriptions,
            ),
        ):
            runner = runner_module.ProxyRunner()
            with self.assertRaisesRegex(RuntimeError, "bad config"):
                runner.connect()
            runner.connect()
            runner.close()

        subscriptions.close.assert_called_once_with()
        session.close.assert_called_once_with()

    def test_proxy_close_cancels_blocked_connect_without_touching_next_generation(
        self,
    ) -> None:
        import zenoh
        from zenoh_proxy import runner as runner_module

        runner = runner_module.ProxyRunner()
        old_open_started = threading.Event()
        release_old_open = threading.Event()
        old_cleanup_started = threading.Event()
        release_old_cleanup = threading.Event()
        old_session = MagicMock()
        new_session = MagicMock()
        old_subscriptions = MagicMock()
        new_subscriptions = MagicMock()
        connect_errors: list[BaseException] = []
        open_count = 0
        open_lock = threading.Lock()

        def open_session(_config):
            nonlocal open_count
            with open_lock:
                open_count += 1
                call = open_count
            if call == 1:
                old_open_started.set()
                self.assertTrue(release_old_open.wait(timeout=2.0))
                return old_session
            return new_session

        def block_old_cleanup() -> None:
            old_cleanup_started.set()
            self.assertTrue(release_old_cleanup.wait(timeout=2.0))

        old_subscriptions.close.side_effect = block_old_cleanup

        def first_connect() -> None:
            try:
                runner.connect()
            except BaseException as error:
                connect_errors.append(error)

        with (
            patch.object(zenoh, "Config", side_effect=[_Config(), _Config()]),
            patch.object(zenoh, "open", side_effect=open_session),
            patch.object(
                runner_module,
                "SubscriptionManager",
                side_effect=[old_subscriptions, new_subscriptions],
            ),
        ):
            worker = threading.Thread(target=first_connect, daemon=True)
            worker.start()
            self.assertTrue(old_open_started.wait(timeout=1.0))

            runner.close()
            # close() invalidates the generation but deliberately leaves the
            # in-flight token owned by the first attempt until it returns.
            with self.assertRaisesRegex(RuntimeError, "already connected"):
                runner.connect()

            release_old_open.set()
            self.assertTrue(old_cleanup_started.wait(timeout=1.0))

            # The first attempt has relinquished its token, but is still
            # cleaning up local resources.  A new generation may now connect.
            runner.connect()
            self.assertIs(runner.session, new_session)

            release_old_cleanup.set()
            worker.join(timeout=2.0)
            self.assertFalse(worker.is_alive())
            self.assertIs(runner.session, new_session)
            runner.close()

        self.assertEqual(len(connect_errors), 1)
        self.assertRegex(str(connect_errors[0]), "closed while connecting")
        old_subscriptions.close.assert_called_once_with()
        old_session.close.assert_called_once_with()
        new_subscriptions.close.assert_called_once_with()
        new_session.close.assert_called_once_with()

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

    def test_proxy_close_preserves_first_cleanup_failure(self) -> None:
        from zenoh_proxy import runner as runner_module

        runner = runner_module.ProxyRunner()
        subscriptions = MagicMock()
        subscriptions.close.side_effect = RuntimeError("subscription cleanup")
        session = MagicMock()
        session.close.side_effect = RuntimeError("session cleanup")
        runner._sub_mgr = subscriptions
        runner._session = session

        with self.assertRaisesRegex(RuntimeError, "subscription cleanup"):
            runner.close()
        subscriptions.close.assert_called_once_with()
        session.close.assert_called_once_with()

    def test_proxy_spin_restores_state_and_signal_after_cleanup_failure(self) -> None:
        import zenoh
        from zenoh_proxy import runner as runner_module

        runner = runner_module.ProxyRunner()
        failing_subscriptions = MagicMock()
        failing_subscriptions.close.side_effect = RuntimeError("cleanup failed")
        runner._sub_mgr = failing_subscriptions
        runner._session = MagicMock()
        original_handler = object()
        installed_handlers: list[object] = []
        replacement_session = MagicMock()
        replacement_subscriptions = MagicMock()
        with (
            patch.object(
                runner_module.time,
                "sleep",
                side_effect=KeyboardInterrupt,
            ),
            patch.object(
                runner_module.signal,
                "getsignal",
                return_value=original_handler,
            ),
            patch.object(
                runner_module.signal,
                "signal",
                side_effect=lambda _sig, handler: installed_handlers.append(handler),
            ),
            self.assertRaisesRegex(RuntimeError, "cleanup failed"),
        ):
            runner.spin()

        self.assertFalse(runner.is_running)
        self.assertFalse(runner._spin_active)
        self.assertEqual(installed_handlers[-1], original_handler)

        with (
            patch.object(zenoh, "Config", return_value=_Config()),
            patch.object(zenoh, "open", return_value=replacement_session),
            patch.object(
                runner_module,
                "SubscriptionManager",
                return_value=replacement_subscriptions,
            ),
        ):
            runner.connect()
            runner.close()

        replacement_subscriptions.close.assert_called_once_with()
        replacement_session.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
