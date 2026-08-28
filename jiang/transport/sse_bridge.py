#!/usr/bin/env python3
"""
Bridge Zenoh CDR data to browser-compatible HTTP SSE.

取代不可靠的 Zenoh REST SSE 端点。浏览器直接连接这个 HTTP SSE 服务器。

需要 ROS2 环境 (rclpy) 来解码 CDR 二进制消息。

用法:
    source /opt/ros/humble/setup.bash
    python3 -m transport.sse_bridge           # 默认: localhost:8001（在 jiang/ 目录下）
    python3 -m transport.sse_bridge --port 8001

浏览器:
    new EventSource('http://localhost:8001/xczs/odom')
    new EventSource('http://localhost:8001/xczs/joint_states')
"""

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from queue import Empty, Queue
from urllib.parse import urlparse, unquote

import zenoh

from .zenoh_key import normalize_ros_key
from .zenoh_session import configure_client_session

# Lazy imports for CDR decoding (require rclpy)
_cdr_decoder = None


def _get_cdr_decoder():
    """Lazy-init the CDR decoder using proxy's converter and registry."""
    global _cdr_decoder
    if _cdr_decoder is not None:
        return _cdr_decoder

    from .zenoh_proxy import register_standard_types
    from .zenoh_proxy.converter import deserialize_and_convert
    from .zenoh_proxy.registry import get_registry

    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from trajectory_msgs.msg import JointTrajectory
    from .zenoh_proxy.handler import (
        add_timestamp,
        flatten_joint_state,
        flatten_twist,
    )

    register_standard_types()

    registry = get_registry()

    @registry.register_topic(
        "xczs/cmd_vel",
        Twist,
        description="XCZS cmd_vel",
    )
    def _h1(topic, data):
        data = flatten_twist(topic, data)
        data = add_timestamp(topic, data)
        return data

    @registry.register_topic(
        "xczs/joint_states",
        JointState,
        description="XCZS joint_states",
    )
    def _h2(topic, data):
        data = flatten_joint_state(topic, data)
        data = add_timestamp(topic, data)
        return data

    for topic, message_type, description in (
        ("xczs/odom", Odometry, "XCZS odometry"),
        (
            "xczs/joint_trajectory",
            JointTrajectory,
            "XCZS joint trajectory",
        ),
        ("robot_description", String, "Robot description"),
        ("tf", TFMessage, "Dynamic transforms"),
        ("tf_static", TFMessage, "Static transforms"),
        ("clock", Clock, "Simulation clock"),
    ):
        registry.register(
            topic,
            message_type,
            handler=add_timestamp,
            description=description,
        )

    class Decoder:
        def __init__(self):
            self.registry = registry
            self.deserialize_and_convert = deserialize_and_convert

        def decode(self, topic, payload_bytes):
            reg = self.registry.lookup(topic)
            if reg is None:
                return None
            try:
                return self.deserialize_and_convert(payload_bytes, reg, topic)
            except Exception:
                return None

    _cdr_decoder = Decoder()
    return _cdr_decoder

# ============================================================================
# Zenoh subscriber manager (shared across SSE connections)
# ============================================================================


class ZenohSource:
    """Manages Zenoh subscriptions and fans out data to SSE clients."""

    def __init__(self, connect_endpoint: str = "tcp/localhost:7447"):
        conf = zenoh.Config()
        configure_client_session(conf, connect_endpoint)
        self._session = zenoh.open(conf)
        self._subs: dict[str, zenoh.Subscriber] = {}
        self._listeners: dict[str, list] = {}  # key -> list of callback functions
        self._last: dict[str, tuple[str, str]] = {}  # key -> (topic, json_str)
        self._lock = threading.RLock()
        self._closed = False

    def add_listener(self, key: str, callback) -> None:
        """Register a callback for a Zenoh key expression. Auto-subscribes.

        A late subscriber (one that attaches after the source already received
        the last sample for ``key``) is replayed the cached sample so it does
        not miss latched (transient-local) topics such as ``tf_static``.  The
        underlying Zenoh bridge already replays its retained transient-local
        value to the first subscriber that declares a subscription; the replay
        here covers every *subsequent* subscriber that joins an already-live
        key, which would otherwise only see samples published after it joined.
        """
        with self._lock:
            if self._closed:
                raise RuntimeError("Zenoh source is closed.")
            if key not in self._subs:
                self._subs[key] = self._session.declare_subscriber(
                    key, self._make_callback(key)
                )
                self._listeners[key] = []
                print(f"[zenoh] subscribed: {key}", file=sys.stderr)
            self._listeners[key].append(callback)
            cached = self._last.get(key)

        if cached is not None:
            try:
                callback(*cached)
            except Exception:
                pass

    def remove_listener(self, key: str, callback) -> None:
        """Remove a callback. Unsubscribes when last listener is removed."""
        subscriber = None
        with self._lock:
            if key in self._listeners:
                self._listeners[key] = [
                    c for c in self._listeners[key] if c is not callback
                ]
                if not self._listeners[key]:
                    subscriber = self._subs.pop(key, None)
                    del self._listeners[key]
                    # The latest-sample cache only exists to replay to a later
                    # listener on an already-live key; once the last listener
                    # leaves, drop it so the cache cannot grow without bound.
                    self._last.pop(key, None)
        if subscriber is not None:
            try:
                subscriber.undeclare()
            except Exception:
                pass
            print(f"[zenoh] unsubscribed: {key}", file=sys.stderr)

    def _make_callback(self, key: str):
        def _callback(sample: zenoh.Sample):
            payload = bytes(sample.payload)
            topic = str(sample.key_expr)
            # Try CDR decode first (ROS2 binary), then JSON, then hex fallback
            json_str = None
            try:
                decoder = _get_cdr_decoder()
            except Exception:
                # rclpy/ROS message modules unavailable: fall through to the
                # JSON/hex fallback instead of letting the ImportError escape
                # the Zenoh callback (which would skip the _last update and
                # every listener).
                decoder = None
            if decoder:
                result = decoder.decode(topic, payload)
                if result:
                    json_str = result
            if json_str is None:
                try:
                    data = json.loads(payload.decode())
                    json_str = json.dumps(data, ensure_ascii=False)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    json_str = json.dumps(
                        {
                            "_raw_hex": payload.hex()[:60] + "...",
                            "_topic": topic,
                            "_len": len(payload),
                        }
                    )
            # Store the latest decoded sample so late listeners can be replayed.
            with self._lock:
                self._last[key] = (topic, json_str)
                listeners = list(self._listeners.get(key, []))
            for cb in listeners:
                try:
                    cb(topic, json_str)
                except Exception:
                    pass

        return _callback

    def close(self) -> None:
        subscribers = ()
        with self._lock:
            if self._closed:
                return
            self._closed = True
            subscribers = tuple(self._subs.values())
            self._subs.clear()
            self._listeners.clear()
            self._last.clear()
        for sub in subscribers:
            try:
                sub.undeclare()
            except Exception:
                pass
        self._session.close()

    @property
    def session(self):
        return self._session


# ============================================================================
# HTTP SSE server
# ============================================================================

_zenoh_source: ZenohSource | None = None
MONITOR_UPDATE_INTERVAL_SECONDS = 1.0


class SSEHandler(BaseHTTPRequestHandler):
    """Serves Zenoh data as SSE (Server-Sent Events)."""

    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):
        print(f"[http] {args[0]}", file=sys.stderr)

    def _reflect_origin_header(self):
        """Reflect the request Origin only when it shares the request's host.

        与 FastAPI 传感器路由的同源规则一致：同一主机（任意端口/协议）的页面
        可以读取 SSE 流——这是 LAN 监控页 ``<host>:8090`` 跨端口访问所需的。
        跨主机的 Origin 通过「不下发 ``Access-Control-Allow-Origin``」来拒绝，
        浏览器会阻止跨源读取；这也封堵了通配 ``*`` 无法防住的 DNS rebinding。
        """
        headers = getattr(self, "headers", None)
        origin = headers.get("Origin") if headers is not None else None
        host = headers.get("Host") if headers is not None else None
        if not origin or not host:
            return None
        parsed = urlparse(origin)
        if parsed.scheme not in {"http", "https"}:
            return None
        origin_host = (parsed.hostname or "").lower()
        request_host = host.split(":", 1)[0].lower()
        if not origin_host or origin_host != request_host:
            return None
        return origin

    def do_GET(self):
        global _zenoh_source
        parsed = urlparse(self.path)
        raw_key = unquote(
            parsed.path[1:] if parsed.path.startswith("/") else parsed.path
        )
        try:
            key = normalize_ros_key(raw_key)
        except ValueError as error:
            self.send_error(400, str(error))
            return

        # Connect to Zenoh and fan out
        # Web 监控只保留每个订阅话题的最新数据，避免高频 ROS 2
        # 话题在浏览器端堆积。该限流不影响 ROS 2 或控制接口频率。
        events: Queue[tuple[str, str]] = Queue(maxsize=1)
        events_lock = threading.Lock()

        def on_data(topic, json_str):
            with events_lock:
                try:
                    events.get_nowait()
                except Empty:
                    pass
                events.put_nowait((topic, json_str))

        # Capture the source used for registration.  Global shutdown can swap
        # or clear _zenoh_source while this request is active; cleanup must
        # always target the same source.  Do not commit HTTP 200 until the
        # upstream subscription is known to be usable.
        source = _zenoh_source
        if source is None:
            self.send_error(503, "Zenoh source is unavailable")
            return
        try:
            source.add_listener(key, on_data)
        except Exception:
            self.send_error(503, "Zenoh subscription is unavailable")
            return

        next_send_time = 0.0
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Vary", "Origin")
            reflected_origin = self._reflect_origin_header()
            if reflected_origin:
                self.send_header("Access-Control-Allow-Origin", reflected_origin)
            self.end_headers()
            while True:
                try:
                    latest_item = events.get(timeout=15)
                except Empty:
                    # Heartbeat to keep connection alive
                    self.wfile.write(b":heartbeat\n\n")
                    self.wfile.flush()
                    continue

                remaining = next_send_time - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                    # 等待期间若有新数据，则用最新值替换待发送值。
                    try:
                        latest_item = events.get_nowait()
                    except Empty:
                        pass

                topic, json_str = latest_item
                ts = int(time.time() * 1000)
                sse_data = json.dumps(
                    {
                        "key": topic,
                        "value": json.loads(json_str),
                        "encoding": "application/json",
                        "timestamp": str(ts),
                    }
                )
                self.wfile.write(
                    f"event:PUT\ndata:{sse_data}\n\n".encode()
                )
                self.wfile.flush()
                next_send_time = (
                    time.monotonic() + MONITOR_UPDATE_INTERVAL_SECONDS
                )
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            source.remove_listener(key, on_data)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Vary", "Origin")
        reflected_origin = self._reflect_origin_header()
        if reflected_origin:
            self.send_header("Access-Control-Allow-Origin", reflected_origin)
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.end_headers()


# ============================================================================
# Main
# ============================================================================


def _build_argument_parser():
    import argparse

    parser = argparse.ArgumentParser(description="Zenoh → SSE bridge")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="HTTP bind host (default: 127.0.0.1)",
    )
    parser.add_argument("--port", type=int, default=8001, help="HTTP port (default: 8001)")
    parser.add_argument("--zenoh", default="tcp/localhost:7447", help="Zenoh endpoint")
    return parser


def main():
    args = _build_argument_parser().parse_args()

    global _zenoh_source
    print(f"[init] Connecting to Zenoh at {args.zenoh}...", file=sys.stderr)
    _zenoh_source = ZenohSource(connect_endpoint=args.zenoh)
    server = None
    serve_started = False
    try:
        server = ThreadingHTTPServer((args.host, args.port), SSEHandler)
        server.daemon_threads = True
        print(
            f"[init] SSE bridge: http://{args.host}:{args.port}/<key>",
            file=sys.stderr,
        )
        try:
            serve_started = True
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    finally:
        try:
            if server is not None:
                try:
                    if serve_started:
                        server.shutdown()
                finally:
                    server.server_close()
        finally:
            try:
                _zenoh_source.close()
            finally:
                _zenoh_source = None


if __name__ == "__main__":
    main()
