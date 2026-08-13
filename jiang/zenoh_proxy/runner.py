
"""
Zenoh session lifecycle manager.

The ``ProxyRunner`` encapsulates connection setup, subscription management,
and the main event loop with graceful Ctrl+C shutdown.
"""

from __future__ import annotations

import signal
import time
from typing import List, Optional, TYPE_CHECKING, Type

from zenoh_session import configure_client_session

if TYPE_CHECKING:
    import zenoh

from .registry import TopicRegistry, get_registry
from .subscriber import SubscriptionManager


class ProxyRunner:
    """Manages Zenoh session lifecycle and subscription management.

    Usage::

        runner = ProxyRunner(host="127.0.0.1", port=7447)
        runner.connect()
        runner.subscribe(["xczs/cmd_vel"])
        runner.spin()  # blocks until Ctrl+C

    Or with auto-subscribe from registry::

        # ... register handlers ...
        runner = ProxyRunner()
        runner.connect()
        runner.subscribe_all_registered()
        runner.spin()
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 7447,
        registry: Optional[TopicRegistry] = None,
        default_msg_type: Optional[Type] = None,
        output_suffix: str = "/json",
    ):
        """
        Args:
            host: Zenoh router hostname/IP.
            port: Zenoh router port.
            registry: ``TopicRegistry`` to use.  Defaults to the global
                      singleton.
            default_msg_type: Fallback ROS2 message type for unmatched topics.
            output_suffix: Suffix for JSON output topics (default ``"/json"``).
        """
        self._host = host
        self._port = port
        self._registry = registry or get_registry()
        self._default_msg_type = default_msg_type
        self._output_suffix = output_suffix

        self._session: "Optional[zenoh.Session]" = None
        self._sub_mgr: Optional[SubscriptionManager] = None
        self._running = False

    # ------------------------------------------------------------------
    # Connect
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the Zenoh router and initialise the subscription manager.

        Must be called before ``subscribe()`` or ``spin()``.
        """
        import zenoh

        if self._session is not None:
            raise RuntimeError("Zenoh proxy is already connected.")
        conf = zenoh.Config()
        configure_client_session(conf, f"tcp/{self._host}:{self._port}")
        session = zenoh.open(conf)
        try:
            subscription_manager = SubscriptionManager(
                session,
                registry=self._registry,
                default_msg_type=self._default_msg_type,
                output_suffix=self._output_suffix,
            )
        except BaseException:
            try:
                session.close()
            except Exception:
                pass
            raise
        self._session = session
        self._sub_mgr = subscription_manager
        print(f"Connected to Zenoh: {self._host}:{self._port}")

    # ------------------------------------------------------------------
    # Subscribe
    # ------------------------------------------------------------------

    def subscribe(self, topics: List[str]) -> None:
        """Subscribe to a list of topics.

        Raises ``RuntimeError`` if ``connect()`` hasn't been called yet.
        """
        if self._sub_mgr is None:
            raise RuntimeError("Cannot subscribe before calling connect()")
        self._sub_mgr.subscribe_multiple(topics)

    def subscribe_all_registered(self) -> None:
        """Auto-subscribe to all patterns registered in the registry.

        Raises ``RuntimeError`` if ``connect()`` hasn't been called yet.
        """
        if self._sub_mgr is None:
            raise RuntimeError("Cannot subscribe before calling connect()")
        self._sub_mgr.subscribe_all_registered()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def spin(self) -> None:
        """Block and keep the process alive.  Press Ctrl+C to stop.

        Handles ``SIGINT`` for graceful shutdown, restoring the original signal
        handler on exit.
        """
        print("Running.  Press Ctrl+C to stop.\n")
        self._running = True

        # Save and override SIGINT for graceful shutdown
        original_sigint = signal.getsignal(signal.SIGINT)

        def _shutdown(sig, frame):
            self._running = False

        signal.signal(signal.SIGINT, _shutdown)

        try:
            while self._running:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()
            signal.signal(signal.SIGINT, original_sigint)

    # ------------------------------------------------------------------
    # Close
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Gracefully close all subscriptions and the Zenoh session."""
        subscription_manager = self._sub_mgr
        session = self._session
        self._sub_mgr = None
        self._session = None
        try:
            if subscription_manager is not None:
                subscription_manager.close()
        finally:
            if session is not None:
                session.close()
                print("Disconnected from Zenoh")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def session(self) -> "Optional[zenoh.Session]":
        """The active Zenoh session, or ``None`` if not connected."""
        return self._session

    @property
    def is_running(self) -> bool:
        """Whether the main loop is currently running."""
        return self._running
