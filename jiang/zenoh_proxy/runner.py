
"""
Zenoh session lifecycle manager.

The ``ProxyRunner`` encapsulates connection setup, subscription management,
and the main event loop with graceful Ctrl+C shutdown.
"""

from __future__ import annotations

import signal
import threading
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
        self._state_lock = threading.RLock()
        self._spin_active = False
        self._connecting = False
        self._connect_token: object | None = None
        self._lifecycle_generation = 0

    # ------------------------------------------------------------------
    # Connect
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the Zenoh router and initialise the subscription manager.

        Must be called before ``subscribe()`` or ``spin()``.
        """
        import zenoh

        with self._state_lock:
            if self._spin_active:
                raise RuntimeError(
                    "Cannot connect until the previous spin loop has exited."
                )
            if self._session is not None or self._connecting:
                raise RuntimeError("Zenoh proxy is already connected.")
            self._connecting = True
            connect_token = object()
            self._connect_token = connect_token
            generation = self._lifecycle_generation
        session = None
        try:
            conf = zenoh.Config()
            configure_client_session(conf, f"tcp/{self._host}:{self._port}")
            session = zenoh.open(conf)
            subscription_manager = SubscriptionManager(
                session,
                registry=self._registry,
                default_msg_type=self._default_msg_type,
                output_suffix=self._output_suffix,
            )
        except BaseException:
            if session is not None:
                try:
                    session.close()
                except Exception:
                    pass
            with self._state_lock:
                if self._connect_token is connect_token:
                    self._connecting = False
                    self._connect_token = None
            raise
        with self._state_lock:
            if (
                generation != self._lifecycle_generation
                or self._connect_token is not connect_token
            ):
                if self._connect_token is connect_token:
                    self._connecting = False
                    self._connect_token = None
                accepted = False
            else:
                self._connecting = False
                self._connect_token = None
                self._session = session
                self._sub_mgr = subscription_manager
                accepted = True
        if not accepted:
            try:
                subscription_manager.close()
            finally:
                session.close()
            raise RuntimeError("Zenoh proxy was closed while connecting.")
        print(f"Connected to Zenoh: {self._host}:{self._port}")

    # ------------------------------------------------------------------
    # Subscribe
    # ------------------------------------------------------------------

    def subscribe(self, topics: List[str]) -> None:
        """Subscribe to a list of topics.

        Raises ``RuntimeError`` if ``connect()`` hasn't been called yet.
        """
        with self._state_lock:
            subscription_manager = self._sub_mgr
            if subscription_manager is None:
                raise RuntimeError("Cannot subscribe before calling connect()")
            # SubscriptionManager mutates resources owned by this lifecycle.
            # Keep it serialized with close() so a completed subscribe cannot
            # write into a manager that close() has already detached.
            subscription_manager.subscribe_multiple(topics)

    def subscribe_all_registered(self) -> None:
        """Auto-subscribe to all patterns registered in the registry.

        Raises ``RuntimeError`` if ``connect()`` hasn't been called yet.
        """
        with self._state_lock:
            subscription_manager = self._sub_mgr
            if subscription_manager is None:
                raise RuntimeError("Cannot subscribe before calling connect()")
            subscription_manager.subscribe_all_registered()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def spin(self) -> None:
        """Block and keep the process alive.  Press Ctrl+C to stop.

        Handles ``SIGINT`` for graceful shutdown, restoring the original signal
        handler on exit.
        """
        with self._state_lock:
            if self._spin_active:
                raise RuntimeError("Zenoh proxy is already spinning.")
            if self._connecting or self._session is None:
                raise RuntimeError(
                    "Cannot spin before connect() has completed."
                )
            self._spin_active = True
            self._running = True

        def _shutdown(sig, frame):
            del sig, frame
            with self._state_lock:
                self._running = False

        # Everything after claiming the spin generation is covered by one
        # cleanup path.  In particular, logging and signal setup can invoke
        # user-controlled hooks and therefore must be treated as fallible.
        manages_sigint = False
        original_sigint = None
        original_sigint_captured = False
        signal_install_attempted = False
        primary_error: BaseException | None = None
        try:
            print("Running.  Press Ctrl+C to stop.\n")
            # Python only permits installing signal handlers on the main
            # thread. A background spin remains useful to embedders and is
            # stopped through ``close``.
            manages_sigint = (
                threading.current_thread() is threading.main_thread()
            )
            if manages_sigint:
                original_sigint = signal.getsignal(signal.SIGINT)
                original_sigint_captured = True
                # Mark the attempt first: a wrapper may install the handler
                # and then raise, in which case restoration is still needed.
                signal_install_attempted = True
                signal.signal(signal.SIGINT, _shutdown)
            while self.is_running:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        except BaseException as error:
            primary_error = error
        finally:
            cleanup_error: BaseException | None = None
            try:
                self.close()
            except BaseException as error:
                cleanup_error = error
            try:
                if (
                    manages_sigint
                    and original_sigint_captured
                    and signal_install_attempted
                ):
                    signal.signal(signal.SIGINT, original_sigint)
            except BaseException as error:
                if cleanup_error is None:
                    cleanup_error = error
            finally:
                with self._state_lock:
                    self._spin_active = False
            if primary_error is not None:
                raise primary_error
            if cleanup_error is not None:
                raise cleanup_error

    # ------------------------------------------------------------------
    # Close
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Gracefully close all subscriptions and the Zenoh session."""
        # ``close`` is also the thread-safe stop signal for a concurrently
        # running ``spin`` loop.  Resource fields are detached before calling
        # external cleanup methods, which keeps repeated calls idempotent.
        with self._state_lock:
            self._running = False
            self._lifecycle_generation += 1
            subscription_manager = self._sub_mgr
            session = self._session
            self._sub_mgr = None
            self._session = None
        cleanup_error: BaseException | None = None
        if subscription_manager is not None:
            try:
                subscription_manager.close()
            except BaseException as error:
                cleanup_error = error
        if session is not None:
            try:
                session.close()
                print("Disconnected from Zenoh")
            except BaseException as error:
                if cleanup_error is None:
                    cleanup_error = error
        if cleanup_error is not None:
            raise cleanup_error

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
        with self._state_lock:
            return self._running
