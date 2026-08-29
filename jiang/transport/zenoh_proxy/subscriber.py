
"""
Multi-topic subscription manager for Zenoh.

The ``SubscriptionManager`` handles multiple Zenoh subscriptions and dispatches
incoming samples through the registry lookup → deserialize → handler → publish
pipeline.
"""

from __future__ import annotations

import traceback
from typing import Dict, Optional, TYPE_CHECKING, Type

if TYPE_CHECKING:
    import zenoh

from .converter import deserialize_and_convert, msg_to_dict
from .registry import TopicRegistration, TopicRegistry, get_registry


class SubscriptionManager:
    """Manages multiple Zenoh subscriptions.

    On each incoming sample it looks up the best-matching registration,
    deserializes the CDR payload, applies the optional handler, and publishes
    the result as JSON on ``{topic}/json``.

    Supports three subscription modes:
      - Explicit single topic / list of topics
      - Wildcard / pattern subscription (e.g. ``"robot/**"``)
      - Auto-subscribe to all registered patterns
    """

    def __init__(
        self,
        session: "zenoh.Session",
        registry: Optional[TopicRegistry] = None,
        default_msg_type: Optional[Type] = None,
        output_suffix: str = "/json",
    ):
        """
        Args:
            session: An open Zenoh session.
            registry: The ``TopicRegistry`` to use for lookup.  Defaults to
                      the process-global singleton.
            default_msg_type: Fallback ROS2 message type for topics that don't
                              match any registration.  ``None`` (default) means
                              unmatched topics are silently skipped.
            output_suffix: Suffix appended to the topic for JSON output
                           (default ``"/json"``).
        """
        self._session = session
        self._registry = registry or get_registry()
        self._default_msg_type = default_msg_type
        self._output_suffix = output_suffix
        self._subscribers: Dict[str, "zenoh.Subscriber"] = {}

    # ------------------------------------------------------------------
    # Subscribe
    # ------------------------------------------------------------------

    def subscribe(self, topic: str) -> None:
        """Subscribe to a single topic or wildcard pattern.

        Args:
            topic: Zenoh key expression (e.g. ``"xczs/cmd_vel"`` or
                   ``"robot/**"``).
        """
        if topic in self._subscribers:
            return  # Already subscribed
        sub = self._session.declare_subscriber(topic, self._on_sample)
        self._subscribers[topic] = sub
        print(f"  [subscribe] {topic}")

    def subscribe_multiple(self, topics: list) -> None:
        """Subscribe to a list of topics at once."""
        for topic in topics:
            self.subscribe(topic)

    def subscribe_all_registered(self) -> None:
        """Subscribe to every pattern that has been registered in the registry.

        This is the recommended mode for plugin-style usage: register your
        handlers, then call ``subscribe_all_registered()`` and the proxy
        automatically picks up all patterns.

        A fully-literal pattern (e.g. ``xczs/cmd_vel``) is skipped when a
        broader registered pattern (e.g. ``*/cmd_vel``) already matches it:
        the broader subscriber delivers the sample and the registry lookup
        resolves it to the most specific registration, so adding a second
        subscriber would only make Zenoh deliver the same sample twice and
        republish the JSON twice.
        """
        seen = set()
        for reg in self._registry.registrations:
            if reg.pattern in seen:
                continue
            seen.add(reg.pattern)
            if self._is_shadowed_literal(reg):
                continue
            self.subscribe(reg.pattern)

    def _is_shadowed_literal(self, reg: TopicRegistration) -> bool:
        """Whether ``reg`` is a literal pattern matched by a broader pattern.

        Only fully-literal patterns can be shadowed: a wildcard pattern carries
        topics no other single pattern necessarily covers, so it is always
        subscribed.
        """
        if "*" in reg.pattern:
            return False
        return any(
            other is not reg and other.regex.match(reg.pattern)
            for other in self._registry.registrations
        )

    # ------------------------------------------------------------------
    # Close
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Undeclare all subscribers."""
        for topic, sub in self._subscribers.items():
            try:
                sub.undeclare()
            except Exception:
                pass
        self._subscribers.clear()

    # ------------------------------------------------------------------
    # Internal callback
    # ------------------------------------------------------------------

    def _on_sample(self, sample: "zenoh.Sample") -> None:
        """Zenoh subscriber callback — the core processing pipeline."""
        topic_str = str(sample.key_expr)

        try:
            payload_bytes = bytes(sample.payload)

            # --- Look up the best-matching registration ---
            registration = self._registry.lookup(topic_str)

            if registration is not None:
                # Registered topic — full pipeline
                json_str = deserialize_and_convert(
                    payload_bytes, registration, topic_str
                )
            elif self._default_msg_type is not None:
                # Fallback: use default msg_type, no handler
                import json as _json
                from rclpy.serialization import deserialize_message
                msg = deserialize_message(payload_bytes, self._default_msg_type)
                data = msg_to_dict(msg)
                json_str = _json.dumps(data)
                print(f"  [fallback] {topic_str} (default type)")
            else:
                # Skip unregistered topics
                print(f"  [skip] {topic_str} — no registration")
                return

            # --- Publish JSON ---
            json_topic = f"{topic_str}{self._output_suffix}"
            self._session.put(
                json_topic,
                json_str.encode(),
                encoding="application/json",
            )
            print(f"  {topic_str} → {json_topic}")

        except Exception:
            print(f"  [error] processing {topic_str}:")
            traceback.print_exc()
