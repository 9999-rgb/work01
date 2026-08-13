
"""
TopicRegistry — central registration and lookup for ROS2 topic handlers.

Provides:
  - TopicRegistration: holds pattern, compiled regex, msg_type, handler
  - TopicRegistry: singleton registry with register() / lookup() / register_topic()
  - Pattern matching: literal, * (single segment), ** (multi segment)

Usage:
    registry = get_registry()

    # Pure passthrough (no handler — just deserialize → JSON → publish)
    registry.register("/cmd_vel", Twist)

    # With custom handler
    @registry.register_topic("/robot/cmd_vel", Twist)
    def handle_cmd_vel(topic, data):
        data["processed"] = True
        return data
"""

from __future__ import annotations

import re
from typing import Any, Callable, Dict, List, Optional, Pattern, Type

# ---------------------------------------------------------------------------
# Handler protocol: (topic: str, data: dict) -> dict
# Return the (possibly modified) data dict to be JSON-serialized.
# Return None to indicate the handler is None (passthrough).
# ---------------------------------------------------------------------------
HandlerFunc = Optional[Callable[[str, Dict[str, Any]], Dict[str, Any]]]


# ============================================================================
# TopicRegistration
# ============================================================================

class TopicRegistration:
    """A single registration: topic pattern → message type + optional handler."""

    __slots__ = ("pattern", "regex", "msg_type", "handler", "description")

    def __init__(
        self,
        pattern: str,
        msg_type: Type,
        handler: HandlerFunc = None,
        description: str = "",
    ):
        self.pattern = pattern
        self.regex = self._compile_pattern(pattern)
        self.msg_type = msg_type
        self.handler = handler
        self.description = description

    # ------------------------------------------------------------------
    # Pattern → regex
    # ------------------------------------------------------------------
    @staticmethod
    def _compile_pattern(pattern: str) -> Pattern:
        """Convert a topic pattern with * and ** wildcards to a compiled regex.

        Rules:
          - ``*``   matches exactly one path segment (no ``/``)
          - ``**``  matches zero or more segments
          - Everything else is literal (regex-escaped)

        Examples:
          ``/robot/*/pose``  matches ``/robot/arm_1/pose`` but NOT ``/robot/a/b/pose``
          ``/robot/**/pose`` matches both.
        """
        parts: List[str] = []
        for part in pattern.split("/"):
            # Adjacent globstars have the same semantics as one globstar and
            # are collapsed to keep separator handling deterministic.
            if part == "**" and parts and parts[-1] == "**":
                continue
            parts.append(part)

        expression = ""
        for index, part in enumerate(parts):
            if part == "**":
                if len(parts) == 1:
                    expression += r"(?:[^/]+(?:/[^/]+)*)?"
                elif index == 0:
                    # Consume complete leading segments, including their
                    # separator, so the following segment also works when
                    # the globstar matches zero segments.
                    expression += r"(?:[^/]+/)*"
                else:
                    expression += r"(?:/[^/]+)*"
                continue

            if index > 0 and not (
                parts[index - 1] == "**" and index - 1 == 0
            ):
                expression += "/"
            expression += r"[^/]+" if part == "*" else re.escape(part)
        return re.compile("^" + expression + "$")

    # ------------------------------------------------------------------
    # Specificity score — higher = more specific (used for sorting)
    # ------------------------------------------------------------------
    @property
    def specificity(self) -> int:
        """Return a specificity score.  Higher = more specific.

        Scoring (heuristic):
          - Each literal character: +1
          - Each single-wildcard ``*``: -10
          - Each globstar ``**``: -100
        """
        score = len(self.pattern)
        score -= self.pattern.count("**") * 100
        score -= (self.pattern.count("*") - self.pattern.count("**")) * 10
        return score

    def __repr__(self) -> str:
        desc = f" desc={self.description!r}" if self.description else ""
        handler = " (handler)" if self.handler else ""
        return (
            f"TopicRegistration({self.pattern!r} → {self.msg_type.__name__}"
            f"{handler}{desc})"
        )


# ============================================================================
# TopicRegistry
# ============================================================================

class TopicRegistry:
    """Singleton-like registry mapping topic patterns → message types/handlers.

    Registrations are kept sorted by specificity (most-specific first) so that
    ``lookup()`` always returns the best match.
    """

    def __init__(self):
        self._registrations: List[TopicRegistration] = []

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def register(
        self,
        pattern: str,
        msg_type: Type,
        handler: HandlerFunc = None,
        description: str = "",
    ) -> TopicRegistration:
        """Register a topic pattern with a ROS2 message type.

        Args:
            pattern: Topic string or pattern (``"xczs/cmd_vel"``,
                     ``"/robot/*/pose"``).
            msg_type: A ROS2 message class (e.g. ``geometry_msgs.msg.Twist``).
            handler: Optional callback ``(topic, data) -> data`` for
                     post-deserialization processing.
            description: Human-readable description for debugging.

        Returns:
            The ``TopicRegistration`` object.

        Note:
            Leading slashes are stripped from the pattern to comply with Zenoh
            key expression rules.
        """
        pattern = pattern.lstrip("/")
        registration = TopicRegistration(pattern, msg_type, handler, description)
        self._registrations.append(registration)
        # Keep sorted: most-specific first
        self._registrations.sort(key=lambda r: r.specificity, reverse=True)
        return registration

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------

    def lookup(self, topic: str) -> Optional[TopicRegistration]:
        """Find the best-matching registration for a topic string.

        Returns the most specific registration whose regex matches *topic*,
        or ``None`` if no registration matches.
        """
        for reg in self._registrations:
            if reg.regex.match(topic):
                return reg
        return None

    # ------------------------------------------------------------------
    # Decorator API
    # ------------------------------------------------------------------

    def register_topic(
        self,
        pattern: str,
        msg_type: Type,
        description: str = "",
    ) -> Callable[[Callable], Callable]:
        """Decorator: register a handler for a topic pattern.

        Usage::

            @registry.register_topic("/robot/cmd_vel", Twist)
            def handle_cmd_vel(topic, data):
                data["_ts"] = time.time()
                return data

        The decorated function is returned unchanged so it can still be called
        directly for testing.
        """
        def decorator(func: Callable[[str, Dict[str, Any]], Dict[str, Any]]):
            self.register(pattern, msg_type, handler=func, description=description)
            return func
        return decorator

    # ------------------------------------------------------------------
    # Bulk registration
    # ------------------------------------------------------------------

    def register_map(
        self,
        mapping: Dict[str, Type],
        handler: HandlerFunc = None,
    ) -> List[TopicRegistration]:
        """Register a simple ``{pattern: msg_type}`` mapping.

        Useful for loading from config or migration from hardcoded chains.
        """
        results = []
        for pattern, msg_type in mapping.items():
            results.append(self.register(pattern, msg_type, handler=handler))
        return results

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    @property
    def registrations(self) -> List[TopicRegistration]:
        """Return a copy of the registration list (read-only)."""
        return list(self._registrations)

    def list_registrations(self) -> List[str]:
        """Return human-readable list of all registrations."""
        return [repr(r) for r in self._registrations]


# ============================================================================
# Module-level singleton
# ============================================================================

_registry: Optional[TopicRegistry] = None


def get_registry() -> TopicRegistry:
    """Get or create the process-global ``TopicRegistry`` singleton."""
    global _registry
    if _registry is None:
        _registry = TopicRegistry()
    return _registry
