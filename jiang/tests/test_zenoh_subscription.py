"""Zenoh proxy subscription dedup tests.

``subscribe_all_registered`` must not subscribe to a fully-literal pattern that
a broader registered pattern already matches (e.g. ``xczs/cmd_vel`` under
``*/cmd_vel``): Zenoh delivers the same sample to every matching subscriber, so
a redundant second subscription would process the sample and republish the JSON
twice.  The registry lookup resolves each delivered sample to its most specific
registration, so the broader subscriber alone is sufficient.
"""

import sys
import unittest
from pathlib import Path


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from transport.zenoh_proxy.registry import TopicRegistry  # noqa: E402
from transport.zenoh_proxy.subscriber import SubscriptionManager  # noqa: E402


class _Twist:
    __name__ = "Twist"


class _JointState:
    __name__ = "JointState"


class _FakeSubscriber:
    def __init__(self, topic: str) -> None:
        self.topic = topic

    def undeclare(self) -> None:
        pass


class _FakeSession:
    def __init__(self) -> None:
        self.subscriptions: list[str] = []

    def declare_subscriber(self, topic: str, callback):
        self.subscriptions.append(topic)
        return _FakeSubscriber(topic)


class SubscriptionShadowingTest(unittest.TestCase):
    def test_literal_pattern_shadowed_by_wildcard_is_not_subscribed(self) -> None:
        registry = TopicRegistry()
        registry.register("*/cmd_vel", _Twist)
        registry.register("*/joint_states", _JointState)
        registry.register("xczs/cmd_vel", _Twist)
        registry.register("xczs/joint_states", _JointState)

        session = _FakeSession()
        manager = SubscriptionManager(session, registry=registry)
        manager.subscribe_all_registered()

        # Only the broader wildcards are subscribed; the exact xczs patterns
        # are matched by them and resolved by the registry lookup.
        self.assertEqual(
            {"*/cmd_vel", "*/joint_states"},
            set(session.subscriptions),
        )

    def test_literal_pattern_without_broader_pattern_is_subscribed(self) -> None:
        registry = TopicRegistry()
        registry.register("*/cmd_vel", _Twist)
        registry.register("xczs/unique_topic", _JointState)

        session = _FakeSession()
        manager = SubscriptionManager(session, registry=registry)
        manager.subscribe_all_registered()

        self.assertEqual(
            {"*/cmd_vel", "xczs/unique_topic"},
            set(session.subscriptions),
        )

    def test_wildcard_patterns_are_always_subscribed(self) -> None:
        registry = TopicRegistry()
        registry.register("*/cmd_vel", _Twist)
        registry.register("xczs/**", _JointState)

        session = _FakeSession()
        manager = SubscriptionManager(session, registry=registry)
        manager.subscribe_all_registered()

        # A wildcard cannot be proven covered by another single pattern, so it
        # is always subscribed.
        self.assertEqual(
            {"*/cmd_vel", "xczs/**"},
            set(session.subscriptions),
        )


if __name__ == "__main__":
    unittest.main()
