#!/usr/bin/env python3
"""
Zenoh CDR → JSON Proxy — ROS2 message pass-through via Zenoh.

Supports plugin-style topic registration with the ``@register_topic`` decorator,
dynamic loading of custom ROS2 message types, and multi-topic subscription.

Usage::

    # Legacy mode — single topic (backward compatible)
    python proxy.py turtle1/cmd_vel

    # Multiple topics
    python proxy.py /robot/cmd_vel /robot/odom

    # Auto-subscribe to all registered patterns
    python proxy.py --auto

    # With custom message search path
    python proxy.py --auto --msg-path ./ros2_ws/install/lib/python3.10/site-packages

    # With a fallback default message type for unmatched topics
    python proxy.py --auto --default-type geometry_msgs.msg.Twist

For custom message types, write your own launcher::

    # my_proxy.py
    from zenoh_proxy import get_registry, load_message_type, add_message_path
    from zenoh_proxy import register_standard_types
    from zenoh_proxy.runner import ProxyRunner

    add_message_path("./install/lib/python3.10/site-packages")
    register_standard_types()

    registry = get_registry()
    MyMsg = load_message_type("my_msgs.msg.MyMessage")

    @registry.register_topic("/robot/custom", MyMsg)
    def handle_custom(topic, data):
        data["processed"] = True
        return data

    runner = ProxyRunner()
    runner.connect()
    runner.subscribe_all_registered()
    runner.spin()
"""

from __future__ import annotations

import argparse
import json
import sys

from zenoh_proxy import (
    add_message_path,
    get_registry,
    load_bulk_mapping,
    load_message_type,
    register_standard_types,
)
from zenoh_proxy.runner import ProxyRunner

# ============================================================================
# CLI
# ============================================================================


def main():
    parser = argparse.ArgumentParser(
        description="Zenoh CDR → JSON Proxy with plugin-style topic registration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "topics", nargs="*",
        help="Topics to subscribe to (legacy mode: single positional arg)",
    )
    parser.add_argument(
        "--host", default="127.0.0.1",
        help="Zenoh router host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port", type=int, default=7447,
        help="Zenoh router port (default: 7447)",
    )
    parser.add_argument(
        "--msg-path", action="append", default=[],
        help="Add a search path for custom ROS2 message packages "
             "(repeatable)",
    )
    parser.add_argument(
        "--auto", action="store_true",
        help="Auto-subscribe to all registered topic patterns",
    )
    parser.add_argument(
        "--default-type", default=None,
        help="Fallback ROS2 message type for unmatched topics "
             "(e.g. geometry_msgs.msg.Twist)",
    )
    parser.add_argument(
        "--output-suffix", default="/json",
        help="Suffix for JSON output topics (default: /json)",
    )
    parser.add_argument(
        "--list", action="store_true",
        help="List registered topics and exit",
    )

    args = parser.parse_args()

    # -- Step 1: add custom message search paths --------------------------
    for path in args.msg_path:
        add_message_path(path)

    # -- Step 2: register standard types (replaces old if-elif chain) -----
    register_standard_types()

    # -- Step 3: list and exit? -------------------------------------------
    if args.list:
        print("Registered topics:")
        for line in get_registry().list_registrations():
            print(f"  {line}")
        return

    # -- Step 4: resolve default fallback type -----------------------------
    default_type = None
    if args.default_type:
        default_type = load_message_type(args.default_type)
        if default_type is None:
            print(
                f"WARNING: Could not load default message type "
                f"'{args.default_type}'"
            )

    # -- Step 5: connect --------------------------------------------------
    runner = ProxyRunner(
        host=args.host,
        port=args.port,
        default_msg_type=default_type,
        output_suffix=args.output_suffix,
    )
    runner.connect()

    # -- Step 6: subscribe ------------------------------------------------
    if args.auto:
        runner.subscribe_all_registered()
    elif args.topics:
        runner.subscribe(args.topics)
    else:
        # Legacy fallback
        topic = "turtle1/cmd_vel"
        print(f"No topics specified, using default: {topic}")
        runner.subscribe([topic])

    # -- Step 7: run ------------------------------------------------------
    runner.spin()


if __name__ == "__main__":
    main()
