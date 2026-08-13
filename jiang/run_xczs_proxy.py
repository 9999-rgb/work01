#!/usr/bin/env python3
"""
XCZS 巡操机器人 — Zenoh CDR → JSON 代理启动器.

Registers all XCZS inspection robot topics with appropriate handlers
(flatting, timestamps) and auto-subscribes to them via the Zenoh bridge.

Usage::

    python3 run_xczs_proxy.py
    python3 run_xczs_proxy.py --host 192.168.1.100 --port 7447

The proxy subscribes to ROS2 topics forwarded by zenoh-bridge-ros2dds,
deserializes the CDR payload, applies handlers, and republishes JSON
on ``{topic}/json`` for the browser monitoring dashboard.
"""

from __future__ import annotations

import argparse
import sys

from zenoh_proxy import (
    get_registry,
    register_standard_types,
)


def register_xczs_topics() -> None:
    """
    Register XCZS-specific topic patterns with custom handlers.

    These are MORE SPECIFIC than the wildcard registrations in
    ``register_standard_types()``, so they win the specificity contest and
    get applied to xczs robot topics.
    """
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from zenoh_proxy.handler import (
        add_timestamp,
        flatten_joint_state,
        flatten_twist,
    )

    registry = get_registry()

    # -- XCZS cmd_vel: flatten + timestamp -------------------------------
    @registry.register_topic(
        "xczs/cmd_vel",
        Twist,
        description="XCZS base velocity (flattened)",
    )
    def handle_xczs_cmd_vel(topic: str, data: dict) -> dict:
        data = flatten_twist(topic, data)
        data = add_timestamp(topic, data)
        return data

    # -- XCZS joint_states: flatten + timestamp --------------------------
    @registry.register_topic(
        "xczs/joint_states",
        JointState,
        description=(
            "XCZS joint states "
            "(flattened positions/velocities/efforts)"
        ),
    )
    def handle_xczs_joint_states(topic: str, data: dict) -> dict:
        data = flatten_joint_state(topic, data)
        data = add_timestamp(topic, data)
        return data

    # -- XCZS odom: timestamp only (already flat enough) -----------------
    @registry.register_topic(
        "xczs/odom",
        Odometry,
        description="XCZS odometry (timestamped)",
    )
    def handle_xczs_odom(topic: str, data: dict) -> dict:
        data = add_timestamp(topic, data)
        return data

    # -- XCZS joint_trajectory: timestamp only ---------------------------
    @registry.register_topic(
        "xczs/joint_trajectory",
        JointTrajectory,
        description="XCZS joint trajectory commands (timestamped)",
    )
    def handle_xczs_joint_trajectory(topic: str, data: dict) -> dict:
        data = add_timestamp(topic, data)
        return data


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="XCZS inspection robot Zenoh proxy",
    )
    parser.add_argument(
        "--host", default="127.0.0.1",
        help="Zenoh bridge host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port", type=int, default=7447,
        help="Zenoh bridge TCP port (default: 7447)",
    )
    parser.add_argument(
        "--control-port", type=int, default=0,
        help="Legacy unauthenticated HTTP control port (default: disabled)",
    )
    parser.add_argument(
        "--list", action="store_true",
        help="List registered topics and exit",
    )
    return parser


def _start_legacy_control_server(port: int):
    """Start the loopback-only compatibility server when explicitly asked."""
    from control_gateway import ControlServer

    print(
        "WARNING: --control-port enables the legacy unauthenticated control "
        "API on loopback only. Prefer control_server.py for JWT-protected "
        "Web control.",
        file=sys.stderr,
    )
    server = ControlServer(host="127.0.0.1", port=port)
    try:
        server.start()
    except BaseException:
        try:
            server.stop()
        except Exception:
            pass
        raise
    return server


def main() -> None:
    from zenoh_proxy.runner import ProxyRunner

    args = _build_argument_parser().parse_args()

    # -- Register all types ----------------------------------------------
    register_standard_types()
    register_xczs_topics()

    # -- List and exit? --------------------------------------------------
    if args.list:
        print("Registered topics:")
        for line in get_registry().list_registrations():
            print(f"  {line}")
        return

    # -- Start HTTP control server ---------------------------------------
    ctrl = None
    runner = None
    ctrl_port = int(args.control_port) if args.control_port > 0 else 0
    try:
        if ctrl_port > 0:
            ctrl = _start_legacy_control_server(ctrl_port)
            print(f"Control server: http://127.0.0.1:{ctrl_port}")
            print("  POST /cmd_vel")
            print("  POST /joint_trajectory")
            print("  POST /cabinet/press")
            print("  GET  /cabinet/controls")
            print("  GET  /cabinet/status")
            print("  GET  /health")

        # -- Connect and run ---------------------------------------------
        runner = ProxyRunner(host=args.host, port=args.port)
        runner.connect()
        runner.subscribe_all_registered()
        runner.spin()
    finally:
        try:
            if runner is not None:
                runner.close()
        finally:
            if ctrl is not None:
                ctrl.stop()


if __name__ == "__main__":
    main()
