#!/usr/bin/env python3
"""Start the XCZS HTTP-to-ROS 2 control and cabinet-operation gateway."""

import argparse
from pathlib import Path
import signal
import threading

from control_gateway import ControlServer


def main() -> None:
    """Parse options and run until SIGINT or SIGTERM."""
    workspace = Path(__file__).resolve().parents[1]
    control_config = workspace / "xczs_inspection_robot_control" / "config"
    parser = argparse.ArgumentParser(
        description="XCZS HTTP-to-ROS 2 autonomous control gateway",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--max-linear-speed", type=float, default=0.25)
    parser.add_argument("--max-angular-speed", type=float, default=0.60)
    parser.add_argument("--command-timeout", type=float, default=0.30)
    parser.add_argument(
        "--allowed-origin",
        action="append",
        dest="allowed_origins",
        help="允许访问控制 API 的浏览器 Origin；可重复指定。",
    )
    parser.add_argument(
        "--cabinet-instances",
        default=str(control_config / "cabinet_instances.yaml"),
        help="柜体实例 inventory YAML。",
    )
    parser.add_argument(
        "--cabinet-scene",
        default=str(control_config / "cabinet_scene.yaml"),
        help="包含 navigation_station 的共用场景参数包。",
    )
    args = parser.parse_args()

    server = ControlServer(
        host=args.host,
        port=args.port,
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
        command_timeout=args.command_timeout,
        cabinet_instances_path=args.cabinet_instances,
        cabinet_scene_path=args.cabinet_scene,
        allowed_origins=args.allowed_origins,
    ).start()
    print(f"Control server: http://{args.host}:{args.port}")
    shutdown_event = threading.Event()

    def request_shutdown(
        signal_number: int,
        frame: object,
    ) -> None:
        del signal_number, frame
        shutdown_event.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        while not shutdown_event.wait(timeout=1.0):
            continue
    finally:
        server.stop()


if __name__ == "__main__":
    main()
