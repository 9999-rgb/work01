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
    parser.add_argument(
        "--cabinet-robot-adapter",
        default=str(control_config / "cabinet_robot_adapter.yaml"),
        help="包含 navigation_base_frame 的机器人适配参数包。",
    )
    parser.add_argument(
        "--cabinet-controls",
        default=str(control_config / "cabinet_controls.yaml"),
        help="控制柜目录参数包；用于记录清单哈希。",
    )
    parser.add_argument(
        "--cabinet-pose",
        default=str(control_config / "cabinet_pose.yaml"),
        help="控制柜位姿参数包；用于记录清单哈希。",
    )
    parser.add_argument(
        "--robot-control",
        default=str(control_config / "robot_control.yaml"),
        help="机器人路由参数包；用于记录清单哈希。",
    )
    parser.add_argument(
        "--recordings-root",
        default=str(workspace / "recordings"),
        help="rosbag2、清单、时间线和任务场景的本地数据目录。",
    )
    args = parser.parse_args()

    shutdown_event = threading.Event()

    def request_shutdown(
        signal_number: int,
        frame: object,
    ) -> None:
        del signal_number, frame
        shutdown_event.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    server = ControlServer(
        host=args.host,
        port=args.port,
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
        command_timeout=args.command_timeout,
        cabinet_instances_path=args.cabinet_instances,
        cabinet_controls_path=args.cabinet_controls,
        cabinet_scene_path=args.cabinet_scene,
        cabinet_pose_path=args.cabinet_pose,
        cabinet_robot_adapter_path=args.cabinet_robot_adapter,
        robot_control_path=args.robot_control,
        recordings_root=args.recordings_root,
        allowed_origins=args.allowed_origins,
    )
    try:
        if shutdown_event.is_set():
            return
        server.start()
        print(f"Control server: http://{args.host}:{args.port}")
        while not shutdown_event.wait(timeout=1.0):
            continue
    finally:
        server.stop()


if __name__ == "__main__":
    main()
