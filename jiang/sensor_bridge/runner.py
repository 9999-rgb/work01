"""Command-line entry point for the XCZS web sensor stream service."""

from __future__ import annotations

import argparse

from aiohttp import web

from .ros_node import SensorRosRuntime
from .state import SensorStreamState
from .web_server import create_sensor_app


def main() -> None:
    parser = argparse.ArgumentParser(
        description="XCZS ROS 2 camera and lidar web stream",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8003)
    parser.add_argument(
        "--camera-topic",
        default="/xczs/camera/arm_camera/image_raw",
    )
    parser.add_argument(
        "--lidar-topic",
        default="/xczs/lidar/scan",
    )
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--camera-fps", type=float, default=10.0)
    parser.add_argument("--lidar-fps", type=float, default=10.0)
    args = parser.parse_args()

    state = SensorStreamState()
    ros_runtime = SensorRosRuntime(
        state=state,
        camera_topic=args.camera_topic,
        lidar_topic=args.lidar_topic,
        jpeg_quality=args.jpeg_quality,
        camera_fps=args.camera_fps,
    ).start()
    app = create_sensor_app(
        state=state,
        stream_fps=args.camera_fps,
        lidar_fps=args.lidar_fps,
    )
    print(
        f"Sensor stream server: http://{args.host}:{args.port} "
        "(camera MJPEG + lidar WebSocket)"
    )
    try:
        web.run_app(
            app,
            host=args.host,
            port=args.port,
            print=None,
        )
    finally:
        ros_runtime.stop()


if __name__ == "__main__":
    main()
