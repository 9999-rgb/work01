#!/usr/bin/env python3
"""启动 XCZS HTTP → ROS 2 控制网关（FastAPI/uvicorn，三合一）。

一个 uvicorn 进程承载：控制网关 REST API + 任务 SSE + 传感器流（MJPEG/
LiDAR WebSocket）+ Zenoh SSE + 用户认证 + 静态页面（monitor.html）。

参数与旧版兼容。传感器与 Zenoh 子系统默认尝试启动；依赖缺失或启动失败时
降级为仅控制 API（不影响主网关可用性）。
"""

import argparse
import logging
import os
import signal
import threading
from pathlib import Path
from typing import Callable

import uvicorn

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

WORKSPACE = Path(__file__).resolve().parents[1]
CONTROL_CONFIG = WORKSPACE / "xczs_inspection_robot_control" / "config"

ExecutorFatalCallback = Callable[[str, BaseException], None]


def _build_process_fatal_callback() -> ExecutorFatalCallback:
    """Build an idempotent callback that asks uvicorn to shut down.

    Uvicorn owns the process signal handlers before application lifespan
    startup, so SIGTERM reaches its normal graceful-shutdown path.  The outer
    ``run_all.sh`` watchdog then observes this persistent process exiting and
    tears down the complete managed process group.
    """
    lock = threading.Lock()
    signal_sent = False

    def request_shutdown(component: str, error: BaseException) -> None:
        nonlocal signal_sent
        with lock:
            if signal_sent:
                return
            signal_sent = True
        logger.critical(
            "%s failed fatally (%s): %s; requesting process shutdown",
            component,
            type(error).__name__,
            str(error) or repr(error),
        )
        os.kill(os.getpid(), signal.SIGTERM)

    return request_shutdown


def _effective_allowed_origins(args: argparse.Namespace) -> list[str]:
    """用同一组 Origin 配置 ControlServer 与 FastAPI。

    优先级：命令行 ``--allowed-origin`` > XCZS_ALLOWED_ORIGINS >
    按当前 ``--port`` 生成的本地同源默认值。
    """
    if args.allowed_origins:
        return list(args.allowed_origins)
    configured = os.environ.get("XCZS_ALLOWED_ORIGINS", "")
    if configured.strip():
        return [origin.strip() for origin in configured.split(",") if origin.strip()]
    return [
        f"http://localhost:{args.port}",
        f"http://127.0.0.1:{args.port}",
    ]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="XCZS HTTP-to-ROS 2 autonomous control gateway",
    )
    parser.add_argument("--host", default="0.0.0.0")
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
        default=str(CONTROL_CONFIG / "cabinet_instances.yaml"),
        help="柜体实例 inventory YAML。",
    )
    parser.add_argument(
        "--cabinet-scene",
        default=str(CONTROL_CONFIG / "cabinet_scene.yaml"),
        help="包含 navigation_station 的共用场景参数包。",
    )
    parser.add_argument(
        "--cabinet-robot-adapter",
        default=str(CONTROL_CONFIG / "cabinet_robot_adapter.yaml"),
        help="包含 navigation_base_frame 的机器人适配参数包。",
    )
    parser.add_argument(
        "--cabinet-controls",
        default=str(CONTROL_CONFIG / "cabinet_controls.yaml"),
        help="控制柜目录参数包；用于记录清单哈希。",
    )
    parser.add_argument(
        "--cabinet-pose",
        default=str(CONTROL_CONFIG / "cabinet_pose.yaml"),
        help="控制柜位姿参数包；用于记录清单哈希。",
    )
    parser.add_argument(
        "--robot-control",
        default=str(CONTROL_CONFIG / "robot_control.yaml"),
        help="机器人路由参数包；用于记录清单哈希。",
    )
    parser.add_argument(
        "--recordings-root",
        default=str(WORKSPACE / "recordings"),
        help="rosbag2、清单、时间线和任务场景的本地数据目录。",
    )
    # ── 子系统（三合一） ───────────────────────────────────────────
    parser.add_argument(
        "--camera-topic",
        default="/xczs/camera/arm_camera/image_raw",
        help="相机图像话题。",
    )
    parser.add_argument(
        "--lidar-topic",
        default="/xczs/lidar/scan",
        help="LiDAR 扫描话题。",
    )
    parser.add_argument(
        "--zenoh",
        default="tcp/localhost:7447",
        help="Zenoh 桥端点（SSE 数据源）。",
    )
    parser.add_argument(
        "--no-sensors",
        action="store_true",
        help="不启动传感器流（相机/LiDAR）。",
    )
    parser.add_argument(
        "--no-sse",
        action="store_true",
        help="不启动 Zenoh SSE 桥。",
    )
    return parser


def _build_sensor_subsystem(
    args: argparse.Namespace,
    fatal_callback: ExecutorFatalCallback | None = None,
):
    """构造 SensorStreamState + SensorRosRuntime；失败则返回 (None, None)。"""
    from sensor_bridge.state import SensorStreamState

    state = SensorStreamState()
    try:
        from sensor_bridge.ros_node import SensorRosRuntime

        runtime = SensorRosRuntime(
            state=state,
            camera_topic=args.camera_topic,
            lidar_topic=args.lidar_topic,
            jpeg_quality=80,
            camera_fps=10.0,
            fatal_callback=fatal_callback,
        )
    except Exception as error:  # noqa: BLE001 - 传感器子系统尽力而为
        logger.warning("传感器子系统未启动：%s", error)
        return None, None
    return state, runtime


def _build_zenoh_source(args: argparse.Namespace):
    """构造 ZenohSource；失败则返回 None。"""
    try:
        from sse_bridge import ZenohSource

        return ZenohSource(connect_endpoint=args.zenoh)
    except Exception as error:  # noqa: BLE001 - SSE 子系统尽力而为
        logger.warning("Zenoh SSE 桥未启动：%s", error)
        return None


def main() -> None:
    """解析参数、构造子系统、创建 FastAPI app 并启动 uvicorn。"""
    args = _parser().parse_args()
    allowed_origins = _effective_allowed_origins(args)
    fatal_callback = _build_process_fatal_callback()

    from control_gateway import ControlServer

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
        allowed_origins=allowed_origins,
        fatal_callback=fatal_callback,
    )

    sensor_state = None
    sensor_runtime = None
    if not args.no_sensors:
        sensor_state, sensor_runtime = _build_sensor_subsystem(
            args,
            fatal_callback,
        )
    zenoh_source = None if args.no_sse else _build_zenoh_source(args)

    from app.main import create_app

    app = create_app(
        control_server=server,
        sensor_state=sensor_state,
        sensor_runtime=sensor_runtime,
        zenoh_source=zenoh_source,
        enable_db=True,
        static_dir=Path(__file__).resolve().parent,
        allowed_origins=allowed_origins,
    )
    print(f"Control server: http://{args.host}:{args.port}")
    print(f"API 文档:      http://localhost:{args.port}/docs")
    if sensor_state is not None:
        print(f"相机 MJPEG:   http://localhost:{args.port}/camera.mjpg")
        print(f"LiDAR WS:     ws://localhost:{args.port}/lidar/ws")
    if zenoh_source is not None:
        print(f"SSE 数据:     http://localhost:{args.port}/sse/<key>")
    print(f"监控面板:     http://localhost:{args.port}/monitor.html")

    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        log_level="info",
        # EventSource/MJPEG/WebSocket 需要在查询参数传递 token。
        # 禁用请求行日志，防止凭证落入终端或日志文件。
        access_log=False,
    )


if __name__ == "__main__":
    main()
