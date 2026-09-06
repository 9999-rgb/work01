#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取 ROS 2 图像话题单帧存 PNG（G 配色截图证据用）。

用法:
  ros_image_snapshot.py [-t /xczs/camera/image_raw] [-o /tmp/scene.png] [-n 3]

订阅 best_effort 图像话题，收到首个完整帧即写盘（等 n 帧可让相机白平衡稳定）。
文件放 scripts/tools/ 属开发取证工具，不入验收。
"""
import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


def decode(img: Image) -> np.ndarray:
    if img.encoding in ("rgb8", "bgr8", "rgba8", "bgra8"):
        n = int(img.encoding[0] == "b" and img.encoding.startswith("bgra") and 4 or 0)
        if img.encoding == "rgb8":
            return cv2.cvtColor(np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3), cv2.COLOR_RGB2BGR)
        if img.encoding == "bgr8":
            return np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3).copy()
        if img.encoding == "rgba8":
            return cv2.cvtColor(np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 4), cv2.COLOR_RGBA2BGR)
        if img.encoding == "bgra8":
            return cv2.cvtColor(np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 4), cv2.COLOR_BGRA2BGR)
    raise ValueError("unsupported encoding: " + img.encoding)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--topic", default="/xczs/camera/image_raw")
    ap.add_argument("-o", "--out", default="/tmp/scene_snapshot.png")
    ap.add_argument("-n", "--frames", type=int, default=3, help="订阅帧数后才落盘（稳定曝光）")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("image_snapshot")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST, depth=2)
    got = []
    sub = node.create_subscription(Image, args.topic, lambda m: got.append(m), qos)
    try:
        import time
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(got) >= args.frames:
                frame = decode(got[-1])
                cv2.imwrite(args.out, frame)
                print("saved %dx%d -> %s (frames=%d, stamp=%d.%d)" % (
                    frame.shape[1], frame.shape[0], args.out, len(got),
                    got[-1].header.stamp.sec, got[-1].header.stamp.nanosec))
                return 0
        print("no frame on %s within %.0fs" % (args.topic, args.timeout), file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
