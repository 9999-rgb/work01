#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""等待统一入口 (run_all.sh --web) 就绪：/xczs/toolset/status JSON state=="ready"。
超时非零退出并打印已看到的状态。用法：python3 wait_stack_ready.py [timeout_s]"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy)
from std_msgs.msg import String

TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
TOPIC = "/xczs/toolset/status"


class Probe(Node):
    def __init__(self):
        super().__init__("wait_stack_ready_probe")
        self.status = None
        # toolset_supervisor 发布 QoS = RELIABLE + TRANSIENT_LOCAL；用相同
        # 耐久度才能收到已保留的最近状态（状态只在初值与状态迁移时发布，
        # 无周期心跳）。
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(
            String, TOPIC, self.cb, qos)
        self.sub  # keep ref

    def cb(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            self.status = {"raw": msg.data[:200]}


def main():
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + TIMEOUT
    last = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            s = node.status
            if s is not None and s.get("state") == "ready":
                print("READY", json.dumps(s, ensure_ascii=False))
                return 0
            if s != last:
                print("seen:", json.dumps(s, ensure_ascii=False))
                last = s
        print("TIMEOUT after %.0fs, last=%s" % (TIMEOUT, json.dumps(last, ensure_ascii=False)))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
