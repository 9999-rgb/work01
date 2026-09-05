#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sample gazebo-entity ground truth during a capped db1 run.

Reads /get_entity_state (gazebo world = map authority) for the two tool base
links plus the chassis and drawer, so the hook/support rod tips can be located
exactly (base pose x business point, q-independent) instead of inferred from
stalls.  Also logs the five rod joints from /xczs/joint_states.

Usage:
  python3 tip_entity_probe.py --out /tmp/tip_probe.log --console <console.log>
Waits for the operator's "arrival dwell active" marker in the console log
(sampled until it appears), then records one line per ~100 ms for `--sample`
seconds.  Every line carries a wall-clock epoch so it can be aligned with the
operator console and rod-trace logs.
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from sensor_msgs.msg import JointState

BASES = {
    "L": ("l_two_cyl_base", (-0.034, -0.061735, -0.3885)),   # L hook rod end, q=0 (adapter contract)
    "R": ("r_three_cyl_base", (0.034, -0.061735, -0.3885)),  # R hook rod end, q=0
}
# The real rods: finger-link origins (the actual cylinders that press the
# plates).  Sampled with their full orientation so the rod axis can be read
# directly instead of trusting the adapter contact-offset frame.
FINGERS = ["l_two_cyl_finger1", "l_two_cyl_finger2",
           "r_three_cyl_finger1", "r_three_cyl_finger2",
           "r_three_cyl_finger3"]
RODS = ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint",
        "r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
        "r_three_cyl_finger3_joint"]
SCENE = ["body", "b1"]


def apply_quat(p, q):
    """Rotate point p by quaternion q=(x,y,z,w)."""
    x, y, z, w = q
    qv = (x, y, z)
    cross = (y * p[2] - z * p[1], z * p[0] - x * p[2], x * p[1] - y * p[0])
    dot = x * p[0] + y * p[1] + z * p[2]
    return tuple(p[i] + 2.0 * (w * cross[i] + (qv[(i + 1) % 3] * cross[(i + 2) % 3] -
                                               qv[(i + 2) % 3] * cross[(i + 1) % 3])) for i in range(3))


class TipEntityProbe(Node):
    def __init__(self, out_path, console_log):
        super().__init__("tip_entity_probe")
        self._out = open(out_path, "w", buffering=1)
        self._lock = threading.Lock()
        self._pos = {}
        self._js = self.create_subscription(
            JointState, "/xczs/joint_states", self._on_js, 20)
        self._console = console_log
        self._client = self.create_client(GetEntityState, "/get_entity_state")
        while not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("waiting for /get_entity_state ...")
        self._executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()

    def _on_js(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in RODS:
                    self._pos[name] = msg.position[i]

    def _entity(self, name):
        """Blocking entity state call (responded by the background executor)."""
        req = GetEntityState.Request()
        req.name = name
        resp = self._client.call(req)
        if resp is None or not resp.success:
            return None
        p = resp.state.pose.position
        o = resp.state.pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    def sample_once(self):
        with self._lock:
            rods = " ".join(f"{self._pos.get(r, float('nan')):.5f}" for r in RODS)
        stamp = time.time()
        self._out.write(f"{stamp:14.3f} RODS {rods}\n")
        for label, (link, contact) in BASES.items():
            got = self._entity(link)
            if got is None:
                self._out.write(f"{stamp:14.3f} {label} {link} NONE\n")
                continue
            pos, quat = got
            tip = apply_quat(contact, quat)
            tip = tuple(pos[i] + tip[i] for i in range(3))
            self._out.write(
                f"{stamp:14.3f} {label} {link} base "
                f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} | "
                f"q {quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f} | "
                f"tip0 {tip[0]:.6f} {tip[1]:.6f} {tip[2]:.6f}\n")
        for name in FINGERS:
            got = self._entity(name)
            if got is None:
                self._out.write(f"{stamp:14.3f} {name} NONE\n")
                continue
            pos, quat = got
            self._out.write(
                f"{stamp:14.3f} {name} "
                f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
                f"q {quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}\n")
        for name in SCENE:
            got = self._entity(name)
            if got is None:
                self._out.write(f"{stamp:14.3f} {name} NONE\n")
                continue
            pos, quat = got
            self._out.write(
                f"{stamp:14.3f} {name} {pos[0]:.6f} {pos[1]:.6f} "
                f"{pos[2]:.6f} qz {quat[2]:.6f}\n")
        self._out.flush()

    def _wait_marker(self, marker, timeout_s):
        """Watch only bytes appended after probe start (old runs also contain
        the same marker in the shared console file)."""
        try:
            offset = open(self._console, "rb").seek(0, 2)
        except OSError:
            offset = 0
        end = time.time() + timeout_s
        while time.time() < end:
            try:
                with open(self._console, "rb") as f:
                    f.seek(offset)
                    if marker.encode() in f.read():
                        return True
                    offset = f.tell()
            except OSError:
                pass
            time.sleep(0.25)
        return False

    def run(self, sample_s, pre_s, dwell_wait_s):
        end_pre = time.time() + pre_s
        while time.time() < end_pre:            # pre-dwell background sampling
            self.sample_once()
            time.sleep(0.2)
        if self._wait_marker("arrival dwell active", dwell_wait_s):
            self._out.write(f"{time.time():14.3f} DWELL_SEEN\n")
        else:
            self._out.write(f"{time.time():14.3f} NO_DWELL_MARKER\n")
        end = time.time() + sample_s
        while time.time() < end:
            self.sample_once()
            time.sleep(0.15)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/tip_probe.log")
    parser.add_argument("--console", default="/tmp/xczs_restart_v11_console.log")
    parser.add_argument("--pre", type=float, default=15.0)
    parser.add_argument("--sample", type=float, default=45.0)
    parser.add_argument("--dwell-wait", type=float, default=300.0)
    args = parser.parse_args()

    rclpy.init()
    probe = TipEntityProbe(args.out, args.console)
    try:
        probe.run(args.sample, args.pre, args.dwell_wait)
    finally:
        # 必须先停掉 spin 线程再销毁：executor 还在 spin 时直接
        # destroy_node/shutdown 会触发 "terminate called without an active
        # exception" 核心转储（rclpy 退出路径要求 spin 线程已退出）。
        # 与 db1_stage_cap_driver.py stop() 的处置一致。
        if rclpy.ok():
            probe._executor.shutdown()
            probe._spin.join(timeout=5.0)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
