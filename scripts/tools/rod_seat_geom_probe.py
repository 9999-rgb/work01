#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Live geometry probe for the db1 hook-press seat-takeover failure.

Samples gazebo /get_entity_state world poses for the drawer (b1) and the two
gripper finger links plus both tool bases, and the five rod joints, every
~100 ms from launch (no console gating).  Each line carries a wall epoch so it
can be aligned with the operator console and rod-trace logs.

Purpose: locate the left/right gripper-disk contact point (finger origin +
q*(0,0,-rod_len)) against the drawer handle-plate window during the hook press /
3s hold / seat takeover, to decide whether the right disk is centered on the
plate face (y in [4.685,4.701], z in [0.904,1.000], front face x ~ 0.099) or
riding the plate edge/chamfer (grind-creep + spring-back signature).

Usage:
  python3 scripts/tools/rod_seat_geom_probe.py --out /tmp/geom_probe.log \
      --seconds 300
"""
import argparse
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from sensor_msgs.msg import JointState

RODS = ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint",
        "r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
        "r_three_cyl_finger3_joint"]
# entity-name, (contact offset local on the finger link)
FINGERS = {
    "l_two_cyl_finger1": (0.0, 0.0, -0.075),
    "l_two_cyl_finger2": (0.0, 0.0, -0.095),
    "r_three_cyl_finger1": (0.0, 0.0, -0.090),
    "r_three_cyl_finger2": (0.0, 0.0, -0.075),
    "r_three_cyl_finger3": (0.0, 0.0, -0.013002),
}
SCENE = ["b1"]


def apply_quat(p, q):
    x, y, z, w = q
    qv = (x, y, z)
    cross = (y * p[2] - z * p[1], z * p[0] - x * p[2], x * p[1] - y * p[0])
    dot = x * p[0] + y * p[1] + z * p[2]
    return tuple(p[i] + 2.0 * (w * cross[i] + (qv[(i + 1) % 3] * cross[(i + 2) % 3] -
                                               qv[(i + 2) % 3] * cross[(i + 1) % 3])) for i in range(3))


class GeomProbe(Node):
    def __init__(self, out_path):
        super().__init__("rod_seat_geom_probe")
        self._out = open(out_path, "w", buffering=1)
        self._lock = threading.Lock()
        self._pos = {}
        self._js = self.create_subscription(
            JointState, "/xczs/joint_states", self._on_js, 20)
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
        req = GetEntityState.Request()
        req.name = name
        try:
            resp = self._client.call(req)
        except Exception:
            return None
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
        for name in FINGERS:
            got = self._entity(name)
            if got is None:
                self._out.write(f"{stamp:14.3f} {name} NONE\n")
                continue
            pos, quat = got
            c = apply_quat(FINGERS[name], quat)
            tip = tuple(pos[i] + c[i] for i in range(3))
            self._out.write(
                f"{stamp:14.3f} {name} {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
                f"q {quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f} "
                f"tip {tip[0]:.6f} {tip[1]:.6f} {tip[2]:.6f}\n")
        for name in SCENE:
            got = self._entity(name)
            if got is None:
                self._out.write(f"{stamp:14.3f} {name} NONE\n")
                continue
            pos, quat = got
            self._out.write(f"{stamp:14.3f} {name} {pos[0]:.6f} {pos[1]:.6f} "
                            f"{pos[2]:.6f} qz {quat[2]:.6f}\n")
        self._out.flush()

    def run(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.sample_once()
            time.sleep(0.1)
        self._spin_executor_shutdown = True
        self._executor.shutdown()
        self._out.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=300.0)
    args = ap.parse_args()
    rclpy.init()
    node = GeomProbe(args.out)
    node.run(args.seconds)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
