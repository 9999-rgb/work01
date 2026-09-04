#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Record rod joints + drawer state during a capped drawer run, for post-hoc
diagnosis of hook/support rod travel (contact stalls etc.).

Logs one line per 100 ms to the given file:
  <t> r1..r5 | L2ref L2act L2err | R1ref R1act R1err | drawer drawer_state
where r1..r5 are the ACTUAL rod positions (joint_states, the same source the
operator's real-JS cache consumes), L2 = l_two_cyl_finger2 (left gripper),
R1 = r_three_cyl_finger1 (right gripper) commanded-vs-actual from each
cylinder controller's JointTrajectoryControllerState (reference=desired,
feedback=actual, error=ref-act), and drawer is db1's slide.
"""
import argparse
import time
import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from xczs_inspection_robot_interfaces.msg import CabinetControlState

NS = "/xczs/cabinet/electrical_mezzanine"
RODS = ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint",
        "r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
        "r_three_cyl_finger3_joint"]
CTRL = {  # controller_state topic -> (label, grip, support)
    "/xczs/two_cylinder_controller/controller_state":
        ("L", "l_two_cyl_finger2_joint", "l_two_cyl_finger1_joint"),
    "/xczs/three_cylinder_controller/controller_state":
        ("R", "r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint"),
}


class RodTraceRecorder(Node):
    def __init__(self, out_path):
        super().__init__("rod_trace_recorder")
        self._out = open(out_path, "w", buffering=1)  # line-buffered
        self._lock = threading.Lock()
        self._pos = {}
        self._ctrl = {t: None for t in CTRL}
        self._drawer = (float("nan"), "")
        self._subs = []
        try:
            self._subs.append(self.create_subscription(
                JointState, "/xczs/joint_states", self._on_joint_state, 50))
        except Exception:  # noqa: BLE001
            pass
        for t in CTRL:
            self._subs.append(self.create_subscription(
                JointTrajectoryControllerState, t,
                self._on_ctrl_state(t), 50))
        self._state_sub = self.create_subscription(
            CabinetControlState, f"{NS}/db1/state", self._on_state, 50)
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()
        self._t0 = time.monotonic()

    def _on_joint_state(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in RODS:
                    self._pos[name] = msg.position[i]

    def _on_ctrl_state(self, topic):
        def cb(msg):
            with self._lock:
                self._ctrl[topic] = (msg.joint_names, msg.reference, msg.feedback)
        return cb

    def _on_state(self, msg):
        with self._lock:
            self._drawer = (msg.position, msg.state_id)

    def stop(self):
        if getattr(self, "_spin", None) and self._spin.is_alive():
            self._spin_executor.shutdown()
            self._spin.join(timeout=3.0)
        self._out.close()

    def run(self, duration_s):
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            with self._lock:
                vals = [self._pos.get(r, float("nan")) for r in RODS]
                refs = {}
                for t, (label, grip_j, sup_j) in CTRL.items():
                    entry = self._ctrl.get(t)
                    if not entry:
                        continue
                    names, ref, fb = entry
                    for role, jn in (("g", grip_j), ("s", sup_j)):
                        if names and jn in names:
                            i = names.index(jn)
                            refs[label + role] = (ref.positions[i],
                                                  fb.positions[i],
                                                  ref.positions[i] - fb.positions[i])
                dp, ds = self._drawer
            bits = " ".join(f"{v:.5f}" for v in vals)
            l2 = refs.get("Lg", (float("nan"),) * 3)
            r1 = refs.get("Rg", (float("nan"),) * 3)
            self._out.write(
                f"{time.monotonic() - self._t0:8.2f} {bits} | "
                f"L {l2[0]:8.5f}/{l2[1]:8.5f}({l2[2]:+8.5f}) | "
                f"R {r1[0]:8.5f}/{r1[1]:8.5f}({r1[2]:+8.5f}) | "
                f"{dp:.5f} {ds}\n")
            time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/rod_trace.log")
    parser.add_argument("--duration", type=float, default=900.0)
    args = parser.parse_args()

    rclpy.init()
    rec = RodTraceRecorder(args.out)
    try:
        rec.run(args.duration)
    finally:
        rec.stop()
        rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
