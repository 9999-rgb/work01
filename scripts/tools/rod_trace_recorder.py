#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Record rod/arm joints + drawer state during a capped drawer run, for
post-hoc diagnosis of hook/support rod travel (contact stalls etc.).

Logs one line per 100 ms to the given file:
  <t> L0..L6 R0..R6 | r1..r5 | e1..e5 | Lg Ls Rg Rs Ru | drawer drawer_state
where L0..L6/R0..R6 are the ACTUAL left/right arm positions (joint_states,
the same source the operator's real-JS cache consumes), r1..r5 the ACTUAL
rod positions, e1..r5 their ACTUAL efforts, and each of Lg/Ls/Rg/Rs/Ru is a
commanded-vs-actual triple (ref/act/err where err=ref-act) read from the
cylinder controllers' JointTrajectoryControllerState, covering AGENT §12.4's
target-vs-measured columns for both hook grippers, both side supports and
the right unlock rod:
  Lg = l_two_cyl_finger2 (left gripper)      Ls = l_two_cyl_finger1 (left support)
  Rg = r_three_cyl_finger1 (right gripper)   Rs = r_three_cyl_finger2 (right support)
  Ru = r_three_cyl_finger3 (right unlock rod)
and drawer is db1's slide.  Column order below the header comment:
  L0..L6 = l_arm_0_joint..l_arm_6_joint
  R0..R6 = r_arm_0_joint..r_arm_6_joint
  r1..r5 = l_two_cyl_finger1/2, r_three_cyl_finger1/2/3
  e1..e5 = efforts of r1..r5
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
ARMS = [f"l_arm_{i}_joint" for i in range(7)] + \
       [f"r_arm_{i}_joint" for i in range(7)]
RODS = ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint",
        "r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
        "r_three_cyl_finger3_joint"]
ALL = ARMS + RODS
CTRL = {  # controller_state topic -> (label, roles by joint)
    "/xczs/two_cylinder_controller/controller_state":
        ("L", {"g": "l_two_cyl_finger2_joint", "s": "l_two_cyl_finger1_joint"}),
    "/xczs/three_cylinder_controller/controller_state":
        ("R", {"g": "r_three_cyl_finger1_joint", "s": "r_three_cyl_finger2_joint",
               "u": "r_three_cyl_finger3_joint"}),
}
# Output order of the commanded-vs-actual triples.
ROLE_ORDER = ("Lg", "Ls", "Rg", "Rs", "Ru")


class RodTraceRecorder(Node):
    def __init__(self, out_path):
        super().__init__("rod_trace_recorder")
        self._out = open(out_path, "w", buffering=1)  # line-buffered
        self._lock = threading.Lock()
        self._pos = {}
        self._eff = {}
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
                if name in ALL:
                    self._pos[name] = msg.position[i]
                    if len(msg.effort) > i:
                        self._eff[name] = msg.effort[i]

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
                ap = [self._pos.get(r, float("nan")) for r in ALL]
                rp = [self._pos.get(r, float("nan")) for r in RODS]
                re = [self._eff.get(r, float("nan")) for r in RODS]
                refs = {}
                for t, (label, role_joints) in CTRL.items():
                    entry = self._ctrl.get(t)
                    if not entry:
                        continue
                    names, ref, fb = entry
                    for role, jn in role_joints.items():
                        if names and jn in names:
                            i = names.index(jn)
                            refs[label + role] = (ref.positions[i],
                                                  fb.positions[i],
                                                  ref.positions[i] - fb.positions[i])
                dp, ds = self._drawer
            def _triple(key: str) -> str:
                v = refs.get(key, (float("nan"),) * 3)
                return f"{v[0]:9.5f}/{v[1]:9.5f}({v[2]:+8.5f})"
            self._out.write(
                f"{time.monotonic() - self._t0:8.2f} "
                f"{' '.join(f'{v:.5f}' for v in ap)} | "
                f"{' '.join(f'{v:.5f}' for v in rp)} | "
                f"{' '.join(f'{v:+.1f}' for v in re)} | "
                f"{' '.join(_triple(key) for key in ROLE_ORDER)} | "
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
