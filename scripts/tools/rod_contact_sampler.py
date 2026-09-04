#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cap2 hook-stage live sampler: does the rod get DRIVEN but blocked, or is
it never driven at all?

While an operator capped drawer run is in progress, log one line per 50 ms:
  <t> f1_pos f1_vel f1_eff  f2_pos f2_vel f2_eff  r1_pos r2_pos r3_pos  \
      ref2 err2  tip_x tip_y tip_z  drawer_pos drawer_state

Channels:
  /xczs/joint_states          - measured rod position/velocity/effort (the
                                effort answers "motor pushing?").
  /xczs/two_cylinder_controller/controller_state - JTC reference trajectory
                                (did the ramp run?) and error (= ref - fb).
  /tf                          - map->l_two_cyl_finger2 contact tip, so the
                                real rod tip position can be compared with the
                                fin face at x=0.099 (fixture geometry is fixed
                                in the map frame). Tip local offset = adapter
                                contact_point_local [0,0,-0.095].
"""
import argparse
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from xczs_inspection_robot_interfaces.msg import CabinetControlState

import tf2_ros


def _quat_rotate(q, v):
    """Rotate vector v by quaternion (x, y, z, w). No external dep."""
    x, y, z, w = q
    # v' = v + 2*w*(q x v) + 2*(q x (q x v))
    cx, cy, cz = y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]
    dx, dy, dz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (v[0] + 2.0 * (w * cx + dx),
            v[1] + 2.0 * (w * cy + dy),
            v[2] + 2.0 * (w * cz + dz))

NS = "/xczs"
RODS = ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint",
        "r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
        "r_three_cyl_finger3_joint"]
FINGER2 = "l_two_cyl_finger2"
TIP_LOCAL_Z = -0.095  # gripper_contact_point_local (adapter drawer_tools.left)


class RodContactSampler(Node):
    def __init__(self, out_path):
        super().__init__("rod_contact_sampler")
        self._out = open(out_path, "w", buffering=1)
        self._lock = threading.Lock()
        self._rods = {r: (float("nan"), float("nan"), float("nan")) for r in RODS}
        self._jtc = None       # (state.state, ref positions list or None)
        self._tip = None       # (x, y, z)
        self._drawer = (float("nan"), "")
        self._buf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._buf, self, spin_thread=True)
        for topic in ("/joint_states", NS + "/joint_states"):
            try:
                self.create_subscription(
                    JointState, topic, self._on_joint_state, 100)
            except Exception:  # noqa: BLE001
                pass
        self.create_subscription(
            JointTrajectoryControllerState,
            NS + "/two_cylinder_controller/controller_state",
            self._on_jtc, 100)
        self.create_subscription(
            CabinetControlState,
            NS + "/cabinet/electrical_mezzanine/db1/state",
            self._on_state, 100)
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()
        self._t0 = time.monotonic()
        self._out.write(f"# wall_t0={time.time():.3f} (sampler-monotonic "
                        f"{self._t0:.3f})\n")

    def stop(self):
        if getattr(self, "_spin", None) and self._spin.is_alive():
            self._spin_executor.shutdown()
            self._spin.join(timeout=3.0)
        self._out.close()

    def _on_joint_state(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self._rods:
                    self._rods[name] = (
                        msg.position[i] if len(msg.position) > i else float("nan"),
                        msg.velocity[i] if len(msg.velocity) > i else float("nan"),
                        msg.effort[i] if len(msg.effort) > i else float("nan"))

    def _on_jtc(self, msg):
        try:
            with self._lock:
                refs = [p for p in (msg.reference.positions
                                    if msg.reference.positions else [])]
                errs = [p for p in (msg.error.positions
                                    if msg.error.positions else [])]
                self._jtc = (refs, errs)
        except Exception as exc:  # noqa: BLE001 — never kill the spin thread
            self.get_logger().error(f"jtc callback: {exc}")

    def _on_state(self, msg):
        with self._lock:
            self._drawer = (msg.position, msg.state_id)

    def _sample_tip(self):
        """Contact tip in world: finger2 link pose applied to the adapter
        contact_point_local [0,0,-0.095] (tool -z is the rod extension axis)."""
        try:
            t = self._buf.lookup_transform(
                "map", FINGER2, rclpy.time.Time(), Duration(seconds=0.2))
        except Exception:  # noqa: BLE001
            try:
                t = self._buf.lookup_transform(
                    "odom", FINGER2, rclpy.time.Time(), Duration(seconds=0.2))
            except Exception:  # noqa: BLE001
                return None
        t0 = t.transform.translation
        q = (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w)
        rel = _quat_rotate(q, (0.0, 0.0, TIP_LOCAL_Z))
        return (t0.x + rel[0], t0.y + rel[1], t0.z + rel[2])

    def run(self, duration_s):
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            tip = self._sample_tip()
            with self._lock:
                vals = []
                for r in RODS:
                    vals += [v for v in self._rods[r]]
                refs, errs = self._jtc if self._jtc else ([], [])
                ref2 = refs[1] if len(refs) > 1 else float("nan")
                err2 = errs[1] if len(errs) > 1 else float("nan")
                self._tip = tip
                dp, ds = self._drawer
            self._out.write(
                f"{time.monotonic() - self._t0:8.2f} "
                + " ".join(f"{v:8.5f}" for v in vals)
                + f"  {ref2:8.5f} {err2:8.5f}  "
                + (f"{tip[0]:8.5f} {tip[1]:8.5f} {tip[2]:8.5f}"
                   if tip else "      nan       nan       nan")
                + f"  {dp:.5f} {ds}\n")
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/rod_contact_sampler.log")
    parser.add_argument("--duration", type=float, default=420.0)
    args = parser.parse_args()

    rclpy.init()
    rec = RodContactSampler(args.out)
    try:
        rec.run(args.duration)
    finally:
        rec.stop()
        rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
