#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rod-channel / drawer telemetry sampler for AGENT §7.2 live sealing runs.

Writes one line per ~0.25 s to <out> (default /tmp/rod_contact.log):
  wall_t f1p f1v f1e f2p f2v f2e r1p r1v r1e r2p r2v r2e r3p r3v r3e
        ref err tipx tipy tipz dpos dstate
(f1/f2 = two_cylinder [0,1]; r1-r3 = three_cylinder [0,1,2]; ref/err =
two_cylinder controller second-joint reference/position-error; tip =
l_two_cyl_finger2 + local (0,0,-0.095) in map; dpos/dstate = drawer db1.)

Purpose: capture rod drive evidence (arrival, overshoot, ODE sleep/stick,
frozen JTC refs) and drawer state during operator stage runs, so JTC
time-based "success" can be cross-checked against measured behavior.
"""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from xczs_inspection_robot_interfaces.msg import CabinetControlState
import tf2_ros

NS = "/xczs"
F1 = "l_two_cyl_finger1_joint"
F2 = "l_two_cyl_finger2_joint"
R1 = "r_three_cyl_finger1_joint"
R2 = "r_three_cyl_finger2_joint"
R3 = "r_three_cyl_finger3_joint"
RODS = [F1, F2, R1, R2, R3]
F2_LINK = "l_two_cyl_finger2"  # TF frame (link), NOT the _joint name
TIP_LOCAL = (0.0, 0.0, -0.095)  # finger2 tip offset (same as /tmp probes)


class RodSampler(Node):
    def __init__(self):
        super().__init__("rod_channel_sampler")
        self.lock = threading.Lock()
        self.js = {}
        self.refs = []
        self.errs = []
        self.drawer = None
        self._odom_off = None
        self.create_subscription(JointState, NS + "/joint_states",
                                 self._on_js, 100)
        try:
            self.create_subscription(JointState, "/joint_states",
                                     self._on_js, 100)
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
        self._buf = tf2_ros.Buffer()
        # No spin_thread: the main loop's spin_once feeds this buffer (a
        # second internal executor would fight the main executor for the
        # node and starve /tf).
        self._tfl = tf2_ros.TransformListener(self._buf, self)
        self.tip = (float("nan"), float("nan"), float("nan"))

    def _on_js(self, msg):
        with self.lock:
            for i, n in enumerate(msg.name):
                if n in RODS:
                    self.js[n] = (msg.position[i], msg.velocity[i],
                                  msg.effort[i])

    def _on_jtc(self, msg):
        with self.lock:
            self.refs = list(msg.reference.positions or [])
            self.errs = list(msg.error.positions or [])

    def _on_state(self, msg):
        with self.lock:
            self.drawer = (msg.position, msg.state_id)

    def _tip_in_map(self):
        """Tip frame = finger2 + TIP_LOCAL; anchor map via finger2 TF, else
        odom-anchored chain with map->odom offset.

        Latest-only reads, NO timeout: a timed wait_for_transform spins this
        node from the caller thread and fights the main-loop executor (rows
        stall for the full wait).  A miss keeps the previous value."""
        try:
            t = self._buf.lookup_transform("map", F2_LINK, rclpy.time.Time())
            tr = t.transform.translation
            return (tr.x, tr.y, tr.z)
        except Exception:  # noqa: BLE001
            try:
                t = self._buf.lookup_transform("odom", F2_LINK,
                                               rclpy.time.Time())
                m = self._buf.lookup_transform("map", "odom",
                                               rclpy.time.Time())
                tr, mo = t.transform.translation, m.transform.translation
                return (tr.x + mo.x, tr.y + mo.y, tr.z + mo.z)
            except Exception:  # noqa: BLE001
                return None  # caller keeps previous value

    def snap(self):
        with self.lock:
            row = []
            for r in RODS:
                p, v, e = self.js.get(r, (float("nan"), float("nan"),
                                          float("nan")))
                row += [p, v, e]
            ref = self.refs[1] if len(self.refs) > 1 else (
                self.refs[0] if self.refs else float("nan"))
            err = self.errs[1] if len(self.errs) > 1 else (
                self.errs[0] if self.errs else float("nan"))
            drawer = self.drawer if self.drawer else (float("nan"), "?")
            return (row, ref, err, self._tip_in_map(), drawer)


def main():
    out_path = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv \
        else "/tmp/rod_contact.log"
    duration = 900.0
    if "--duration" in sys.argv:
        duration = float(sys.argv[sys.argv.index("--duration") + 1])

    rclpy.init()
    node = RodSampler()
    out = open(out_path, "w", buffering=1)
    out.write(f"# wall_t0={time.time():.3f} (sampler-monotonic "
              f"{time.monotonic():.3f})\n")
    def _tip_loop():
        while rclpy.ok():
            fresh = node._tip_in_map()
            if fresh is not None:
                node.tip = fresh
            time.sleep(1.0)

    threading.Thread(target=_tip_loop, daemon=True).start()
    t0 = time.monotonic()
    t_last = float("-inf")
    while rclpy.ok() and time.monotonic() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        if now - t_last >= 0.25:
            t_last = now
            row, ref, err, _, drawer = node.snap()
            fields = [f"{now - t0:8.2f}"]
            fields += [f"{x:+9.5f}" for x in row]
            fields += [f"{ref:+9.5f}", f"{err:+9.5f}"]
            fields += [f"{node.tip[0]:9.4f}", f"{node.tip[1]:9.4f}",
                       f"{node.tip[2]:9.4f}"]
            fields += [f"{drawer[0]:9.5f} {drawer[1]}"]
            out.write(" ".join(fields) + "\n")
    out.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
