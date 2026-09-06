#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Controlled single-rod sweep at the current pose (parked), to localize the
right three-cylinder tool's internal contact that pins finger1/finger2 with
~11-37 N at home.

Commands ONE rod through a slow position ladder and logs, per step, the
settled actual position and the controller output effort (from the raw
controller_state topic arrays).  A free-space rod tracks the ladder with
effort ~0; a collision ridge shows as a stick / effort spike / position lag.

Usage:
  python3 scripts/tools/rod_sweep_probe.py --rod r_three_cyl_finger1_joint \
      --ladder 0.000,0.004,0.008,0.012,0.016,0.020,0.016,0.012,0.008,0.004,0.000 \
      --hold 2.0
"""
import argparse
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import builtin_interfaces.msg

CTRL = {
    "r_three_cyl_finger1_joint": "/xczs/three_cylinder_controller/follow_joint_trajectory",
    "r_three_cyl_finger2_joint": "/xczs/three_cylinder_controller/follow_joint_trajectory",
    "r_three_cyl_finger3_joint": "/xczs/three_cylinder_controller/follow_joint_trajectory",
    "l_two_cyl_finger1_joint": "/xczs/two_cylinder_controller/follow_joint_trajectory",
    "l_two_cyl_finger2_joint": "/xczs/two_cylinder_controller/follow_joint_trajectory",
}
ST_TOPIC = {
    "r_three_cyl_finger1_joint": "/xczs/three_cylinder_controller/controller_state",
    "r_three_cyl_finger2_joint": "/xczs/three_cylinder_controller/controller_state",
    "r_three_cyl_finger3_joint": "/xczs/three_cylinder_controller/controller_state",
    "l_two_cyl_finger1_joint": "/xczs/two_cylinder_controller/controller_state",
    "l_two_cyl_finger2_joint": "/xczs/two_cylinder_controller/controller_state",
}
ALL_RODS = list(CTRL.keys())


class Sweep(Node):
    def __init__(self, rod, hold):
        super().__init__("rod_sweep_probe")
        self._rod = rod
        self._hold = hold
        self._act = {}
        self._eff = {}
        self._lock = threading.Lock()
        self._js = self.create_subscription(
            JointState, "/xczs/joint_states", self._on_js, 20)
        self._st = self.create_subscription(
            JointTrajectoryControllerState, ST_TOPIC[rod], self._on_st, 20)
        self._client = ActionClient(
            self, FollowJointTrajectory, CTRL[rod])
        while not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("waiting for FJT server ...")
        self._executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()
        time.sleep(0.5)

    def _on_js(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in ALL_RODS:
                    self._act[name] = msg.position[i]

    def _on_st(self, msg):
        # JointTrajectoryControllerState: output.effort[] aligned with joint_names
        with self._lock:
            for i, name in enumerate(msg.joint_names):
                if name == self._rod:
                    if len(msg.output.effort) > i:
                        self._eff[name] = msg.output.effort[i]

    def snap(self):
        with self._lock:
            return (self._act.get(self._rod), self._eff.get(self._rod))

    def cmd(self, ref):
        # The controller rejects partial-joint goals; include every joint of the
        # tool's controller, holding the non-swept rods at their current pos.
        goal = FollowJointTrajectory.Goal()
        joints = sorted(self._ctrl_joints())
        goal.trajectory.joint_names = joints
        with self._lock:
            holds = {j: self._act.get(j, 0.0) for j in joints}
        pt = JointTrajectoryPoint()
        pt.positions = [ref if j == self._rod else holds[j] for j in joints]
        pt.velocities = [0.0] * len(joints)
        pt.time_from_start = builtin_interfaces.msg.Duration(sec=1)
        goal.trajectory.points = [pt]
        self._client.send_goal_async(goal)

    def _ctrl_joints(self):
        # joints that share the same controller as self._rod
        if self._rod.startswith("r_three"):
            return ["r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
                    "r_three_cyl_finger3_joint"]
        return ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint"]

    def run(self, ladder):
        for ref in ladder:
            self.cmd(ref)
            deadline = time.monotonic() + self._hold
            last = None
            while time.monotonic() < deadline:
                a, e = self.snap()
                if a is not None:
                    last = (a, e)
                time.sleep(0.05)
            if last:
                print(f"ref={ref:.4f}  act={last[0]:.4f}  "
                      f"eff={'%7.1f' % last[1] if last[1] is not None else 'NA'}")
            else:
                print(f"ref={ref:.4f}  act=NA")
        self._executor.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rod", required=True)
    ap.add_argument("--ladder", required=True,
                    help="comma list of refs, e.g. 0,0.004,...,0")
    ap.add_argument("--hold", type=float, default=2.0)
    args = ap.parse_args()
    ladder = [float(x) for x in args.ladder.split(",")]
    rclpy.init()
    n = Sweep(args.rod, args.hold)
    n.run(ladder)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
