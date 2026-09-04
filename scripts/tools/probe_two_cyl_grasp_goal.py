#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Probe: reproduce the drawer hook-stage two-cylinder grasp rod goal in
isolation and watch the measured joint, to answer three questions:

  1. How far does l_two_cyl_finger2 actually travel toward 0.010 m?
  2. Does the controller terminate the goal (SUCCESS when tolerance met /
     GOAL_TOLERANCE_VIOLATED at the goal_time_tolerance deadline), or hold it
     indefinitely while the rod rests short of tolerance?
  3. Where does the measured rod rest relative to commanded 0.010?

Sends exactly the goal the operator builds in send_cylinder_full_joint_goal:
2 points (t=0 current -> t=2 s target), per-joint tolerance 0.005 for the
moving joint / 0.002 for held joints, goal_time_tolerance 5 s.  Samples
/joint_states every 200 ms for up to 20 s and prints the measured
l_two_cyl_finger{1,2} trajectory plus the goal outcome.
"""
import time
import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NS = "/xczs"
CONTROLLER = NS + "/two_cylinder_controller"
JOINTS = ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint"]
TARGET_FINGER2 = 0.010
RAMP_S = 2.0
GTT_S = 5.0
SAMPLE_S = 0.2
WATCH_S = 20.0


class TwoCylProbe(Node):
    def __init__(self):
        super().__init__("two_cyl_grasp_goal_probe")
        self._pos = {j: float("nan") for j in JOINTS}
        self._stamp = 0.0
        # joint_states may be bridged at /joint_states or namespaced.
        self._subs = []
        for topic in ("/joint_states", NS + "/joint_states"):
            try:
                self._subs.append(self.create_subscription(
                    JointState, topic, self._on_joint_state, 10))
            except Exception:  # noqa: BLE001
                pass
        self._client = ActionClient(
            self, FollowJointTrajectory, CONTROLLER + "/follow_joint_trajectory")
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    def stop(self):
        if getattr(self, "_spin", None) and self._spin.is_alive():
            self._spin_executor.shutdown()
            self._spin.join(timeout=3.0)

    def _on_joint_state(self, msg):
        self._stamp = time.monotonic()
        for i, name in enumerate(msg.name):
            if name in self._pos:
                self._pos[name] = msg.position[i]

    def _current(self):
        return [self._pos[j] for j in JOINTS]

    def run(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if all(p == p for p in self._current()):  # not NaN
                break
            time.sleep(0.1)
        start = self._current()
        if any(p != p for p in start):
            raise RuntimeError("no joint_states sample within 10 s")
        self.get_logger().info(
            f"current rods: {dict(zip(JOINTS, start))}")

        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("two_cylinder action server unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINTS
        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.velocities = [0.0, 0.0]
        p0.accelerations = [0.0, 0.0]
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = [start[0], TARGET_FINGER2]
        p1.velocities = [0.0, 0.0]
        p1.accelerations = [0.0, 0.0]
        p1.time_from_start.sec = int(RAMP_S)
        goal.trajectory.points = [p0, p1]
        goal.goal_time_tolerance.sec = int(GTT_S)
        for j, name in enumerate(JOINTS):
            tol = JointTolerance()
            tol.name = name
            tol.position = 0.005 if abs(TARGET_FINGER2 - start[1]) > 1e-4 \
                else 0.002
            goal.goal_tolerance.append(tol)

        send = self._client.send_goal_async(goal)
        # accept
        acc_deadline = time.monotonic() + 10.0
        while not send.done() and time.monotonic() < acc_deadline:
            time.sleep(0.02)
        if not send.done():
            raise RuntimeError("goal not accepted within 10 s")
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError(f"goal rejected: {gh!r}")
        self.get_logger().info(
            f"goal ACCEPTED at t=0; finger2 target {TARGET_FINGER2} m, "
            f"2 s ramp, tol 0.005, gtt {GTT_S} s. Sampling {WATCH_S} s...")

        result = gh.get_result_async()
        t0 = time.monotonic()
        prev = self._current()
        outcome = "NO RESULT"
        while time.monotonic() - t0 < WATCH_S:
            if result.done():
                r = result.result()
                outcome = (f"{r.status} (code {r.status}) "
                           f"error={r.result.error_code} "
                           f"{r.result.error_string or ''}")
                break
            cur = self._current()
            dt = time.monotonic() - t0
            if abs(cur[1] - prev[1]) >= 0.0002 or dt < 0.5:
                self.get_logger().info(
                    f"t={dt:6.2f}  finger1={cur[0]:.4f}  "
                    f"finger2={cur[1]:.4f}  err2={TARGET_FINGER2 - cur[1]:+.4f}")
            prev = cur
            time.sleep(SAMPLE_S)
        cur = self._current()
        self.get_logger().info(
            f"final: finger1={cur[0]:.4f} finger2={cur[1]:.4f} "
            f"err2={TARGET_FINGER2 - cur[1]:+.4f}")
        self.get_logger().info(f"goal outcome: {outcome}")


def main():
    rclpy.init()
    probe = TwoCylProbe()
    try:
        probe.run()
    finally:
        probe.stop()
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
