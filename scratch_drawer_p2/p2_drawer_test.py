#!/usr/bin/env python3
"""P2 drawer physics acceptance driver.

Establishes an operation session (active_control + heartbeat) against the
plugin at /xczs/cabinet/test and exercises the drawer services:
  * single-hand grasp must be rejected for a drawer,
  * unlock requires the right tool inside the logical unlock zone,
  * bimanual attach requires BOTH tools inside the grasp threshold,
  * detach/reset leaves no residual runtime joints,
  * unlock=false and reset re-engage the rail latch.
"""
import subprocess
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from xczs_inspection_robot_interfaces.srv import (
    SetCabinetBimanualGrasp, SetCabinetGrasp, SetCabinetUnlock)


NS = "/xczs/cabinet/test"
CONTROL = "test_drawer"
LEASE = "lease-001"
ROBOT_BOTH = "robot_both"
ROBOT_UNLOCK = "robot_unlock"
ROBOT_LEFT_ONLY = "robot_left_only"
ROBOT_FAR = "robot_far"
ORIGIN = {"x": 0.0, "y": 0.0, "z": 0.0}


def point():
    from geometry_msgs.msg import Point
    return Point(x=0.0, y=0.0, z=0.0)


class DrawerP2Driver(Node):
    def __init__(self):
        super().__init__("drawer_p2_driver")
        # The plugin's active_control subscription is transient_local; a
        # volatile publisher is QoS-incompatible and silently drops all
        # messages, which the real operator avoids by matching durability.
        self.active_pub = self.create_publisher(
            String, NS + "/active_control",
            QoSProfile(depth=10,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))
        self.hb_pub = self.create_publisher(String, NS + "/operation_heartbeat", 10)
        self.grasp_cli = self.create_client(SetCabinetGrasp, NS + "/grasp")
        self.bimanual_cli = self.create_client(SetCabinetBimanualGrasp, NS + "/bimanual_grasp")
        self.unlock_cli = self.create_client(SetCabinetUnlock, NS + "/unlock")
        self.reset_cli = self.create_client(Trigger, NS + "/reset_physics")
        self.state = None
        self.state_sub = self.create_subscription(
            type(self)._state_type(), NS + "/test_drawer/state", self._on_state, 10)
        self._hb_stop = False
        # NB: rclpy Node.executor is a property that stores only a weak
        # reference, so the executor must be kept alive by a differently-named
        # strong attribute (self._spin_executor) or it is GC'd immediately.
        self._spin_executor = rclpy.executors.MultiThreadedExecutor()
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    @staticmethod
    def _call(cli, req, timeout=20.0):
        """Blocking service call that works with the background spin thread."""
        future = cli.call_async(req)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                raise TimeoutError(f"service call timed out: {cli.srv_name}")
            time.sleep(0.02)
        return future.result()

    @staticmethod
    def _state_type():
        from xczs_inspection_robot_interfaces.msg import CabinetControlState
        return CabinetControlState

    def _on_state(self, msg):
        self.state = msg

    def start_session(self):
        self.active_pub.publish(String(data=CONTROL))
        self.hb_pub.publish(String(data=LEASE))
        t = threading.Thread(target=self._hb_loop, daemon=True)
        t.start()

    def _hb_loop(self):
        while not self._hb_stop:
            self.active_pub.publish(String(data=CONTROL))
            self.hb_pub.publish(String(data=LEASE))
            time.sleep(0.4)

    def stop_session(self):
        self._hb_stop = True

    def wait_services(self, timeout=15.0):
        for cli in (self.grasp_cli, self.bimanual_cli, self.unlock_cli, self.reset_cli):
            assert cli.wait_for_service(timeout), f"service {cli.srv_name} unavailable"

    def grasp(self, attach):
        req = SetCabinetGrasp.Request()
        req.control_id = CONTROL
        req.operation_lease_id = LEASE
        req.robot_model = ROBOT_BOTH
        req.robot_link = "left_tool"
        req.robot_grasp_point = point()
        req.robot_base_link = "base_link"
        req.attach = attach
        return DrawerP2Driver._call(self.grasp_cli, req)

    def unlock(self, robot_model, unlock):
        req = SetCabinetUnlock.Request()
        req.control_id = CONTROL
        req.operation_lease_id = LEASE
        req.robot_model = robot_model
        req.right_robot_link = "right_tool"
        req.right_robot_grasp_point = point()
        req.unlock_zone_point = point()
        req.unlock = unlock
        return DrawerP2Driver._call(self.unlock_cli, req)

    def bimanual(self, robot_model, attach):
        req = SetCabinetBimanualGrasp.Request()
        req.control_id = CONTROL
        req.operation_lease_id = LEASE
        req.robot_model = robot_model
        req.left_robot_link = "left_tool"
        req.right_robot_link = "right_tool"
        req.left_robot_grasp_point = point()
        req.right_robot_grasp_point = point()
        req.left_handle_point = point()
        req.right_handle_point = point()
        req.robot_base_link = "base_link"
        req.attach = attach
        return DrawerP2Driver._call(self.bimanual_cli, req)

    def reset(self):
        return DrawerP2Driver._call(self.reset_cli, Trigger.Request())


def check(name, cond, detail=""):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}" + (f"  ({detail})" if detail else ""))
    return cond


def residual_runtime_joints():
    out = subprocess.run(
        ["gz", "model", "-m", "drawer_test_cabinet", "-i"],
        capture_output=True, text=True).stdout
    return [line for line in out.splitlines()
            if "xczs_cabinet_runtime" in line]


def main():
    rclpy.init()
    driver = DrawerP2Driver()
    driver.start_session()
    driver.wait_services()
    results = []

    try:
        # T1: single-hand grasp on a drawer must be rejected.
        r = driver.grasp(attach=True)
        results.append(check("T1 single grasp rejected on drawer",
                             not r.success, r.message))

        # T2: unlock with right tool far away must fail.
        r = driver.unlock(ROBOT_FAR, True)
        results.append(check("T2 unlock far rejected",
                             not r.success, r.message))

        # T3: unlock with right tool at the logical unlock zone must succeed.
        r = driver.unlock(ROBOT_UNLOCK, True)
        results.append(check("T3 unlock at zone succeeds",
                             r.success and r.drawer_unlocked, r.message))

        # T4: bimanual attach with only one tool in range must be rejected.
        r = driver.bimanual(ROBOT_LEFT_ONLY, True)
        results.append(check("T4 single-side bimanual rejected",
                             not r.success, r.message))

        # T5: bimanual attach with both tools in range succeeds.
        r = driver.bimanual(ROBOT_BOTH, True)
        results.append(check("T5 bimanual attach succeeds",
                             r.success and r.left_grasped and r.right_grasped,
                             r.message))

        # T6: single-hand grasp still rejected while bimanual is active.
        r = driver.grasp(attach=True)
        results.append(check("T6 single grasp rejected while bimanual",
                             not r.success, r.message))

        # T7: idempotent re-attach succeeds.
        r = driver.bimanual(ROBOT_BOTH, True)
        results.append(check("T7 idempotent bimanual re-attach",
                             r.success, r.message))

        # T8: detach succeeds.
        r = driver.bimanual(ROBOT_BOTH, False)
        results.append(check("T8 bimanual detach succeeds", r.success, r.message))

        # T9: no residual runtime joints after detach.
        time.sleep(0.5)
        residual = residual_runtime_joints()
        results.append(check("T9 no residual runtime joints after detach",
                             not residual, "; ".join(residual)))

        # T10: unlock again while still unlocked (never opened) is idempotent.
        r = driver.unlock(ROBOT_UNLOCK, True)
        results.append(check("T10 unlock idempotent while unlocked",
                             r.success, r.message))

        # T11: explicit unlock=false re-engages the latch.
        r = driver.unlock(ROBOT_UNLOCK, False)
        results.append(check("T11 unlock=false re-engages latch",
                             r.success, r.message))

        # T12: bimanual attach must be refused while latched again.
        r = driver.bimanual(ROBOT_BOTH, True)
        results.append(check("T12 attach refused after re-lock",
                             not r.success, r.message))

        # T13: reset cleans up and re-latches.
        r = driver.reset()
        results.append(check("T13 reset succeeds", r.success, r.message))
        residual = residual_runtime_joints()
        results.append(check("T13b no residual runtime joints after reset",
                             not residual, "; ".join(residual)))

        # T14: attach must be refused after reset re-latched the drawer.
        r = driver.bimanual(ROBOT_BOTH, True)
        results.append(check("T14 attach refused after reset re-lock",
                             not r.success, r.message))

        # T15: full unlock+attach+detach cycle still works after the above.
        r = driver.unlock(ROBOT_UNLOCK, True)
        ok_unlock = r.success
        r = driver.bimanual(ROBOT_BOTH, True)
        ok_attach = r.success
        r = driver.bimanual(ROBOT_BOTH, False)
        ok_detach = r.success
        results.append(check("T15 full cycle unlock/attach/detach",
                             ok_unlock and ok_attach and ok_detach,
                             f"unlock={ok_unlock} attach={ok_attach} detach={ok_detach}"))

        if driver.state is not None:
            results.append(check("T16 drawer stayed at closed position",
                                 abs(driver.state.position) < 0.05,
                                 f"position={driver.state.position:.4f}"))
    finally:
        driver.stop_session()
        driver.destroy_node()
        rclpy.shutdown()

    passed = sum(results)
    print(f"\n{passed}/{len(results)} checks passed")
    sys.exit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
