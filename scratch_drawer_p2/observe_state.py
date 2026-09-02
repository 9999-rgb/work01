#!/usr/bin/env python3
"""Observe the synthetic drawer + button states right after spawn."""
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from xczs_inspection_robot_interfaces.msg import CabinetControlState


class Observer(Node):
    def __init__(self):
        super().__init__("p2_observer")
        self.drawer = None
        self.button = None
        self.sub = self.create_subscription(
            CabinetControlState, "/xczs/cabinet/test/test_drawer/state",
            self._on_drawer, 10)
        self.sub2 = self.create_subscription(
            CabinetControlState, "/xczs/cabinet/test/test_drawer_unlock_button/state",
            self._on_button, 10)
        self._spin_executor = rclpy.executors.MultiThreadedExecutor()
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    def _on_drawer(self, msg):
        self.drawer = msg

    def _on_button(self, msg):
        self.button = msg


def main():
    rclpy.init()
    obs = Observer()
    start = time.time()
    while time.time() - start < 10.0:
        d = obs.drawer
        b = obs.button
        if d is not None or b is not None:
            ds = f"drawer pos={d.position:.4f} state={d.state_id} unlocked={d.activated}" if d else "drawer: none"
            bs = f"button pos={b.position:.4f} state={b.state_id} activated={b.activated}" if b else "button: none"
            print(f"t={time.time()-start:5.2f}  {ds}  |  {bs}", flush=True)
        time.sleep(0.5)
    obs.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
