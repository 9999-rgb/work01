#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""db1 抽屉播放 RELEASE 保持语义探针（2026-09-07 AGENT T3 keep semantics 活验）。

直接调 /xczs/cabinet/electrical_mezzanine/playback（不经 Web/operator），把抽屉经
零戳立即播放开到指定轨位后 RELEASE，再用 db1/state 连续观察轨位是否保持：
  * open     开到业务开位 0.30 → RELEASE → 观察 ≥hold 秒：不得回弹/漂移。
  * mid25    开到 0.25（非档位中间）→ RELEASE → 观察：012 前此处被闭锁弹簧拉向
             开档位（实测 ~1 mm/s 漂移），本探针判定是否已静态保持。
  * mid15    开到 0.15 → RELEASE → 观察：不得被拉向关档位。
  * relatch  关回 0.0 → RELEASE → 观察：应立即复闩锁定、保持闭位不漂移。
每个位置判据：观察窗内最大 |Δrail| ≤ drift_tol，且终态不发散。
顺序 all = relatch → mid25 → mid15 → open（先归零再逐点，避免上一点的保持干扰）。
成功路径的 RELEASE 后一律不再运动，避免留下孤儿播放租约或错误中间态。

用法:
  python3 db1_hold_probe.py all
  python3 db1_hold_probe.py open|mid25|mid15|relatch
  python3 db1_hold_probe.py --pos 0.22 --hold 12     # 任意位置
"""
import argparse
import sys
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy)
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xczs_inspection_robot_interfaces.srv import SetCabinetPlayback
from xczs_inspection_robot_interfaces.msg import CabinetControlState

CONTROL_ID = "db1"
PLAYBACK_SERVICE = "/xczs/cabinet/electrical_mezzanine/playback"
STATE_TOPIC = "/xczs/cabinet/electrical_mezzanine/db1/state"
BUSINESS_OPEN = 0.30
BUSINESS_CLOSED = 0.0


def _sim_now(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


class Db1HoldProbe(Node):
    def __init__(self, hold, drift_tol, move_secs):
        super().__init__("db1_hold_probe")
        self.hold = float(hold)
        self.drift_tol = float(drift_tol)
        self.move_secs = float(move_secs)
        self.lease = "hold-probe-" + uuid.uuid4().hex[:10]
        self.position = None
        self.state_id = None
        self.velocity = None
        self.client = self.create_client(
            SetCabinetPlayback, PLAYBACK_SERVICE)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.state_sub = self.create_subscription(
            CabinetControlState, STATE_TOPIC, self.on_state, qos)
        # 后台 executor 强引用：订阅回调持续运转；service 用 call_async+轮询
        # future.done()，绝不主线程再 spin 同一 node（012 探针踩过的双 spin 坑）。
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(
            target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    def on_state(self, msg):
        self.position = msg.position
        self.state_id = msg.state_id
        self.velocity = msg.velocity

    def stop(self):
        if getattr(self, "_spin", None) and self._spin.is_alive():
            self._spin_executor.shutdown()
            self._spin.join(timeout=3.0)

    def wait_ready(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.client.service_is_ready() and self.position is not None:
                return
            time.sleep(0.1)
        raise RuntimeError("playback 服务/抽屉状态不可用")

    # --------------------------------------------------------------- service
    def _await(self, future, what, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("%s 请求超时" % what)
            time.sleep(0.02)
        return future.result()

    def _play(self, start_q, end_q, secs):
        req = SetCabinetPlayback.Request()
        req.command = SetCabinetPlayback.Request.COMMAND_START
        req.control_id = CONTROL_ID
        req.operation_lease_id = self.lease
        req.trajectory.joint_names = [CONTROL_ID]
        n = max(2, int(secs * 10.0))
        for i in range(n + 1):
            t = secs * i / n
            p = JointTrajectoryPoint()
            p.time_from_start = Duration(seconds=t).to_msg()
            p.positions = [start_q + (end_q - start_q) * i / n]
            p.velocities = [0.0]
            req.trajectory.points.append(p)
        return self._await(self.client.call_async(req), "START")

    def _release(self):
        req = SetCabinetPlayback.Request()
        req.command = SetCabinetPlayback.Request.COMMAND_RELEASE
        req.control_id = CONTROL_ID
        req.operation_lease_id = self.lease
        try:
            return self._await(self.client.call_async(req), "RELEASE")
        except RuntimeError:
            return None

    # ------------------------------------------------------------------ core
    def _move_and_release(self, target, what):
        """立即播放开到 target，到位后 RELEASE；返回 (resp, release_pos)。"""
        start_q = self.position
        resp = self._play(start_q, target, self.move_secs)
        if not resp.success:
            print("  %s 开到 %.3f START 失败: %s" % (what, target, resp.message))
            self._release()
            return None, None
        # 按位置等待到位（不依赖绝对 sim 阈值）
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.move_secs + 10.0:
            if self.position is not None and abs(self.position - target) < 0.006:
                break
            time.sleep(0.05)
        rel = self._release()
        time.sleep(0.3)
        if rel is not None and not rel.success:
            print("  %s RELEASE 失败: %s" % (what, rel.message))
            return None, None
        return rel, self.position

    def _hold_check(self, pos0, what):
        """观察 hold 秒：记录最大 |Δrail|、单调漂移、终态 velocity；返回 bool。"""
        rows = []
        max_delta = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.hold:
            q = self.position
            v = self.velocity
            if q is not None:
                max_delta = max(max_delta, abs(q - pos0))
                rows.append((time.monotonic() - t0, q, v))
            time.sleep(0.1)
        # 收尾读一次
        qf = self.position
        if qf is not None:
            max_delta = max(max_delta, abs(qf - pos0))
            rows.append((self.hold, qf, self.velocity))
        step = max(1, len(rows) // 14)
        trace = [(round(t, 1), round(q, 5)) for t, q, _ in rows[::step]]
        final_v = rows[-1][2] if rows else None
        ok = max_delta <= self.drift_tol
        print("  hold %.0fs: pos0=%.5f  max|Δ|=%.5f (tol %.4f)  final_v=%s  %s" %
              (self.hold, pos0, max_delta, self.drift_tol,
               "%.5f" % final_v if final_v is not None else "n/a",
               "PASS" if ok else "FAIL"))
        print("  trace(t,rail): %s" % trace)
        return ok

    def run_one(self, target, what):
        pos_before = self.position
        rel, pos0 = self._move_and_release(target, what)
        if pos0 is None:
            return False
        print("%-8s: 播放开到 %.3f 并 RELEASE，释放轨位=%.5f m state=%s" %
              (what.upper(), target, pos0, self.state_id))
        # 等 ~0.5s 排除播放停表尾巴，然后进入正式观察窗
        time.sleep(0.5)
        ok = self._hold_check(pos0, what)
        if abs(pos_before - target) < 0.006:
            print("  (起点已在目标，实际无行程——观察仍有效)")
        return ok

    def run(self, mode):
        self.wait_ready()
        print("drawer start rail=%.4f m state=%s (lease %s)" %
              (self.position, self.state_id, self.lease))
        results = {}
        order = []
        if mode == "all":
            order = ["relatch", "mid25", "mid15", "open"]
        else:
            order = [mode]
        try:
            # 每段之间把抽屉归到目标位即可，无需额外归零；relatch 先跑建立闭位。
            for m in order:
                target = {"open": BUSINESS_OPEN, "mid25": 0.25,
                          "mid15": 0.15, "relatch": BUSINESS_CLOSED}.get(m)
                if target is None:  # 任意位置
                    target = self._pos_arg
                results[m] = self.run_one(target, m)
        finally:
            # 收尾关回闭位并释放，保证离开时是干净的复闩闭位。
            if self.position is not None and abs(self.position - 0.0) > 0.01:
                self._play(self.position, 0.0, 2.0)
                time.sleep(2.5)
            self._release()
        print("RESULT   : %s" % {k: ("PASS" if v else "FAIL")
                                 for k, v in results.items()})
        return all(results.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", nargs="?", default="all",
                        choices=["all", "open", "mid25", "mid15", "relatch"])
    parser.add_argument("--pos", type=float, default=None,
                        help="任意测试轨位（配合 mode=all 之外任意占位）")
    parser.add_argument("--hold", type=float, default=12.0,
                        help="观察保持秒数（默认 12）")
    parser.add_argument("--drift-tol", type=float, default=0.008,
                        help="保持期最大允许 |Δrail| (m)，默认 0.008")
    parser.add_argument("--move-secs", type=float, default=4.0,
                        help="开到目标位的调度时长 (s)")
    args = parser.parse_args()

    rclpy.init()
    probe = Db1HoldProbe(args.hold, args.drift_tol, args.move_secs)
    probe._pos_arg = args.pos
    try:
        ok = probe.run(args.mode)
    finally:
        probe.stop()
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
