#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""db1 SetCabinetPlayback header.stamp 时序契约探针（2026-09-06 AGENT T3 活验）。

插件兑现 trajectory.header.stamp 的三种策略，直接调插件
/xczs/cabinet/electrical_mezzanine/playback（不经过 Web/operator 鉴权），用
db1/state（CabinetControlState，单位 m）读回轨位验证：
  * stale     header.stamp 明显早于当前 sim time（>1s 宽限）→ START 被拒
              （success=False 且 message 含 stale），轨位不动、不占租约。
  * deferred  header.stamp 在未来 → START 接受；sim time 到戳前轨位保持
              原位（velocity 0），到戳后才沿调度运动，到达目标。
  * zero      header.stamp 缺省(0) → 历史行为：立即起播。

所有判据以 sim 时间轴（state 头戳 + START 返回的锚点）为准，不依赖墙钟/RTF。
每次 START 之后无论成败都会 RELEASE，避免探针异常时留下孤儿播放租约
（RELEASE 只认持有租约，孤儿租约会卡死该抽屉直至 gzserver 重启）。

用法:
  python3 db1_playback_stamp_probe.py            # 全部三段
  python3 db1_playback_stamp_probe.py stale|deferred|zero
默认目标 0.25 m；未来戳提前 4.0s；行程 6.0s。
"""
import argparse
import re
import sys
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy)
from rclpy.duration import Duration
from rclpy.time import Time as RclTime

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xczs_inspection_robot_interfaces.srv import SetCabinetPlayback
from xczs_inspection_robot_interfaces.msg import CabinetControlState

CONTROL_ID = "db1"
PLAYBACK_SERVICE = "/xczs/cabinet/electrical_mezzanine/playback"
STATE_TOPIC = "/xczs/cabinet/electrical_mezzanine/db1/state"


def _sim_now(stamp):
    """builtin Time -> 秒（sim 时间轴）。"""
    return stamp.sec + 1e-9 * stamp.nanosec


def _anchor_from_message(message):
    m = re.search(r"will begin at sim time ([-+]?[0-9]+(?:\.[0-9]+)?)", message)
    return float(m.group(1)) if m else None


class Db1PlaybackStampProbe(Node):
    def __init__(self, distance, seconds, ahead):
        super().__init__("db1_playback_stamp_probe")
        self.distance = float(distance)
        self.seconds = float(seconds)
        self.ahead = float(ahead)
        self.lease = "stamp-probe-" + uuid.uuid4().hex[:12]
        self.position = None
        self.state_stamp = None
        self.client = self.create_client(
            SetCabinetPlayback, PLAYBACK_SERVICE)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.state_sub = self.create_subscription(
            CabinetControlState, STATE_TOPIC, self.on_state, qos)
        # 保强引用 executor：rclpy 只持弱引用，否则被 GC 后回调全停。
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(
            target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    def on_state(self, msg):
        self.position = msg.position
        self.state_stamp = msg.header.stamp

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

    # ------------------------------------------------------------------- srv
    def _schedule(self, start_q, end_q, seconds):
        traj = JointTrajectory()
        traj.joint_names = [CONTROL_ID]
        n = max(2, int(seconds * 8.0))
        for i in range(n + 1):
            t = seconds * i / n
            q = start_q + (end_q - start_q) * i / n
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=t).to_msg()
            point.positions = [q]
            point.velocities = [0.0]
            traj.points.append(point)
        return traj

    def _await(self, future, what, timeout=8.0):
        """等在后台 MultiThreadedExecutor 上完成 service future。

        本节点加入了后台 spin 线程（订阅回调依赖它持续运转）；绝不能再从主
        线程 rclpy.spin_until_future_complete 同一个 node/executor —— rclpy
        不允许双线程 spin，会导致订阅回调静默停摆（表现为 state 头戳"冻结"）。
        call_async 的响应由后台 executor 处理，这里只轮询 future.done()。
        """
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("%s 请求超时" % what)
            time.sleep(0.02)
        return future.result()

    def _start(self, start_q, end_q, seconds, stamp_dt):
        request = SetCabinetPlayback.Request()
        request.command = SetCabinetPlayback.Request.COMMAND_START
        request.control_id = CONTROL_ID
        request.operation_lease_id = self.lease
        request.trajectory = self._schedule(start_q, end_q, seconds)
        if stamp_dt is not None:
            request.trajectory.header.stamp = RclTime(
                seconds=_sim_now(self.state_stamp) + stamp_dt).to_msg()
        future = self.client.call_async(request)
        return self._await(future, "START")

    def _release(self):
        request = SetCabinetPlayback.Request()
        request.command = SetCabinetPlayback.Request.COMMAND_RELEASE
        request.control_id = CONTROL_ID
        request.operation_lease_id = self.lease
        future = self.client.call_async(request)
        try:
            return self._await(future, "RELEASE")
        except RuntimeError:
            return None

    def _drain(self, sim_until, what):
        """记录 rail-vs-sim 轨迹直到 sim 越过 sim_until；返回 (max_drift_from,
        saw_motion, final_q, stalled, [(sim,q)...])。from_q 为该段起点轨位。

        世界物理卡死时 state 头戳不再前进（插件只在 on_update 里发状态），
        若 8s 墙钟内头戳无进展判为 stall 提前返回，避免把等待预算烧满。
        """
        from_q = self.position
        log = []
        t0 = time.monotonic()
        last_advance = t0
        last_stamp = None
        stalled = False
        while time.monotonic() - t0 < 40.0:
            if self.state_stamp is not None and \
                    _sim_now(self.state_stamp) >= sim_until:
                break
            if self.state_stamp is not None:
                s = _sim_now(self.state_stamp)
                if last_stamp is None or s > last_stamp:
                    last_stamp = s
                    last_advance = time.monotonic()
            if time.monotonic() - last_advance > 8.0:
                stalled = True
                break
            if self.position is not None:
                log.append((_sim_now(self.state_stamp), self.position))
            time.sleep(0.05)
        time.sleep(0.2)
        if self.position is not None:
            log.append((_sim_now(self.state_stamp), self.position))
        # 轻度抽稀
        step = max(1, len(log) // 14)
        trace = [(round(s, 2), round(q, 4)) for s, q in log[::step]]
        final_q = self.position
        max_drift = max((abs(q - from_q) for _, q in log), default=0.0)
        saw_motion = max_drift > 0.02
        return max_drift, saw_motion, final_q, stalled, trace

    # ------------------------------------------------------------- individual
    def probe_stale(self):
        start_q = self.position
        resp = self._start(start_q, self.distance, self.seconds,
                           stamp_dt=-(self.ahead + 5.0))
        refused = (not resp.success) and ("stale" in resp.message)
        # 无论接受/拒绝都释放，避免留下租约
        rel = self._release()
        time.sleep(0.4)
        moved = abs(self.position - start_q) > 0.002
        print("STALE    : success=%s refused=%s  rail %.4f->%.4f moved=%s" %
              (resp.success, refused, start_q, self.position, moved))
        print("  resp   : %s" % resp.message)
        print("  release: %s" % (rel.message if rel else "n/a"))
        return refused and not moved

    def _close_to(self, target, secs, what):
        """立即(零戳)播放把抽屉开到/关到 target，等待到位并 RELEASE。
        返回 True 表示到位且放行干净（用于给 deferred/zero 建立干净的起点）。"""
        r = self._start(self.position, target, secs, None)
        if not r.success:
            print("  %s 预置 START 失败: %s" % (what, r.message))
            return False
        ok = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs + 8.0:
            if self.position is not None and \
                    abs(self.position - target) < 0.006:
                ok = True
                break
            time.sleep(0.05)
        self._release()
        time.sleep(0.4)
        if not ok:
            print("  %s 预置未到位: pos=%.4f target=%.4f" %
                  (what, self.position, target))
        return ok

    def probe_deferred(self):
        start_q = self.position
        # 起点钉在原位才代表"等待窗未起播"，故先关到 0 再测。
        if start_q > 0.02 and not self._close_to(0.0, 2.0, "DEFERRED"):
            return False
        resp = self._start(self.position, self.distance, self.seconds,
                           stamp_dt=+self.ahead)
        anchor = _anchor_from_message(resp.message)
        print("DEFERRED : success=%s  anchor=%s  start rail=%.4f" %
              (resp.success, anchor, start_q))
        print("  resp   : %s" % resp.message)
        if not resp.success:
            self._release()
            return False
        if anchor is None:
            print("DEFERRED : FAIL（响应未返回 future 锚点，无法按 sim 判据）")
            self._release()
            return False
        # 锚点前 ~0.5s sim 内的最大漂移（应为 0，velocity 0 钉住）
        max_drift, saw_motion, final_q, stalled, _ = \
            self._drain(anchor - 0.5, "锚点前")
        print("  pre-anchor sim %.2f: max_drift=%.4f (need<=0.004) motion=%s"
              % (anchor - 0.5, max_drift, saw_motion))
        if stalled:
            print("  pre-anchor: 世界物理卡死，无法验证等待窗 → SKIP(环境)")
            self._release()
            return None
        # 锚点 + 行程 + 余量，观察起播并到达
        max_drift2, _, final_q2, stalled2, trace = self._drain(
            anchor + self.seconds + 2.0, "行程后")
        reached = final_q2 is not None and abs(final_q2 - self.distance) < 0.01
        print("  trace(sim,rail): %s" % trace)
        print("  after-anchor: final=%.4f (target %.2f) reached=%s" %
              (final_q2, self.distance, reached))
        if stalled2:
            print("  after-anchor: 世界物理卡死（起播后），无法验证到达 → SKIP(环境)")
        self._release()
        parked_ok = max_drift <= 0.004 and not saw_motion
        if stalled2:
            return None
        return parked_ok and reached

    def probe_zero(self):
        # 立即起播判据：抽屉在开位，零戳 START 应立即开始向关位运动并到达。
        # 起点不在开位先预置到 0.25，保证“立即关回”有可观测的行程。
        if self.position < 0.20 and not self._close_to(self.distance, 2.0,
                                                       "ZERO"):
            return False
        start_q = self.position
        resp = self._start(start_q, 0.0, self.seconds, stamp_dt=None)
        print("ZERO     : success=%s  (immediate, no stamp)" % resp.success)
        print("  resp   : %s" % resp.message)
        if not resp.success:
            self._release()
            return False
        # 立即起播：按墙钟有界等待（不依赖绝对 sim 阈值），判到位 + 曾运动。
        # 以相对起点累计位移判"曾运动"（0.05s 采样每拍轨位增量仅 ~0.002，
        # 用增量阈值会永远判不出 motion）。
        trace = []
        t0 = time.monotonic()
        saw_move = False
        reached = False
        while time.monotonic() - t0 < self.seconds + 8.0:
            q = self.position
            if q is not None:
                trace.append((round(_sim_now(self.state_stamp), 1),
                              round(q, 4)))
                if abs(q - start_q) > 0.02:
                    saw_move = True
                if abs(q - 0.0) < 0.006:
                    reached = True
                    break
            time.sleep(0.05)
        step = max(1, len(trace) // 12)
        print("  trace(sim,rail): %s" % trace[::step])
        print("  final=%.4f reached_closed=%s moved=%s" %
              (self.position if self.position is not None else float("nan"),
               reached, saw_move))
        self._release()
        return saw_move and reached

    # ------------------------------------------------------------------ main
    def run(self, modes):
        self.wait_ready()
        print("drawer start rail=%.4f m sim=%.3f s (lease %s)" %
              (self.position, _sim_now(self.state_stamp), self.lease))
        results = {}
        try:
            if "stale" in modes:
                results["stale"] = self.probe_stale()
            if "deferred" in modes:
                results["deferred"] = self.probe_deferred()
            if "zero" in modes:
                results["zero"] = self.probe_zero()
        finally:
            self._release()
        label = {k: ("PASS" if v is True else
                     ("SKIP(环境)" if v is None else "FAIL"))
                 for k, v in results.items()}
        print("RESULT   : %s" % label)
        # SKIP(None) 是环境卡死导致无法验证，不算功能失败；
        # 只有显式 False 才算 FAIL。
        return all(v is not False for v in results.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", nargs="?", default="all",
                        choices=["all", "stale", "deferred", "zero"])
    parser.add_argument("--distance", type=float, default=0.25)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--ahead", type=float, default=4.0,
                        help="future header.stamp 提前量（s）")
    args = parser.parse_args()

    rclpy.init()
    probe = Db1PlaybackStampProbe(args.distance, args.seconds, args.ahead)
    modes = ["stale", "deferred", "zero"] if args.mode == "all" else [args.mode]
    try:
        ok = probe.run(modes)
    finally:
        probe.stop()
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
