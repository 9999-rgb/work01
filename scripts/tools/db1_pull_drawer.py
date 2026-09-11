#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""联动抽拉抽屉：双手后拉 + 抽屉同步抽出（视觉合唱，不是真实拖拽）。

现场要求（2026-09-11）：**不需要机器人真的把抽屉拽出来**——只要"看起来是机器人
拉的"。做法是两条运动在时间上对齐：

  ① 抽屉轨道：用插件的 `SetCabinetPlayback`(START) 下发一条一维位移时序，
     让抽屉被**运动学驱动**（插件在此期间旁路闩锁/弹簧/耦合物理）；
  ② 双臂：沿世界 +x（远离柜体）平移同样距离，时间轴缩放到同一时长。

两边同距同时长，视觉上就是"手抓着把手把抽屉拉出来"。

为什么不用真的拖：物理侧钩爪与把手是"跨在板两侧"的视觉合唱姿态（见
taught_poses/db1_teach_node001_raise30.yaml），不构成真实咬合，硬拖会把
姿态拽散（本会话已实测过多次）。

用法：
  python3 scripts/tools/db1_pull_drawer.py                      # 默认拉出 0.30 m / 6 s
  python3 scripts/tools/db1_pull_drawer.py --distance 0.20 --duration 4
  python3 scripts/tools/db1_pull_drawer.py --dry-run            # 只算路径与时限，不下发

先决条件：活栈就绪、机器人已在 001 启动位姿、抽屉处于 closed。
"""
import argparse
import sys
import threading
import time
from pathlib import Path

import rclpy
from moveit_msgs.srv import GetCartesianPath
from rclpy.parameter import Parameter
from xczs_inspection_robot_interfaces.msg import CabinetControlState
from xczs_inspection_robot_interfaces.srv import (
    ManageOperationLease, SetCabinetPlayback)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nudge_tool_translate import ToolTranslateNudge  # noqa: E402
from xczs_controllers import ARMS, wait_for  # noqa: E402

CABINET = "electrical_mezzanine"
LEASE_SERVICE = "/xczs/operation_lease"
OWNER_ID = "db1_pull_drawer"
RAIL_LIMIT = 0.38  # 抽屉轨道行程上限（URDF prismatic joint upper）
# 租约协调器把单次租约时长限死在 maximum_lease_duration（实测 5.0 s），
# 所以长动作不能靠"要一个长租约"，必须**周期性续租**（与 operator 同做法）。
LEASE_SECONDS = 5.0
LEASE_RENEW_PERIOD = 1.5


class DrawerPuller(ToolTranslateNudge):
    def __init__(self, control_id):
        super().__init__()
        self._control = control_id
        self._rail = None
        self.create_subscription(
            CabinetControlState,
            "/xczs/cabinet/%s/%s/state" % (CABINET, control_id),
            self._on_rail, 10)
        self._lease_cli = self.create_client(ManageOperationLease, LEASE_SERVICE)
        self._playback_cli = self.create_client(
            SetCabinetPlayback,
            "/xczs/cabinet/%s/playback" % CABINET)

    def _on_rail(self, msg):
        self._rail = msg.position

    # ------------------------------------------------------------------ lease
    def acquire_lease(self, seconds):
        request = ManageOperationLease.Request()
        request.command = ManageOperationLease.Request.ACQUIRE
        request.owner_id = OWNER_ID
        request.lease_id = ""
        request.requested_duration = float(seconds)
        response = self._call(self._lease_cli, request, 15.0, LEASE_SERVICE)
        if not response.success:
            raise RuntimeError("获取操作租约失败: %s" % response.message)
        print("已获取操作租约 %s（剩余 %.1f s）" % (response.lease_id,
                                                response.remaining_duration))
        return response.lease_id

    def start_renew(self, lease_id):
        """后台线程周期性续租；租约到期播放会被回收，所以必须续。"""
        self._renew_stop = threading.Event()

        def loop():
            while not self._renew_stop.wait(LEASE_RENEW_PERIOD):
                request = ManageOperationLease.Request()
                request.command = ManageOperationLease.Request.RENEW
                request.owner_id = OWNER_ID
                request.lease_id = lease_id
                request.requested_duration = LEASE_SECONDS
                try:
                    response = self._call(self._lease_cli, request, 5.0,
                                          LEASE_SERVICE)
                    if not response.success:
                        print("续租被拒: %s" % response.message)
                except Exception:  # noqa: BLE001
                    pass  # 单次续租失败不致命，下一轮再试

        self._renew_thread = threading.Thread(target=loop, daemon=True)
        self._renew_thread.start()

    def stop_renew(self):
        stop = getattr(self, "_renew_stop", None)
        if stop is not None:
            stop.set()
        thread = getattr(self, "_renew_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)

    def release_lease(self, lease_id):
        if not lease_id:
            return
        request = ManageOperationLease.Request()
        request.command = ManageOperationLease.Request.RELEASE
        request.owner_id = OWNER_ID
        request.lease_id = lease_id
        request.requested_duration = 0.0
        try:
            self._call(self._lease_cli, request, 10.0, LEASE_SERVICE)
            print("已释放操作租约")
        except Exception as exc:  # noqa: BLE001
            print("释放租约时出错（不影响动作结果）: %s" % exc)

    # --------------------------------------------------------------- playback
    def start_playback(self, lease_id, start, distance, duration):
        """下发抽屉轨道时序：从 start 到 start+distance，历时 duration 秒。"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [self._control]
        # header.stamp 契约（插件侧）：0 = 立即开始；非零 = 钉住等到那一刻，
        # 且**用节点墙钟判定**，过期 >1s 直接拒。本工具要的是"立刻开始"，
        # 所以留 0；若填仿真时间会被判成"过去 17 亿秒"而拒收。
        trajectory.header.stamp.sec = 0
        trajectory.header.stamp.nanosec = 0
        for elapsed, value in ((0.0, start), (duration, start + distance)):
            point = JointTrajectoryPoint()
            point.positions = [value]
            point.time_from_start = rclpy.duration.Duration(
                seconds=elapsed).to_msg()
            trajectory.points.append(point)
        request = SetCabinetPlayback.Request()
        request.command = SetCabinetPlayback.Request.COMMAND_START
        request.control_id = self._control
        request.operation_lease_id = lease_id
        request.trajectory = trajectory
        response = self._call(
            self._playback_cli, request, 15.0, "playback")
        if not response.success:
            raise RuntimeError("抽屉播放被拒: %s" % response.message)
        print("抽屉播放已开始：轨道 %.4f → %.4f m，历时 %.1f s"
              % (start, start + distance, duration))
        return response

    def release_playback(self, lease_id):
        request = SetCabinetPlayback.Request()
        request.command = SetCabinetPlayback.Request.COMMAND_RELEASE
        request.control_id = self._control
        request.operation_lease_id = lease_id
        try:
            self._call(self._playback_cli, request, 10.0, "playback")
            print("已结束抽屉播放（交回插件默认控制）")
        except Exception as exc:  # noqa: BLE001
            print("结束播放时出错（不影响动作结果）: %s" % exc)

    # ------------------------------------------------------------------- main
    def run(self, distance, duration, dry_run, max_step, timeout):
        self._ready(timeout)
        wait_for(lambda: self._rail is not None, 15.0, "抽屉状态")
        # 插件校验 `q < lower || q > upper` 直接拒收，而关到位时轨道位置实测是
        # 负的微小值（−2.1e-05），下限 0 —— 起点必须先钳进限位，否则播放被拒。
        start = max(0.0, min(RAIL_LIMIT, self._rail))
        if abs(start - self._rail) > 1e-9:
            print("轨道起点 %.6f 越限，钳到 %.6f" % (self._rail, start))
        if start + distance > RAIL_LIMIT:
            raise RuntimeError("目标 %.4f m 超过轨道上限 %.2f m"
                               % (start + distance, RAIL_LIMIT))

        plans = {}
        print("\n==== 联动抽拉 ====")
        print("抽屉起点 %.4f m，目标 %.4f m，历时 %.1f s"
              % (start, start + distance, duration))
        for side in ARMS:
            solution, fraction, tip, goal = self.plan_translate(
                side, "x", distance, max_step, timeout, True)
            if fraction < 0.99:
                raise RuntimeError("%s 笛卡尔路径完整度仅 %.4f，整批放弃"
                                   % (side, fraction))
            if len(solution.points) == 0:
                raise RuntimeError("%s 轨迹为空" % side)
            plans[side] = (solution, tip)
            print("%-5s 末端 %+.4f → %+.4f m（Δx %+.4f）完整度 %.4f"
                  % (side, tip[0], goal[0], distance, fraction))

        if dry_run:
            print("--dry-run：未下发。")
            return

        # 把手臂轨迹缩放到与抽屉同一时长，两边看起来才是"一起走"。
        plan_time = max(
            (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9)
            for solution, _tip in plans.values() for p in solution.points)
        slowdown = max(1.0, duration / plan_time) if plan_time > 0 else 1.0
        print("臂轨迹原时长 %.2f s → 缩放到 %.2f s（×%.2f）"
              % (plan_time, plan_time * slowdown, slowdown))

        lease_id = self.acquire_lease(LEASE_SECONDS)
        self.start_renew(lease_id)
        try:
            self.start_playback(lease_id, start, distance, duration)
            for side, (solution, _tip) in plans.items():
                print("执行 %-5s ..." % side)
                result = self.execute(side, solution, slowdown,
                                      max(timeout, duration * 4))
                print("%-5s 控制器 error_code=%s" % (side, result.error_code))
            for side in ARMS:
                if not self._tip_settled(ARMS[side]["tip"], timeout=15.0):
                    print("警告: %s 末端 15 s 内未静止" % side)
            time.sleep(1.0)
            print("\n---- 结果 ----")
            print("抽屉轨道 %.4f → %.4f m（Δ %+.4f m）"
                  % (start, self._rail, self._rail - start))
            for side in ARMS:
                (nx, ny, nz), _ = self._tip_pose(ARMS[side]["tip"])
                print("%-5s 末端 x %+.4f（Δ %+.4f m）" % (side, nx, nx - plans[side][1][0]))
            print("================")
        finally:
            self.release_playback(lease_id)
            self.stop_renew()
            self.release_lease(lease_id)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", default="db1", help="抽屉控制 id，默认 db1")
    parser.add_argument("--distance", type=float, default=0.30,
                        help="抽出距离（m），默认 0.30")
    parser.add_argument("--duration", type=float, default=6.0,
                        help="动作时长（s），默认 6.0")
    parser.add_argument("--dry-run", action="store_true", help="只算不下发")
    parser.add_argument("--max-step", type=float, default=0.005)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = DrawerPuller(args.control)
    try:
        node.run(args.distance, args.duration, args.dry_run,
                 args.max_step, args.timeout)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
