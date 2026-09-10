#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双源杆端采样探针：物理真值 vs FK，外加电缸关节实测值（开发排查工具）。

为什么需要它
------------
operator 的「动作起始到位门」（verify_drawer_action_start_gate）用
`/get_entity_state` 读连杆世界位姿 + 同一个 local 接触偏移，得到杆端**物理真值**，
与「本次指令下发的那个工作位姿」比对。2026-09-10 活验里左侧残差 4.6 mm（放行
线内）、右侧 30.9 mm 且**几乎全在 x（外向轴）**上 —— y/z 残差与左侧同量级。
轴向单边偏大只有两种解释，本探针就是用来分辨它们的：

  A. 该侧电缸当时并不在「收拢」位（关节实测值 ≠ 0）→ 杆端沿工具轴多伸了 ~30 mm；
  B. 电缸确实收拢，但连杆的物理位姿与运动学模型不同步（臂/工具基座物理漂移）。

判据：**电缸关节实测值**是杆伸缩的独立真值。关节 ≈0 而物理杆端仍偏 30 mm ⇒ B；
关节 ≈0.03 ⇒ A。同时打印 TF（FK）杆端作为第三条参照 —— 物理与 FK 同步移动
⇒ 真位移；只有物理动 ⇒ 物理/模型分叉。

采样项（每周期一行，制表符分隔，便于 awk/cut）
----------------------------------------------
  t          相对启动秒数
  <rod>_q    该杆电缸关节实测位置（/xczs/joint_states）
  <rod>_phys 物理真值杆端（/get_entity_state 连杆位姿 ⊗ local 接触偏移）
  <rod>_fk   FK 杆端（TF，含同一 local 偏移）—— 与 phys 之差即「模型 vs 物理」

杆角色（config-B 现役，与 adapter drawer_tools 同源）：
  L 钩 = l_two_cyl_finger1 (端深 -0.075)   R 钩 = r_three_cyl_finger2 (端深 -0.075)
  L 支 = l_two_cyl_finger2 (端深 -0.095)   R 支 = r_three_cyl_finger1 (端深 -0.090)
  R 解锁 = r_three_cyl_finger3 (端深 -0.013002)

用法:
  ROS_DOMAIN_ID=42 python3 scripts/tools/probe_rod_tip_truth.py \
      [--every 0.2] [--seconds 180] [--out /tmp/rod_truth.tsv]
"""

import argparse
import sys
import threading
import time

import rclpy
from gazebo_msgs.srv import GetEntityState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

# 杆名 → (电缸关节名, local 接触偏移 z)。偏移与 adapter
# drawer_tools.*.support/gripper_contact_point_local 一致（工具 STL z-extent 实测）。
RODS = {
    "L_hook": ("l_two_cyl_finger1_joint", "l_two_cyl_finger1", -0.075),
    "L_support": ("l_two_cyl_finger2_joint", "l_two_cyl_finger2", -0.095),
    "R_support": ("r_three_cyl_finger1_joint", "r_three_cyl_finger1", -0.090),
    "R_hook": ("r_three_cyl_finger2_joint", "r_three_cyl_finger2", -0.075),
    "R_unlock": ("r_three_cyl_finger3_joint", "r_three_cyl_finger3", -0.013002),
}
TOOL_FRAME_FALLBACK = {
    # TF 里杆 link 全部在 model 树内，直接用同名 frame 取（URDF 已含关节值）。
}


def quat_rot(q, v):
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


class RodTruthProbe(Node):
    def __init__(self) -> None:
        super().__init__("rod_tip_truth_probe")
        self._buf = tf2_ros.Buffer()
        self._lst = tf2_ros.TransformListener(self._buf, self)
        self._entity_cli = self.create_client(GetEntityState, "/get_entity_state")
        self._joints = {}
        self.create_subscription(JointState, "/xczs/joint_states", self._on_joints, 10)

    def _on_joints(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self._joints[name] = position

    def wait_for_service(self, timeout: float = 20.0) -> bool:
        return self._entity_cli.wait_for_service(timeout_sec=timeout)

    def physics_tip(self, link: str, local_z: float):
        """物理真值杆端：连杆世界位姿 ⊗ (0,0,local_z)。与 operator
        drawer_physics_link_point 同源同式（同一服务、同一 local 偏移）。"""
        request = GetEntityState.Request()
        request.name = link
        future = self._entity_cli.call_async(request)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        response = future.result() if future.done() else None
        if response is None or not response.success:
            return None
        pose = response.state.pose
        p = pose.position
        q = pose.orientation
        offset = quat_rot((q.x, q.y, q.z, q.w), (0.0, 0.0, local_z))
        return (p.x + offset[0], p.y + offset[1], p.z + offset[2])

    def fk_tip(self, link: str, local_z: float):
        """FK 杆端：TF 里同名 link frame ⊗ 同一 local 偏移（含关节值）。"""
        for parent in ("odom", "base_link", "body"):
            try:
                tf = self._buf.lookup_transform(parent, link, tf2_ros.Time())
            except Exception:
                continue
            tr = tf.transform.translation
            q = tf.transform.rotation
            offset = quat_rot((q.x, q.y, q.z, q.w), (0.0, 0.0, local_z))
            return (tr.x + offset[0], tr.y + offset[1], tr.z + offset[2])
        return None


def fmt(point):
    if point is None:
        return "n/a"
    return "%.5f,%.5f,%.5f" % point


def fmt_q(value):
    return "n/a" if value is None else "%.6f" % value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--every", type=float, default=0.2, help="采样周期（秒）")
    parser.add_argument("--seconds", type=float, default=180.0, help="总时长（秒）")
    parser.add_argument("--out", default="", help="同时写 TSV 到该路径")
    args = parser.parse_args()

    rclpy.init()
    node = RodTruthProbe()
    if not node.wait_for_service():
        print("FATAL: /get_entity_state 不可用（gazebo_ros_state 插件未起）")
        rclpy.shutdown()
        return 2

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    sink = open(args.out, "w", encoding="utf-8") if args.out else None
    header = ["t"]
    for label in RODS:
        header += [label + "_q", label + "_phys", label + "_fk"]
    line = "\t".join(header)
    print(line, flush=True)
    if sink:
        sink.write(line + "\n")

    started = time.time()
    try:
        while time.time() - started < args.seconds:
            row = ["%.3f" % (time.time() - started)]
            for label, (joint, link, local_z) in RODS.items():
                row.append(fmt_q(node._joints.get(joint)))
                row.append(fmt(node.physics_tip(link, local_z)))
                row.append(fmt(node.fk_tip(link, local_z)))
            line = "\t".join(row)
            print(line, flush=True)
            if sink:
                sink.write(line + "\n")
                sink.flush()
            time.sleep(args.every)
    except KeyboardInterrupt:
        pass
    finally:
        if sink:
            sink.close()
        # 先停 executor 并等 spin 线程真正退出，再拆节点：反过来会在回调仍
        # 在跑时销毁实体（实测 "terminate called without an active exception"
        # 直接 core dump，把一个正常采样收尾变成吓人的崩溃）。
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
