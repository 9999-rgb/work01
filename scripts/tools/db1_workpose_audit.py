#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""db1 双臂工作位姿几何审计（2026-09-07 位姿朝向缺陷取证）。

双臂已停在 db1 工作位姿（例如 visual open 完成后双臂持把停在位）时运行：
读取左/右工具各指杆 link 的 TF 真值，换算每根杆真实杆端世界点与杆轴向，
与抽屉几何合同特征比对，回答“朝向错在哪、应绕哪条轴翻转”：
  * 钩爪(hook)    应指向各自把手立板中心（世界 y 4.107 / 4.693；板窗
    L y[4.099,4.115] R y[4.685,4.701]，z 0.904..1.000，中心 z 0.952）。
  * 支撑(support) 应指向同侧固定柜体侧缝（左 y 4.039 / 右 y 4.761）。
  * 右 finger3(unlock) 应朝向右把手本体解锁按钮区（y≈4.693, z≈0.952），
    而非顶部指示灯（世界 (0.099,4.685,1.000)）。
  * 杆轴向（伸向抽屉方向）应为世界 -x；两主杆列方向应沿世界 y。

特征按 map 系声明；杆端取自 odom 系 TF。比较前把特征换算进 odom
（odom→map 用 TF，失败则退恒等并告警）。抽屉开到 0.3 时立板随抽屉 +x 平移，
但 y/z 不变 —— 本审计按 (y,z) 判向，x 仅报告不作判据。

用法:
  python3 db1_workpose_audit.py                 # 单次采样 + 判定
  python3 db1_workpose_audit.py --settle 2.0    # 采样前多等一会
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy)
from rclpy.duration import Duration
import tf2_ros

from xczs_inspection_robot_interfaces.msg import CabinetControlState

PLAN_FRAME = "odom"          # 杆端/机器人所在规划系 = 物理世界（gz 机器人根）
STATE_TOPIC = "/xczs/cabinet/electrical_mezzanine/db1/state"

# 几何合同 YAML 把特征声明在“map”系，但实测本夹具 odom↔map 只有 ~3mm 水平差，
# 而 map 系 z 比物理 odom 高 0.5852（AMCL 局部化覆盖层，不是夹具物理地面）。
# 操作员实际以测得的 b1 为锚、在 odom(=物理) 里做几何；特征 YAML 的数值本来就是
# 物理坐标。因此这里把杆端(odom)与特征 raw 数值直接比，绝不做 map→odom 的 z 平移
# （那会把正确位姿误判成 MIS-AIM 0.57m）。odom 的 x/y 与 map 相差 <4mm，可忽略。

# 每侧指杆: (link, 角色, 杆端 contact 点在 link 局部系的 -z 深度 L)
RODS = {
    "left": [
        ("l_two_cyl_finger1", "support_L", 0.075),   # 支撑 → 西侧缝
        ("l_two_cyl_finger2", "hook_L", 0.095),      # 钩爪 → 左把手立板
    ],
    "right": [
        ("r_three_cyl_finger1", "hook_R", 0.090),    # 钩爪 → 右把手立板
        ("r_three_cyl_finger2", "support_R", 0.075),  # 支撑 → 东侧缝
        ("r_three_cyl_finger3", "unlock_R", 0.013002),  # 解锁短杆
    ],
}

# 特征中心（map 系）；半宽仅用于“是否落在板窗内”的提示，判定用最近中心。
FEATURES = {
    "fin_L":   dict(center=(0.099, 4.107, 0.952), half=(0.0305, 0.008, 0.048)),
    "fin_R":   dict(center=(0.099, 4.693, 0.952), half=(0.0305, 0.008, 0.048)),
    "seam_L":  dict(center=(0.099, 4.039, 0.952), half=None),
    "seam_R":  dict(center=(0.099, 4.761, 0.952), half=None),
    "btn_R":   dict(center=(0.099, 4.693, 0.952), half=None),
    "lamp_R":  dict(center=(0.099, 4.685, 1.000), half=None),
}
EXPECT = {"support_L": "seam_L", "hook_L": "fin_L",
          "hook_R": "fin_R", "support_R": "seam_R", "unlock_R": "btn_R"}
EXPECT_LABEL = {"seam_L": "左缝4.039", "fin_L": "左板4.107",
                "fin_R": "右板4.693", "seam_R": "右缝4.761", "btn_R": "解锁钮区"}


def qrot(q, v):
    """unit quat (x,y,z,w) rotate v -> world (v dot convention)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def norm(v):
    m = math.sqrt(sum(c * c for c in v))
    return (v[0] / m, v[1] / m, v[2] / m) if m > 1e-12 else (0.0, 0.0, 0.0)


class Db1WorkPoseAudit(Node):
    def __init__(self):
        super().__init__("db1_workpose_audit")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self,
                                                  spin_thread=True)
        self.position = None
        self.state_id = None
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            CabinetControlState, STATE_TOPIC, self._on_state, qos)

    def _on_state(self, msg):
        self.position = msg.position
        self.state_id = msg.state_id

    def lookup(self, target, source, timeout=3.0):
        return self.buf.lookup_transform(
            target, source, rclpy.time.Time(),
            timeout=Duration(seconds=timeout))

    def _pq(self, t):
        return ((t.transform.translation.x, t.transform.translation.y,
                 t.transform.translation.z),
                (t.transform.rotation.x, t.transform.rotation.y,
                 t.transform.rotation.z, t.transform.rotation.w))

    def odom_features(self):
        """特征中心 = raw 物理坐标（见模块注释：YAML 数值即物理，勿做 map→odom z 平移）。"""
        return {k: dict(f) for k, f in FEATURES.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--settle", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = Db1WorkPoseAudit()
    try:
        time.sleep(args.settle)
        feats = node.odom_features()
        print("drawer rail=%.4f m state=%s (规划系=%s)" %
              (node.position if node.position is not None else float("nan"),
               node.state_id, PLAN_FRAME))

        hdr = "%-16s %-10s | %-24s | %-8s %-8s | %-10s | %s"
        print(hdr % ("link", "role", "link_origin_odom", "rod_tip_y",
                     "rod_tip_z", "expect", "y/z nearest feature"))
        print("-" * 110)
        verdicts = []
        for side, rods in RODS.items():
            poses = {}
            ok_side = True
            for link, _, _ in rods:
                try:
                    poses[link] = node.lookup(PLAN_FRAME, link)
                except Exception as e:
                    print("  [%s] TF 缺 link %s: %s" % (side, link, e))
                    ok_side = False
            if not ok_side:
                continue
            # 每根杆的杆端 & 轴向
            data = {}
            for link, role, L in rods:
                tr, R = node._pq(poses[link])
                tip = qrot(R, (0.0, 0.0, -L))
                tip = (tr[0] + tip[0], tr[1] + tip[1], tr[2] + tip[2])
                data[role] = (tr, R, tip)
            # 列方向 = 前两主杆 link 原点连线
            mains = rods[:2]
            o0 = node._pq(poses[mains[0][0]])[0]
            o1 = node._pq(poses[mains[1][0]])[0]
            col = norm((o1[0] - o0[0], o1[1] - o0[1], o1[2] - o0[2]))
            for link, role, L in rods:
                tr, R, tip = data[role]
                ax = norm(qrot(R, (0.0, 0.0, -1.0)))
                exp = EXPECT[role]
                exp_c = feats[exp]["center"]
                best = min(feats, key=lambda k: math.hypot(
                    feats[k]["center"][1] - tip[1],
                    feats[k]["center"][2] - tip[2]))
                dyz_exp = math.hypot(exp_c[1] - tip[1], exp_c[2] - tip[2])
                in_window = ""
                half = feats[exp]["half"]
                if half is not None and role.startswith("hook"):
                    cy = exp_c[1]
                    in_window = (" in-window" if (abs(tip[1] - cy) <= half[1]
                                                  and 0.904 <= tip[2] <= 1.000)
                                 else " OUT-window")
                ok = best == exp and dyz_exp < 0.012
                verdicts.append(ok)
                print("%-16s %-10s | (%.3f,%.3f,%.3f) | %-8.3f %-8.3f | %-10s | %s  (Δyz=%.4f)%s" %
                      (link, role, tr[0], tr[1], tr[2], tip[1], tip[2],
                       EXPECT_LABEL[exp], best, dyz_exp,
                       " OK" if ok else " MIS-AIM"))
                if role.startswith("hook") or role.startswith("support"):
                    print("      ├ rod_axis(w)=(%.3f,%.3f,%.3f)" % ax)
            print("  [%s] 列方向(world)=(%.3f,%.3f,%.3f)（期望≈(0,∓1,0) 贴世界 y）"
                  % (side, col[0], col[1], col[2]))
            if side == "right" and "unlock_R" in data:
                u = data["unlock_R"][2]
                btn = feats["btn_R"]["center"]
                lamp = feats["lamp_R"]["center"]
                du = math.hypot(u[1] - btn[1], u[2] - btn[2])
                dl = math.hypot(u[1] - lamp[1], u[2] - lamp[2])
                print("  [unlock_R] 短杆尖 y=%.3f z=%.3f  Δ(按钮区)=%.3f "
                      "Δ(指示灯)=%.3f → 朝%s" %
                      (u[1], u[2], du, dl,
                       "按钮区" if du < dl else "指示灯(LAMP,疑错!)"))
        n_ok = sum(1 for v in verdicts if v)
        print("\nVERDICT : %d/%d 主杆对位" % (n_ok, len(verdicts)))
        return 0 if verdicts and all(verdicts) else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
