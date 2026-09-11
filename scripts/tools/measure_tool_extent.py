#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量测末端执行器（工具）在给定坐标系下的真实三维包围盒。

"末端"= 所有 visual 网格落在 `meshes/tools/` 下的连杆（工具基座 + 电缸杆），
不含机械臂本体。做法：从 /robot_description 取 URDF，解析这些连杆的 visual
网格与 origin，逐网格算 STL 包围盒，再用 TF 把包围盒八个角变换到目标坐标系，
取并集。这样得到的是"当前位姿下工具实际占多大"，而不是连杆原点的散布。

为什么不用连杆原点估算：连杆原点只在杆的轴线上，量不出工具的横向/纵向厚度；
而本工具是为了回答"末端的高度/宽度是多少"这类几何问题（现场教学里按比例的
微量动作都以它为分母）。

用法：
  python3 scripts/tools/measure_tool_extent.py                    # 世界系 odom
  python3 scripts/tools/measure_tool_extent.py --side left        # 只看左手工具
  python3 scripts/tools/measure_tool_extent.py --frame body       # 在底盘系里量
  python3 scripts/tools/measure_tool_extent.py --mesh-detail      # 逐网格列出

先决条件：活栈已就绪（robot_state_publisher 在发 /robot_description 与 TF）。
本工具只读，不下发任何指令。
"""
import argparse
import math
import re
import struct
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import rclpy.executors
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from rclpy.time import Time
from std_msgs.msg import String

ROBOT_DESCRIPTION_TOPIC = "/robot_description"
TOOL_MESH_MARKER = "meshes/tools/"

_AXIS_NAMES = ("X", "Y", "Z")
_VERTEX_RE = re.compile(
    rb"vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)")


def load_stl_vertices(path):
    """读 STL 全部顶点 (N,3) float64。二进制与 ASCII 都认。

    二进制判定必须用 `84 + 50*n == size`：早先写成 `(size-84)/50 == n` 时，
    三角形数刚好凑整的文件会被误判成 ASCII，随后按 UTF-8 解码炸掉。
    """
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) >= 84:
        count = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * count == len(data):
            record = np.frombuffer(data[84:], dtype=np.uint8).reshape(count, 50)
            # 每条记录：12 B 法向 + 36 B 三顶点 + 2 B 属性
            flat = record[:, 12:48].copy().view("<f4").reshape(-1, 3)
            return flat.astype(np.float64)
    matches = _VERTEX_RE.findall(data)
    if not matches:
        raise ValueError("%s 既不是合法二进制 STL，也没有 vertex 记录" % path)
    return np.array(matches, dtype=np.float64)


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def quat_to_matrix(quat):
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def parse_vec(text, default=(0.0, 0.0, 0.0)):
    if not text:
        return np.array(default, dtype=np.float64)
    parts = [float(v) for v in text.split()]
    if len(parts) != 3:
        raise ValueError("期望三个分量，得到 %r" % text)
    return np.array(parts, dtype=np.float64)


def resolve_mesh_path(filename):
    """package://<pkg>/<rel> → 安装目录下的绝对路径。"""
    if filename.startswith("package://"):
        rest = filename[len("package://"):]
        package, _, relative = rest.partition("/")
        base = get_package_share_directory(package)
        return "%s/%s" % (base, relative)
    return filename


def parse_tool_meshes(urdf_xml, side):
    """返回 [{'link','mesh','origin_xyz','origin_rpy','scale'}]，只含工具网格。"""
    root = ET.fromstring(urdf_xml)
    entries = []
    for link in root.findall("link"):
        link_name = link.get("name")
        if side != "both" and not link_name.startswith(side[0] + "_"):
            continue
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename") or ""
            if TOOL_MESH_MARKER not in filename:
                continue
            origin = visual.find("origin")
            entries.append({
                "link": link_name,
                "mesh": filename,
                "origin_xyz": parse_vec(origin.get("xyz") if origin is not None
                                        else None),
                "origin_rpy": parse_vec(origin.get("rpy") if origin is not None
                                        else None),
                "scale": parse_vec(mesh.get("scale"), (1.0, 1.0, 1.0)),
            })
    return entries


class ToolExtentMeasurer(Node):
    def __init__(self, frame):
        super().__init__("tool_extent_measurer", parameter_overrides=[
            # 与仿真同钟：TF 时间戳来自 gzserver 的 /clock。
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self._frame = frame
        self._urdf = None
        latch = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._urdf_sub = self.create_subscription(
            String, ROBOT_DESCRIPTION_TOPIC, self._on_urdf, latch)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        # rclpy Node.executor 只存弱引用：必须用不同名强属性保住 executor，
        # 否则立即被 GC，回调全停（与 db1_stage_cap_driver.py 同一坑）。
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(
            target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    def stop(self):
        if getattr(self, "_spin", None) and self._spin.is_alive():
            self._spin_executor.shutdown()
            self._spin.join(timeout=3.0)

    def _on_urdf(self, msg):
        self._urdf = msg.data

    def _wait(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        raise RuntimeError("timeout waiting for %s" % what)

    def _link_to_frame(self, link_name, timeout=5.0):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._frame, link_name, Time(), Duration(seconds=0.5))
                rotation = transform.transform.rotation
                translation = transform.transform.translation
                quat = np.array([rotation.x, rotation.y, rotation.z,
                                 rotation.w], dtype=np.float64)
                return np.array([translation.x, translation.y,
                                 translation.z], dtype=np.float64), quat
            except Exception as exc:  # noqa: BLE001
                last = exc
                time.sleep(0.1)
        raise RuntimeError("tf %s→%s 不可用: %s" % (self._frame, link_name, last))

    # ------------------------------------------------------------------- main
    def measure(self, entries, detail):
        self._wait(lambda: self._urdf is not None, 20.0, ROBOT_DESCRIPTION_TOPIC)
        corners = []
        rows = []
        for entry in entries:
            try:
                translation, quat = self._link_to_frame(entry["link"])
            except RuntimeError as exc:
                print("  跳过 %-28s %s" % (entry["link"], exc))
                continue
            vertices = load_stl_vertices(resolve_mesh_path(entry["mesh"]))
            vertices = vertices * entry["scale"]
            local_min = vertices.min(axis=0)
            local_max = vertices.max(axis=0)
            box = np.array([[x, y, z]
                            for x in (local_min[0], local_max[0])
                            for y in (local_min[1], local_max[1])
                            for z in (local_min[2], local_max[2])])
            box = box @ rpy_to_matrix(*entry["origin_rpy"]).T + entry["origin_xyz"]
            box = box @ quat_to_matrix(quat).T + translation
            corners.append(box)
            rows.append((entry["link"], entry["mesh"].rsplit("/", 1)[-1],
                         (local_max - local_min) * 1000.0, translation))
        if not corners:
            raise RuntimeError("没有任何工具网格变换成功，无法量测")
        return np.vstack(corners), rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frame", default="odom",
                        help="量测所在坐标系，默认 odom")
    parser.add_argument("--side", choices=["left", "right", "both"],
                        default="both", help="只量哪一侧的工具，默认两侧")
    parser.add_argument("--mesh-detail", action="store_true",
                        help="逐网格列出自身尺寸与连杆原点")
    args = parser.parse_args()

    rclpy.init()
    node = ToolExtentMeasurer(args.frame)
    try:
        entries = None
        node._wait(lambda: node._urdf is not None, 20.0,
                   ROBOT_DESCRIPTION_TOPIC)
        entries = parse_tool_meshes(node._urdf, args.side)
        if not entries:
            raise RuntimeError("URDF 里没找到 %s 下的工具网格" % TOOL_MESH_MARKER)
        corners, rows = node.measure(entries, args.mesh_detail)
        low = corners.min(axis=0)
        high = corners.max(axis=0)
        extent = (high - low) * 1000.0

        print("\n==== 末端工具包围盒（%s 系）====" % args.frame)
        print("网格数 %d，连杆 %d 个" %
              (len(rows), len({r[0] for r in rows})))
        if args.mesh_detail:
            print("-- 逐网格 --")
            for link, mesh, size, origin in rows:
                print("  %-28s %-20s 自身 %6.1f×%6.1f×%6.1f mm"
                      "  原点 (%+.4f %+.4f %+.4f)"
                      % (link, mesh, size[0], size[1], size[2],
                         origin[0], origin[1], origin[2]))
        print("-- 并集包围盒 --")
        for i, axis in enumerate(_AXIS_NAMES):
            print("  %s: %+.4f … %+.4f m   跨度 %.2f mm"
                  % (axis, low[i], high[i], extent[i]))
        print("-- 按比例换算（竖直方向 = Z 轴）--")
        for i, axis in enumerate(_AXIS_NAMES):
            print("  %s 的 1/4 = %.2f mm" % (axis, extent[i] / 4.0))
        print("================================")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
