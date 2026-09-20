#!/usr/bin/env python3
"""只读记录 Gazebo 实体姿态下的旋钮接触与穿透量（JSONL）。

输出柜体盒的有符号距离、闭合网格双向顶点穿入深度和双钳口到轴表面的
距离。开放网格仅计算无符号距离，避免装配网格法向产生虚假穿入。
顶点采样不是完整连续碰撞证明；应与 MoveIt 碰撞检查及 Gazebo 接触合看。
"""
import argparse
import json
import signal
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
import vtk
from vtk.util.numpy_support import vtk_to_numpy, numpy_to_vtk

CLOSED_SURFACES = {}
BOX_SURFACES = {}

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/tools'))
from measure_tool_extent import quat_to_matrix, rpy_to_matrix


def matrix(pose):
    result = np.eye(4)
    result[:3, :3] = quat_to_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return result


def mesh(path, offset, rotation, scale):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    poly = vtk.vtkPolyData()
    poly.DeepCopy(reader.GetOutput())
    points = vtk_to_numpy(poly.GetPoints().GetData()).astype(float)
    points = (points * scale) @ rotation.T + offset
    vp = vtk.vtkPoints()
    vp.SetData(numpy_to_vtk(points, deep=True))
    poly.SetPoints(vp)
    distance = vtk.vtkImplicitPolyDataDistance()
    distance.SetInput(poly)
    edges = vtk.vtkFeatureEdges()
    edges.SetInputData(poly)
    edges.BoundaryEdgesOn(); edges.NonManifoldEdgesOn()
    edges.FeatureEdgesOff(); edges.ManifoldEdgesOff()
    edges.Update()
    CLOSED_SURFACES[id(distance)] = edges.GetOutput().GetNumberOfCells() == 0
    return points, distance


def signed(points, distance):
    if id(distance) in BOX_SURFACES:
        center, size = BOX_SURFACES[id(distance)]
        q = np.abs(points - center) - size / 2
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
    values = vtk.vtkDoubleArray()
    distance.EvaluateFunction(numpy_to_vtk(np.ascontiguousarray(points), deep=True), values)
    result = vtk_to_numpy(values)
    # 开放的 CAD 装配网格没有可靠的内部/外部；法向符号不能作为穿透证据。
    return result if CLOSED_SURFACES.get(id(distance), True) else np.abs(result)


def load_geometry():
    desc = ROOT / 'xczs_inspection_robot_description'
    urdf = ET.fromstring(subprocess.check_output(['xacro', str(desc / 'urdf/xczs_inspection_robot.urdf.xacro'), 'toolset:=B']))
    scene = ET.parse(desc / 'urdf/scenes/generator_plant.xacro').getroot().find('model')
    shapes = {}
    for link in urdf.findall('link'):
        name = link.get('name')
        if not (name.startswith(('r_rotbtn', 'r_arm_', 'l_rocker', 'l_arm_'))):
            continue
        # 钳口使用原可见网格审计外观穿模；物理代理接触深度由插件独立校验。
        geometry_tag = 'visual' if name in ('r_rotbtn_jaw1', 'r_rotbtn_jaw2') else 'collision'
        for col in link.findall(geometry_tag):
            m = col.find('geometry/mesh')
            if m is None:
                continue
            origin = col.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            shapes['xczs_inspection_robot::' + name] = mesh(ROOT / m.get('filename').removeprefix('package://'), xyz, rpy_to_matrix(*rpy), np.fromstring(m.get('scale', '1 1 1'), sep=' '))
    boxes = []
    for col in scene.find("link[@name='fadianground']").findall('collision'):
        boxes.append((col.get('name'), np.fromstring(col.findtext('pose'), sep=' ')[:3], np.fromstring(col.findtext('geometry/box/size'), sep=' ')))
    for link in scene.findall('link'):
        if link.get('name') == 'fadianground':
            continue
        col = link.find('collision')
        m = col.find('geometry/mesh') if col is not None else None
        if col is None:
            continue
        if m is None:
            box = col.find('geometry/box/size')
            if box is None:
                # 独立按钮采用圆柱碰撞体，外观网格保留源模型的形状和位姿。
                visual = link.find('visual')
                vm = visual.find('geometry/mesh') if visual is not None else None
                if vm is not None:
                    pose = np.fromstring(visual.findtext('pose', '0 0 0 0 0 0'), sep=' ')
                    shapes['xczs_scene_floor::' + link.get('name')] = mesh(
                        ROOT / vm.findtext('uri').removeprefix('model://'),
                        pose[:3], rpy_to_matrix(*pose[3:]),
                        np.fromstring(vm.findtext('scale', '1 1 1'), sep=' '))
                continue
            pose = np.fromstring(col.findtext('pose', '0 0 0 0 0 0'), sep=' ')
            size = np.fromstring(box.text, sep=' ')
            cube = vtk.vtkCubeSource()
            cube.SetCenter(*pose[:3])
            cube.SetXLength(size[0]); cube.SetYLength(size[1]); cube.SetZLength(size[2])
            tri = vtk.vtkTriangleFilter(); tri.SetInputConnection(cube.GetOutputPort()); tri.Update()
            distance = vtk.vtkImplicitPolyDataDistance(); distance.SetInput(tri.GetOutput())
            BOX_SURFACES[id(distance)] = (pose[:3], size)
            shapes['xczs_scene_floor::' + link.get('name')] = (vtk_to_numpy(tri.GetOutput().GetPoints().GetData()).astype(float), distance)
            continue
        pose = np.fromstring(col.findtext('pose', '0 0 0 0 0 0'), sep=' ')
        shapes['xczs_scene_floor::' + link.get('name')] = mesh(ROOT / m.findtext('uri').removeprefix('model://'), pose[:3], rpy_to_matrix(*pose[3:]), np.fromstring(m.findtext('scale', '1 1 1'), sep=' '))
    return shapes, boxes


def measure(shapes, boxes, poses):
    world = {k: points @ poses[k][:3, :3].T + poses[k][:3, 3] for k, (points, _) in shapes.items() if k in poses}
    box_min = [100., '', '']
    plate_min = [100., '', '']
    shaft_dist = {}
    for name, points in world.items():
        if not name.startswith('xczs_inspection_robot::'):
            continue
        for box, center, size in boxes:
            q = np.abs(points-center)-size/2
            d = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
            low = float(d.min())
            if low < box_min[0]:
                box_min = [low, name, box]
        if 'rotbtn' not in name and 'rocker' not in name:
            continue
        for fixture, fp in world.items():
            if not fixture.startswith('xczs_scene_floor::') or np.linalg.norm(fp.mean(0)-points.mean(0)) > .3:
                continue
            local = (points - poses[fixture][:3, 3]) @ poses[fixture][:3, :3]
            d1 = signed(local, shapes[fixture][1])
            local = (fp - poses[name][:3, 3]) @ poses[name][:3, :3]
            d2 = signed(local, shapes[name][1])
            low = float(min(d1.min(), d2.min()))
            if (fixture.endswith('1') or '::button' in fixture or fixture.endswith('rotatebutton')) and low < plate_min[0]:
                plate_min = [low, name, fixture]
            if fixture.endswith('1') and 'jaw' in name:
                shaft_dist[name + ':' + fixture] = low
    return {'cabinet_signed_m': box_min, 'button_signed_m': plate_min,
            'jaw_plate_signed_m': shaft_dist,
            'unsigned_open_meshes': [k for k, (_, distance) in shapes.items()
                                     if not CLOSED_SURFACES.get(id(distance), True)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=1800)
    parser.add_argument('--interval', type=float, default=.25)
    parser.add_argument('--replay', type=Path, help='按保存的真实位姿重新计算网格距离')
    args = parser.parse_args()
    shapes, boxes = load_geometry()
    if args.replay:
        for line in args.replay.read_text().splitlines():
            frame = json.loads(line)
            poses = {k: np.array(v) for k, v in frame['link_poses'].items()}
            metrics = measure(shapes, boxes, poses)
            # 旧记录未保存臂段位姿；保留原始、解析盒距离的全臂审计结果。
            metrics['cabinet_signed_m'] = frame['cabinet_signed_m']
            frame.update(metrics)
            print(json.dumps(frame), flush=True)
        return
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    # 先结束采样循环再销毁 ROS 实体，避免 SIGINT 与 take_message 并发。
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = rclpy.create_node('generator_contact_audit')
    poses, joints = {}, {}
    node.create_subscription(LinkStates, '/link_states', lambda m: poses.update({k: matrix(p) for k, p in zip(m.name, m.pose) if k in shapes or k == "xczs_inspection_robot::body"}), 10)
    node.create_subscription(JointState, '/xczs/joint_states', lambda m: joints.update({k: v for k, v in zip(m.name, m.position)}), 10)
    started = time.monotonic()
    last = 0
    try:
        while not stop_requested and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() - last < args.interval:
                continue
            last = time.monotonic()
            metrics = measure(shapes, boxes, poses)
            print(json.dumps({'wall_time': time.time(), 'elapsed': last-started, 'joints': joints, 'link_poses': {k: p.tolist() for k, p in poses.items()}, **metrics}), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
