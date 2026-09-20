#!/usr/bin/env python3
"""保守筛查电气夹层末端的柜后穿模与滑动副偏转；不替代完整网格碰撞检测。"""
import argparse
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import vtk

ROOT = Path(__file__).resolve().parents[2]


def tool_bounds():
    document = ET.parse(ROOT / 'xczs_inspection_robot_description/urdf/components/tools.xacro')
    bounds = {}
    for link in document.findall('.//link'):
        name = link.get('name', '')
        if '_two_cyl_' not in name and '_three_cyl_' not in name:
            continue
        name = name.replace('${prefix}', 'l' if '_two_cyl_' in name else 'r')
        visual = link.find('visual')
        origin = visual.find('origin')
        if origin is not None and any(float(v) != 0 for v in
                (origin.get('xyz', '0 0 0') + ' ' + origin.get('rpy', '0 0 0')).split()):
            raise RuntimeError('工具 visual 原点已变化，需更新几何检查: ' + name)
        mesh = visual.find('geometry/mesh')
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(ROOT / mesh.get('filename').split('://', 1)[1]))
        reader.Update()
        if reader.GetOutput().GetNumberOfPoints() == 0:
            raise RuntimeError('无法读取工具网格: ' + name)
        box = reader.GetOutput().GetBounds()
        bounds[name] = np.array(list(itertools.product(box[:2], box[2:4], box[4:6])))
    return bounds


def world_points(corners, pose):
    x, y, z, w = pose[3:]
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return corners @ rotation.T + np.array(pose[:3])



def check_repeated_commands(frames, web_file):
    evidence = json.loads(web_file.read_text())
    measurements = []
    for check in evidence['checks']:
        task = check['task']
        if not task['result'].get('already_at_target'):
            continue
        samples = [frame for frame in frames
                   if task['started_at'] <= frame['time'] <= task['completed_at']]
        if not samples:
            raise RuntimeError('重复指令期间没有位姿采样')
        maximum = max(float(np.linalg.norm(
            np.array(frame['poses'][name][:3]) - np.array(samples[0]['poses'][name][:3])))
            for frame in samples for name in ('l_two_cyl_base', 'r_three_cyl_base'))
        measurements.append({'task_id': task['task_id'], 'maximum_translation_m': maximum})
    return measurements


def support_strokes(frames):
    document = ET.parse(ROOT / 'xczs_inspection_robot_description/urdf/components/tools.xacro')
    measurements = {}
    for joint in document.findall('.//joint'):
        child = joint.find('child').get('link', '')
        prefix = 'l' if '_two_cyl_' in child else 'r'
        child = child.replace('${prefix}', prefix)
        if child not in ('l_two_cyl_finger2', 'r_three_cyl_finger1'):
            continue
        parent = joint.find('parent').get('link').replace('${prefix}', prefix)
        origin = joint.find('origin')
        if any(float(v) != 0 for v in origin.get('rpy', '0 0 0').split()):
            raise RuntimeError('支撑杆关节原点旋转已变化，需更新行程检查')
        axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
        axis /= np.linalg.norm(axis)
        offset = np.dot(np.array([float(v) for v in origin.get('xyz').split()]), axis)
        strokes = []
        for frame in frames:
            base, rod = frame['poses'][parent], frame['poses'][child]
            world_axis = world_points(np.array([axis]), base)[0] - np.array(base[:3])
            strokes.append(float(np.dot(np.array(rod[:3])-np.array(base[:3]), world_axis)-offset))
        measurements[child] = {'minimum_m': min(strokes), 'maximum_m': max(strokes)}
    if len(measurements) != 2:
        raise RuntimeError('支撑杆关节定义缺失')
    return measurements


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-file', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--web-file', type=Path, help='检查重复指令期间的实测末端位移')
    parser.add_argument('--control', choices=['db1', 'dm1', 'ds2', 'ds3'])
    parser.add_argument('--max-rod-angle', type=float, default=5.0)
    args = parser.parse_args()
    frames = [json.loads(line) for line in args.motion_file.read_text().splitlines()]
    if len(frames) < 2:
        raise RuntimeError('实际运动采样不足')
    gaps = np.diff([frame['time'] for frame in frames])
    if not np.isfinite(gaps).all() or np.any(gaps <= 0):
        raise RuntimeError('采样时间无效或不递增')
    # 仿真慢于实时时，墙钟间隔大并不等于运动漏采；用仿真时间核对覆盖度。
    simulation_gaps = None
    if all('simulation_time' in frame for frame in frames):
        simulation_gaps = np.diff([frame['simulation_time'] for frame in frames])
        if not np.isfinite(simulation_gaps).all() or np.any(simulation_gaps < 0):
            raise RuntimeError('仿真时间无效或发生重置')
    result = {'motion_file': str(args.motion_file), 'samples': len(frames),
              'maximum_sampling_gap_s': float(gaps.max()),
              'maximum_simulation_gap_s': (float(simulation_gaps.max())
                                           if simulation_gaps is not None else None),
              'tool_clearance': {}, 'rod_rotation': {},
              'success': bool((simulation_gaps if simulation_gaps is not None else gaps).max() <= .25)}
    bounds = tool_bounds()
    rods = {name for name in bounds if 'finger' in name}
    for frame in frames:
        angles = frame.get('rod_rotation_deg', {})
        if not rods.issubset(angles) or not all(np.isfinite(angles[name]) for name in rods):
            raise RuntimeError('杆件偏转记录缺失或无效')
    for name, corners in bounds.items():
        lowest = (float('inf'), None)
        for frame in frames:
            if name not in frame['poses']:
                raise RuntimeError('运动记录缺少工具: ' + name)
            points = world_points(corners, frame['poses'][name])
            if not np.isfinite(points).all():
                raise RuntimeError('工具位姿包含无效读数: ' + name)
            lower, upper = points.min(0), points.max(0)
            # 当前四只抽拉柜所在区域；保守包围盒进入 x<0 需人工复查。
            if upper[2] < .65 or lower[2] > 2.3 or upper[1] < 3.8 or lower[1] > 5.9:
                continue
            if lower[0] < lowest[0]:
                lowest = (float(lower[0]), frame['time'])
        result['tool_clearance'][name] = {'minimum_x_m': lowest[0], 'time': lowest[1]}
        if lowest[1] is None or lowest[0] < 0.0:
            result['success'] = False
    for name in sorted(rods):
        worst = max(frames, key=lambda f: f['rod_rotation_deg'].get(name, 0))
        angle = worst['rod_rotation_deg'][name]
        result['rod_rotation'][name] = {'maximum_deg': angle, 'time': worst['time']}
        if not np.isfinite(angle) or angle > args.max_rod_angle:
            result['success'] = False
    if args.web_file:
        result['repeat_commands'] = check_repeated_commands(frames, args.web_file)
        if any(item['maximum_translation_m'] > .001 for item in result['repeat_commands']):
            result['success'] = False
    if args.control:
        result['support_strokes'] = support_strokes(frames)
        if args.control in ('ds2', 'ds3') and any(
                max(abs(item['minimum_m']), abs(item['maximum_m'])) > .001
                for item in result['support_strokes'].values()):
            result['success'] = False
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print('PASS' if result['success'] else 'FAIL', args.motion_file, 'samples=', len(frames))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
