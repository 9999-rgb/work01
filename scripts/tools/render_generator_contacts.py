#!/usr/bin/env python3
"""以 Gazebo 实测 link 位姿渲染第二套末端与发电机控件，避免截取用户桌面。"""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import rclpy
import vtk
from gazebo_msgs.srv import GetEntityState
from rcl_interfaces.srv import GetParameters

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xczs_controllers import SpinNode, wait_for

ROOT = Path(__file__).resolve().parents[2]


def transform(position, quaternion):
    x, y, z, w = quaternion
    rows = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), position[0]],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), position[1]],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), position[2]],
            [0, 0, 0, 1]]
    matrix = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            matrix.SetElement(i, j, rows[i][j])
    return matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--motion-file', type=Path,
                        help='回放 generator_contact_audit 记录的实测位姿')
    parser.add_argument('--at-time', type=float, help='选择最接近的记录时间（Unix 秒）')
    parser.add_argument('--view', choices=['oblique', 'side'], default='oblique',
                        help='斜视或沿柜面侧视，用于检查接触深度')
    args = parser.parse_args()
    recorded = {}
    if args.motion_file:
        frames = [json.loads(line) for line in args.motion_file.read_text().splitlines()]
        if args.at_time is None:
            parser.error('--motion-file 需要同时指定 --at-time')
        frame = min(frames, key=lambda f: abs(f['wall_time'] - args.at_time))
        recorded = {k.split('::')[-1]: v for k, v in frame['link_poses'].items()}
        print('recorded_time', frame['wall_time'])
    rclpy.init()
    node = SpinNode('render_generator_contacts')
    entity = node.create_client(GetEntityState, '/get_entity_state')
    params = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')

    def call(client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('仿真服务不可用')
        future = client.call_async(request)
        wait_for(future.done, 10, '仿真读数')
        return future.result()

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.93, 0.94, 0.96)
    try:
        req = GetParameters.Request()
        req.names = ['robot_description']
        robot = ET.fromstring(call(params, req).values[0].string_value)
        scene = ET.parse(ROOT / 'xczs_inspection_robot_description/urdf/scenes/generator_plant.xacro')
        links = [(link, False) for link in robot.findall('link')
                 if link.find('visual') is not None and
                 any(part in link.attrib['name'] for part in ('rotbtn', 'rocker'))]
        links += [(link, True) for link in scene.findall('./model/link')
                  if link.find('visual') is not None]
        for link, sdf in links:
            name = link.attrib['name']
            req = GetEntityState.Request()
            req.name = name
            response = call(entity, req)
            if not response.success:
                continue
            pose = response.state.pose
            matrix = transform([getattr(pose.position, a) for a in 'xyz'],
                               [getattr(pose.orientation, a) for a in 'xyzw'])
            if name in recorded:
                matrix = vtk.vtkMatrix4x4()
                for i in range(4):
                    for j in range(4):
                        matrix.SetElement(i, j, recorded[name][i][j])
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                uri = mesh.findtext('uri') if sdf else mesh.attrib['filename']
                path = ROOT / uri.split('://', 1)[1]
                reader = vtk.vtkSTLReader()
                reader.SetFileName(str(path))
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(reader.GetOutputPort())
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                origin = visual.find('origin')
                values = ([float(v) for v in visual.findtext('pose', '0 0 0 0 0 0').split()]
                          if sdf else [float(v) for v in
                          ((origin.get('xyz', '0 0 0') + ' ' + origin.get('rpy', '0 0 0'))
                           if origin is not None else '0 0 0 0 0 0').split()])
                local = vtk.vtkTransform()
                local.SetMatrix(matrix)
                local.Translate(*values[:3])
                local.RotateZ(math.degrees(values[5]))
                local.RotateY(math.degrees(values[4]))
                local.RotateX(math.degrees(values[3]))
                actor.SetUserMatrix(local.GetMatrix())
                actor.GetProperty().SetColor((0.55, 0.57, 0.6) if sdf else
                                             ((0.9, 0.58, 0.1) if 'finger' in name else (0.12, 0.18, 0.25)))
                renderer.AddActor(actor)
        import yaml
        config = yaml.safe_load((ROOT / 'xczs_inspection_robot_control/config/scene_controls/generator_plant_controls.yaml').read_text())
        params_root = next(iter(config.values()))['ros__parameters']
        x, y, z = params_root['controls'][args.control]['local_position']
        camera = renderer.GetActiveCamera()
        camera.SetPosition(x + 1.5, y + (.12 if args.view == 'side' else .8), z + .3)
        camera.SetFocalPoint(x, y + .08, z)
        camera.SetViewUp(0,0,1)
        camera.ParallelProjectionOn()
        camera.SetParallelScale(.24)
        window = vtk.vtkRenderWindow()
        window.SetOffScreenRendering(1)
        window.AddRenderer(renderer)
        window.SetSize(1200,900)
        window.Render()
        capture = vtk.vtkWindowToImageFilter()
        capture.SetInput(window)
        capture.Update()
        writer = vtk.vtkPNGWriter()
        args.out.parent.mkdir(parents=True, exist_ok=True)
        writer.SetFileName(str(args.out))
        writer.SetInputConnection(capture.GetOutputPort())
        writer.Write()
        window.Finalize()
        print(args.out)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
