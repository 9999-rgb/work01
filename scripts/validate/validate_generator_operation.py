#!/usr/bin/env python3
"""验证发电机控件的 Web 操作、实测行程及停止后的终态；不负责导航。"""
import argparse
import json
import math
from pathlib import Path
import time
import sys
import xml.etree.ElementTree as ET
import urllib.request
from urllib.error import URLError


def verify_rocker_withdrawal():
    """读取 Gazebo 实体与真实关节，证明停止后的退出和收臂。"""
    import rclpy
    from gazebo_msgs.srv import GetEntityState
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / 'scripts/tools'))
    from derive_taught_pose import PoseDeriver
    rclpy.init()
    node = PoseDeriver()
    try:
        node._wait(lambda: node._joint_state is not None, 10, 'joint state')
        srdf = ET.parse(root / 'xczs_inspection_robot_moveit_config/config/xczs_inspection_robot_toolset_B.srdf')
        desired = {j.get('name'): float(j.get('value')) for j in
                   srdf.findall("./group_state[@name='home'][@group='left_arm']/joint")}
        assert len(desired) == 7, '缺少左臂收臂配置'
        measured = dict(zip(node._joint_state.name, node._joint_state.position))
        error = max(abs(measured[name] - value) for name, value in desired.items())
        assert math.isfinite(error) and error < .12, f'停止后左臂未收回: {error}'
        client = node.create_client(GetEntityState, '/get_entity_state')
        assert client.wait_for_service(timeout_sec=5), 'Gazebo 实体服务未就绪'
        points = []
        for name in ('l_rocker_rotor', 'rotatebutton'):
            request = GetEntityState.Request()
            request.name = name
            future = client.call_async(request)
            node._wait(future.done, 5, name)
            response = future.result()
            assert response.success, name
            position = response.state.pose.position
            points.append([position.x, position.y, position.z])
        distance = math.dist(*points)
        assert math.isfinite(distance) and distance > .06, '摇杆未退出插口'
        return {'home_max_joint_error_rad': error, 'tip_socket_distance_m': distance}
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api', default='http://127.0.0.1:8090')
    parser.add_argument('--control', required=True)
    parser.add_argument('--target', default='turned')
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=360)
    parser.add_argument('--task-id', help='接续采样已经启动的任务，不重新发送动作')
    parser.add_argument('--turns', type=float, default=1.0, help='摇杆停止前至少转动的圈数')
    args = parser.parse_args()
    if not math.isfinite(args.turns) or args.turns <= 0:
        parser.error('--turns 必须为正数')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    evidence = {'control': args.control, 'success': False, 'samples': []}

    def request(path, payload=None):
        req = urllib.request.Request(args.api + path,
            data=None if payload is None else json.dumps(payload).encode(),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=5) as response:
            return json.load(response)

    def controls():
        return {c['control_id']: c for c in request('/cabinets/generator_plant/controls')['controls']}

    ready_deadline = time.monotonic() + 30
    while True:
        try:
            before = controls()
        except URLError:
            if time.monotonic() > ready_deadline:
                raise
            time.sleep(.2)
            continue
        position = before[args.control].get('current_position')
        if isinstance(position, (int, float)) and math.isfinite(position):
            break
        if time.monotonic() > ready_deadline:
            raise TimeoutError('控件实时状态尚未就绪，没有发送动作')
        time.sleep(.2)
    control = before[args.control]
    continuous = control.get('continuous_rotation', False)
    button = control['control_type'] == 0
    payload = {'cabinet': 'generator_plant', 'control_id': args.control,
               'command': 'press' if button else 'set_state'}
    if button:
        payload['force'] = control['default_force']
    else:
        payload['target_state'] = 'running' if continuous else args.target
    task = args.task_id or request('/task/operate', payload)['task_id']
    evidence.update(task_id=task, request=payload, before=before)
    print('TASK', task, payload, flush=True)
    deadline = time.monotonic() + args.timeout
    stopped = False
    rotation_start = None
    last_message = None
    try:
        while time.monotonic() < deadline:
            status = request(f'/task/{task}/status')
            snapshot = controls()
            message = status.get('message')
            if message != last_message:
                print(status.get('status'), message, flush=True)
                last_message = message
            evidence['samples'].append({'time': time.time(), 'status': status,
                'positions': {k: v['current_position'] for k, v in snapshot.items()},
                'velocities': {k: v.get('velocity') for k, v in snapshot.items()}})
            measured = snapshot[args.control].get('current_position')
            if not isinstance(measured, (int, float)) or not math.isfinite(measured):
                raise RuntimeError('操作中控件实时状态失效')
            travel = abs(measured - control['current_position'])
            if continuous and not stopped:
                if '持续旋转' in (message or '') and rotation_start is None:
                    rotation_start = measured
                    evidence['rotation_started_at'] = time.time()
                    evidence['rotation_start_position'] = measured
                if rotation_start is None:
                    assert abs(snapshot[args.control]['velocity']) < .5, '插入期间插口异常高速转动'
                    assert travel < .1, '插入期间插口被异常带动'
                else:
                    travel = abs(measured - rotation_start)
            if continuous and rotation_start is not None and not stopped and travel >= 2 * math.pi * args.turns + .2:
                request(f'/task/{task}/cancel', {})
                stopped = True
                evidence['stop_requested_at'] = time.time()
                print('STOP after', travel / (2 * math.pi), 'turns', flush=True)
            if status['status'] in ('success', 'failed', 'canceled', 'cancelled', 'error'):
                if continuous and status['status'] in ('canceled', 'cancelled') and not status.get('backend_termination_confirmed'):
                    time.sleep(.1)
                    continue
                evidence['terminal'] = status
                if continuous:
                    assert stopped and status['status'] in ('canceled', 'cancelled'), status
                    assert not status.get('reservation_active'), '后台仍占用机器人'
                else:
                    assert status['status'] == 'success', status
                break
            time.sleep(.1)
        else:
            request(f'/task/{task}/cancel', {})
            raise TimeoutError('操作超时，已请求取消；检查后端停止状态')
        time.sleep(1)
        after = controls()
        evidence['after'] = after
        if continuous:
            assert abs(after[args.control]['velocity']) < .02, '停止后插口仍在旋转'
            rotating = [s for s in evidence['samples']
                        if s['time'] < evidence['stop_requested_at'] and
                        '持续旋转' in s['status'].get('message', '')]
            peak_velocity = max(abs(s['velocities'][args.control]) for s in rotating)
            evidence['rotation_peak_velocity_rad_s'] = peak_velocity
            assert peak_velocity < 2.0, f'旋转速度异常，存在抖动: {peak_velocity}'
            evidence['withdrawal'] = verify_rocker_withdrawal()
        elif button:
            assert abs(after[args.control]['current_position']) < .001, '按钮未释放'
            peak = max(s['positions'][args.control] for s in evidence['samples'])
            assert peak >= .003, '没有采到有效按下行程'
            assert peak <= control['max_position'] + .001, '按钮行程超限'
        else:
            child = args.control.replace('_knob', '_button')
            axial_peak = max(s['positions'][child] for s in evidence['samples'])
            assert axial_peak >= .007, '没有拉出到位证据'
            assert axial_peak <= before[child]['max_position'] + .001, '旋钮轴向行程超限'
            assert abs(after[child]['current_position']) < .001, '旋钮未插回'
            target = control['state_positions'][control['state_ids'].index(args.target)]
            assert abs(after[args.control]['current_position'] - target) < .05, '旋钮角度未到位'
            peak_velocity = max(abs(s['velocities'][args.control]) for s in evidence['samples'])
            evidence['rotation_peak_velocity_rad_s'] = peak_velocity
            assert math.isfinite(peak_velocity) and peak_velocity < 2.0, '旋钮出现异常高速抖动'
        allowed = {args.control}
        if not continuous and not button:
            allowed.add(args.control.replace('_knob', '_button'))
        disturbed = [name for name in before if name not in allowed and
                     isinstance(before[name].get('current_position'), (int, float)) and
                     abs(after[name]['current_position'] - before[name]['current_position']) > .005]
        evidence['disturbed_controls'] = disturbed
        assert not disturbed, f'旁路控件发生位移: {disturbed}'
        transient = [name for name in before if name not in allowed and any(
            not isinstance(sample['positions'][name], (int, float)) or
            not math.isfinite(sample['positions'][name]) or
            abs(sample['positions'][name] - before[name]['current_position']) > .005
            for sample in evidence['samples'])]
        evidence['transient_disturbed_controls'] = transient
        assert not transient, f'操作过程中旁路控件发生串动: {transient}'
        evidence['success'] = True
        print('PASS', args.control, flush=True)
    except (Exception, KeyboardInterrupt) as error:
        evidence['error'] = str(error)
        if not evidence.get('terminal'):
            try:
                evidence['cancel_response'] = request(f'/task/{task}/cancel', {})
            except Exception as cancel_error:
                evidence['cancel_error'] = str(cancel_error)
        raise
    finally:
        output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
