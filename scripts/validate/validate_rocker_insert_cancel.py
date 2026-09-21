#!/usr/bin/env python3
"""经 Web 验证摇杆已对准但尚未插到底时取消，确认退出和碰撞恢复。先预定位到 frb。"""
import argparse
import json
from pathlib import Path
import time
import urllib.request
from validate_generator_operation import verify_rocker_withdrawal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api', default='http://127.0.0.1:8090')
    parser.add_argument('--runtime-log', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    evidence = {'success': False, 'scenario': 'cancel_during_verified_insertion'}

    def api(path, data=None):
        request = urllib.request.Request(args.api + path,
            data=None if data is None else json.dumps(data).encode(),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=10) as response:
            return json.load(response)

    def position():
        controls = api('/cabinets/generator_plant/controls')['controls']
        return next(c['current_position'] for c in controls if c['control_id'] == 'frb')

    offset = args.runtime_log.stat().st_size
    initial = position()
    task = api('/task/operate', {'cabinet': 'generator_plant', 'control_id': 'frb',
                                'command': 'set_state', 'target_state': 'running'})['task_id']
    evidence.update(task_id=task, initial_position=initial)
    stopped = False
    terminal = False
    try:
        deadline = time.monotonic() + 240
        while time.monotonic() < deadline:
            log = args.runtime_log.read_bytes()[offset:].decode(errors='replace')
            status = api(f'/task/{task}/status')
            if not stopped and "Verified insertion alignment for 'frb'" in log:
                evidence['cancel_response'] = api(f'/task/{task}/cancel', {})
                evidence['stop_requested_at'] = time.time()
                stopped = True
                print('STOP during verified insertion', flush=True)
            if status.get('backend_termination_confirmed'):
                terminal = True
                evidence['terminal'] = status
                assert stopped and status['status'] in ('canceled', 'cancelled'), status
                assert not status.get('reservation_active'), '机器人占用未释放'
                break
            time.sleep(.1)
        else:
            raise TimeoutError('插入取消未在期限内结束')
        time.sleep(1)
        log = args.runtime_log.read_bytes()[offset:].decode(errors='replace')
        evidence['collision_restored'] = "Restored 1 actuation collision(s) for cabinet control 'frb'" in log
        assert evidence['collision_restored'], '退出后配合面碰撞未恢复'
        evidence['withdrawal'] = verify_rocker_withdrawal()
        evidence['final_position'] = position()
        assert abs(evidence['final_position'] - initial) < .1, '未开始传动时插口发生明显转动'
        evidence['success'] = True
        print('PASS insertion cancel', flush=True)
    except BaseException as error:
        evidence['error'] = str(error)
        raise
    finally:
        if not terminal:
            try:
                api(f'/task/{task}/cancel', {})
            except Exception as error:
                evidence['cancel_error'] = str(error)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(evidence, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
