#!/usr/bin/env python3
"""通过 Web 操作验证电气夹层仿真抽拉往返；运行前需位于目标工位。"""
import argparse
import json
import os
from pathlib import Path
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api', default='http://127.0.0.1:8090')
    parser.add_argument('--control', choices=['db1', 'dm1', 'ds1', 'ds2', 'ds3'], required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    headers = {'Content-Type': 'application/json'}
    token = os.environ.get('XCZS_CONTROL_TOKEN')
    if token:
        headers['Authorization'] = 'Bearer ' + token

    def api(path, payload=None):
        request = urllib.request.Request(args.api + path, headers=headers,
                                         data=json.dumps(payload).encode() if payload else None)
        with urllib.request.urlopen(request, timeout=15) as response:
            return json.load(response)

    def controls():
        return {item['control_id']: item for item in
                api('/cabinets/electrical_mezzanine/controls')['controls']}

    baseline = controls()
    evidence = {'control': args.control, 'checks': [], 'success': False}
    try:
        for state, expected in [('open', .05), ('closed', 0.0)]:
            task = api('/task/operate', {'cabinet': 'electrical_mezzanine',
                                        'control_id': args.control,
                                        'command': 'set_state', 'target_state': state})
            deadline = time.monotonic() + 180
            while task['status'] not in ('success', 'failed', 'canceled'):
                if time.monotonic() > deadline:
                    raise RuntimeError('Web 操作超时')
                time.sleep(.5)
                task = api('/task/' + task['task_id'] + '/status')
            if task['status'] != 'success':
                raise RuntimeError(task.get('message'))
            # 任务结束后继续观测，检查回弹和其他抽屉串动。
            positions = []
            for _ in range(10):
                snapshot = controls()
                item = snapshot[args.control]
                position = float(item['current_position'])
                positions.append(position)
                if abs(position - expected) > .003 or item['current_state'] != state:
                    raise RuntimeError('轨道未稳定在目标: %s, %.6f' % (state, position))
                for cid, other in snapshot.items():
                    if cid != args.control and abs(other['current_position'] - baseline[cid]['current_position']) > .003:
                        raise RuntimeError('其他抽屉发生串动: ' + cid)
                time.sleep(.5)
            evidence['checks'].append({'state': state, 'task': task,
                                       'positions_m': positions})
            print(args.control, state, 'PASS', positions[-1], flush=True)
        evidence['success'] = True
    finally:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(evidence, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
