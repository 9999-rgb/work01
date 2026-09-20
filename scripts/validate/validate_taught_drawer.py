#!/usr/bin/env python3
"""通过 Web 操作验证电气夹层仿真抽拉往返；运行前需位于目标工位。"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api', default='http://127.0.0.1:8090')
    parser.add_argument('--control', choices=['db1', 'dm1', 'ds1', 'ds2', 'ds3'], required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--repeat-target', action='store_true')
    parser.add_argument('--expect-direct-close', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.cycles <= 20:
        parser.error('--cycles 必须在 1..20')
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
    evidence = {'control': args.control, 'baseline': baseline, 'checks': [], 'success': False}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    try:
        operations = [('open', .05), ('closed', 0.0)]
        if args.repeat_target:
            operations = [item for item in operations for _ in range(2)]
        for index, (state, expected) in enumerate(operations * args.cycles):
            task = api('/task/operate', {'cabinet': 'electrical_mezzanine',
                                        'control_id': args.control,
                                        'command': 'set_state', 'target_state': state})
            progress_trace = []
            deadline = time.monotonic() + 180
            while task['status'] not in ('success', 'failed', 'canceled'):
                progress_trace.append({'time': time.time(), 'phase': task.get('phase'),
                                       'progress': task.get('progress'), 'message': task.get('message')})
                if time.monotonic() > deadline:
                    raise RuntimeError('Web 操作超时')
                time.sleep(.5)
                task = api('/task/' + task['task_id'] + '/status')
            if task['status'] != 'success':
                evidence['failed_task'] = task
                evidence['failed_progress_trace'] = progress_trace
                raise RuntimeError(task.get('message'))
            if args.repeat_target and index % 2 == 1:
                if not task.get('result', {}).get('already_at_target'):
                    raise RuntimeError('重复目标仍执行了抽拉动作')
            elif args.expect_direct_close and state == 'closed':
                if not task.get('result', {}).get('continued_from_open'):
                    evidence['failed_task'] = task
                    evidence['failed_progress_trace'] = progress_trace
                    raise RuntimeError('扣手开位未直接推回，重复执行了准备动作')
            # 任务结束后继续观测，检查回弹和其他抽屉串动。
            positions = []
            for _ in range(10):
                snapshot = controls()
                item = snapshot[args.control]
                position = float(item['current_position'])
                positions.append(position)
                if (not math.isfinite(position) or abs(position - expected) > .003
                        or item['current_state'] != state):
                    raise RuntimeError('轨道未稳定在目标: %s, %.6f' % (state, position))
                for cid, other in snapshot.items():
                    if cid != args.control and (not math.isfinite(other['current_position'])
                            or abs(other['current_position'] - baseline[cid]['current_position']) > .003):
                        evidence['unexpected_motion'] = {'control': cid,
                            'before': baseline[cid]['current_position'],
                            'after': other['current_position']}
                        raise RuntimeError('其他抽屉发生串动: ' + cid)
                time.sleep(.5)
            evidence['checks'].append({'cycle': index // len(operations) + 1,
                                       'state': state, 'task': task,
                                       'positions_m': positions,
                                       'progress_trace': progress_trace})
            args.out.write_text(json.dumps(evidence, ensure_ascii=False, indent=2))
            print(args.control, state, 'PASS', positions[-1], flush=True)
        evidence['success'] = True
    finally:
        args.out.write_text(json.dumps(evidence, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
