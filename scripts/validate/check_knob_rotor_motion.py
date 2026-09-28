#!/usr/bin/env python3
"""核对旋钮旋转阶段：转盘转动，机械臂关节保持；依赖实测录制数据。"""
import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operation', type=Path, required=True)
    parser.add_argument('--motion-file', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    operation = json.loads(args.operation.read_text())
    samples = operation['samples']
    rotating = [s for s in samples if '由夹爪转盘旋转旋钮' in
                s['status'].get('message', '')]
    if not rotating:
        raise RuntimeError('未录到独立转盘旋转阶段')
    start = rotating[0]['time']
    end = next(s['time'] for s in samples if s['time'] > rotating[-1]['time'])
    recorded = [json.loads(line) for line in args.motion_file.read_text().splitlines()]
    frames = [f for f in recorded if start <= f['wall_time'] <= end and f.get('joints')]
    # Web 轮询可能晚于旋转实际开始；加入紧邻的前一帧，避免漏掉起始角度。
    preceding = [f for f in recorded if start - 0.5 <= f['wall_time'] < start
                 and f.get('joints')]
    if preceding:
        frames.insert(0, max(preceding, key=lambda f: f['wall_time']))
    names = [f'r_arm_{i}_joint' for i in range(7)] + ['r_rotbtn_rotate_joint']
    if len(frames) < 3 or any(n not in f['joints'] for f in frames for n in names):
        raise RuntimeError('旋转阶段实测关节样本不足')
    spans = {}
    for name in names:
        values = [f['joints'][name] for f in frames]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('关节反馈非有限值')
        spans[name] = math.degrees(max(values) - min(values))
    arm_span = max(spans[n] for n in names[:-1])
    rotor_span = spans[names[-1]]
    terminal = max((f for f in recorded if f['wall_time'] <= samples[-1]['time']
                    and f.get('joints')), key=lambda f: f['wall_time'])
    reset_error = abs(terminal['joints']['r_rotbtn_rotate_joint'])
    success = (bool(operation['success']) and arm_span < 1.0 and
               40.0 < rotor_span < 48.0 and reset_error < 0.01)
    result = {'success': success, 'samples': len(frames), 'start': start, 'end': end,
              'joint_span_deg': spans, 'max_arm_span_deg': arm_span,
              'rotor_span_deg': rotor_span,
              'final_rotor_error_rad': reset_error,
              'limitation': '关节离散采样，仅验证旋转分工，不替代接触与完整动作验收。'}
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print('PASS' if success else 'FAIL', f'arm={arm_span:.3f}°, rotor={rotor_span:.3f}°')
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
