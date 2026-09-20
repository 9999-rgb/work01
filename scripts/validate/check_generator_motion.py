#!/usr/bin/env python3
"""检查发电机层实测网格审计记录；允许 1 mm 数值接触容差，不替代连续碰撞证明。"""
import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-file', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    count = 0
    minima = {}
    for line in args.motion_file.read_text().splitlines():
        frame = json.loads(line)
        if not frame.get('link_poses'):
            continue
        count += 1
        for key in ['cabinet_signed_m', 'button_signed_m']:
            entry = frame[key]
            if not math.isfinite(entry[0]):
                raise RuntimeError('网格距离无效')
            if key not in minima or entry[0] < minima[key]['distance_m']:
                minima[key] = {'distance_m': entry[0], 'links': entry[1:],
                               'time': frame['wall_time']}
    success = count >= 10 and len(minima) == 2 and all(
        -0.001 <= entry['distance_m'] < 100 for entry in minima.values())
    result = {'success': success, 'samples': count, 'minima': minima,
              'contact_tolerance_m': .001,
              'limitation': '顶点采样；开放 CAD 网格使用无符号距离，不能证明完整连续无碰撞。'}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print('PASS' if success else 'FAIL', args.motion_file, count, 'samples')
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
