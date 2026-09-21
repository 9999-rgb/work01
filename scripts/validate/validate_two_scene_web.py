#!/usr/bin/env python3
"""从 Web 切换两层场景和末端，逐项复用动作验收器；工位采用仿真预定位。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api', default='http://127.0.0.1:8090')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--drawer-cycles', type=int, default=2)
    parser.add_argument('--phase', choices=['all', 'electrical', 'generator', 'generator_aux', 'generator_rocker', 'generator_buttons', 'generator_knobs'], default='all')
    args = parser.parse_args()
    if not 1 <= args.drawer_cycles <= 20:
        parser.error('--drawer-cycles 必须在 1..20')
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    report = {'success': False, 'steps': [], 'prepositioned_simulation': True, 'phase': args.phase}
    headers = {'Content-Type': 'application/json'}
    if os.environ.get('XCZS_CONTROL_TOKEN'):
        headers['Authorization'] = 'Bearer ' + os.environ['XCZS_CONTROL_TOKEN']

    def api(path, data=None, timeout=180):
        request = urllib.request.Request(args.api + path, headers=headers,
            data=None if data is None else json.dumps(data).encode())
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.load(response)

    def run(script, *arguments):
        command = [sys.executable, str(ROOT / script), *map(str, arguments)]
        print('RUN', script, *arguments, flush=True)
        subprocess.run(command, cwd=ROOT, check=True)
        report['steps'].append({'script': script, 'arguments': list(map(str, arguments))})

    def select(scene, toolset):
        result = api('/scene/switch', {'name': scene})
        assert result.get('status') in ('switched', 'unchanged', 'reconciled'), result
        status = api('/robot/toolset/status')
        if status.get('active_toolset') != toolset:
            api('/robot/toolset/switch', {'toolset': toolset,
                'expected_generation': status.get('generation', 0)})
        deadline = time.monotonic() + 420
        while time.monotonic() < deadline:
            status = api('/robot/toolset/status')
            if status.get('state') == 'ready' and status.get('active_toolset') == toolset:
                break
            if status.get('state') == 'failed':
                raise RuntimeError('末端切换失败: ' + str(status.get('message')))
            time.sleep(2)
        else:
            raise TimeoutError('末端切换未就绪')
        assert api('/scenes')['active'] == scene, '末端切换后场景不一致'
        report['steps'].append({'scene': scene, 'toolset': toolset, 'switch': result})
        print('READY', scene, toolset, flush=True)

    def position(control, scene, toolset):
        run('scripts/tools/preposition_base.py', '--control', control, '--cabinet', scene,
            '--toolset', toolset, '--adapter', ROOT / 'xczs_inspection_robot_control/config/scene_controls' / (scene + '_adapter.yaml'))

    def start_recording(script, *arguments, output=None):
        stream = output.open('w') if output else subprocess.DEVNULL
        process = subprocess.Popen([sys.executable, str(ROOT / script), *map(str, arguments)],
            cwd=ROOT, stdout=stream)
        return process, stream

    def stop_recording(recording):
        process, stream = recording
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=10)
        if hasattr(stream, 'close'):
            stream.close()

    try:
        catalog = api('/scenes')
        report['scene_catalog'] = catalog
        assert {s['name'] for s in catalog['scenes']} == {'electrical_mezzanine', 'generator_plant'}, catalog
        if args.phase in ('all', 'electrical'):
            select('electrical_mezzanine', 'A')
            for control in ['db1', 'dm1', 'ds2', 'ds3']:
                position(control, 'electrical_mezzanine', 'A')
                motion = args.out / (control + '_motion.jsonl')
                web = args.out / (control + '_web.json')
                recorder = start_recording('scripts/validate/record_drawer_motion.py', '--out', motion, '--seconds', 1200)
                try:
                    time.sleep(2)
                    run('scripts/validate/validate_taught_drawer.py', '--api', args.api,
                        '--control', control, '--cycles', args.drawer_cycles, '--repeat-target',
                        '--expect-direct-close', '--out', web)
                finally:
                    stop_recording(recorder)
                run('scripts/validate/check_drawer_motion.py', '--control', control,
                    '--motion-file', motion, '--web-file', web, '--out', args.out / (control + '_geometry.json'))
                geometry = json.loads((args.out / (control + '_geometry.json')).read_text())
                worst = max(geometry['rod_rotation'].values(), key=lambda value: value['maximum_deg'])
                run('scripts/tools/render_drawer_contacts.py', '--control', control,
                    '--motion-file', motion, '--at-time', worst['time'], '--view', 'side',
                    '--out', args.out / (control + '_worst.png'))
        if args.phase in ('all', 'generator', 'generator_aux', 'generator_rocker', 'generator_buttons', 'generator_knobs'):
            select('generator_plant', 'B')
            recorder = start_recording('scripts/validate/generator_contact_audit.py',
                '--duration', 3600, '--interval', .4, output=args.out / 'generator_contact.jsonl')
            try:
                controls = ['fr135_knob', 'fr2222_knob', 'fr4332_knob',
                            'fr12452_knob', 'fr20422_knob', 'fr25452_knob',
                            'fbutton1', 'fbutton2', 'fbutton3', 'fbutton4', 'frb']
                if args.phase == 'generator_knobs':
                    controls = [control for control in controls if control.endswith('_knob')]
                if args.phase == 'generator_aux':
                    controls = ['fbutton1', 'fbutton2', 'fbutton3', 'fbutton4', 'frb']
                if args.phase == 'generator_buttons':
                    controls = ['fbutton1', 'fbutton2', 'fbutton3', 'fbutton4']
                if args.phase == 'generator_rocker':
                    controls = ['frb']
                for control in controls:
                    position(control, 'generator_plant', 'B')
                    for target in (['turned', 'center'] if control.endswith('_knob') else ['press']):
                        run('scripts/validate/validate_generator_operation.py', '--api', args.api,
                            '--control', control, '--target', target,
                            '--output', args.out / (control + '_' + target + '.json'))
                if args.phase == 'generator_aux':
                    for control in ['fbutton1', 'fbutton2', 'fbutton3', 'fbutton4']:
                        position(control, 'generator_plant', 'B')
                        run('scripts/validate/validate_generator_operation.py', '--api', args.api,
                            '--control', control, '--output', args.out / (control + '_repeat.json'))
                    position('frb', 'generator_plant', 'B')
                if args.phase not in ('generator_buttons', 'generator_knobs'):
                    run('scripts/validate/validate_generator_operation.py', '--api', args.api,
                        '--control', 'frb', '--turns', .1, '--output', args.out / 'frb_repeat.json')
            finally:
                stop_recording(recorder)
            run('scripts/validate/check_generator_motion.py', '--motion-file',
                args.out / 'generator_contact.jsonl', '--out', args.out / 'generator_geometry.json')
            for control, suffix, measured in [('fr135_knob', 'turned', 'fr135_button'),
                                               ('fbutton4', 'press', 'fbutton4'),
                                               ('frb', 'press', 'frb')]:
                if args.phase == 'generator_knobs' and not control.endswith('_knob'):
                    continue
                if args.phase == 'generator_buttons' and control != 'fbutton4':
                    continue
                if args.phase == 'generator_rocker' and control != 'frb':
                    continue
                if args.phase == 'generator_aux' and control.endswith('_knob'):
                    continue
                operation = json.loads((args.out / (control + '_' + suffix + '.json')).read_text())
                sample = max(operation['samples'], key=(
                    (lambda item: abs(item['velocities']['frb'])) if control == 'frb' else
                    (lambda item: item['positions'][measured])))
                run('scripts/tools/render_generator_contacts.py', '--control', control,
                    '--motion-file', args.out / 'generator_contact.jsonl', '--at-time', sample['time'],
                    '--view', 'side', '--out', args.out / (control + '_side.png'))
        if args.phase == 'all':
            select('electrical_mezzanine', 'A')
        report['success'] = True
    except BaseException as error:
        report['error'] = str(error)
        raise
    finally:
        (args.out / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
