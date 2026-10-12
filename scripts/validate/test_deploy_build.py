"""Deployment failure handling and cache boundaries; no live robot required."""
import importlib.util
import os
import tempfile
import socket
import subprocess
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('update_rosdep', ROOT / 'scripts/deploy/update_rosdep.py')
UPDATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(UPDATE)
DOCKER = (ROOT / 'scripts/deploy/Dockerfile').read_text().replace('\\\n', '')
RUNS = [line[4:] for line in DOCKER.splitlines() if line.startswith('RUN ')]


class DeploymentTests(unittest.TestCase):
    def retry(self, outcomes, expected, calls):
        with patch.object(UPDATE.subprocess, 'run', side_effect=outcomes) as run, \
                patch.object(UPDATE.time, 'sleep'):
            self.assertEqual(UPDATE.main(), expected)
            self.assertEqual(run.call_count, calls)
            self.assertEqual(run.call_args.kwargs['timeout'], 300)

    def test_retry_recovers(self):
        self.retry([SimpleNamespace(returncode=1), SimpleNamespace(returncode=0)], 0, 2)

    def test_partial_cache_is_failure(self):
        self.retry([SimpleNamespace(returncode=1)] * 3, 1, 3)

    def test_timeouts_are_bounded(self):
        self.retry([subprocess.TimeoutExpired('rosdep', 300)] * 3, 1, 3)

    def test_ipv4_and_timeout_apply_before_rosdep_import(self):
        original_argv = UPDATE.sys.argv
        try:
            with patch.object(UPDATE.socket, 'getaddrinfo') as resolver, \
                    patch.object(UPDATE.urllib.request, 'urlopen') as opener:
                def run_rosdep(*args, **kwargs):
                    UPDATE.socket.getaddrinfo('example.test', 443)
                    UPDATE.urllib.request.urlopen('https://example.test', timeout=15)
                with patch.object(UPDATE.runpy, 'run_path', side_effect=run_rosdep):
                    UPDATE.update_once()
                self.assertEqual(resolver.call_args.args[2], socket.AF_INET)
                self.assertEqual(opener.call_args.kwargs['timeout'], 60)
        finally:
            UPDATE.sys.argv = original_argv

    def test_all_run_shell_syntax(self):
        for command in RUNS:
            with self.subTest(command=command[:70]):
                subprocess.run(['bash', '-n', '-c', command], check=True)

    def test_version_check_rejects_failures(self):
        command = next(c for c in RUNS if 'gzserver --version' in c)
        check = command[command.index('(status=0;'):]
        for code, version, expected in [(0, 'version 11.10.2', 0), (255, 'version 11.10.2', 0),
                                        (1, 'version 11.10.2', 1), (124, 'version 11.10.2', 1),
                                        (255, 'loader error', 1), (0, 'version 12.0', 1)]:
            with self.subTest(code=code, version=version):
                mock = f"timeout() {{ printf '%s\\n' '{version}'; return {code}; }}; "
                result = subprocess.run(['bash', '-c', mock + check], capture_output=True)
                self.assertEqual(result.returncode == 0, expected == 0)

    def test_virtual_display_readiness_gate(self):
        entrypoint = (ROOT / 'scripts/deploy/entrypoint.sh').read_text()
        command = entrypoint.split("xvfb-run -a", 1)[1].split("bash -c '", 1)[1].split("' &", 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            probe = temp / 'glxinfo'
            probe.write_text('#!/bin/sh\nexit "$PROBE_STATUS"\n')
            probe.chmod(0o755)
            launch = temp / 'run_all.sh'
            launch.write_text('#!/bin/sh\nprintf "LAUNCH %s %s %s\\n" "$XCZS_DISPLAY_SETTLE_SECONDS" "$XCZS_DISPLAY_SETTLE_MAX_WAIT" "$1"\n')
            launch.chmod(0o755)
            for status in ('0', '1', '124'):
                with self.subTest(probe_status=status):
                    env = {**os.environ, 'PATH': str(temp) + ':' + os.environ['PATH'], 'PROBE_STATUS': status}
                    env.pop('XCZS_DISPLAY_SETTLE_SECONDS', None)
                    env.pop('XCZS_DISPLAY_SETTLE_MAX_WAIT', None)
                    result = subprocess.run(['bash', '-c', command], cwd=temp, env=env,
                                            capture_output=True, text=True, timeout=5)
                    if status == '0':
                        self.assertEqual(result.returncode, 0)
                        self.assertIn('LAUNCH 0 0 --web', result.stdout)
                    else:
                        self.assertNotEqual(result.returncode, 0)
                        self.assertNotIn('LAUNCH', result.stdout)
            env = {**os.environ, 'PATH': str(temp) + ':' + os.environ['PATH'], 'PROBE_STATUS': '0',
                   'XCZS_DISPLAY_SETTLE_SECONDS': '20', 'XCZS_DISPLAY_SETTLE_MAX_WAIT': '15'}
            result = subprocess.run(['bash', '-c', command], cwd=temp, env=env,
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0)
            self.assertIn('LAUNCH 20 15 --web', result.stdout)

    def test_network_compile_checks_are_separate_layers(self):
        for token in ['update_rosdep.py', 'rosdep install', 'colcon build', 'gzserver --version']:
            self.assertEqual(sum(token in command for command in RUNS), 1)
        indices = [next(i for i, c in enumerate(RUNS) if token in c) for token in
                   ['update_rosdep.py', 'rosdep install', 'colcon build', 'gzserver --version']]
        self.assertEqual(len(set(indices)), 4)
        self.assertGreater(DOCKER.index('COPY . .'), DOCKER.index('colcon build'))
        for manifest in ROOT.glob('xczs_inspection_robot_*/package.xml'):
            self.assertIn(f'COPY {manifest.parent.name}/ {manifest.parent.name}/', DOCKER)


if __name__ == '__main__':
    unittest.main()
