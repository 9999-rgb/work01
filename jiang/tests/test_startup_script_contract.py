from __future__ import annotations

import os
import signal
import socket
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
STARTUP = ROOT / "jiang" / "start_xczs_bridge.sh"
WEB_VALIDATOR = ROOT / "scripts" / "validate_cabinet_web"
RECORDING_VALIDATOR = ROOT / "scripts" / "validate_recording_replay"
ZENOH_BRIDGE_CONFIG = (
    ROOT
    / "xczs_inspection_robot_control"
    / "config"
    / "zenoh_bridge.json5"
)


class StartupScriptContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.startup_source = STARTUP.read_text(encoding="utf-8")
        cls.validator_source = WEB_VALIDATOR.read_text(encoding="utf-8")
        cls.recording_validator_source = RECORDING_VALIDATOR.read_text(
            encoding="utf-8"
        )
        cls.zenoh_bridge_config_source = ZENOH_BRIDGE_CONFIG.read_text(
            encoding="utf-8"
        )

    def test_preflight_checks_runtime_contract_and_ports(self) -> None:
        preflight_exit = self.startup_source.index(
            'if [ "$PREFLIGHT_ONLY" = "true" ]'
        )
        contract_check = self.startup_source.index("check_adapter_contract")
        port_check = self.startup_source.index(
            '_require_port_available "$ZENOH_BIND_HOST" "$BRIDGE_TCP_PORT"'
        )

        self.assertLess(contract_check, preflight_exit)
        self.assertLess(port_check, preflight_exit)
        self.assertIn(
            'if [ "$ZENOH_REQUIRED" = "true" ] && '
            '[ ! -x "$ZENOH_BRIDGE" ]',
            self.startup_source,
        )
        self.assertIn("create_app", self.startup_source)
        self.assertIn("from control_gateway import ControlServer", self.startup_source)
        for package in (
            "xczs_inspection_robot_control",
            "xczs_inspection_robot_description",
            "gazebo_ros",
            "robot_state_publisher",
            "controller_manager",
            "moveit_ros_move_group",
            "nav2_bringup",
        ):
            self.assertIn(package, self.startup_source)
        self.assertIn(
            'if [ "$GAZEBO_ENABLED" = "true" ] || '
            '[ "$ROBOT_BRINGUP" = "true" ] ||',
            self.startup_source,
        )
        self.assertIn(
            'if [ "$MOVEIT_ENABLED" = "true" ] &&',
            self.startup_source,
        )
        self.assertIn(
            'if [ "$NAV2_ENABLED" = "true" ]', self.startup_source
        )

    def test_network_isolation_is_explicit_and_consistent(self) -> None:
        self.assertIn(
            'ZENOH_LAN_ENABLED="${XCZS_ZENOH_LAN_ENABLED:-false}"',
            self.startup_source,
        )
        self.assertIn('ZENOH_BIND_HOST="127.0.0.1"', self.startup_source)
        self.assertIn('ZENOH_BIND_HOST="0.0.0.0"', self.startup_source)
        self.assertIn(
            'ZENOH_SCOUTING_ARGS=(--no-multicast-scouting)',
            self.startup_source,
        )
        self.assertIn(
            '--listen "tcp/$ZENOH_BIND_HOST:$BRIDGE_TCP_PORT"',
            self.startup_source,
        )
        self.assertIn(
            '--rest-http-port "$ZENOH_BIND_HOST:$BRIDGE_REST_PORT"',
            self.startup_source,
        )
        self.assertIn(
            '--zenoh "tcp/127.0.0.1:$BRIDGE_TCP_PORT"',
            self.startup_source,
        )
        self.assertIn(
            '_require_port_available "$ZENOH_BIND_HOST" "$BRIDGE_REST_PORT"',
            self.startup_source,
        )
        self.assertIn(
            'export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"',
            self.startup_source,
        )
        self.assertIn(
            'ROS_LOCALHOST_ONLY 必须为 0 或 1', self.startup_source
        )
        self.assertIn(
            'if [ -z "${CYCLONEDDS_URI:-}" ]', self.startup_source
        )
        self.assertIn("MaxAutoParticipantIndex>60", self.startup_source)
        self.assertIn(
            'echo "  ROS 2 域:   $ROS_DOMAIN_ID"', self.startup_source
        )
        self.assertIn(
            'echo "  Zenoh 网络: $ZENOH_NETWORK_LABEL"', self.startup_source
        )

    def test_headless_warning_discloses_camera_rendering_limit(self) -> None:
        self.assertIn("DISPLAY 未设置", self.startup_source)
        self.assertIn(
            'timeout --kill-after=1 3 xdpyinfo -display "$DISPLAY"',
            self.startup_source,
        )
        self.assertIn(
            "基于渲染的 RGB 相机也不会发布图像",
            self.startup_source,
        )

    def test_http_readiness_calls_are_bounded(self) -> None:
        for function_name, next_name in (
            ("_wait_for_http()", "_task_stack_is_ready()"),
            ("_wait_for_task_stack()", "_wait_for_stable_processes()"),
            ("_monitor_managed_processes()", "_start_bridge()"),
        ):
            start = self.startup_source.index(function_name)
            end = self.startup_source.index(next_name, start)
            function = self.startup_source[start:end]
            self.assertIn("curl -fsS --connect-timeout 1 --max-time 2", function)

    def test_zenoh_and_web_start_after_ros_stack_stabilizes(self) -> None:
        stable_wait = self.startup_source.index(
            '_wait_for_stable_processes 50'
        )
        bridge_start = self.startup_source.index(
            '_start_bridge\n',
            stable_wait,
        )
        bridge_stable = self.startup_source.index(
            '_wait_for_stable_processes 25',
            bridge_start,
        )
        web_start = self.startup_source.index(
            '_start_web_control\n',
            bridge_stable,
        )
        task_readiness = self.startup_source.index(
            '_wait_for_task_stack \\\n',
            web_start,
        )
        self.assertLess(stable_wait, bridge_start)
        self.assertLess(bridge_start, bridge_stable)
        self.assertLess(bridge_stable, web_start)
        self.assertLess(web_start, task_readiness)

    def test_zenoh_bridge_does_not_reinject_raw_camera_topics(self) -> None:
        self.assertIn(
            'ZENOH_BRIDGE_CONFIG="${ZENOH_BRIDGE_CONFIG:-',
            self.startup_source,
        )
        self.assertIn(
            '--config "$ZENOH_BRIDGE_CONFIG"',
            self.startup_source,
        )
        self.assertIn(
            '_require_file "$ZENOH_BRIDGE_CONFIG"',
            self.startup_source,
        )
        self.assertIn('publishers: ["/xczs/camera/.*"]',
                      self.zenoh_bridge_config_source)
        self.assertIn('subscribers: ["/xczs/camera/.*"]',
                      self.zenoh_bridge_config_source)

    def test_gazebo_master_is_derived_from_domain_and_preflighted(self) -> None:
        self.assertIn(
            'GAZEBO_MASTER_URI:-http://127.0.0.1:$((11345 + ROS_DOMAIN_ID))',
            self.startup_source,
        )
        self.assertIn(
            'GAZEBO_MASTER_URI 必须为 http://host:port', self.startup_source
        )
        self.assertIn(
            '0.0.0.0 "$GAZEBO_MASTER_PORT" "Gazebo Master"',
            self.startup_source,
        )
        self.assertIn(
            "ipaddress.ip_address(parsed.hostname).is_loopback",
            self.startup_source,
        )
        self.assertIn(
            "read -r GAZEBO_MASTER_HOST GAZEBO_MASTER_PORT "
            "GAZEBO_MASTER_IS_LOCAL",
            self.startup_source,
        )
        self.assertIn(
            '远端 master，跳过本机端口预检', self.startup_source
        )
        self.assertIn(
            '本地 Gazebo 未启用，跳过端口预检', self.startup_source
        )
        self.assertIn(
            'echo "  Gazebo URI: $GAZEBO_MASTER_URI"', self.startup_source
        )

    def test_entire_ipv4_loopback_range_is_preflighted_as_local(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind(("127.0.0.1", 0))
            shared_port = probe.getsockname()[1]
        environment = os.environ.copy()
        environment.update(
            BRIDGE_TCP_PORT=str(shared_port),
            GAZEBO_MASTER_URI=f"http://127.0.0.2:{shared_port}",
            XCZS_PREFLIGHT_ONLY="true",
        )
        result = subprocess.run(
            [str(STARTUP), "--web"],
            text=True,
            capture_output=True,
            env=environment,
            timeout=10,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "GAZEBO_MASTER_URI 端口不能与 Zenoh 或 Web 端口相同",
            result.stdout + result.stderr,
        )

    def test_gazebo_master_uri_rejects_whitespace_and_control_characters(
        self,
    ) -> None:
        for uri in (
            "http://local host:17577",
            "http://local\thost:17577",
        ):
            environment = os.environ.copy()
            environment.update(
                GAZEBO_MASTER_URI=uri,
                XCZS_PREFLIGHT_ONLY="true",
            )
            result = subprocess.run(
                [str(STARTUP), "--web"],
                text=True,
                capture_output=True,
                env=environment,
                timeout=10,
                check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(
                "GAZEBO_MASTER_URI 必须为 http://host:port",
                result.stdout + result.stderr,
            )

    def test_leading_zero_domain_is_normalized_as_decimal(self) -> None:
        reserved_sockets = []
        ports = []
        try:
            for _ in range(3):
                probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                probe.bind(("127.0.0.1", 0))
                reserved_sockets.append(probe)
                ports.append(probe.getsockname()[1])
        finally:
            for probe in reserved_sockets:
                probe.close()
        environment = os.environ.copy()
        environment.update(
            ROS_DOMAIN_ID="08",
            BRIDGE_TCP_PORT=str(ports[0]),
            BRIDGE_REST_PORT=str(ports[1]),
            CONTROL_PORT=f"0{ports[2]}",
            XCZS_STARTUP_TIMEOUT_SEC="08",
            CONTROL_HOST="127.0.0.1",
            XCZS_PREFLIGHT_ONLY="true",
        )
        result = subprocess.run(
            [str(STARTUP), "--web"],
            text=True,
            capture_output=True,
            env=environment,
            timeout=30,
            check=False,
        )
        self.assertEqual(
            result.returncode,
            0,
            msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
        )
        self.assertIn("ROS 2 域:   8", result.stdout)
        self.assertIn("Gazebo URI: http://127.0.0.1:11353", result.stdout)
        self.assertIn(
            "Web Origins: "
            f"http://localhost:{ports[2]},"
            f"http://127.0.0.1:{ports[2]}",
            result.stdout,
        )

    def test_plain_keyboard_mode_does_not_require_zenoh(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-display-test-") as tmp:
            fake_xdpyinfo = Path(tmp) / "xdpyinfo"
            fake_xdpyinfo.write_text(
                "#!/usr/bin/env bash\nexit 0\n",
                encoding="utf-8",
            )
            fake_xdpyinfo.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                DISPLAY=":xczs-test",
                ZENOH_BRIDGE="/definitely/missing/zenoh-bridge",
                BRIDGE_TCP_PORT="not-a-port",
                BRIDGE_REST_PORT="also-not-a-port",
                XCZS_ZENOH_LAN_ENABLED="not-a-boolean",
                GAZEBO_ENABLED="false",
                SPAWN_CABINET="false",
                XCZS_PREFLIGHT_ONLY="true",
            )
            result = subprocess.run(
                [str(STARTUP), "--keyboard"],
                text=True,
                capture_output=True,
                env=environment,
                timeout=30,
                check=False,
            )
        self.assertEqual(
            result.returncode,
            0,
            msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
        )
        self.assertIn("完全跳过（键盘模式无需 Zenoh）", result.stdout)
        self.assertIn("Zenoh 网络: 未启用", result.stdout)

    def test_ipv6_wildcard_uses_ipv6_loopback_for_readiness(self) -> None:
        self.assertIn('::) _READY_HOST="::1"', self.startup_source)
        self.assertNotIn(
            '0.0.0.0|::|\'[::]\') _READY_HOST="127.0.0.1"',
            self.startup_source,
        )

    def test_keyboard_proxy_still_requires_zenoh(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-display-test-") as tmp:
            fake_xdpyinfo = Path(tmp) / "xdpyinfo"
            fake_xdpyinfo.write_text(
                "#!/usr/bin/env bash\nexit 0\n",
                encoding="utf-8",
            )
            fake_xdpyinfo.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                DISPLAY=":xczs-test",
                ZENOH_BRIDGE="/definitely/missing/zenoh-bridge",
                XCZS_PREFLIGHT_ONLY="true",
            )
            result = subprocess.run(
                [str(STARTUP), "--keyboard", "--with-proxy"],
                text=True,
                capture_output=True,
                env=environment,
                timeout=10,
                check=False,
            )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "zenoh-bridge-ros2dds 不存在",
            result.stdout + result.stderr,
        )

    def test_unreachable_display_is_rejected_for_keyboard_mode(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-display-test-") as tmp:
            fake_xdpyinfo = Path(tmp) / "xdpyinfo"
            fake_xdpyinfo.write_text(
                "#!/usr/bin/env bash\nexit 1\n",
                encoding="utf-8",
            )
            fake_xdpyinfo.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                DISPLAY=":broken-display",
                XCZS_PREFLIGHT_ONLY="true",
            )
            result = subprocess.run(
                [str(STARTUP), "--keyboard"],
                text=True,
                capture_output=True,
                env=environment,
                timeout=10,
                check=False,
            )
        self.assertNotEqual(result.returncode, 0)
        output = result.stdout + result.stderr
        self.assertIn("无法连接 DISPLAY=:broken-display", output)
        self.assertIn("RGB 相机也不会发布图像", output)
        self.assertIn("需要可连接的 DISPLAY", output)

    def test_display_probe_force_kills_a_term_ignoring_process(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-display-test-") as tmp:
            fake_xdpyinfo = Path(tmp) / "xdpyinfo"
            fake_xdpyinfo.write_text(
                "#!/usr/bin/env bash\n"
                "trap '' TERM\n"
                "while :; do sleep 10; done\n",
                encoding="utf-8",
            )
            fake_xdpyinfo.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                DISPLAY=":term-ignoring-display",
                GAZEBO_ENABLED="false",
                SPAWN_CABINET="false",
                CABINET_BRINGUP="false",
                MOVEIT_ENABLED="false",
                XCZS_PREFLIGHT_ONLY="true",
            )
            started = time.monotonic()
            result = subprocess.run(
                [str(STARTUP), "--keyboard"],
                text=True,
                capture_output=True,
                env=environment,
                timeout=8,
                check=False,
            )
            elapsed = time.monotonic() - started

        self.assertNotEqual(result.returncode, 0)
        self.assertLess(elapsed, 7.0)
        self.assertIn(
            "无法连接 DISPLAY=:term-ignoring-display",
            result.stdout + result.stderr,
        )

    def test_launcher_only_stops_its_registered_process_groups(self) -> None:
        self.assertNotIn("pkill", self.startup_source)
        self.assertIn("MANAGED_PIDS", self.startup_source)
        self.assertIn("MANAGED_PGIDS", self.startup_source)
        self.assertIn(
            '_signal_process_group "$process_group" TERM', self.startup_source
        )
        self.assertNotIn('kill "-$signal" "$pid"', self.startup_source)
        self.assertIn(
            '_start_managed_process LAUNCH_PID "ROS 2 launch"',
            self.startup_source,
        )
        self.assertIn(
            "ros2 launch xczs_inspection_robot_control",
            self.startup_source,
        )
        self.assertIn("xczs-managed-guardian", self.startup_source)
        self.assertIn("_monitor_managed_processes", self.startup_source)
        self.assertNotIn("ROS 2 launch 已正常结束", self.startup_source)
        self.assertIn(
            'echo "ERROR: ${MANAGED_LABELS[$index]} 意外退出（status=$status）。"',
            self.startup_source,
        )
        launch_start = self.startup_source.index(
            '_start_managed_process LAUNCH_PID "ROS 2 launch"'
        )
        task_readiness = self.startup_source.rindex("_wait_for_task_stack")
        self.assertLess(launch_start, task_readiness)
        self.assertIn(
            'health.get("map_available") is True', self.startup_source
        )
        self.assertIn("occupancy map", self.startup_source)
        self.assertIn('.map_available == true', self.validator_source)
        self.assertIn(
            '_wait_for_http "http://${_READY_URL_HOST}:$CONTROL_PORT/health"',
            self.startup_source,
        )
        self.assertIn("_task_stack_is_ready", self.startup_source)
        self.assertIn(
            'CABINET_ACTION_REQUIRED="false"', self.startup_source
        )
        self.assertIn(
            '[ "$CABINET_BRINGUP" = "true" ] && '
            '[ "$MOVEIT_ENABLED" = "true" ]',
            self.startup_source,
        )
        self.assertIn(
            '_task_stack_is_ready "$CABINET_ACTION_REQUIRED"',
            self.startup_source,
        )
        monitor_start = self.startup_source.index(
            "_monitor_managed_processes()"
        )
        monitor_end = self.startup_source.index("_start_bridge()", monitor_start)
        runtime_monitor = self.startup_source[monitor_start:monitor_end]
        self.assertIn("consecutive_health_failures", runtime_monitor)
        self.assertIn("_task_stack_is_ready", runtime_monitor)
        self.assertIn("连续 3 次未通过运行期健康检查", runtime_monitor)

    def test_cleanup_stops_group_after_leader_has_exited(self) -> None:
        helpers_start = self.startup_source.index("_register_process() {")
        helpers_end = self.startup_source.index("trap _cleanup_on_exit EXIT")
        helpers = self.startup_source[helpers_start:helpers_end]
        harness = f"""
set -Eeo pipefail
CLEANUP_DONE=false
SHUTDOWN_TIMEOUT_SEC=1
MANAGED_PIDS=()
MANAGED_PGIDS=()
MANAGED_LABELS=()
{helpers}
trap cleanup EXIT

sleep 5 &
unmanaged_pid=$!
setsid bash -c 'trap "" TERM; sleep 5 &' &
leader=$!
_register_process "$leader" "leader-exit regression"
wait "$leader"
process_group="${{MANAGED_PGIDS[0]}}"
if _managed_leader_is_running "$leader" "$process_group"; then
    echo "leader should already have exited" >&2
    exit 31
fi
if ! _process_group_is_running "$process_group"; then
    echo "child should still be alive in the registered group" >&2
    exit 32
fi
# 让显式调用模拟退出状态为 0 时触发的 EXIT trap。
true
cleanup
if _process_group_is_running "$process_group"; then
    echo "registered child process survived cleanup" >&2
    exit 33
fi
if ! kill -0 "$unmanaged_pid" 2>/dev/null; then
    echo "cleanup touched an unmanaged process" >&2
    exit 34
fi
kill -TERM "$unmanaged_pid"
wait "$unmanaged_pid" 2>/dev/null || true
"""
        result = subprocess.run(
            ["bash"],
            input=harness,
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        self.assertEqual(
            result.returncode,
            0,
            msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
        )
        self.assertIn("强制终止", result.stderr)

    def test_exit_trap_propagates_managed_group_cleanup_failure(self) -> None:
        helpers_start = self.startup_source.index("_register_process() {")
        helpers_end = self.startup_source.index("trap _cleanup_on_exit EXIT")
        helpers = self.startup_source[helpers_start:helpers_end]
        harness = f"""
set -Eeo pipefail
CLEANUP_DONE=false
SHUTDOWN_TIMEOUT_SEC=1
MANAGED_PIDS=(424242)
MANAGED_PGIDS=(424242)
MANAGED_LABELS=(stubbed-residual)
{helpers}
# Simulate an unkillable residual without addressing any real process.
_process_group_is_running() {{ return 0; }}
_signal_process_group() {{ return 0; }}
_wait_for_managed_groups() {{ return 1; }}
wait() {{ return 0; }}
trap _cleanup_on_exit EXIT
exit 0
"""
        result = subprocess.run(
            ["bash"],
            input=harness,
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("受管进程组 PGID=424242 仍未退出", result.stderr)

    def test_cleanup_removes_shell_owned_launch_runtime_directory(self) -> None:
        helpers_start = self.startup_source.index("_register_process() {")
        helpers_end = self.startup_source.index("trap _cleanup_on_exit EXIT")
        helpers = self.startup_source[helpers_start:helpers_end]
        with tempfile.TemporaryDirectory(
            prefix="xczs_runtime_contract_", dir="/tmp"
        ) as runtime_directory:
            marker = Path(runtime_directory) / "generated" / "artifact.yaml"
            marker.parent.mkdir()
            marker.write_text("generated", encoding="utf-8")
            harness = f"""
set -Eeo pipefail
CLEANUP_DONE=false
SHUTDOWN_TIMEOUT_SEC=1
MANAGED_PIDS=()
MANAGED_PGIDS=()
MANAGED_LABELS=()
LAUNCH_RUNTIME_DIRECTORY="$RUNTIME_DIRECTORY"
export XCZS_LAUNCH_RUNTIME_DIRECTORY="$LAUNCH_RUNTIME_DIRECTORY"
{helpers}
expected_runtime="$LAUNCH_RUNTIME_DIRECTORY"
cleanup
[ ! -e "$expected_runtime" ]
[ -z "${{XCZS_LAUNCH_RUNTIME_DIRECTORY+x}}" ]
"""
            result = subprocess.run(
                ["bash"],
                input=harness,
                text=True,
                capture_output=True,
                timeout=5,
                check=False,
                env={**os.environ, "RUNTIME_DIRECTORY": runtime_directory},
            )
        self.assertEqual(
            result.returncode,
            0,
            msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
        )

    def test_guardian_reaps_group_if_supervisor_is_killed(self) -> None:
        helpers_start = self.startup_source.index("_register_process() {")
        helpers_end = self.startup_source.index("trap _cleanup_on_exit EXIT")
        helpers = self.startup_source[helpers_start:helpers_end]
        with tempfile.TemporaryDirectory(
            prefix="xczs-managed-guardian-test-"
        ) as tmp:
            tmp_path = Path(tmp)
            unmanaged_file = tmp_path / "unmanaged.pid"
            managed_file = tmp_path / "managed.pgid"
            harness = f"""
set -Eeo pipefail
CLEANUP_DONE=false
SHUTDOWN_TIMEOUT_SEC=1
MANAGED_GUARD_TIMEOUT_SEC=1
MANAGED_PIDS=()
MANAGED_PGIDS=()
MANAGED_LABELS=()
{helpers}

sleep 30 &
printf '%s' "$!" >"$UNMANAGED_FILE"
_start_managed_process managed_process_id "orphan regression" \\
    bash -c 'trap "" TERM; sleep 30 & wait'
printf '%s' "$managed_process_id" >"$MANAGED_FILE"
while true; do sleep 1; done
"""
            environment = os.environ.copy()
            environment.update(
                UNMANAGED_FILE=str(unmanaged_file),
                MANAGED_FILE=str(managed_file),
            )
            supervisor = subprocess.Popen(
                ["bash"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
            )
            unmanaged_pid = 0
            managed_group = 0
            managed_running = True
            try:
                assert supervisor.stdin is not None
                supervisor.stdin.write(harness)
                supervisor.stdin.close()
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    if unmanaged_file.exists() and managed_file.exists():
                        unmanaged_text = unmanaged_file.read_text().strip()
                        managed_text = managed_file.read_text().strip()
                        if unmanaged_text and managed_text:
                            unmanaged_pid = int(unmanaged_text)
                            managed_group = int(managed_text)
                            break
                    if supervisor.poll() is not None:
                        break
                    time.sleep(0.05)
                self.assertGreater(unmanaged_pid, 0)
                self.assertGreater(managed_group, 0)

                os.kill(supervisor.pid, signal.SIGKILL)
                supervisor.wait(timeout=3)

                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    group_probe = subprocess.run(
                        [
                            "ps",
                            "-eo",
                            "pgid=,stat=",
                        ],
                        text=True,
                        capture_output=True,
                        check=True,
                    )
                    managed_running = any(
                        fields[0] == str(managed_group)
                        and not fields[1].startswith("Z")
                        for line in group_probe.stdout.splitlines()
                        if len(fields := line.split()) >= 2
                    )
                    if not managed_running:
                        break
                    time.sleep(0.1)
                self.assertFalse(
                    managed_running,
                    "guardian left a non-zombie process in its managed PGID",
                )
                os.kill(unmanaged_pid, 0)
            finally:
                if supervisor.poll() is None:
                    supervisor.kill()
                    supervisor.wait(timeout=3)
                if managed_group > 0:
                    try:
                        os.killpg(managed_group, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                if unmanaged_pid > 0:
                    try:
                        os.kill(unmanaged_pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                if supervisor.stdout is not None:
                    supervisor.stdout.close()
                if supervisor.stderr is not None:
                    supervisor.stderr.close()

    def test_guardian_preserves_a_clean_target_exit_status(self) -> None:
        helpers_start = self.startup_source.index("_register_process() {")
        helpers_end = self.startup_source.index("trap _cleanup_on_exit EXIT")
        helpers = self.startup_source[helpers_start:helpers_end]
        harness = f"""
set -Eeo pipefail
CLEANUP_DONE=false
SHUTDOWN_TIMEOUT_SEC=1
MANAGED_GUARD_TIMEOUT_SEC=1
MANAGED_PIDS=()
MANAGED_PGIDS=()
MANAGED_LABELS=()
{helpers}
_start_managed_process target_pid "clean-exit regression" bash -c 'exit 23'
set +e
wait "$target_pid"
status=$?
set -e
[ "$status" -eq 23 ]
if _process_group_is_running "$target_pid"; then
    echo "clean target left its managed process group" >&2
    exit 41
fi
"""
        result = subprocess.run(
            ["bash"],
            input=harness,
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
        self.assertEqual(
            result.returncode,
            0,
            msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
        )

    def test_web_validator_supports_bearer_and_login_authentication(self) -> None:
        self.assertIn("XCZS_CONTROL_TOKEN", self.validator_source)
        self.assertIn('"$API_URL/auth/login"', self.validator_source)
        self.assertIn(
            'AUTH_CURL_ARGS=(-H "Authorization: Bearer $API_TOKEN")',
            self.validator_source,
        )
        self.assertIn(
            'curl -sS -N --connect-timeout 5 "${AUTH_CURL_ARGS[@]}"',
            self.validator_source,
        )
        self.assertIn("XCZS_CONTROL_TOKEN", self.recording_validator_source)
        self.assertIn(
            '"${AUTH_CURL_ARGS[@]}"', self.recording_validator_source
        )
        self.assertIn(
            "timeout --kill-after=2 10 ros2 topic list",
            self.recording_validator_source,
        )
        readiness_start = self.validator_source.index("wait_for_system_ready()")
        readiness_end = self.validator_source.index(
            "start_sse_capture()",
            readiness_start,
        )
        readiness = self.validator_source[readiness_start:readiness_end]
        self.assertIn("--connect-timeout 2 --max-time 5", readiness)

    def test_validator_timeout_arguments_are_bounded_decimal(self) -> None:
        web_result = subprocess.run(
            [str(WEB_VALIDATOR), "--timeout", "999999"],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
        self.assertNotEqual(web_result.returncode, 0)
        self.assertIn("1..86400", web_result.stderr)

        recording_result = subprocess.run(
            [
                str(RECORDING_VALIDATOR),
                "--record-seconds",
                "08",
                "--include-sensors",
            ],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
        self.assertNotEqual(recording_result.returncode, 0)
        self.assertIn("必须与 --runtime 一起使用", recording_result.stderr)
        self.assertNotIn("底数", recording_result.stderr)

    def test_spawn_z_validation_survives_python_optimize(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-display-test-") as tmp:
            fake_xdpyinfo = Path(tmp) / "xdpyinfo"
            fake_xdpyinfo.write_text(
                "#!/usr/bin/env bash\nexit 0\n",
                encoding="utf-8",
            )
            fake_xdpyinfo.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                DISPLAY=":xczs-test",
                PYTHONOPTIMIZE="1",
                GAZEBO_ENABLED="false",
                SPAWN_CABINET="false",
                CABINET_BRINGUP="false",
                MOVEIT_ENABLED="false",
                XCZS_PREFLIGHT_ONLY="true",
                SPAWN_Z="nan",
            )
            result = subprocess.run(
                [str(STARTUP), "--keyboard"],
                text=True,
                capture_output=True,
                env=environment,
                timeout=30,
                check=False,
            )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SPAWN_Z", result.stdout + result.stderr)

    def test_recording_validator_claims_unexpected_start_before_asserting(self) -> None:
        request_start = self.recording_validator_source.index("request_json() {")
        request_end = self.recording_validator_source.index(
            "expect_code() {",
            request_start,
        )
        request_json = self.recording_validator_source[request_start:request_end]
        claim = request_json.index('OWN_RECORDING="true"')
        json_assertion = request_json.index("jq -e .")
        self.assertLess(claim, json_assertion)
        self.assertIn('"$path" == "/recording/start"', request_json)
        self.assertIn('"$RESPONSE_CODE" =~ ^2[0-9][0-9]$', request_json)
        self.assertIn("OWN_RECORDING_ID", request_json)
        replay_claim = request_json.index('OWN_REPLAY="true"')
        self.assertLess(replay_claim, json_assertion)
        self.assertIn('"$path" == "/replay/data/start"', request_json)
        self.assertIn('"$path" == "/replay/task/start"', request_json)

        post_start = self.recording_validator_source.index("post_json() {")
        post_end = self.recording_validator_source.index("wait_for_status() {")
        post_json = self.recording_validator_source[post_start:post_end]
        self.assertLess(
            post_json.index("request_json POST"),
            post_json.index('expect_code "$expected"'),
        )
        self.assertIn(
            'if [[ "$OWN_RECORDING" == "true" ]]',
            self.recording_validator_source,
        )
        self.assertIn(
            '"$active_recording_id" == "$OWN_RECORDING_ID"',
            self.recording_validator_source,
        )
        self.assertIn(
            '"$API_URL/recording/stop"',
            self.recording_validator_source,
        )
        data_runtime_start = self.recording_validator_source.index(
            "run_data_runtime() {"
        )
        task_runtime_start = self.recording_validator_source.index(
            "run_task_runtime() {"
        )
        data_runtime = self.recording_validator_source[
            data_runtime_start:task_runtime_start
        ]
        cancel_request = data_runtime.rindex("post_json /replay/cancel")
        ownership_release = data_runtime.rindex('OWN_REPLAY="false"')
        idle_wait = data_runtime.rindex("wait_for_status")
        self.assertLess(cancel_request, ownership_release)
        self.assertLess(ownership_release, idle_wait)
        task_runtime = self.recording_validator_source[task_runtime_start:]
        recording_match = task_runtime.index(
            '"$task_recording_id" == "$recording_id"'
        )
        active_branch = task_runtime.index("running|canceling)", recording_match)
        interlock_probe = task_runtime.index("post_json /cmd_vel", active_branch)
        terminal_branch = task_runtime.index(
            "success|failed|canceled)", interlock_probe
        )
        terminal_release = task_runtime.index(
            'OWN_REPLAY="false"', terminal_branch
        )
        result_dispatch = task_runtime.index(
            'case "$task_status" in', terminal_release
        )
        self.assertLess(recording_match, active_branch)
        self.assertLess(active_branch, interlock_probe)
        self.assertLess(interlock_probe, terminal_branch)
        self.assertLess(
            terminal_release,
            result_dispatch,
        )

    def test_task_replay_accepts_a_terminal_first_status_without_interlock(self) -> None:
        task_start = self.recording_validator_source.index(
            "run_task_runtime() {"
        )
        task_end = self.recording_validator_source.index(
            "\n}\n\ncheck_read_only_contract",
            task_start,
        ) + 2
        task_function = self.recording_validator_source[task_start:task_end]

        with tempfile.TemporaryDirectory(prefix="xczs-validator-test-") as tmp:
            harness = Path(tmp) / "task_replay_harness.sh"
            harness.write_text(
                """#!/usr/bin/env bash
set -euo pipefail
TASK_RECORDING=quick_failure
EXPECT_TASK_FAILURE=true
TIMEOUT_SECONDS=2
OWN_REPLAY=false
RESPONSE_BODY=''
STATUS_CALLS=0
CMD_VEL_CALLS=0
fail() { printf 'ERROR: %s\\n' "$*" >&2; exit 1; }
get_json() {
  case "$1" in
    /recordings/quick_failure)
      RESPONSE_BODY='{"artifacts":{"scenario":true}}'
      ;;
    /replay/status)
      STATUS_CALLS=$((STATUS_CALLS + 1))
      if (( STATUS_CALLS == 1 )); then
        RESPONSE_BODY='{"mode":"idle","read_only":false,"task_replay":{"status":"idle","recording_id":null}}'
      else
        RESPONSE_BODY='{"mode":"idle","read_only":false,'
        RESPONSE_BODY+='"task_replay":{"status":"failed",'
        RESPONSE_BODY+='"recording_id":"quick_failure",'
        RESPONSE_BODY+='"error":"expected fast failure"}}'
      fi
      ;;
    *) fail "unexpected GET $1" ;;
  esac
}
post_json() {
  case "$1" in
    /replay/task/start)
      [[ "$3" == 202 ]] || fail "wrong task start expectation"
      RESPONSE_BODY='{"mode":"task_replay","read_only":true,"task_replay":{"status":"running","recording_id":"quick_failure"}}'
      ;;
    /cmd_vel)
      CMD_VEL_CALLS=$((CMD_VEL_CALLS + 1))
      ;;
    *) fail "unexpected POST $1" ;;
  esac
}
wait_for_status() { :; }
"""
                + task_function
                + """
run_task_runtime
printf 'owner=%s cmd_vel=%s status_calls=%s\\n' \
  "$OWN_REPLAY" "$CMD_VEL_CALLS" "$STATUS_CALLS"
""",
                encoding="utf-8",
            )
            harness.chmod(0o755)
            result = subprocess.run(
                [str(harness)],
                text=True,
                capture_output=True,
                timeout=5,
                check=False,
            )

        self.assertEqual(
            result.returncode,
            0,
            msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
        )
        self.assertIn("owner=false cmd_vel=0 status_calls=2", result.stdout)

    def test_recording_validator_stops_an_unexpectedly_accepted_probe(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-validator-test-") as tmp:
            tmp_path = Path(tmp)
            curl_log = tmp_path / "curl.log"
            fake_curl = tmp_path / "curl"
            fake_curl.write_text(
                """#!/usr/bin/env bash
set -euo pipefail
output_file=""
write_format=""
url=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -o) output_file="$2"; shift 2 ;;
    -w) write_format="$2"; shift 2 ;;
    -H|-X|--data|--data-binary|--connect-timeout|--max-time) shift 2 ;;
    -sS) shift ;;
    http://*) url="$1"; shift ;;
    *) shift ;;
  esac
done
case "$url" in
  */recordings)
    body='{"recordings":[],"count":0}'; code=200 ;;
  */replay/status)
    body='{"mode":"idle","read_only":false,'
    body+='"recording":{"status":"recording",'
    body+='"recording_id":"leak_probe"},'
    body+='"playback":null,"task_replay":null}'; code=200 ;;
  */recording/start)
    body='{"recording_id":"leak_probe"}'; code=202 ;;
  */recording/stop)
    printf '%s\n' "$url" >>"$CURL_LOG"
    body='{"status":"stopped"}'; code=202 ;;
  *)
    body='{"error":"unexpected test URL"}'; code=500 ;;
esac
if [[ -n "$output_file" ]]; then
  printf '%s' "$body" >"$output_file"
else
  printf '%s' "$body"
fi
if [[ -n "$write_format" ]]; then
  printf '%s' "$code"
fi
""",
                encoding="utf-8",
            )
            fake_curl.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                CURL_LOG=str(curl_log),
                XCZS_CONTROL_API_URL="http://127.0.0.1:9",
                XCZS_CONTROL_TOKEN="",
                XCZS_CONTROL_USERNAME="",
                XCZS_CONTROL_PASSWORD="",
                XCZS_ADMIN_PASSWORD="",
            )
            result = subprocess.run(
                [str(RECORDING_VALIDATOR)],
                text=True,
                capture_output=True,
                env=environment,
                timeout=10,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("期望 HTTP 400，实际 202", result.stderr)
            self.assertEqual(
                curl_log.read_text(encoding="utf-8").strip(),
                "http://127.0.0.1:9/recording/stop",
            )

    def test_recording_validator_does_not_stop_a_foreign_recording(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-validator-test-") as tmp:
            tmp_path = Path(tmp)
            curl_log = tmp_path / "curl.log"
            fake_curl = tmp_path / "curl"
            fake_curl.write_text(
                """#!/usr/bin/env bash
set -euo pipefail
output_file=""
write_format=""
url=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -o) output_file="$2"; shift 2 ;;
    -w) write_format="$2"; shift 2 ;;
    -H|-X|--data|--data-binary|--connect-timeout|--max-time) shift 2 ;;
    -sS|-fsS) shift ;;
    http://*) url="$1"; shift ;;
    *) shift ;;
  esac
done
case "$url" in
  */recordings)
    body='{"recordings":[],"count":0}'; code=200 ;;
  */replay/status)
    body='{"mode":"idle","read_only":false,'
    body+='"recording":{"status":"recording",'
    body+='"recording_id":"foreign_recording"},'
    body+='"playback":null,"task_replay":null}'; code=200 ;;
  */recording/start)
    body='{"recording_id":"our_probe"}'; code=202 ;;
  */recording/stop)
    printf '%s\\n' "$url" >>"$CURL_LOG"
    body='{"status":"stopped"}'; code=202 ;;
  *)
    body='{"error":"unexpected test URL"}'; code=500 ;;
esac
if [[ -n "$output_file" ]]; then
  printf '%s' "$body" >"$output_file"
else
  printf '%s' "$body"
fi
if [[ -n "$write_format" ]]; then
  printf '%s' "$code"
fi
""",
                encoding="utf-8",
            )
            fake_curl.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                CURL_LOG=str(curl_log),
                XCZS_CONTROL_API_URL="http://127.0.0.1:9",
                XCZS_CONTROL_TOKEN="",
                XCZS_CONTROL_USERNAME="",
                XCZS_CONTROL_PASSWORD="",
                XCZS_ADMIN_PASSWORD="",
            )
            result = subprocess.run(
                [str(RECORDING_VALIDATOR)],
                text=True,
                capture_output=True,
                env=environment,
                timeout=10,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("当前活动录制 foreign_recording 不属于本脚本", result.stderr)
            self.assertFalse(curl_log.exists())

    def test_recording_validator_does_not_stop_without_an_owned_id(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-validator-test-") as tmp:
            tmp_path = Path(tmp)
            curl_log = tmp_path / "curl.log"
            fake_curl = tmp_path / "curl"
            fake_curl.write_text(
                """#!/usr/bin/env bash
set -euo pipefail
output_file=""
write_format=""
url=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -o) output_file="$2"; shift 2 ;;
    -w) write_format="$2"; shift 2 ;;
    -H|-X|--data|--data-binary|--connect-timeout|--max-time) shift 2 ;;
    -sS|-fsS) shift ;;
    http://*) url="$1"; shift ;;
    *) shift ;;
  esac
done
case "$url" in
  */recordings)
    body='{"recordings":[],"count":0}'; code=200 ;;
  */replay/status)
    body='{"mode":"idle","read_only":false,"recording":null,'
    body+='"playback":null,"task_replay":null}'; code=200 ;;
  */recording/start)
    body='not-json'; code=202 ;;
  */recording/stop)
    printf '%s\\n' "$url" >>"$CURL_LOG"
    body='{"status":"stopped"}'; code=202 ;;
  *)
    body='{"error":"unexpected test URL"}'; code=500 ;;
esac
if [[ -n "$output_file" ]]; then
  printf '%s' "$body" >"$output_file"
else
  printf '%s' "$body"
fi
if [[ -n "$write_format" ]]; then
  printf '%s' "$code"
fi
""",
                encoding="utf-8",
            )
            fake_curl.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                CURL_LOG=str(curl_log),
                XCZS_CONTROL_API_URL="http://127.0.0.1:9",
                XCZS_CONTROL_TOKEN="",
                XCZS_CONTROL_USERNAME="",
                XCZS_CONTROL_PASSWORD="",
                XCZS_ADMIN_PASSWORD="",
            )
            result = subprocess.run(
                [str(RECORDING_VALIDATOR)],
                text=True,
                capture_output=True,
                env=environment,
                timeout=10,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("响应缺少 recording_id", result.stderr)
            self.assertFalse(curl_log.exists())

    def test_recording_validator_cancels_malformed_accepted_replay(self) -> None:
        with tempfile.TemporaryDirectory(prefix="xczs-validator-test-") as tmp:
            tmp_path = Path(tmp)
            curl_log = tmp_path / "curl.log"
            fake_curl = tmp_path / "curl"
            fake_curl.write_text(
                """#!/usr/bin/env bash
set -euo pipefail
output_file=""
write_format=""
url=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -o) output_file="$2"; shift 2 ;;
    -w) write_format="$2"; shift 2 ;;
    -H|-X|--data|--data-binary|--connect-timeout|--max-time) shift 2 ;;
    -sS) shift ;;
    http://*) url="$1"; shift ;;
    *) shift ;;
  esac
done
case "$url" in
  */recordings)
    body='{"recordings":[],"count":0}'; code=200 ;;
  */replay/status)
    body='{"mode":"idle","read_only":false,"recording":null,'
    body+='"playback":null,"task_replay":null}'; code=200 ;;
  */recording/start|*/recording/stop|*/replay/data/pause|\
  */replay/data/resume|*/replay/data/rate|*/replay/task/start)
    body='{"detail":"invalid"}'; code=400 ;;
  */replay/data/start)
    body='not-json'; code=202 ;;
  */replay/cancel)
    printf '%s\n' "$url" >>"$CURL_LOG"
    body='{"status":"canceled"}'; code=202 ;;
  *)
    body='{"error":"unexpected test URL"}'; code=500 ;;
esac
if [[ -n "$output_file" ]]; then
  printf '%s' "$body" >"$output_file"
else
  printf '%s' "$body"
fi
if [[ -n "$write_format" ]]; then
  printf '%s' "$code"
fi
""",
                encoding="utf-8",
            )
            fake_curl.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                PATH=f"{tmp}:{environment['PATH']}",
                CURL_LOG=str(curl_log),
                XCZS_CONTROL_API_URL="http://127.0.0.1:9",
                XCZS_CONTROL_TOKEN="",
                XCZS_CONTROL_USERNAME="",
                XCZS_CONTROL_PASSWORD="",
                XCZS_ADMIN_PASSWORD="",
            )
            result = subprocess.run(
                [str(RECORDING_VALIDATOR)],
                text=True,
                capture_output=True,
                env=environment,
                timeout=10,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("返回的不是有效 JSON", result.stderr)
            self.assertEqual(
                curl_log.read_text(encoding="utf-8").strip(),
                "http://127.0.0.1:9/replay/cancel",
            )


if __name__ == "__main__":
    unittest.main()
