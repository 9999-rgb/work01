from __future__ import annotations

import os
import socket
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
STARTUP = ROOT / "jiang" / "start_xczs_bridge.sh"
WEB_VALIDATOR = ROOT / "scripts" / "validate_cabinet_web"
RECORDING_VALIDATOR = ROOT / "scripts" / "validate_recording_replay"


class StartupScriptContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.startup_source = STARTUP.read_text(encoding="utf-8")
        cls.validator_source = WEB_VALIDATOR.read_text(encoding="utf-8")
        cls.recording_validator_source = RECORDING_VALIDATOR.read_text(
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
        self.assertIn('if [ ! -x "$ZENOH_BRIDGE" ]', self.startup_source)
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
            "基于渲染的 RGB 相机也不会发布图像",
            self.startup_source,
        )

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
            CONTROL_PORT=str(ports[2]),
            CONTROL_HOST="127.0.0.1",
            XCZS_CONTROL_ORIGINS=(
                f"http://localhost:{ports[2]},"
                f"http://127.0.0.1:{ports[2]}"
            ),
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

    def test_launcher_only_stops_its_registered_process_groups(self) -> None:
        self.assertNotIn("pkill", self.startup_source)
        self.assertIn("MANAGED_PIDS", self.startup_source)
        self.assertIn("MANAGED_PGIDS", self.startup_source)
        self.assertIn(
            '_signal_process_group "$process_group" TERM', self.startup_source
        )
        self.assertNotIn('kill "-$signal" "$pid"', self.startup_source)
        self.assertIn(
            "setsid ros2 launch xczs_inspection_robot_control",
            self.startup_source,
        )
        self.assertIn('_register_process "$LAUNCH_PID"', self.startup_source)
        self.assertIn("_monitor_managed_processes", self.startup_source)
        self.assertNotIn("ROS 2 launch 已正常结束", self.startup_source)
        self.assertIn(
            'echo "ERROR: ${MANAGED_LABELS[$index]} 意外退出（status=$status）。"',
            self.startup_source,
        )
        launch_start = self.startup_source.index("setsid ros2 launch")
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
            '"$API_URL/recording/stop"',
            self.recording_validator_source,
        )

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
    body='{"mode":"idle","read_only":false,"recording":null,"playback":null,"task_replay":null}'; code=200 ;;
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


if __name__ == "__main__":
    unittest.main()
