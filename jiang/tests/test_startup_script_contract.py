from __future__ import annotations

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
            '_require_port_available 0.0.0.0 "$BRIDGE_TCP_PORT"'
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

    def test_launcher_only_stops_its_registered_process_groups(self) -> None:
        self.assertNotIn("pkill", self.startup_source)
        self.assertIn("MANAGED_PIDS", self.startup_source)
        self.assertIn('_signal_process_group "$pid" TERM', self.startup_source)
        self.assertIn(
            "setsid ros2 launch xczs_inspection_robot_control",
            self.startup_source,
        )
        self.assertIn('_register_process "$LAUNCH_PID"', self.startup_source)
        self.assertIn("_monitor_managed_processes", self.startup_source)
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


if __name__ == "__main__":
    unittest.main()
