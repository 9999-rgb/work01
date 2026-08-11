from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

from cabinet_validation_targets import (  # noqa: E402
    NoAlternativeTargetError,
    select_non_noop_target_state,
)


class CabinetValidationTargetTests(unittest.TestCase):
    def test_selects_state_different_from_live_state(self) -> None:
        target = select_non_noop_target_state(
            ["left", "center", "right"],
            "left",
            control_id="box_7_knob",
        )

        self.assertEqual("center", target)

    def test_rejects_catalog_without_alternative_state(self) -> None:
        with self.assertRaisesRegex(
            NoAlternativeTargetError,
            "NOT APPLICABLE.*no-op reachability validation",
        ):
            select_non_noop_target_state(
                ["closed", "closed"],
                "closed",
                control_id="cabinet_rear_door",
            )

    def test_rejects_live_state_outside_catalog(self) -> None:
        with self.assertRaisesRegex(
            NoAlternativeTargetError,
            "NOT APPLICABLE.*no advertised live current_state",
        ):
            select_non_noop_target_state(
                ["off", "on"],
                "unknown",
                control_id="cabinet_side_switch",
            )

    def test_web_validator_uses_live_non_noop_target(self) -> None:
        source = (SCRIPTS / "validate_cabinet_web").read_text(
            encoding="utf-8"
        )
        selector_start = source.index("operation_body_for_control()")
        selector_end = source.index(
            "\n}\n\nrun_unreachable_operation()",
            selector_start,
        )
        selector = source[selector_start:selector_end]
        runner_start = selector_end
        runner_end = source.index("\n}\n\nvalidate_sse_capture()", runner_start)
        runner = source[runner_start:runner_end]

        self.assertIn('get_json "/cabinets/$CABINET/controls"', selector)
        self.assertIn(".current_state", selector)
        self.assertIn(".state_ids[] | select(. != $current)", selector)
        self.assertNotIn(".state_ids[0]", selector)
        self.assertIn("NOT APPLICABLE", selector)
        self.assertLess(
            runner.index('navigate_for_control "$control_id"'),
            runner.index('operation_body_for_control "$control_id"'),
        )

    def test_web_validator_covers_limited_knob_command_matrix(self) -> None:
        source = (SCRIPTS / "validate_cabinet_web").read_text(
            encoding="utf-8"
        )
        contract_start = source.index(
            "run_unavailable_knob_command_contracts()"
        )
        contract_end = source.index(
            "\n}\n\nvalidate_sse_capture()",
            contract_start,
        )
        contract = source[contract_start:contract_end]

        self.assertIn("set_position false", contract)
        self.assertIn("toggle false", contract)
        self.assertIn("run_unavailable_knob_command_contracts \\", source)
        self.assertIn(".result.validation_performed == true", source)
        self.assertIn(".result.operation_executed == false", source)

    def test_web_validator_snapshots_all_controls_for_crosstalk(self) -> None:
        source = (SCRIPTS / "validate_cabinet_web").read_text(
            encoding="utf-8"
        )
        snapshot_start = source.index("cabinet_controls_snapshot()")
        snapshot_end = source.index(
            "\n}\n\nrun_successful_operation()",
            snapshot_start,
        )
        snapshot_contract = source[snapshot_start:snapshot_end]

        self.assertIn("length == 33", snapshot_contract)
        self.assertIn("position:.current_position", snapshot_contract)
        self.assertNotIn("position:.position", snapshot_contract)
        self.assertIn("transition_sequence", snapshot_contract)
        self.assertIn("$delta >= -0.0001", snapshot_contract)
        self.assertIn(
            'jq -ne --arg excluded "$excluded_control"',
            snapshot_contract,
        )
        self.assertIn(
            'assert_cabinet_controls_unchanged "$state_before" "$control_id"',
            source,
        )
        self.assertIn(
            'assert_cabinet_controls_unchanged "$states_before"',
            source,
        )

    def test_low_level_matrix_requires_one_prepositioned_control(self) -> None:
        source = (SCRIPTS / "validate_cabinet_simulation").read_text(
            encoding="utf-8"
        )

        self.assertIn("if len(args.control) > 1:", source)
        self.assertIn("if args.exhaustive and not args.inventory_only:", source)
        self.assertIn(
            "Low-level --exhaustive motion is unsafe without Nav2",
            source,
        )
        self.assertIn("selected_knobs = [", source)
        self.assertIn("for entry in controls", source)
        self.assertIn("COMMAND_SET_POSITION", source)
        self.assertIn("COMMAND_TOGGLE", source)
        self.assertIn("require_planning_validation=True", source)


if __name__ == "__main__":
    unittest.main()
