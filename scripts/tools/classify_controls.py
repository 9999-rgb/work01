#!/usr/bin/env python3
"""Classify each cabinet control's physical reachability via the planning-only
Action.

For every requested control it teleports the robot base onto the control's dock
station (``preposition_base.teleport_to_station``) and then sends ONE non-noop
goal with ``navigate_to_staging_pose=False``.  The operator's planning-only
branch (``validate_inoperable_control_path``) validates the full
ready -> approach -> manipulate -> retreat -> transport chain against live
MoveIt / TF / planning scene but never executes a trajectory.

No motion is ever sent: the teleport is a Gazebo entity-state write, and every
Action that this script triggers is planning-only by construction.

Per control it prints one ``ID<TAB>CLASS<TAB>stage<TAB>frac<TAB>err<TAB>...``
row plus the operator's reason text.  CLASS values:

    ACHIEVABLE      chain validated to completion (fraction >= 0.999)
    FAIL@<stage>    planning-only chain stopped at a specific diagnostic stage
    UNREACHABLE     operator aborted before completing the chain
    TOOLSET_MISMATCH  tool not served for this control
    NO_STATION      control has no navigation station configured
    ERROR           unexpected failure / time-out

Usage:
    scripts/tools/classify_controls.py [--controls box_3_knob,box_4_knob]
                                 [--all-unavailable] [--cabinet cabinet_a]
                                 [--toolset B] [--settle 6.0]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "jiang"))

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from xczs_inspection_robot_interfaces.action import OperateCabinetControl
from xczs_inspection_robot_interfaces.msg import CabinetControl
from xczs_inspection_robot_interfaces.msg import CabinetControlCatalog
from xczs_inspection_robot_interfaces.msg import CabinetControlState

from cabinet_validation_targets import select_non_noop_target_state
from preposition_base import teleport_to_station

import control_gateway.robot_adapter as robot_adapter  # noqa: E402

DEFAULT_ADAPTER = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "config"
    / "cabinet_robot_adapter.yaml"
)
ROBOT_ENTITY = "xczs_inspection_robot"


class Classifier(Node):
    def __init__(
        self,
        *,
        cabinet: str,
        adapter_path: str,
        toolset: str,
    ) -> None:
        self.cabinet = cabinet
        self.namespace = f"/xczs/cabinet/{cabinet}"
        self.cabinet_frame = f"{cabinet}_frame"
        super().__init__(f"xczs_{cabinet}_control_classifier")
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.catalog: CabinetControlCatalog | None = None
        self.states: dict[str, CabinetControlState] = {}
        self.state_received_at: dict[str, float] = {}
        self._state_subscriptions = []
        self.create_subscription(
            CabinetControlCatalog,
            f"{self.namespace}/control_catalog",
            self._catalog_callback,
            retained,
        )
        self.client = ActionClient(
            self,
            OperateCabinetControl,
            f"{self.namespace}/operate_cabinet_control",
        )
        self._retained_qos = retained
        self.adapter = robot_adapter.load(adapter_path, toolset=toolset)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _catalog_callback(self, message: CabinetControlCatalog) -> None:
        if self.catalog is not None:
            return
        self.catalog = message
        for control in message.controls:
            state_topic = self._resolve_cabinet_topic(control.state_topic)
            self._state_subscriptions.append(
                self.create_subscription(
                    CabinetControlState,
                    state_topic,
                    lambda state, selected_id=control.control_id: (
                        self._state_callback(selected_id, state)
                    ),
                    self._retained_qos,
                )
            )

    def _resolve_cabinet_topic(self, topic: str) -> str:
        value = str(topic).strip()
        if not value:
            raise RuntimeError("Catalog contains an empty cabinet topic.")
        if value.startswith("/"):
            return value
        return f"{self.namespace}/{value.lstrip('/')}"

    def _state_callback(
        self, control_id: str, state: CabinetControlState
    ) -> None:
        if state.control_id == control_id:
            self.states[control_id] = state
            self.state_received_at[control_id] = time.monotonic()

    def _spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_catalog(self, timeout: float) -> CabinetControlCatalog:
        deadline = time.monotonic() + timeout
        while self.catalog is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.25)
        if self.catalog is None:
            raise RuntimeError("Control catalog was never received.")
        return self.catalog

    def wait_for_fresh_state(
        self, control_id: str, timeout: float
    ) -> CabinetControlState:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.25)
            received = self.state_received_at.get(control_id, 0.0)
            if (
                control_id in self.states
                and time.monotonic() - received < 5.0
            ):
                return self.states[control_id]
        raise RuntimeError(
            f"No fresh valid state for '{control_id}' within {timeout:.1f}s."
        )

    def wait_for_base_settled(
        self, world_x: float, world_y: float, timeout: float
    ) -> None:
        """Wait until odom->base_link x/y matches the teleported pose."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tf = self.tf_buffer.lookup_transform(
                    "odom", "base_link", Time()
                )
                dx = abs(tf.transform.translation.x - world_x)
                dy = abs(tf.transform.translation.y - world_y)
                if dx < 0.05 and dy < 0.05:
                    return
            except TransformException:
                pass
        raise RuntimeError(
            f"Robot base did not settle near teleported pose "
            f"({world_x:.3f}, {world_y:.3f})."
        )

    def _build_goal(
        self, control: CabinetControl, live_state: CabinetControlState
    ) -> OperateCabinetControl.Goal:
        goal = OperateCabinetControl.Goal()
        goal.control_id = control.control_id
        goal.command = OperateCabinetControl.Goal.COMMAND_PRESS
        goal.force = float(control.default_force)
        goal.navigate_to_staging_pose = False
        if control.control_type != CabinetControl.TYPE_BUTTON:
            target_state = select_non_noop_target_state(
                control.state_ids,
                live_state.state_id,
                control_id=str(control.control_id),
            )
            goal.command = OperateCabinetControl.Goal.COMMAND_SET_STATE
            goal.target_state = target_state
        return goal

    def classify(
        self, control_id: str, settle: float,
        standoff_override: float | None = None,
    ) -> str:
        station = self.adapter.control_navigation_station(control_id)
        if station is None:
            return f"{control_id}\tNO_STATION\t-\t-\t-"
        control = next(
            entry
            for entry in self.catalog.controls
            if entry.control_id == control_id
        )
        live_state = self.wait_for_fresh_state(control_id, 10.0)
        world_x, world_y, _z, _body_yaw, _base_yaw = teleport_to_station(
            self,
            station=station,
            cabinet_frame=self.cabinet_frame,
            control_id=control_id,
            standoff_override=standoff_override,
        )
        self.wait_for_base_settled(world_x, world_y, 20.0)
        self._spin(settle)

        goal = self._build_goal(control, live_state)
        if not self.client.wait_for_server(timeout_sec=15.0):
            return f"{control_id}\tERROR\t-\t-\t- (action server unavailable)"
        send_future = self.client.send_goal_async(goal)
        deadline = time.monotonic() + 120.0
        while not send_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send_future.done():
            send_future.cancel()
            return f"{control_id}\tERROR\t-\t-\t- (goal send timed out)"
        handle = send_future.result()
        if handle is None or not handle.accepted:
            status = "rejected" if handle is None else "not accepted"
            return f"{control_id}\tERROR\t-\t-\t- ({status})"
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 180.0
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=3.0)
            return f"{control_id}\tERROR\t-\t-\t- (result timed out)"

        result = result_future.result().result
        stage = str(getattr(result, "diagnostic_stage", "")).strip()
        frac = float(getattr(result, "path_fraction", math.nan))
        req = float(getattr(result, "required_fraction", math.nan))
        err = int(getattr(result, "error_code", -1))
        validation = bool(getattr(result, "validation_performed", False))
        executed = bool(getattr(result, "operation_executed", True))
        policy = str(getattr(result, "policy_reason", "")).strip()
        message = str(getattr(result, "message", "")).strip().replace("\n", " ")

        if not validation and err == OperateCabinetControl.Result.TOOLSET_MISMATCH:
            cls = "TOOLSET_MISMATCH"
        elif validation and stage == "complete" and frac >= 0.999:
            cls = "ACHIEVABLE"
        elif validation:
            cls = f"FAIL@{stage}" if stage else "FAIL"
        elif err == OperateCabinetControl.Result.UNREACHABLE:
            cls = "UNREACHABLE"
        elif err == OperateCabinetControl.Result.PLANNING_FAILED:
            cls = "PLANNING_FAILED"
        else:
            cls = "OTHER"
        frac_text = f"{frac:.3f}" if math.isfinite(frac) else "?"
        req_text = f"{req:.3f}" if math.isfinite(req) else "?"
        executed_text = "EXEC" if executed else "PLAN"
        return (
            f"{control_id}\t{cls}\t{stage}\t{frac_text}/{req_text}\t"
            f"{err}\t{executed_text}"
            + (f"\tpolicy={policy}" if policy else "")
            + (f"\tmsg={message[:140]}" if message else "")
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--controls",
        help="comma-separated control ids to classify",
    )
    parser.add_argument(
        "--all-unavailable",
        action="store_true",
        help="classify every non-operable control with a station",
    )
    parser.add_argument(
        "--cabinet", default="cabinet_a", help="cabinet instance name"
    )
    parser.add_argument(
        "--toolset", default="B", choices=("A", "B"), help="active toolset"
    )
    parser.add_argument(
        "--adapter",
        default=str(DEFAULT_ADAPTER),
        help="cabinet_robot_adapter.yaml path",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=6.0,
        help="seconds to wait after base settling before planning",
    )
    parser.add_argument(
        "--standoff",
        type=float,
        default=None,
        help="override the adapter station standoff for this run",
    )
    parser.add_argument(
        "--standoff-sweep",
        default="",
        help="comma-separated standoff values to sweep, e.g. 0.65,0.70,0.77,0.85",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    rclpy.init()
    node = Classifier(
        cabinet=args.cabinet,
        adapter_path=args.adapter,
        toolset=args.toolset,
    )
    try:
        catalog = node.wait_for_catalog(20.0)
        if args.controls:
            control_ids = [c.strip() for c in args.controls.split(",") if c.strip()]
        elif args.all_unavailable:
            control_ids = [
                str(entry.control_id)
                for entry in catalog.controls
                if not entry.operable
            ]
        else:
            control_ids = [
                str(entry.control_id)
                for entry in catalog.controls
                if not entry.operable
            ]
        if not control_ids:
            print("No controls selected.")
            return 0
        if args.standoff_sweep:
            standoffs = [
                float(value.strip())
                for value in args.standoff_sweep.split(",")
                if value.strip()
            ]
        else:
            standoffs = [args.standoff]
        print(f"\n=== CLASSIFICATION (standoffs: {standoffs}) ===")
        for standoff in standoffs:
            if len(control_ids) > 1:
                print(f"\n--- standoff={standoff} ---")
            for control_id in control_ids:
                row = node.classify(
                    control_id, args.settle, standoff_override=standoff
                )
                print(row)
        return 0
    except Exception as error:  # noqa: BLE001 - CLI must always surface the cause
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
