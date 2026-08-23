"""Regression contract for isolated physical cabinet-knob grasping."""

from __future__ import annotations

import math
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
CABINET_XACROS = (
    WORKSPACE
    / "xczs_inspection_robot_description"
    / "urdf"
    / "control_cabinet.urdf.xacro",
    WORKSPACE
    / "jiang"
    / "samples"
    / "demo_cabinet"
    / "control_cabinet.urdf.xacro",
)
ROBOT_ADAPTERS = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "config"
    / "cabinet_robot_adapter.yaml",
    WORKSPACE / "jiang" / "samples" / "demo_cabinet" / "cabinet_robot_adapter.yaml",
)
SCENE_CONFIGS = (
    WORKSPACE / "xczs_inspection_robot_control" / "config" / "cabinet_scene.yaml",
    WORKSPACE / "jiang" / "samples" / "demo_cabinet" / "cabinet_scene.yaml",
)
OPERATOR_SOURCE = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "src"
    / "cabinet_button_operator.cpp"
)
PLUGIN_SOURCE = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "src"
    / "gazebo"
    / "cabinet_state_plugin.cpp"
)
PLANNING_SCENE_SOURCE = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "src"
    / "cabinet_planning_scene.cpp"
)


def _expanded_cabinet(path: Path) -> ET.Element:
    completed = subprocess.run(
        ["xacro", str(path), "cabinet_name:=knob_grasp_contract"],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(completed.stdout)


@pytest.mark.parametrize("cabinet_xacro", CABINET_XACROS)
def test_every_knob_uses_compliant_twist_coupling(
    cabinet_xacro: Path,
) -> None:
    """No knob may fall back to the cross-model rigid-joint path."""
    root = _expanded_cabinet(cabinet_xacro)
    plugin = next(
        element
        for element in root.findall(".//plugin")
        if element.get("filename") == "libxczs_cabinet_state.so"
    )
    knobs = [
        control
        for control in plugin.findall("control")
        if control.findtext("control_type") == "knob"
    ]
    assert len(knobs) == 11

    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    for knob in knobs:
        control_id = knob.findtext("control_id")
        values = {
            name: float(knob.findtext(name, "nan"))
            for name in (
                "detent_stiffness",
                "grasp_coupling_stiffness",
                "grasp_coupling_damping",
                "grasp_coupling_max_effort",
            )
        }
        assert all(math.isfinite(value) for value in values.values()), control_id
        assert values["grasp_coupling_stiffness"] > 0.0, control_id
        assert values["grasp_coupling_damping"] > 0.0, control_id
        assert values["grasp_coupling_max_effort"] > 0.0, control_id

        detents = [float(value) for value in knob.findtext("detents", "").split()]
        assert len(detents) == 3 and detents[0] < detents[1] < detents[2]
        midpoint = 0.5 * (detents[1] + detents[2])
        joint = joints[knob.findtext("joint_name")]
        friction = float(joint.find("dynamics").get("friction", "0"))

        # At the center/right boundary, the wrist coupling must still beat
        # the old detent's restoring torque and joint friction.  Otherwise the
        # physical knob can never cross into the requested right detent.
        available_torque = min(
            values["grasp_coupling_max_effort"],
            values["grasp_coupling_stiffness"] * (detents[2] - midpoint),
        )
        resisting_torque = (
            values["detent_stiffness"] * abs(midpoint - detents[1])
            + friction
        )
        assert available_torque > resisting_torque, control_id


def _operator_parameters(path: Path) -> dict[str, object]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document["/**/xczs_cabinet_button_operator"]["ros__parameters"]


def test_box_5_knob_keeps_safe_standoff_for_planning_only_calibration() -> None:
    builtin = _operator_parameters(ROBOT_ADAPTERS[0])
    sample = _operator_parameters(ROBOT_ADAPTERS[1])
    assert builtin == sample

    controls = builtin["controls"]
    assert isinstance(controls, dict)
    calibration = controls["box_5_knob"]
    assert calibration["navigation_station"]["local_anchor"] == [
        0.717470,
        1.115,
        0.0,
    ]
    # The chassis half-extent along the cabinet normal is about 0.426 m.
    # Never trade base/cabinet clearance for an otherwise reachable arm IK.
    assert calibration["navigation_station"]["standoff"] == 0.769
    assert calibration["navigation_station"]["standoff"] > 0.426
    # 0.75 of the 0.785 rad travel puts the tool just past the center/right
    # midpoint (0.589 rad vs 0.393 rad); the detent finishes the travel on
    # release, so the wrist never needs a full-angle Cartesian arc.
    assert calibration["detent_release_fraction"] == 0.75
    assert calibration["ready_joint_seed"] == {
        "joint_names": [f"r_arm_{index}_joint" for index in range(7)],
        "positions": [
            1.221067,
            1.158552,
            -1.448952,
            -1.806018,
            1.998380,
            0.014786,
            -1.656403,
        ],
    }

    # All-zero rolls in the toward_control orientation: with tool +Z pointing
    # at the control, roll=0 keeps the jaw opening axis (tool +Y) parallel to
    # the knob-blade normal (world +-Y), so the 11 mm blade slips into the
    # 36 mm jaw without contact.  The historical non-zero rolls were calibrated
    # along the old outward axis, which turned the jaws vertical (world +-Z)
    # and made the 65 mm blade unable to enter the 36 mm vertical gap.
    assert calibration["tool_roll_offsets"] == [0.0] * 9


def test_operation_watchdog_fits_between_heartbeat_and_lease_expiry() -> None:
    parameters = [_operator_parameters(path) for path in ROBOT_ADAPTERS]
    assert parameters[0] == parameters[1]
    lease_duration = float(parameters[0]["operation_lease_duration"])
    renew_period = float(parameters[0]["operation_lease_renew_period"])

    watchdog_timeouts = []
    for cabinet_xacro in CABINET_XACROS:
        root = _expanded_cabinet(cabinet_xacro)
        plugin = next(
            element
            for element in root.findall(".//plugin")
            if element.get("filename") == "libxczs_cabinet_state.so"
        )
        timeout = float(plugin.findtext("operation_watchdog_timeout", "nan"))
        assert math.isfinite(timeout) and timeout > 0.0
        assert plugin.findtext("operation_heartbeat_topic") == (
            "/xczs/cabinet/knob_grasp_contract/operation_heartbeat"
        )
        assert plugin.findtext("operation_fault_topic") == (
            "/xczs/cabinet/knob_grasp_contract/operation_fault"
        )
        watchdog_timeouts.append(timeout)

    request_timeout = float(parameters[0]["operation_lease_request_timeout"])
    scene_timeouts = []
    for scene_config in SCENE_CONFIGS:
        document = yaml.safe_load(scene_config.read_text(encoding="utf-8"))
        scene_parameters = document["/**/xczs_cabinet_planning_scene"][
            "ros__parameters"
        ]
        scene_timeouts.append(float(scene_parameters["operation_watchdog_timeout"]))

    assert watchdog_timeouts == [2.0, 2.0]
    assert scene_timeouts == watchdog_timeouts
    assert renew_period + request_timeout < watchdog_timeouts[0]
    assert watchdog_timeouts[0] < lease_duration


def test_watchdog_heartbeat_and_physics_cleanup_order_is_explicit() -> None:
    operator = OPERATOR_SOURCE.read_text(encoding="utf-8")
    renew_start = operator.index("  void renew_operation_lease_once()")
    renew_end = operator.index(
        "  void mark_operation_lease_lost(", renew_start
    )
    renew = operator[renew_start:renew_end]
    assert "publish_operation_heartbeat();" in renew
    assert (
        "kOperationHeartbeatTopic, rclcpp::QoS(1).reliable());"
        in operator
    )
    assert "message.data = lease_id;" in operator
    assert (
        operator.count("request->operation_lease_id = operation_lease_id_;")
        == 2
    )
    assert (
        "kOperationFaultTopic, rclcpp::QoS(1).reliable()," in operator
    )
    fault_subscribe = operator.index("kOperationFaultTopic")
    fault_callback_end = operator.index(
        "navigation_client_ =", fault_subscribe
    )
    fault_callback = operator[fault_subscribe:fault_callback_end]
    assert fault_callback.index("if (message->data.empty())") < (
        fault_callback.index("mark_operation_lease_lost_for_exact_lease(")
    )
    mark_lost_start = operator.index("  void mark_operation_lease_lost(")
    mark_lost_end = operator.index(
        "  void release_operation_lease_noexcept()", mark_lost_start
    )
    mark_lost = operator[mark_lost_start:mark_lost_end]
    assert "operation_fault_matches_active_lease(" in mark_lost
    assert "std::lock_guard<std::mutex> lock(operation_lease_mutex_);" in mark_lost
    assert "const std::string * expected_lease_id" in mark_lost

    heartbeat_start = operator.index("  void publish_operation_heartbeat()")
    heartbeat_end = operator.index(
        "  void publish_active_control_message(", heartbeat_start
    )
    heartbeat = operator[heartbeat_start:heartbeat_end]
    assert heartbeat.count("mark_operation_lease_lost_for_exact_lease(") == 2
    assert heartbeat.count("lease_id);") == 2

    active_publish_start = heartbeat_end
    active_publish_end = operator.index(
        "  void reset_controls(", active_publish_start
    )
    active_publish = operator[active_publish_start:active_publish_end]
    assert "const std::string & operation_lease_id" in active_publish
    assert active_publish.count("if (!control_id.empty())") == 2
    assert active_publish.count(
        "mark_operation_lease_lost_for_exact_lease("
    ) == 2
    assert active_publish.count("operation_lease_id);") == 2

    plugin = PLUGIN_SOURCE.read_text(encoding="utf-8")
    assert (
        "operation_heartbeat_topic_, rclcpp::QoS(1).reliable(),"
        in plugin
    )
    assert (
        "operation_fault_topic_, rclcpp::QoS(1).reliable());" in plugin
    )
    destructor_start = plugin.index("  ~CabinetStatePlugin() override")
    destructor_end = plugin.index("  void Load(", destructor_start)
    destructor = plugin[destructor_start:destructor_end]
    disconnect = destructor.index("update_connection_.reset();")
    join_update = destructor.index(
        "std::lock_guard<std::mutex> physics_lock(physics_callback_mutex_);"
    )
    reset_fault_publisher = destructor.index(
        "operation_fault_publisher_.reset();"
    )
    assert disconnect < join_update < reset_fault_publisher
    update_start = plugin.index("  void on_update()")
    update_end = plugin.index("  bool grasp_is_active() const", update_start)
    update = plugin[update_start:update_end]
    assert update.index("enforce_operation_watchdog();") < update.index(
        "process_physics_requests();"
    )

    watchdog_start = plugin.index("  void enforce_operation_watchdog()")
    watchdog_end = plugin.index(
        "  bool actuation_tool_is_clear(", watchdog_start
    )
    watchdog = plugin[watchdog_start:watchdog_end]
    assert "operation_watchdog_has_expired(" in watchdog
    assert "active_control_generation_ != active_control_generation" in watchdog
    assert "operation_heartbeat_sequence_ != heartbeat_sequence" in watchdog
    assert "current_session_heartbeat" in watchdog
    assert "active_grasp_control_generation_ != active_control_generation" in watchdog
    assert "active_operation_control_.clear();" in watchdog
    assert "reset_pregrasp_tracking_locked();" in watchdog
    assert "watchdog_recovery_pending_" in watchdog
    assert watchdog.index("publish_operation_fault(activity_lease_id);") < (
        watchdog.rindex("continue_watchdog_recovery(now);")
    )
    assert "restore_all_actuation_collisions();" not in watchdog

    recovery_start = plugin.index("  void log_watchdog_release_failure(")
    recovery_end = plugin.index("  void enforce_operation_watchdog()")
    recovery = plugin[recovery_start:recovery_end]
    assert "release_grasp_constraint();" in recovery
    assert "catch (const std::exception & error)" in recovery
    assert "kWatchdogReleaseRetryDelay" in recovery
    assert "collision_restore_robot_link.reset()" not in recovery
    assert "restore_all_actuation_collisions();" not in recovery

    assert "kExpiredLeaseHistoryLimit = 64U" in plugin
    assert (
        "expired_operation_lease_history_.size() >\n"
        "      kExpiredLeaseHistoryLimit" in plugin
    )
    assert "remember_expired_operation_lease_locked(activity_lease_id);" in watchdog

    active_receive_start = plugin.index("  void receive_active_control(")
    heartbeat_receive_start = plugin.index(
        "  void receive_operation_heartbeat(", active_receive_start
    )
    active_receive = plugin[active_receive_start:heartbeat_receive_start]
    retire_old_lease = active_receive.index(
        "remember_expired_operation_lease_locked("
    )
    clear_old_lease = active_receive.index(
        "operation_heartbeat_lease_id_.clear();"
    )
    assert retire_old_lease < clear_old_lease

    heartbeat_receive_end = plugin.index(
        "  void remember_expired_operation_lease_locked(",
        heartbeat_receive_start,
    )
    heartbeat_receive = plugin[
        heartbeat_receive_start:heartbeat_receive_end
    ]
    assert "operation_heartbeat_lease_id_ != operation_lease_id" in (
        heartbeat_receive
    )
    retire_predecessor = heartbeat_receive.index(
        "remember_expired_operation_lease_locked("
    )
    install_successor = heartbeat_receive.index(
        "operation_heartbeat_lease_id_ = operation_lease_id;"
    )
    assert retire_predecessor < install_successor

    authorize_start = plugin.index(
        "  bool operation_heartbeat_authorizes_grasp_locked("
    )
    authorize_end = plugin.index(
        "  Control * first_suppressed_control()", authorize_start
    )
    authorize = plugin[authorize_start:authorize_end]
    assert "request.active_control_generation != active_control_generation_" in (
        authorize
    )
    assert "expired_operation_lease_ids_.count(request.operation_lease_id)" in (
        authorize
    )
    assert "operation_session_matches(" in authorize

    release_start = plugin.index("  PhysicsOutcome release_grasp_constraint()")
    release_end = plugin.index("  static void complete_request(", release_start)
    release = plugin[release_start:release_end]
    assert "schedule_actuation_collision_restore(" in release
    assert "released_robot_link" in release

    snapshot = watchdog.index(
        "heartbeat_sequence = operation_heartbeat_sequence_;"
    )
    now = watchdog.index("const auto now = std::chrono::steady_clock::now();")
    assert snapshot < now

    submit_start = plugin.index("  PhysicsOutcome submit_physics_request(")
    submit_end = plugin.index("  void process_physics_requests()", submit_start)
    submit = plugin[submit_start:submit_end]
    assert "request->operation_lease_id =" in submit
    assert "request->active_control_generation = active_control_generation_;" in submit

    grasp_start = plugin.index("  PhysicsOutcome handle_grasp_request(")
    grasp_end = plugin.index(
        "  PhysicsOutcome release_grasp_constraint()", grasp_start
    )
    grasp = plugin[grasp_start:grasp_end]
    assert grasp.count("operation_heartbeat_authorizes_grasp_locked(request)") == 2
    recovery_interlock = grasp.index("if (watchdog_recovery_pending_)")
    idempotent_success = grasp.index("Cabinet grasp is already attached.")
    authorization = grasp.rindex("operation_heartbeat_authorizes_grasp_locked(request)")
    attach_physics = grasp.index("base_brake_joint_ = model_->CreateJoint(")
    assert recovery_interlock < idempotent_success
    assert authorization < attach_physics
    assert "active_grasp_lease_id_ == request.operation_lease_id" in grasp
    assert "active_grasp_lease_id_ = request.operation_lease_id;" in grasp
    release_branch_end = grasp.index("    if (request.operation_lease_id.empty())")
    release_branch = grasp[:release_branch_end]
    idempotent_release = release_branch.index(
        "Cabinet grasp is already released."
    )
    exact_release = release_branch.index("operation_session_matches(")
    physical_release = release_branch.index("return release_grasp_constraint();")
    assert idempotent_release < exact_release < physical_release
    assert "active_grasp_lease_id_" in release_branch
    assert "request.operation_lease_id" in release_branch

    failed_attach_start = plugin.index(
        "  PhysicsOutcome recover_failed_grasp_attach("
    )
    failed_attach_end = plugin.index(
        "  PhysicsOutcome handle_grasp_request(", failed_attach_start
    )
    failed_attach = plugin[failed_attach_start:failed_attach_end]
    assert "catch (...)" in failed_attach
    assert "watchdog_recovery_pending_ = true;" in failed_attach
    assert "set_actuation_collision_enabled(control, true)" not in failed_attach
    assert "suppress_actuation_collision(control" in failed_attach
    assert "schedule_actuation_collision_restore(" in failed_attach
    assert "robot_link, request.robot_grasp_point" in failed_attach

    reset_start = plugin.index("  PhysicsOutcome reset_controls()")
    reset_end = plugin.index(
        "  PhysicsOutcome submit_physics_request(", reset_start
    )
    reset = plugin[reset_start:reset_end]
    clearance = reset.index("!actuation_tool_is_clear(control)")
    restore = reset.index("restore_all_actuation_collisions();")
    assert clearance < restore


def test_planning_scene_exemption_requires_live_exact_lease() -> None:
    source = PLANNING_SCENE_SOURCE.read_text(encoding="utf-8")
    assert '"operation_watchdog_timeout", 2.0' in source
    assert "operation_watchdog_timeout_is_valid(" in source
    assert (
        '"operation_heartbeat", rclcpp::QoS(1).reliable(),' in source
    )
    assert '"operation_fault", rclcpp::QoS(1).reliable(),' in source
    assert "kExpiredLeaseHistoryLimit = 64U" in source

    heartbeat_start = source.index("  void receive_operation_heartbeat(")
    heartbeat_end = source.index("  void receive_operation_fault(", heartbeat_start)
    heartbeat = source[heartbeat_start:heartbeat_end]
    assert "active_control_id_.empty()" in heartbeat
    assert "expired_operation_lease_ids_.count(operation_lease_id)" in heartbeat
    assert "if (!collision_exemption_was_active)" in heartbeat
    assert "++scene_revision_;" in heartbeat

    fault_start = heartbeat_end
    fault_end = source.index("  void enforce_operation_watchdog()", fault_start)
    fault = source[fault_start:fault_end]
    assert "operation_fault_matches_active_lease(" in fault
    assert "clear_active_operation_locked(true);" in fault

    watchdog_start = fault_end
    watchdog_end = source.index("  bool has_door() const", watchdog_start)
    watchdog = source[watchdog_start:watchdog_end]
    assert "std::chrono::steady_clock::now()" in watchdog
    assert "operation_watchdog_has_expired(" in watchdog
    assert "clear_active_operation_locked(true);" in watchdog

    publish_start = source.index("  void publish_scene_if_needed()")
    publish_end = source.index(
        "  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>", publish_start
    )
    publish = source[publish_start:publish_end]
    assert publish.index("enforce_operation_watchdog();") < publish.index(
        "lookup_model_transform(observed_model)"
    )
    assert (
        "active_control = operation_heartbeat_received_ ?\n"
        "        active_control_id_ : std::string{};" in publish
    )
    assert "active_control_id_.empty() || !operation_heartbeat_received_" in publish
