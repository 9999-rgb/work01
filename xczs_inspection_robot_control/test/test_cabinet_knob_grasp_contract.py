"""Regression contract for isolated physical cabinet-knob grasping."""

from __future__ import annotations

import math
import re
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
    / "xczs_inspection_robot_gazebo"
    / "src"
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
    # The demo replica must keep the SAME box_5 calibration contract as the
    # builtin.  Scope the equality to the required subtree, not the whole file
    # -- the two adapters legitimately diverge on other controls.
    assert builtin["controls"]["box_5_knob"] == sample["controls"]["box_5_knob"]

    controls = builtin["controls"]
    assert isinstance(controls, dict)
    calibration = controls["box_5_knob"]
    # Aligned dock anchor: shoulder (r_arm_0) laterally aligned with the knob
    # blade centerline (base_y ~ 0.513, the button family's -0.3506 shoulder
    # offset, anchor x = -0.18313) makes pre-press and both detents reachable.
    # The historical x = 0.717 (+0.55 shoulder offset) put the shoulder 0.9 m
    # sideways and made the ready IK fail every time.
    assert calibration["navigation_station"]["local_anchor"] == [
        -0.18313,
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
    ready_joint_seed = calibration["ready_joint_seed"]
    assert ready_joint_seed["joint_names"] == [
        f"r_arm_{index}_joint" for index in range(7)
    ]
    assert ready_joint_seed["positions"] == [
        -1.221067,
        1.158552,
        -1.448952,
        -1.806018,
        1.998380,
        0.014786,
        -1.656403,
    ]

    # Knob-blade grasp contract in the toward_control orientation: tool +Z
    # points at the control, and roll rotates the jaw opening axis (tool +Y)
    # relative to the knob-blade normal (world +-Y).  Because the roll is a
    # constant offset in the tool frame, the jaw<->blade misalignment equals
    # |roll| throughout the whole rotation, so the blade projects
    # 11*|cos roll| + 65*|sin roll| onto the 36 mm jaw opening.  That value
    # must stay below 36 mm, i.e. |roll| <= ~22.5 deg.  The historical
    # non-zero rolls were calibrated along the old outward axis (jaws vertical,
    # world +-Z, 65 mm blade could not enter the 36 mm vertical gap); the
    # calibrated center->left roll (-20 deg) stays inside the fit bound
    # (11*cos20 + 65*sin20 ~ 32.6 mm < 36 mm) while clearing the r_arm_0<->
    # r_arm_2 self-collision that otherwise blocks the left arc.
    offsets = calibration["tool_roll_offsets"]
    assert len(offsets) == 9
    for offset in offsets:
        assert (
            abs(math.cos(offset)) * 0.011 + abs(math.sin(offset)) * 0.065
            < 0.036
        ), f"tool roll {offset} breaks the knob-blade jaw fit"
    # state_ids = ["left","center","right"]; index = source*3 + target.
    # center(1)->left(0) = index 3 is the only non-zero entry.
    assert offsets[3] == -0.349066
    assert all(
        offsets[i] == 0.0 for i in range(9) if i != 3
    )


def test_operation_watchdog_fits_between_heartbeat_and_lease_expiry() -> None:
    # Both adapter replicas must configure the same timing contract.  Read the
    # three required fields from each file and require them equal, instead of
    # whole-file equality.
    lease_values = [
        (
            float(parameters["operation_lease_duration"]),
            float(parameters["operation_lease_renew_period"]),
            float(parameters["operation_lease_request_timeout"]),
        )
        for parameters in (
            _operator_parameters(ROBOT_ADAPTERS[0]),
            _operator_parameters(ROBOT_ADAPTERS[1]),
        )
    ]
    assert lease_values[0] == lease_values[1]
    lease_duration, renew_period, request_timeout = lease_values[0]

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

    # The plugin authorizes physics strictly by the operation lease.  Every
    # lease-gated physical request the operator dispatches -- cabinet grasp,
    # bimanual drawer grasp, drawer unlock: each service whose schema carries
    # operation_lease_id -- must be stamped with the CURRENT lease under the
    # lease mutex, so a stale request can never win a physical race after the
    # operation is retired.  Derive the service set from the schemas and
    # require every concrete request construction of those types to carry the
    # stamp, instead of pinning one whole-file total (P1-P4 added the drawer
    # channels, which is exactly what broke the old fixed count).
    lease_gated_services = {
        path.stem for path in (
            WORKSPACE / "xczs_inspection_robot_interfaces"
            / "srv").glob("*.srv")
        if "operation_lease_id" in path.read_text(encoding="utf-8")
    }
    request_stamps = operator.count(
        "request->operation_lease_id = operation_lease_id_;"
    )
    request_constructions = sum(
        len(re.findall(name + r"::\s*Request>\(\)", operator))
        for name in lease_gated_services
    )
    assert request_constructions >= 1
    assert request_stamps == request_constructions
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
    # A heartbeat publication failure must retire the EXACT lease (the snapshot
    # taken at the top of the function), never the live member that a successor
    # operation may already have taken over.  Every catch handler here must mark
    # the lease lost -- no handler may only log and leave a stale lease
    # authoritative -- so require the mark sites to equal the number of catch
    # handlers rather than a fixed tally.
    assert heartbeat.count(
        "mark_operation_lease_lost_for_exact_lease("
    ) == (
        heartbeat.count("catch (const std::exception & error)")
        + heartbeat.count("} catch (...)")
    )
    assert heartbeat.count("lease_id);") == heartbeat.count(
        "mark_operation_lease_lost_for_exact_lease("
    )

    active_publish_start = heartbeat_end
    active_publish_end = operator.index(
        "  void reset_controls(", active_publish_start
    )
    active_publish = operator[active_publish_start:active_publish_end]
    assert "const std::string & operation_lease_id" in active_publish
    # A non-idle active-control publication failure must retire the exact lease
    # that owns the control; only an idle (empty-control) publication failure
    # may degrade to a log.  Each non-idle failure case marks exactly once, so
    # derive the expectations from the number of non-idle branches instead of a
    # fixed tally.
    non_idle_branches = active_publish.count("if (!control_id.empty()) {")
    assert non_idle_branches >= 1
    assert active_publish.count(
        "mark_operation_lease_lost_for_exact_lease("
    ) == non_idle_branches
    assert active_publish.count("operation_lease_id);") == non_idle_branches

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

    # The physical cleanup lives in the two mutually-exclusive per-session
    # helpers (single and bimanual), and release_grasp_constraint() funnels into
    # both and only succeeds when neither reports a failure.  Do not anchor on
    # the funnel alone: the restore scheduling that makes the just-grasped
    # control's collision come back lives in each helper body.
    release_start = plugin.index(
        "  PhysicsOutcome release_single_grasp_constraint()"
    )
    release_end = plugin.index("  static void complete_request(", release_start)
    release = plugin[release_start:release_end]
    single_start = release.index(
        "  PhysicsOutcome release_single_grasp_constraint()"
    )
    bimanual_start = release.index(
        "  PhysicsOutcome release_bimanual_grasp_constraint()"
    )
    combined_start = release.index(
        "  PhysicsOutcome release_grasp_constraint()"
    )
    single_release = release[single_start:bimanual_start]
    bimanual_release = release[bimanual_start:combined_start]
    combined_release = release[combined_start:]
    for body in (single_release, bimanual_release):
        assert "schedule_actuation_collision_restore(" in body
        assert "publish_grasp_active(false)" in body
    assert "released_robot_link" in single_release
    assert "released_left_link" in bimanual_release
    assert "released_left_grasp_point" in bimanual_release
    assert "release_single_grasp_constraint()" in combined_release
    assert "release_bimanual_grasp_constraint()" in combined_release
    assert "!single.success && !bimanual.success" in combined_release

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
    # Attach must be authorized at two stages: up front (a fresh heartbeat from
    # the exact active control and global lease session) and again, under the
    # session lock, immediately before the irreversible CreateJoint commit.
    # Pin both stages by ordering instead of by a fixed call tally.
    first_authorization = grasp.index(
        "operation_heartbeat_authorizes_grasp_locked(request)"
    )
    recovery_interlock = grasp.index("if (watchdog_recovery_pending_)")
    idempotent_success = grasp.index("Cabinet grasp is already attached.")
    commit_session_lock = grasp.index(
        "std::unique_lock<std::mutex> active_session_lock("
        "active_control_mutex_);"
    )
    commit_authorization = grasp.rindex(
        "operation_heartbeat_authorizes_grasp_locked(request)"
    )
    attach_physics = grasp.index("base_brake_joint_ = model_->CreateJoint(")
    assert recovery_interlock < idempotent_success
    assert first_authorization < idempotent_success
    assert first_authorization < commit_session_lock
    assert commit_session_lock < commit_authorization < attach_physics
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
