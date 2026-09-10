"""Regression contract for MoveIt mobile-base TF synchronization."""

from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "cabinet_button_operator.cpp"
)
PACKAGE_ROOT = SOURCE.parents[1]
INTERFACES_ROOT = (
    SOURCE.parents[2] / "xczs_inspection_robot_interfaces"
)


def test_action_results_expose_conservative_terminal_evidence() -> None:
    """Clients must be able to distinguish side effects from transport."""
    operate = (INTERFACES_ROOT / "action" / "OperateCabinetControl.action").read_text(
        encoding="utf-8"
    )
    press = (INTERFACES_ROOT / "action" / "PressCabinetButton.action").read_text(
        encoding="utf-8"
    )
    fields = (
        "physical_outcome_confirmed",
        "final_state_verified",
        "transport_succeeded",
        "recovery_succeeded",
        "grasp_released",
    )
    for field in fields:
        assert f"bool {field}" in operate
        assert f"bool {field}" in press

    source = SOURCE.read_text(encoding="utf-8")
    for field in fields:
        assert f"result->{field} = false" in source
    assert "result->physical_outcome_confirmed = true" in source
    assert "result->final_state_verified = true" in source
    assert "result->transport_succeeded = true" in source
    assert "result->recovery_succeeded = true" in source
    assert "result->grasp_released = true" in source


def test_unknown_and_busy_goals_are_structured_action_failures() -> None:
    """Goal transport acceptance must not discard the actionable reason."""
    source = SOURCE.read_text(encoding="utf-8")
    assert "GoalResponse::ACCEPT_AND_EXECUTE" in source
    assert "PendingGoalDisposition::INVALID_CONTROL" in source
    assert "PendingGoalDisposition::INVALID_BUTTON" in source
    assert "PendingGoalDisposition::RESOURCE_BUSY" in source
    assert "abort_pending_operate_goal(" in source
    assert "abort_pending_press_goal(" in source
    assert "Failed to start cabinet worker:" in source
    assert "Cabinet worker terminated unexpectedly" in source


def test_every_move_group_interface_reuses_the_warm_tf_buffer() -> None:
    """A fresh TF buffer races odom_joint initialization on every action."""
    source = SOURCE.read_text(encoding="utf-8")

    assert "std::shared_ptr<tf2_ros::Buffer> transform_buffer_;" in source
    # Exactly one warm buffer is ever created for the action lifecycle; no site
    # builds a fresh empty one per action.
    assert source.count("std::make_shared<tf2_ros::Buffer>(") == 1
    assert "std::shared_ptr<tf2_ros::Buffer>()" not in source

    # Every move-group construction must share that warm member buffer, however
    # its group name is spelled (the calibration tool group and the bimanual
    # left group legitimately use names other than move_group_name_).  Anchor on
    # the inline Options(...) each construction passes and require the warm
    # buffer to be the constructor's next argument, instead of tallying a fixed
    # set of call sites.
    probe = source
    warm_constructions = 0
    total_constructions = 0
    while True:
        options_index = probe.find("MoveGroupInterface::Options(")
        if options_index == -1:
            break
        total_constructions += 1
        options_close = probe.find(")", options_index)
        assert options_close != -1
        constructor_end = probe.find(");", options_close)
        assert constructor_end != -1
        if "transform_buffer_," in probe[options_close:constructor_end]:
            warm_constructions += 1
        probe = probe[constructor_end + 1:]
    assert total_constructions >= 1
    assert warm_constructions == total_constructions


def test_physical_paths_reverify_the_stopped_base_before_arm_motion() -> None:
    """Scene settling must not open a gap between docking and arm motion."""
    source = SOURCE.read_text(encoding="utf-8")

    # The reverify entry point exists, and the two anchored snippets below pin
    # its invocation in each physical docking path right after scene settling.
    # Do not tally whole-file call text: the old counter also matched the
    # function definition and broke the moment a new settling branch appeared.
    assert "  void verify_staging_pose_before_arm_motion(" in source
    assert (
        "dock_to_staging_pose(goal_handle, staging_poses.planning_pose);\n"
        "      interruptible_hold(goal_handle, planning_scene_settle_seconds_);\n"
        "      verify_staging_pose_before_arm_motion("
    ) in source
    assert (
        "if (preparation_policy.wait_for_scene_settle) {\n"
        "        interruptible_hold(goal_handle, planning_scene_settle_seconds_);\n"
        "      }\n"
        "      if (preparation_policy.execute_precision_docking) {\n"
        "        verify_staging_pose_before_arm_motion("
    ) in source


def test_operable_station_uses_explicit_base_footprint_clearance_gate() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    assert '"docking_base_footprint"' in source
    assert '"docking_base_footprint_padding"' in source
    assert "button->operable && !station_standoff_is_safe(" in source


def test_rotary_controls_are_stable_at_both_pregrasp_boundaries() -> None:
    """No knob/switch/door grasp may follow an unchecked state change."""
    source = SOURCE.read_text(encoding="utf-8")
    operate_start = source.index("  void execute_operate(")
    operate_end = source.index(
        "  std::pair<double, std::string> resolve_operation_target(",
        operate_start,
    )
    operate = source[operate_start:operate_end]

    # Knobs and doors (the rotary family) must be stable at BOTH pregrasp
    # boundaries: before the ready motion and again right before the grasp
    # attach.  The drawer/slider branches legitimately own their own gates, so
    # do not tally gates inside execute_operate -- pin the two rotary
    # boundaries by stage ordering instead.
    assert (
        "{control.get(), initial_state.state_id, initial_state.position}"
        in operate
    )
    assert (
        "{ancestor, parent_initial_state.state_id,\n"
        "              parent_initial_state.position}"
        in operate
    )

    ready_boundary_gate = operate.index(
        "wait_for_pregrasp_controls_stable("
    )
    # 几何锚定现为 rail-aware 形式 latch_cabinet_transform(drawer, rail)
    # （2026-09-09 cap1 复验败根因修复：抽屉开/关位都按实测轨位反推 frame 真值，
    #  免疫 Nav2 期 AMCL map→odom 漂移）；这里只锚「调用点」的先后，故不带实参。
    assert "latch_cabinet_transform(\n" in operate
    geometry_latch = operate.index("latch_cabinet_transform(")
    ready_motion = operate.index("rotary_poses.ready_pose")
    # The ready-boundary gate runs before geometry is latched and before any
    # rotary ready-pose motion, so the ready plan is computed against a state
    # that was still verified current.
    assert ready_boundary_gate < geometry_latch
    assert ready_boundary_gate < ready_motion

    pregrasp_motion = operate.index(
        "rotary_poses.pregrasp_pose", ready_motion
    )
    # The rotary grasp-boundary gate is the LAST stability gate in
    # execute_operate (the drawer/slider gates appear earlier in the text) and
    # sits between the near-grasp pregrasp plan and the final short Cartesian
    # approach that attaches the grasp.
    grasp_boundary_gate = operate.rindex(
        "wait_for_pregrasp_controls_stable("
    )
    final_cartesian_approach = operate.index(
        "{rotary_poses.grasp_pose}", pregrasp_motion
    )
    attach_grasp = operate.index(
        "set_control_grasp(goal_handle, control->id, true)",
        final_cartesian_approach,
    )
    assert pregrasp_motion < grasp_boundary_gate < final_cartesian_approach
    assert final_cartesian_approach < attach_grasp


def test_pregrasp_stability_gate_is_fresh_continuous_and_fail_closed() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    gate_start = source.index("  void wait_for_pregrasp_controls_stable(")
    gate_end = source.index(
        "  void wait_for_parent_controls_stable(", gate_start
    )
    gate = source[gate_start:gate_end]

    assert "state.structured_received_at > boundary_started_at" in gate
    assert "classify_pregrasp_stability_sample(" in gate
    assert "state.state_id == reference.state_id" in gate
    assert "reference.position" in gate
    assert "stable_state_duration_" in gate
    assert "PregraspStabilitySampleStatus::REFERENCE_CHANGED" in gate
    # Fail-closed: the gate never returns success before the guarded stability
    # interval and never aborts with a non-NOT_READY outcome.  Enforce that
    # every Result reference and every throw in this body is a NOT_READY
    # refusal (adding a new integrity gate must not force this contract to
    # change) and that each of the four failure classes is still present:
    # empty references, an unavailable reference, a changed reference, and
    # deadline expiry after the continuous-stability loop.
    assert "if (references.empty()) {" in gate
    assert "if (!reference.control) {" in gate
    assert "while (std::chrono::steady_clock::now() < deadline) {" in gate
    not_ready_refusals = gate.count(
        "OperateCabinetControl::Result::NOT_READY"
    )
    assert not_ready_refusals == gate.count("OperateCabinetControl::Result::")
    assert not_ready_refusals == gate.count("throw GenericOperationError(")


def test_action_start_gate_expects_the_commanded_work_pose() -> None:
    """The arm starting-posture gate must measure against the pose the pipeline
    actually commands, never against a self-invented stricter reference.

    Regression (2026-09-10 first live run, db1 open): the gate built its
    expected rod-end point from the bare handle point + rail + press + rod
    clearance, omitting ``drawer_pose_aim_offset``.  That aim offset is
    calibrated into the *commanded* work pose (aim_new = aim_old - measured
    residual), so the physical rod end legitimately lands on handle + aim.  The
    gate therefore read the calibration bias as "the arm is not in place"
    (21.0 mm left / 28.6 mm right vs a 0.020 m limit) and refused a healthy
    run.  With the aim term restored the same measurement is 7.9 / 11.6 mm --
    exactly the execution lateral residual the adapter comment records.

    Pinning the raw handle point would be wrong for a second reason: nulling
    the lateral residual onto the handle point is ``self_center``'s job
    (``drawer_world_rail_point``, 2.5 mm band, run AFTER this gate), so the
    gate must not require it to be already done.
    """
    source = SOURCE.read_text(encoding="utf-8")
    gate_start = source.index("  void verify_drawer_action_start_gate(")
    gate_end = source.index(
        "  void plan_and_execute_bimanual_poses(", gate_start
    )
    gate = source[gate_start:gate_end]

    # Same expression as calculate_drawer_side_tool_pose: handle point + rail
    # shift + the side's calibrated aim offset.
    assert "drawer_side_point(control, side) + control.drawer_axis * rail_position" in gate
    assert "tool.drawer_pose_aim_offset" in gate
    # ...and the physical measurement side is the same truth source the seal
    # and self-center stages use, not MoveIt FK.
    assert "drawer_physics_link_point(" in gate
    assert "gripper_contact_point_local" in gate

    # The raw rail/handle point is self-center's reference, not this gate's.
    assert "drawer_world_rail_point(" not in gate

    # Ordering is the reason the above holds: the gate runs before self_center
    # in the visual front end, so the residual it must tolerate is the
    # pre-self-center one.
    backend_start = source.index("  void execute_drawer_visual_backend(")
    backend = source[backend_start:]
    assert (
        backend.index("verify_drawer_action_start_gate(")
        < backend.index("self_center_drawer_work_pose(")
    )


def test_rod_start_posture_is_commanded_and_measured_before_the_action() -> None:
    """The arm reaching its work pose is not the same as the tool being in its
    starting posture: the rods are physically pushed out by the cabinet during
    the arrival approach.

    Regression (2026-09-10 live run, db1 open): the arrival gate refused the
    action with a right hook tip 30.9 / 31.7 mm short of the expected point,
    almost entirely along the outward axis.  A purpose-built probe
    (scripts/tools/probe_rod_tip_truth.py, /xczs/joint_states joint values +
    /get_entity_state physics tips) measured the SAME window: the right hook
    cylinder joint sat at q = +0.019..+0.039 (mean +0.029 -- exactly that
    30 mm) while the left one was quiet at ~0, and no cylinder goal had been
    sent yet (the first one is issued by the gate-timeout recovery).  The rods
    are prismatic, so q maps 1:1 onto tip travel along the tool axis: contact
    with the cabinet shoved them out.  The arm was in place, the rods were
    not -- so the gate was right to refuse.

    The fix must therefore COMMAND the rods back to their stage-"home"
    positions before the action and re-MEASURE them, strictly (a jam must fail
    the run, not be forwarded), and it must run before the arrival gate so the
    gate measures a genuinely retracted tool.  The arm hold dwell is re-armed
    off the controller COMMAND reference, never the measured position.
    """
    source = SOURCE.read_text(encoding="utf-8")
    step_start = source.index("  void establish_drawer_rod_start_posture(")
    step_end = source.index("  void verify_drawer_action_start_gate(", step_start)
    step = source[step_start:step_end]

    # Commands the stage-"home" matrix through the same strict full-joint send
    # the staged front end uses (not the catch-and-warn best-effort wrapper).
    assert 'drawer_rod_stage_desired(' in step
    assert ', "home")' in step
    assert "send_cylinder_full_joint_goal(" in step
    assert "best_effort_drawer_rods_home(" not in step

    # ...and re-measures every rod from the REAL joint values, not from the
    # trajectory controller's SUCCESS verdict.
    assert "read_real_joint_position(" in step
    assert "entry.measured[index]" in step
    assert "drawer_rod_start_tolerance_" in step

    # Strict: a rod that will not return home fails the action loudly, by name,
    # instead of continuing with a displaced tool.
    assert "throw OperationError(" in step
    assert "PressCabinetButton::Result::EXECUTION_FAILED" in step
    assert "stop_active_motion();" in step
    assert "describe_rods()" in step

    # Bounded and fail-closed: waits for a stable in-tolerance window, and the
    # whole step is bounded by its own deadline check.
    assert "drawer_rod_start_stable_seconds_" in step
    assert "drawer_rod_start_timeout_seconds_" in step
    assert "if (std::chrono::steady_clock::now() >= deadline) {" in step

    # The arm hold re-arm is anchored on the controller command reference.
    assert "rearm_drawer_arm_hold_dwell(" in step
    helper_start = source.index("  bool rearm_drawer_arm_hold_dwell(")
    helper_end = source.index(
        "  void establish_drawer_rod_start_posture(", helper_start
    )
    helper = source[helper_start:helper_end]
    assert "arm_controller_reference(left_side, joint_order, terminal)" in helper
    assert "kDwellHoldSeconds" in helper

    # The dwell length is a single source of truth: the hoisted constant plus
    # local aliases, never a second hard-coded 12.0.
    assert source.count("kDwellHoldSeconds = 12.0") == 1

    # Parameter defaults are declared once and bounded like the arrival gate's.
    for name, default in (
        ("drawer_rod_start_tolerance", "0.005"),
        ("drawer_rod_start_stable_seconds", "0.30"),
        ("drawer_rod_start_timeout", "5.0"),
    ):
        assert f'"{name}", {default})' in source
    assert (
        "Drawer rod-start gate tolerances exceed their safety bounds."
        in source
    )

    # Ordering: the rod start posture is established (and the tool therefore
    # retracted) before the arrival gate measures it, and both run before the
    # staged front end starts acting on the cabinet.
    backend = source[source.index("  void execute_drawer_visual_backend("):]
    assert (
        backend.index("establish_drawer_rod_start_posture(")
        < backend.index("verify_drawer_action_start_gate(")
        < backend.index("self_center_drawer_work_pose(")
    )
    assert 'result->diagnostic_stage = "rod_start";' in backend


def test_toolset_mismatch_finishes_before_any_moveit_profile_change() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    operate_start = source.index("  void execute_operate(")
    operate_end = source.index(
        "  std::pair<double, std::string> resolve_operation_target(",
        operate_start,
    )
    operate = source[operate_start:operate_end]

    mismatch = operate.index("if (!tool_serves_control(control->control_type))")
    profile_change = operate.index("apply_tool_profile(control->control_type)")
    assert mismatch < profile_change
    assert "OperateCabinetControl::Result::TOOLSET_MISMATCH" in operate
    assert 'result->diagnostic_stage = "toolset_validation"' in operate
    assert "result->operation_executed = false" in operate


def test_articulated_noop_is_an_explicit_terminal_failure() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    operate_start = source.index("  void execute_operate(")
    operate_end = source.index(
        "  std::pair<double, std::string> resolve_operation_target(",
        operate_start,
    )
    operate = source[operate_start:operate_end]

    target_resolution = operate.index("resolve_operation_target(")
    noop_gate = operate.index(
        "std::abs(target_position - initial_state.position) <="
    )
    # rail-aware 锚定调用点，见 test_rotary_controls_are_stable_at_both_pregrasp_boundaries。
    geometry_latch = operate.index("latch_cabinet_transform(")
    assert target_resolution < noop_gate < geometry_latch
    assert "is already at requested" in operate
    assert "select a different physical detent" in operate
