"""AGENT doc §4.5 (P1) structural contracts for the in-place seat takeover.

Pure source-contract tests (no Gazebo, no compilation) protecting the P1
behaviors that the pure-math projection gtest cannot reach:

  - the seal normal path no longer resets tool controllers (§4.1);
  - a seal that cannot produce a valid seat reference on either side fails the
    whole hook stage instead of forwarding an unsealed depth to support (§4.2 /
    §4.5 "左右任一侧无效时不得进入后续阶段");
  - support / unlock full-joint goals retain the hook seat reference (§4.4);
  - the hold state lives only on the current action's stack (§4.2 / §4.5
    cancel/exception residue).

Assertions are region-scoped to the owning function body (brace-matched) rather
than whole-file counts, and run against a copy with comments and string
literals blanked so a comment or log string can never satisfy or defeat a
token check.  Identifiers are matched as complete constructs (client *usage*,
full service request types), not by word substrings.
"""

from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "cabinet_button_operator.cpp"
)
HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "xczs_inspection_robot_control"
    / "drawer_hook_seat_projection.hpp"
)

SOURCE_TEXT = SOURCE.read_text(encoding="utf-8")
HEADER_TEXT = HEADER.read_text(encoding="utf-8")


def _blank_comments_and_strings(text: str) -> str:
    """Return ``text`` with // comments, /* */ comments, and string/char
    literals replaced by spaces, keeping length and every code token intact.

    This lets token assertions ignore comments and log strings, which may
    legitimately mention the deleted four-chain or controller reset concepts.
    """
    out = list(text)
    n = len(text)
    i = 0
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if c == "/" and nxt == "/":  # line comment
            while i < n and text[i] != "\n":
                out[i] = " "
                i += 1
        elif c == "/" and nxt == "*":  # block comment
            out[i] = " "
            out[i + 1] = " "
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                if text[i] != "\n":
                    out[i] = " "
                i += 1
            if i + 1 < n:
                out[i] = " "
                out[i + 1] = " "
                i += 2
        elif c == '"' or c == "'":  # string / char literal
            quote = c
            out[i] = " "
            i += 1
            while i < n:
                if text[i] == "\\":  # escaped char: blank both
                    out[i] = " "
                    i += 1
                    if i < n:
                        out[i] = " "
                        i += 1
                    continue
                if text[i] == quote:
                    out[i] = " "
                    i += 1
                    break
                if text[i] != "\n":
                    out[i] = " "
                i += 1
        else:
            i += 1
    return "".join(out)


def _body(text: str, head_token: str) -> str:
    """Return the brace-matched body that begins after ``head_token``.

    ``head_token`` must uniquely identify a function/struct definition head
    (return type + name + ``(``).  Matching runs on comment/string-blanked
    text so braces hidden inside literals cannot throw off the depth count;
    the returned slice is from the original text.
    """
    blank = _blank_comments_and_strings(text)
    start = text.find(head_token)
    assert start != -1, f"signature head not found: {head_token}"
    open_idx = blank.find("{", start)
    assert open_idx != -1, f"no body opener after {head_token}"
    depth = 0
    for i in range(open_idx, len(blank)):
        if blank[i] == "{":
            depth += 1
        elif blank[i] == "}":
            depth -= 1
            if depth == 0:
                return text[open_idx:i + 1]
    raise AssertionError(f"unbalanced braces after {head_token}")


def test_seal_normal_path_never_resets_tool_controllers() -> None:
    """§4.1: the deactivate/configure/activate → re-home → re-chase → park
    four-chain must not survive inside the seal's normal success path."""
    body = _body(SOURCE_TEXT, "void seal_drawer_hooks_by_probe(")
    clean = _blank_comments_and_strings(body)

    # The P1 takeover machinery is present: physics home/seat capture, the
    # single-source projection header, and the takeover goals.
    assert "capture_gripper_end(" in clean
    assert "accept_drawer_rod_goal(" in clean
    assert "project_drawer_hook_seat(" in clean
    assert "*in_place_ok = true;" in clean

    # None of the controller-reset infrastructure is referenced inside the
    # seal (comments/logs blanked, so only real usage would match).
    for token in (
        "controller_switch_client_",
        "controller_configure_client_",
        "SwitchController",
    ):
        assert token not in clean, (
            f"seal body references controller-reset infra ({token}); §4.1 "
            "forbids deactivate/configure/activate in the normal path."
        )


def test_projection_is_delegated_to_the_single_source_header() -> None:
    """§4.3: endpoint→joint conversion lives in exactly one tested header; the
    operator must call it rather than re-derive an approximation."""
    body = _body(SOURCE_TEXT, "void seal_drawer_hooks_by_probe(")
    clean = _blank_comments_and_strings(body)

    # The header is the pure, unit-tested projection (drawer_hook_seat_
    # projection_test covers signed-projection + joint clamping).
    assert "inline DrawerHookSeatProjection project_drawer_hook_seat(" in HEADER_TEXT
    assert "double joint_ref{0.0};" in HEADER_TEXT

    # Seal routes the seat/home/axis capture through the header's projection
    # (raw body: these anchors are code and call-site text, not literals).
    assert "return projection.joint_ref;" in body
    assert "project_drawer_hook_seat(" in clean
    assert "const double left_joint_ref = seat_joint_ref(" in body
    assert "const double right_joint_ref = seat_joint_ref(" in body

    # §4.3 p_home baseline must be taken BEFORE the press (phase 1): under
    # load the position channel is not a clean home.  Order on the raw body:
    # home capture → press goal → seat capture (after the 3 s hold).
    home_idx = body.find("left_home_base = capture_home_base(left_tool, left_label)")
    press_idx = body.find("const std::vector<double> left_press_desired = ")
    seat_idx = body.find("left_seat_end = capture_gripper_end(left_tool, left_label, \"seat\")")
    assert home_idx != -1 and press_idx != -1 and seat_idx != -1
    assert home_idx < press_idx < seat_idx


def test_invalid_either_side_fails_the_whole_hook_stage() -> None:
    """§4.2/§4.5: a seal that cannot conclude a valid seat reference on both
    sides must fail the stage, never report invalid and carry on."""
    body = _body(SOURCE_TEXT, "void seal_drawer_hooks_by_probe(")
    clean = _blank_comments_and_strings(body)

    # Exactly one reset-to-invalid (the defensive init at function entry) and
    # one set-valid at the verified exit, in that order — so no code path
    # concludes with in_place_ok=false and a "half-sealed" result cannot be
    # returned normally for the caller to forward into support.
    assert clean.count("*in_place_ok = false;") == 1
    assert clean.count("*in_place_ok = true;") == 1
    assert clean.index("*in_place_ok = false;") < clean.index("*in_place_ok = true;")

    # Both physics-capture-unavailable branches now throw a whole-stage
    # failure (NOT_READY) instead of the removed soft return.  The throw
    # messages live inside string literals, so assert them on the raw body.
    assert "throw OperationError(" in clean
    assert "without a pre-press home base" in body
    assert "cannot project the in-place seat takeover" in body


def test_support_and_unlock_full_joint_goals_retain_the_hook_ref() -> None:
    """§4.4: downstream full-joint goals must carry the seal seat ref, not
    overwrite it with the static gripper_grasp_position."""
    support = _body(SOURCE_TEXT, "void drive_drawer_support_stage(")
    support_clean = _blank_comments_and_strings(support)

    # Support passes each side's seal joint_ref through the gripper override
    # of the full-joint stage goal (NaN only on the legacy unsealed path).
    assert "hook_hold.valid ? hook_hold.left_joint_ref :" in support_clean
    assert "hook_hold.valid ? hook_hold.right_joint_ref :" in support_clean

    side = _body(SOURCE_TEXT, "double execute_drawer_rod_stage_side(")
    side_clean = _blank_comments_and_strings(side)
    assert "drawer_rod_stage_desired(" in side_clean
    assert "gripper_override" in side_clean

    # Unlock drives only the motor joint and holds every other finger at its
    # measured state.  After the takeover the gripper's measured state == the
    # seat ref (the takeover goal landed the rod on the seat), so holding the
    # others at current *is* §4.4's "保持座封/支撑".  The unlock path must NOT
    # rebuild a static-grasp full-joint vector.
    unlock = _body(SOURCE_TEXT, "double drive_unlock_motor(")
    unlock_clean = _blank_comments_and_strings(unlock)
    assert "send_unlock_motor_goal(" in unlock_clean
    assert "start_positions.push_back(current);" in unlock_clean
    assert "drawer_rod_stage_desired" not in unlock_clean


def test_hold_state_is_task_local_and_leaves_no_residue() -> None:
    """§4.2: the hold state exists only for one action execution.  It must not
    be a member, global, or shared cache, so cancel/exception unwinding cannot
    leave a stale seat reference behind."""
    source_clean = _blank_comments_and_strings(SOURCE_TEXT)

    # The only value-instantiation is the execute_operate drawer-branch local
    # (struct params below take it by reference).
    assert source_clean.count("DrawerHookHoldState hook_hold;") == 1
    for stash in (
        "hook_hold_",                      # member-style name
        "static DrawerHookHoldState",      # function-local static
        "shared_ptr<DrawerHookHoldState>", # heap escape
        "DrawerHookHoldState * hook_hold =",  # second owned pointer
    ):
        assert stash not in source_clean, (
            f"hold state may not outlive the task ({stash})"
        )

    # The struct carries exactly the §4.2 fields, in the same shape the seal
    # and support stages rely on.
    struct = _body(SOURCE_TEXT, "struct DrawerHookHoldState")
    assert "double left_joint_ref{0.0};" in struct
    assert "double right_joint_ref{0.0};" in struct
    assert "bool valid{false};" in struct


def test_no_reset_client_is_invoked_anywhere() -> None:
    """§4.1 keeps the reset clients as exception-recovery infrastructure only:
    they may be constructed (dead members), but no code path may invoke them.
    This whole-file check is the backstop to the seal-scoped one above."""
    clean = _blank_comments_and_strings(SOURCE_TEXT)
    assert "controller_switch_client_->" not in clean
    assert "controller_configure_client_->" not in clean
    assert "SwitchController::Request" not in clean
    assert "ConfigureController::Request" not in clean
