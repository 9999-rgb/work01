"""Regression contract for MoveIt mobile-base TF synchronization."""

from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "cabinet_button_operator.cpp"
)


def test_every_move_group_interface_reuses_the_warm_tf_buffer() -> None:
    """A fresh TF buffer races odom_joint initialization on every action."""
    source = SOURCE.read_text(encoding="utf-8")

    assert "std::shared_ptr<tf2_ros::Buffer> transform_buffer_;" in source
    assert "std::make_shared<tf2_ros::Buffer>(get_clock())" in source
    assert "std::shared_ptr<tf2_ros::Buffer>()" not in source
    assert source.count(
        'MoveGroupInterface::Options(\n'
        '          move_group_name_, "robot_description", move_group_namespace_),\n'
        "        transform_buffer_,"
    ) == 2


def test_physical_paths_reverify_the_stopped_base_before_arm_motion() -> None:
    """Scene settling must not open a gap between docking and arm motion."""
    source = SOURCE.read_text(encoding="utf-8")

    assert source.count("verify_staging_pose_before_arm_motion(") == 3
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
