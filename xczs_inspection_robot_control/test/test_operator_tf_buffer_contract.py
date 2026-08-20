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
