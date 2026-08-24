"""Tests for spawn-time initial-pose verification and shared pose contracts."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree

from control_msgs.action import FollowJointTrajectory
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import xacro
import yaml


CONTROL_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = CONTROL_ROOT.parent
DESCRIPTION_ROOT = WORKSPACE_ROOT / "xczs_inspection_robot_description"
MOVEIT_ROOT = WORKSPACE_ROOT / "xczs_inspection_robot_moveit_config"


def _load_module():
    script_path = CONTROL_ROOT / "scripts" / "verify_initial_pose.py"
    spec = spec_from_file_location("verify_initial_pose", script_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def error(self, message):
        self.messages.append(("error", message))


class _Node:
    def __init__(self, message=None):
        self.logger = _Logger()
        self.destroyed = False
        self.message = message
        self.publishers = []

    def get_logger(self):
        return self.logger

    def create_subscription(self, message_type, topic, callback, depth):
        assert message_type is JointState
        assert topic.startswith("/")
        assert depth == 10
        if self.message is not None:
            callback(self.message)
        return object()

    def create_publisher(self, message_type, topic, _qos):
        assert message_type is Bool
        assert topic.startswith("/")
        publisher = _Publisher()
        self.publishers.append((topic, publisher))
        return publisher

    def destroy_node(self):
        self.destroyed = True


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Client:
    def __init__(self, ready=True):
        self.ready = ready

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, _goal):
        raise AssertionError("spawn-pose verification must never send a goal")


def _joint_state(positions):
    message = JointState()
    message.name = list(positions)
    message.position = list(positions.values())
    return message


def _mounted_joint_state(module, arm_positions, toolset="A"):
    positions = dict(arm_positions)
    positions.update({name: 0.0 for name in module.TOOLSET_JOINTS[toolset]})
    return _joint_state(positions)


def _ready_clients(_node, action_type, _action_name):
    assert action_type is FollowJointTrajectory
    return _Client()


def test_verifies_measured_pose_without_sending_a_trajectory():
    module = _load_module()
    expected = {joint: float(index) / 10.0 for index, joint in enumerate(module.ARM_JOINTS)}
    node = _Node(_mounted_joint_state(module, expected))
    with patch.object(module, "ActionClient", side_effect=_ready_clients):
        module.verify_initial_pose(node, expected, timeout_sec=0.0)


def test_missing_controller_is_a_startup_failure():
    module = _load_module()
    expected = {joint: 0.0 for joint in module.ARM_JOINTS}

    def client(_node, _action_type, action_name):
        return _Client(ready="left_arm_controller" not in action_name)

    with patch.object(module, "ActionClient", side_effect=client):
        with pytest.raises(module.InitialPoseError, match="left arm"):
            module.verify_initial_pose(
                _Node(_mounted_joint_state(module, expected)),
                expected,
                timeout_sec=0.0,
            )


def test_pose_mismatch_is_a_startup_failure():
    module = _load_module()
    expected = {joint: 0.0 for joint in module.ARM_JOINTS}
    measured = dict(expected)
    measured["r_arm_6_joint"] = 0.5
    with patch.object(module, "ActionClient", side_effect=_ready_clients):
        with pytest.raises(module.InitialPoseError, match="r_arm_6_joint"):
            module.verify_initial_pose(
                _Node(_mounted_joint_state(module, measured)),
                expected,
                timeout_sec=0.0,
            )


def test_successful_verification_releases_spawn_only_joint_hold():
    module = _load_module()
    node = _Node()

    with patch.object(module.rclpy, "spin_once") as spin_once:
        publisher = module.release_startup_joint_hold(node)

    assert node.publishers == [(module.DEFAULT_JOINT_HOLD_TOPIC, publisher)]
    assert len(publisher.messages) == 1
    assert publisher.messages[0].data is False
    spin_once.assert_called_once_with(
        node,
        timeout_sec=module.POLL_INTERVAL_SEC,
    )


def test_toolset_b_requires_only_its_own_tool_actions_and_joint_state():
    module = _load_module()
    expected = {joint: 0.0 for joint in module.ARM_JOINTS}
    node = _Node(_mounted_joint_state(module, expected, toolset="B"))
    observed_actions = []

    def client(_node, _action_type, action_name):
        observed_actions.append(action_name)
        return _Client()

    with patch.object(module, "ActionClient", side_effect=client):
        module.verify_initial_pose(
            node,
            expected,
            timeout_sec=0.0,
            toolset="B",
        )

    assert set(observed_actions) == {
        action for _label, action in module.ARM_CONTROLLERS
    } | {
        action for _label, action in module.TOOLSET_CONTROLLERS["B"]
    }


def test_missing_mounted_tool_joint_is_a_startup_failure():
    module = _load_module()
    expected = {joint: 0.0 for joint in module.ARM_JOINTS}
    # Build the mapping directly so the missing-joint assertion names the
    # physical B tool rather than an unrelated arm joint.
    measured = dict(expected)
    measured.update({name: 0.0 for name in module.TOOLSET_JOINTS["B"]})
    missing = module.TOOLSET_JOINTS["B"][0]
    measured.pop(missing)
    with patch.object(module, "ActionClient", side_effect=_ready_clients):
        with pytest.raises(module.InitialPoseError, match=missing):
            module.verify_initial_pose(
                _Node(_joint_state(measured)),
                expected,
                timeout_sec=0.0,
                toolset="B",
            )


@pytest.mark.parametrize("value", [True, float("nan"), "0.0"])
def test_initial_position_file_rejects_nonfinite_or_nonnumeric_values(
    tmp_path,
    value,
):
    module = _load_module()
    positions = {joint: 0.0 for joint in module.ARM_JOINTS}
    positions["l_arm_0_joint"] = value
    path = tmp_path / "positions.yaml"
    path.write_text(
        yaml.safe_dump({"initial_positions": positions}),
        encoding="utf-8",
    )
    with pytest.raises(module.InitialPoseError, match="finite number"):
        module.load_expected_positions(path)


def _srdf_home_positions():
    # Each toolset SRDF carries the same 14-joint dual-arm home state; the
    # union over both sets must match the shared spawn/reset pose exactly.
    result = {}
    for srdf_name in (
        "xczs_inspection_robot_toolset_A.srdf",
        "xczs_inspection_robot_toolset_B.srdf",
    ):
        root = ElementTree.parse(
            MOVEIT_ROOT / "config" / srdf_name
        ).getroot()
        for state in root.findall("group_state"):
            if state.attrib.get("name") != "home":
                continue
            result.update(
                {
                    joint.attrib["name"]: float(joint.attrib["value"])
                    for joint in state.findall("joint")
                }
            )
    return result


def _adapter_defaults(path):
    parameters = yaml.safe_load(path.read_text(encoding="utf-8"))[
        "/**"
    ]["ros__parameters"]
    limits = parameters["manual_joint_limits"]
    return {
        joint: float(limits[joint]["default_position"])
        for joint in limits
        if joint.startswith(("l_arm_", "r_arm_"))
    }


def _generated_ros2_control_positions(initial_positions_file):
    document = xacro.process_file(
        str(
            DESCRIPTION_ROOT
            / "urdf"
            / "xczs_inspection_robot.urdf.xacro"
        ),
        mappings={"initial_positions_file": str(initial_positions_file)},
    )
    root = ElementTree.fromstring(document.toxml())
    result = {}
    for joint in root.find("ros2_control").findall("joint"):
        if not joint.attrib["name"].startswith(("l_arm_", "r_arm_")):
            continue
        interface = joint.find("state_interface[@name='position']")
        initial = interface.find("param[@name='initial_value']")
        result[joint.attrib["name"]] = float(initial.text)
    return result


def test_spawn_moveit_home_and_reset_share_one_arm_pose():
    module = _load_module()
    initial_positions_file = DESCRIPTION_ROOT / "config" / "initial_positions.yaml"
    expected = module.load_expected_positions(initial_positions_file)
    builtin_defaults = _adapter_defaults(
        CONTROL_ROOT / "config" / "cabinet_robot_adapter.yaml"
    )
    sample_defaults = _adapter_defaults(
        WORKSPACE_ROOT / "jiang" / "samples" / "demo_cabinet"
        / "cabinet_robot_adapter.yaml"
    )

    assert _generated_ros2_control_positions(initial_positions_file) == expected
    assert _srdf_home_positions() == expected
    assert builtin_defaults == expected
    assert sample_defaults == expected


def test_main_returns_nonzero_and_cleans_up_on_verification_failure():
    module = _load_module()
    node = _Node()
    expected = {joint: 0.0 for joint in module.ARM_JOINTS}
    with (
        patch.object(module, "default_positions_file", return_value="pose.yaml"),
        patch.object(module, "load_expected_positions", return_value=expected),
        patch.object(module.rclpy, "init") as init,
        patch.object(module, "Node", return_value=node),
        patch.object(
            module,
            "verify_initial_pose",
            side_effect=module.InitialPoseError("not ready"),
        ),
        patch.object(module.rclpy, "try_shutdown") as shutdown,
    ):
        assert module.main(args=[]) == 1

    init.assert_called_once_with()
    shutdown.assert_called_once_with()
    assert node.destroyed
    assert (
        "error",
        "Initial pose verification failed: not ready",
    ) in node.logger.messages
