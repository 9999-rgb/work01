"""Pure-contract tests for the world-preserving toolset supervisor.

These tests deliberately do not start Gazebo.  They protect the command-line
contract that keeps the world process separate from the replaceable robot
child, especially the per-toolset MoveIt path expansion.
"""

from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import patch

import pytest


CONTROL_ROOT = Path(__file__).resolve().parents[1]


def _load_module():
    script_path = CONTROL_ROOT / "scripts" / "toolset_supervisor.py"
    spec = spec_from_file_location("toolset_supervisor", script_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_normalizes_only_the_two_declared_toolsets():
    module = _load_module()

    assert module.normalize_toolset(" b ") == "B"
    with pytest.raises(module.ToolsetSupervisorError, match="A or B"):
        module.normalize_toolset("C")


def test_static_child_arguments_cannot_override_supervisor_ownership():
    module = _load_module()

    with pytest.raises(module.ToolsetSupervisorError, match="gazebo"):
        module._parse_launch_arguments(["gazebo:=true"])
    with pytest.raises(
        module.ToolsetSupervisorError,
        match="gazebo_plugin_instance_id",
    ):
        module._parse_launch_arguments(["gazebo_plugin_instance_id:=external"])


@pytest.mark.parametrize(
    "name",
    ("moveit_srdf", "moveit_joint_limits", "moveit_controllers"),
)
def test_toolset_specific_moveit_override_requires_a_safe_template(name):
    module = _load_module()

    with pytest.raises(module.ToolsetSupervisorError, match="toolset"):
        module._parse_launch_arguments([f"{name}:=/tmp/toolset_A.yaml"])


def test_expands_only_the_audited_moveit_path_template():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._launch_arguments = module._parse_launch_arguments(
        (
            "moveit_srdf:=/profiles/xczs_toolset_{toolset}.srdf",
            "moveit_joint_limits:=/profiles/joint_limits_{toolset}.yaml",
            "moveit_controllers:=/profiles/controllers_{toolset}.yaml",
            "robot_xacro:=/profiles/robot_{toolset}.xacro",
        )
    )

    assert supervisor._expanded_launch_arguments("B") == (
        "moveit_srdf:=/profiles/xczs_toolset_B.srdf",
        "moveit_joint_limits:=/profiles/joint_limits_B.yaml",
        "moveit_controllers:=/profiles/controllers_B.yaml",
        "robot_xacro:=/profiles/robot_{toolset}.xacro",
    )


def test_capture_yaw_helper_normalizes_a_nonunit_quaternion():
    module = _load_module()
    orientation = SimpleNamespace(x=0.0, y=0.0, z=2.0, w=0.0)

    assert math.isclose(abs(module._quaternion_yaw(orientation)), math.pi)


def test_switch_request_is_rejected_when_the_supervisor_is_not_ready():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    supervisor._stopping = False
    supervisor._stop_requested = threading.Event()
    supervisor._status = {
        "state": "failed",
        "ready": False,
        "active_toolset": "A",
        "target_toolset": None,
        "generation": 7,
    }
    response = SimpleNamespace()

    result = supervisor._switch_service_callback(
        SimpleNamespace(toolset="B", expected_generation=7),
        response,
    )

    assert result.accepted is False
    assert result.state == "failed"
    assert "not ready" in result.message


def test_stopping_supervisor_cannot_spawn_a_new_robot_child():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    supervisor._stopping = True

    with pytest.raises(module.ToolsetSupervisorError, match="stopping"):
        supervisor._start_robot_child("A", pose=None, spawn_cabinet=False)


def test_each_robot_child_gets_a_unique_valid_gazebo_plugin_instance_id():
    module = _load_module()

    class FakeChild:
        def __init__(self, pid):
            self.pid = pid

    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    supervisor._stopping = False
    supervisor._child_process = None
    supervisor._child_process_group_id = None
    supervisor._child_instance_sequence = 0
    supervisor._launch_package = "xczs_inspection_robot_control"
    supervisor._launch_file = "inspection_robot.launch.py"
    supervisor._launch_arguments = ()
    supervisor._cabinet_bringup = True
    supervisor._intentional_child_stop = False
    supervisor._latest_joint_positions = {}
    supervisor._latest_joint_received_monotonic = None
    supervisor._nav2_lifecycle_active = False
    supervisor._next_nav2_lifecycle_probe_monotonic = 0.0
    supervisor._nav2_lifecycle_status_detail = ""
    supervisor.get_logger = lambda: SimpleNamespace(info=lambda _message: None)

    with (
        patch.object(
            module.subprocess,
            "Popen",
            side_effect=(FakeChild(1001), FakeChild(1002)),
        ) as popen,
        patch.object(
            module.ToolsetSupervisor,
            "_reset_nav2_lifecycle_probe_locked",
        ),
        patch.object(
            module.ToolsetSupervisor,
            "_reset_gazebo_global_args",
        ) as reset_global_args,
    ):
        supervisor._start_robot_child("B", pose=None, spawn_cabinet=False)
        # Model the first child after a completed stop.  A rollback may restore
        # the same B toolset, which must still get a fresh plugin node suffix.
        supervisor._child_process = None
        supervisor._child_process_group_id = None
        supervisor._start_robot_child("B", pose=None, spawn_cabinet=False)

    first_command = popen.call_args_list[0].args[0]
    second_command = popen.call_args_list[1].args[0]
    assert "gazebo_plugin_instance_id:=robot_b_1" in first_command
    assert "gazebo_plugin_instance_id:=robot_b_2" in second_command
    assert module._GAZEBO_PLUGIN_INSTANCE_ID_PATTERN.fullmatch("robot_b_1")
    assert module._GAZEBO_PLUGIN_INSTANCE_ID_PATTERN.fullmatch("robot_b_2")
    assert [call.kwargs for call in reset_global_args.call_args_list] == [
        {"required": True},
        {"required": True},
    ]


def test_global_args_guard_is_required_for_a_toolset_switch():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._global_args_guard_client = object()
    supervisor._service_timeout_sec = 4.0

    with patch.object(
        module.ToolsetSupervisor,
        "_call_service",
        side_effect=module.ToolsetSupervisorError("guard unavailable"),
    ) as call_service:
        with pytest.raises(module.ToolsetSupervisorError, match="guard unavailable"):
            supervisor._reset_gazebo_global_args(required=True)

    assert call_service.call_count == 1
    assert call_service.call_args.kwargs["timeout_sec"] == 4.0


def test_initial_spawn_can_continue_before_global_args_guard_is_ready():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._global_args_guard_client = object()
    supervisor._service_timeout_sec = 4.0
    warnings = []
    supervisor.get_logger = lambda: SimpleNamespace(
        warn=lambda *args: warnings.append(args)
    )

    with patch.object(
        module.ToolsetSupervisor,
        "_call_service",
        side_effect=module.ToolsetSupervisorError("guard unavailable"),
    ) as call_service:
        supervisor._reset_gazebo_global_args(required=False)

    assert call_service.call_count == 1
    assert call_service.call_args.kwargs["timeout_sec"] == module.GUARD_INITIAL_WAIT_SEC
    assert warnings


def test_entity_removal_requires_two_consecutive_absent_observations():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._service_timeout_sec = 1.0
    supervisor._robot_name = "xczs_inspection_robot"
    supervisor._get_model_list_client = object()
    supervisor._ensure_not_stopping = lambda: None

    responses = (
        SimpleNamespace(
            success=True,
            model_names=("ground_plane", "xczs_inspection_robot"),
        ),
        SimpleNamespace(success=True, model_names=("ground_plane",)),
        SimpleNamespace(success=True, model_names=("ground_plane",)),
    )
    with (
        patch.object(
            module.ToolsetSupervisor,
            "_call_service",
            side_effect=responses,
        ) as call_service,
        patch.object(module.time, "sleep") as sleep,
    ):
        supervisor._wait_for_robot_entity_absent()

    assert call_service.call_count == 3
    assert sleep.call_count == 2
    for invocation in call_service.call_args_list:
        assert invocation.args[2] == "verify robot entity removal"
        request = invocation.args[1]
        assert isinstance(request, module.GetModelList.Request)
        assert 0.0 < invocation.kwargs["timeout_sec"] <= 1.0


def test_entity_removal_rejects_a_failed_model_list_query():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._service_timeout_sec = 1.0
    supervisor._robot_name = "xczs_inspection_robot"
    supervisor._get_model_list_client = object()
    supervisor._ensure_not_stopping = lambda: None

    with patch.object(
        module.ToolsetSupervisor,
        "_call_service",
        return_value=SimpleNamespace(success=False, model_names=()),
    ):
        with pytest.raises(module.ToolsetSupervisorError, match="could not list"):
            supervisor._wait_for_robot_entity_absent()


def test_entity_removal_accepts_an_empty_successful_model_list():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._service_timeout_sec = 1.0
    supervisor._robot_name = "xczs_inspection_robot"
    supervisor._get_model_list_client = object()
    supervisor._ensure_not_stopping = lambda: None

    with (
        patch.object(
            module.ToolsetSupervisor,
            "_call_service",
            side_effect=(
                SimpleNamespace(success=True, model_names=()),
                SimpleNamespace(success=True, model_names=()),
            ),
        ) as call_service,
        patch.object(module.time, "sleep"),
    ):
        supervisor._wait_for_robot_entity_absent()

    assert call_service.call_count == 2


def test_pose_capture_failure_preserves_the_unchanged_previous_stack():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    supervisor._stopping = False
    supervisor._stop_requested = threading.Event()
    supervisor._fatal_error = None
    supervisor._status = {
        "generation": 8,
        "state": "switching",
        "stage": "queued",
        "ready": False,
        "active_toolset": "A",
        "target_toolset": "B",
    }
    supervisor._status_publisher = SimpleNamespace(publish=lambda _message: None)
    supervisor.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None,
    )

    with (
        patch.object(
            module.ToolsetSupervisor,
            "_capture_robot_pose",
            side_effect=module.ToolsetSupervisorError("pose service timed out"),
        ),
        patch.object(
            module.ToolsetSupervisor,
            "_wait_for_robot_ready",
        ) as wait_ready,
        patch.object(
            module.ToolsetSupervisor,
            "_stop_current_robot_child",
        ) as stop_child,
        patch.object(
            module.ToolsetSupervisor,
            "_delete_robot_entity",
        ) as delete_entity,
        patch.object(
            module.ToolsetSupervisor,
            "_start_robot_child",
        ) as start_child,
        patch.object(module.ToolsetSupervisor, "_rollback") as rollback,
    ):
        supervisor._switch_worker("A", "B", 8, "toolset-test")

    wait_ready.assert_called_once_with("A")
    stop_child.assert_not_called()
    delete_entity.assert_not_called()
    start_child.assert_not_called()
    rollback.assert_not_called()
    assert supervisor._fatal_error is None
    assert supervisor._status["state"] == "ready"
    assert supervisor._status["stage"] == "preflight_failed"
    assert supervisor._status["ready"] is True
    assert supervisor._status["active_toolset"] == "A"
    assert supervisor._status["target_toolset"] is None
    assert "pose service timed out" in supervisor._status["last_error"]


def test_rollback_stops_a_residual_child_even_before_target_started_flag():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    supervisor._stopping = False
    supervisor._stop_requested = threading.Event()
    supervisor._child_process = object()
    events = []
    supervisor.get_logger = lambda: SimpleNamespace(error=lambda _message: None)

    with (
        patch.object(
            module.ToolsetSupervisor,
            "_set_switch_stage",
            side_effect=lambda *_args: events.append("stage"),
        ),
        patch.object(
            module.ToolsetSupervisor,
            "_stop_current_robot_child",
            side_effect=lambda: events.append("stop"),
        ) as stop_child,
        patch.object(
            module.ToolsetSupervisor,
            "_delete_robot_entity",
            side_effect=lambda **_kwargs: events.append("delete"),
        ),
        patch.object(
            module.ToolsetSupervisor,
            "_start_robot_child",
            side_effect=lambda *_args, **_kwargs: events.append("start"),
        ),
        patch.object(
            module.ToolsetSupervisor,
            "_wait_for_robot_ready",
            side_effect=lambda *_args: events.append("ready"),
        ),
    ):
        restored = supervisor._rollback(
            "A",
            {"x": 0.0, "y": 0.0, "z": 0.58, "yaw": 0.0},
            False,
            9,
        )

    assert restored is True
    stop_child.assert_called_once_with()
    assert events == ["stage", "stop", "delete", "start", "ready"]


def test_supervisor_does_not_forward_child_launch_arguments_to_rcl():
    source = (
        CONTROL_ROOT / "scripts" / "toolset_supervisor.py"
    ).read_text(encoding="utf-8")

    assert "rclpy.init(args=[])" in source


def test_stopping_child_reaps_the_launch_leader_before_group_check():
    module = _load_module()

    class FakeChild:
        def __init__(self):
            self.poll_count = 0

        def poll(self):
            self.poll_count += 1
            return 0

    child = FakeChild()
    with (
        patch.object(
            module.ToolsetSupervisor,
            "_process_group_is_alive",
            side_effect=(True, False),
        ),
        patch.object(module.time, "sleep"),
    ):
        assert module.ToolsetSupervisor._wait_for_process_group_exit(
            child,
            4242,
            1.0,
        )

    assert child.poll_count >= 2


def test_child_cleanup_does_not_skip_a_group_when_its_leader_has_exited():
    module = _load_module()

    class FakeChild:
        pid = 4242

        @staticmethod
        def poll():
            return 0

    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    supervisor._child_process = FakeChild()
    supervisor._child_process_group_id = 4242
    supervisor._intentional_child_stop = False
    supervisor._child_stop_timeout_sec = 1.0

    with (
        patch.object(
            module.ToolsetSupervisor,
            "_process_group_is_alive",
            return_value=True,
        ),
        patch.object(
            module.ToolsetSupervisor,
            "_signal_child_process_group",
        ) as signal_group,
        patch.object(
            module.ToolsetSupervisor,
            "_wait_for_process_group_exit",
            return_value=True,
        ),
    ):
        supervisor._stop_child_process(supervisor._child_process)

    signal_group.assert_called_once_with(4242, module.signal.SIGINT)


def test_nav2_lifecycle_probe_requires_the_standard_trigger_to_succeed():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    client = SimpleNamespace(service_is_ready=lambda: True)
    supervisor._nav2_lifecycle_active_client = client
    supervisor._nav2_lifecycle_client_generation = 1
    supervisor._nav2_lifecycle_active = False
    supervisor._next_nav2_lifecycle_probe_monotonic = 0.0
    supervisor._nav2_lifecycle_status_detail = ""
    supervisor._service_timeout_sec = 10.0
    response = SimpleNamespace(success=False, message="managed nodes are inactive")

    with (
        patch.object(module.time, "monotonic", return_value=1.0),
        patch.object(
            module.ToolsetSupervisor,
            "_call_service",
            return_value=response,
        ) as call_service,
    ):
        active, detail = supervisor._nav2_lifecycle_is_active()

    assert active is False
    assert "not active" in detail
    assert "managed nodes are inactive" in detail
    call_service.assert_called_once()
    assert call_service.call_args.args[0] is client
    assert isinstance(call_service.call_args.args[1], module.Trigger.Request)
    assert call_service.call_args.args[2] == "Nav2 lifecycle active"
    assert call_service.call_args.kwargs["timeout_sec"] == (
        module.NAV2_LIFECYCLE_REQUEST_TIMEOUT_SEC
    )
    assert module.DEFAULT_NAV2_LIFECYCLE_ACTIVE_SERVICE == (
        "/lifecycle_manager_navigation/is_active"
    )


def test_nav2_lifecycle_probe_is_rebound_for_every_robot_child():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    old_client = object()
    new_client = object()
    supervisor._nav2_lifecycle_active_client = old_client
    supervisor._nav2_lifecycle_client_generation = 4
    supervisor._nav2_lifecycle_active = True
    supervisor._next_nav2_lifecycle_probe_monotonic = 99.0
    supervisor._nav2_lifecycle_status_detail = "old stack active"
    destroyed = []
    created = []
    supervisor.destroy_client = lambda client: destroyed.append(client)
    supervisor.create_client = lambda service_type, service_name: (
        created.append((service_type, service_name)) or new_client
    )

    supervisor._reset_nav2_lifecycle_probe_locked()

    assert destroyed == [old_client]
    assert created == [
        (module.Trigger, module.DEFAULT_NAV2_LIFECYCLE_ACTIVE_SERVICE)
    ]
    assert supervisor._nav2_lifecycle_active_client is new_client
    assert supervisor._nav2_lifecycle_client_generation == 5
    assert supervisor._nav2_lifecycle_active is False
    assert supervisor._next_nav2_lifecycle_probe_monotonic == 0.0


def test_late_old_lifecycle_reply_cannot_mark_the_replacement_ready():
    module = _load_module()
    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    old_client = object()
    current_client = object()
    supervisor._nav2_lifecycle_active_client = current_client
    supervisor._nav2_lifecycle_client_generation = 2
    supervisor._nav2_lifecycle_active = False
    supervisor._nav2_lifecycle_status_detail = "new child pending"

    active, detail = supervisor._set_nav2_lifecycle_probe_result(
        old_client,
        1,
        True,
        "old child active",
    )

    assert active is False
    assert "changed during probe" in detail
    assert supervisor._nav2_lifecycle_active is False
    assert supervisor._nav2_lifecycle_status_detail == "new child pending"


def test_timed_out_service_request_is_removed_from_the_client():
    module = _load_module()

    class PendingFuture:
        def __init__(self):
            self.callbacks = []

        def add_done_callback(self, callback):
            self.callbacks.append(callback)

    class PendingClient:
        def __init__(self):
            self.future = PendingFuture()
            self.removed = []

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, _request):
            return self.future

        def remove_pending_request(self, future):
            self.removed.append(future)

    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._service_timeout_sec = 1.0
    supervisor._stop_requested = threading.Event()
    client = PendingClient()

    with pytest.raises(module.ToolsetSupervisorError, match="request timed out"):
        supervisor._call_service(
            client,
            object(),
            "Nav2 lifecycle active",
            timeout_sec=0.001,
        )

    assert client.removed == [client.future]


def test_nav2_lifecycle_inactive_blocks_readiness_even_when_navigation_action_exists():
    module = _load_module()

    class FakeChild:
        @staticmethod
        def poll():
            return None

    supervisor = object.__new__(module.ToolsetSupervisor)
    supervisor._lock = threading.RLock()
    supervisor._stop_requested = threading.Event()
    supervisor._child_process = FakeChild()
    supervisor._latest_joint_received_monotonic = 0.0
    supervisor._latest_joint_positions = {
        name: 0.0 for name in module.ARM_JOINTS + module.TOOLSET_JOINTS["A"]
    }
    supervisor._ready_timeout_sec = 0.01
    supervisor._require_nav2 = True
    supervisor._navigation_client = SimpleNamespace(server_is_ready=lambda: True)
    supervisor._require_cabinet_action = False
    supervisor._cabinet_action_clients = {}
    supervisor.get_logger = lambda: SimpleNamespace(info=lambda _message: None)

    with (
        patch.object(module.ToolsetSupervisor, "_action_ready", return_value=True),
        patch.object(
            module.ToolsetSupervisor,
            "_nav2_lifecycle_is_active",
            return_value=(False, "Nav2 lifecycle manager is not active"),
        ),
        patch.object(module.time, "monotonic", side_effect=(0.0, 0.0, 0.0, 0.02)),
        patch.object(module.time, "sleep"),
    ):
        with pytest.raises(
            module.ToolsetSupervisorError,
            match="Nav2 lifecycle manager is not active",
        ):
            supervisor._wait_for_robot_ready("A")
