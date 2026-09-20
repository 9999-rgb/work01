#!/usr/bin/env python3
"""抽拉执行器回归检查；先 source ROS 2 与工作区环境，再直接运行。"""
from pathlib import Path
from types import SimpleNamespace
import math
import sys
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import taught_sequence_ros as worker


class DrawerSafetyTests(unittest.TestCase):
    def test_pull_timing_uses_cartesian_distance_not_point_index(self):
        samples = iter((0.0, 0.01, 0.04, 0.05))

        def call(*args):
            pose = PoseStamped()
            pose.pose.position.x = next(samples)
            return SimpleNamespace(error_code=SimpleNamespace(val=1), pose_stamped=[pose])

        trajectory = JointTrajectory(joint_names=['test'], points=[
            JointTrajectoryPoint(positions=[float(i)]) for i in range(4)])
        node = SimpleNamespace(_fk_cli=None, _call=call)
        worker.TaughtRosWorker._retime_drawer_path(node, 'left', trajectory, 3.0)
        for point, expected in zip(trajectory.points, (0.0, 0.6, 2.4, 3.0)):
            actual = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            self.assertAlmostEqual(actual, expected)

    def test_transit_obstacle_is_removed_when_planning_fails(self):
        joints = worker.ARMS['left']['joints']
        pose = PoseStamped()
        pose.pose.position.x = .55
        operations = []
        planning_calls = []

        def call(client, request, timeout, label):
            if label == '准备姿态正解':
                return SimpleNamespace(error_code=SimpleNamespace(val=1), pose_stamped=[pose])
            if label == '柜外准备路径规划':
                planning_calls.append(label)
                return SimpleNamespace(motion_plan_response=SimpleNamespace(
                    error_code=SimpleNamespace(val=-2)))
            operations.append(request.scene.world.collision_objects[0].operation)
            return SimpleNamespace(success=True)

        node = SimpleNamespace(
            _fk_cli=None, _motion_plan_cli=None, _planning_scene_cli=None,
            _joint_state=JointState(name=list(joints)), _measured=lambda _: 0.0,
            _tip_pose=lambda _: ((1.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
            _check_cancel=lambda: None, _call=call)
        with self.assertRaisesRegex(RuntimeError, '无法规划柜外准备路径'):
            worker.TaughtRosWorker._move_arm_to(node, 'left', {j: 0.0 for j in joints})
        self.assertEqual(len(planning_calls), 3)
        self.assertEqual(operations, [worker.CollisionObject.ADD, worker.CollisionObject.REMOVE])

    def test_drawer_playback_matches_motion_contract(self):
        root = Path(__file__).resolve().parents[2]
        scene = ET.parse(root / 'xczs_inspection_robot_description/urdf/scenes/electrical_mezzanine.xacro')
        sequences = worker.yaml.safe_load((worker.TAUGHT_POSES_DIR / 'db1_sequence.yaml').read_text())['sequences']
        controls = {c.findtext('control_id'): c for c in scene.findall('.//control')}
        for cid in ('db1', 'dm1', 'ds2', 'ds3'):
            for command, target in (('open', .05), ('closed', 0.0)):
                steps = sequences[cid][command]['steps']
                pull = next(s for s in steps if s['type'] == 'pull_drawer')
                self.assertEqual(pull['target'], target)
                share = .7 if pull.get('support_enabled', True) else 1.0
                self.assertEqual(float(controls[cid].findtext('playback_follow_arm_share')), share)
                if cid in ('ds2', 'ds3'):
                    self.assertEqual(share, 1.0)
                    self.assertFalse(any(s.get('role') == 'support' for s in steps))

    def test_rejects_invalid_joint_feedback(self):
        node = SimpleNamespace(_joint_state=JointState(name=['test'], position=[math.nan]))
        with self.assertRaisesRegex(RuntimeError, '反馈无效'):
            worker.TaughtRosWorker._measured(node, 'test')

    def test_rejects_joint_branch_jump_and_nan(self):
        joints = worker.ARMS['left']['joints']
        for bad in (0.36, math.nan):
            with self.subTest(value=bad):
                trajectory = JointTrajectory(joint_names=list(joints), points=[
                    JointTrajectoryPoint(positions=[0.0] * len(joints)),
                    JointTrajectoryPoint(positions=[bad] + [0.0] * (len(joints)-1))])
                def call(client, request, timeout, label):
                    if label == '直线起点正解':
                        return SimpleNamespace(error_code=SimpleNamespace(val=1),
                                               pose_stamped=[PoseStamped()])
                    return SimpleNamespace(solution=SimpleNamespace(joint_trajectory=trajectory), fraction=1.0)

                node = SimpleNamespace(
                    _tip_pose=lambda _: ((0., 0., 0.), (0., 0., 0., 1.)),
                    get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: JointState().header.stamp)),
                    _joint_state=JointState(), _cartesian_cli=None, _fk_cli=None,
                    _measured=lambda _: 0.0,
                    _call=call)
                with self.assertRaisesRegex(RuntimeError, '关节跳变'):
                    worker.TaughtRosWorker._plan_translate(node, 'left', 'x', .05)

    def test_cartesian_target_uses_same_joint_snapshot_as_start(self):
        joints = worker.ARMS['left']['joints']
        snapshot = JointState(name=list(joints), position=[0.0] * len(joints))
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.orientation.w = 1.0
        trajectory = JointTrajectory(joint_names=list(joints), points=[
            JointTrajectoryPoint(positions=[0.0] * len(joints))])

        def call(client, request, timeout, label):
            if label == '直线起点正解':
                self.assertIs(request.robot_state.joint_state, snapshot)
                # 模拟新反馈到达；不能替换这次已捕获的规划起点。
                node._joint_state = JointState(name=list(joints), position=[.01] * len(joints))
                return SimpleNamespace(error_code=SimpleNamespace(val=1), pose_stamped=[pose])
            self.assertIs(request.start_state.joint_state, snapshot)
            self.assertAlmostEqual(request.waypoints[0].position.x, 1.05)
            return SimpleNamespace(solution=SimpleNamespace(joint_trajectory=trajectory), fraction=1.0)

        node = SimpleNamespace(
            _joint_state=snapshot, _fk_cli=None, _cartesian_cli=None,
            _tip_pose=Mock(return_value=((99., 0., 0.), (0., 0., 0., 1.))),
            _measured=lambda _: 0.0, _call=call,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: JointState().header.stamp)))
        worker.TaughtRosWorker._plan_translate(node, 'left', 'x', .05)
        node._tip_pose.assert_not_called()

    def test_repeat_target_uses_measured_position(self):
        node = SimpleNamespace(_cabinet='test', _rails={'ds3': .05}, _rail_subscriptions={},
                               _rail_cb=lambda _: Mock(),
                               create_subscription=Mock(return_value='subscription'),
                               destroy_subscription=Mock())
        node._ensure_rail_subscription = lambda cid: worker.TaughtRosWorker._ensure_rail_subscription(node, cid)
        with patch.object(worker, 'wait_for'):
            self.assertTrue(worker.TaughtRosWorker.drawer_already_at_target(node, 'ds3', .05))
            self.assertTrue(node.drawer_result['already_at_target'])
            node._rails['ds3'] = .01
            self.assertFalse(worker.TaughtRosWorker.drawer_already_at_target(node, 'ds3', .05))
            node._rails['ds3'] = math.nan
            with self.assertRaisesRegex(RuntimeError, '反馈无效'):
                worker.TaughtRosWorker.drawer_already_at_target(node, 'ds3', .05)
        self.assertEqual(node.create_subscription.call_count, 1)
        node.destroy_subscription.assert_not_called()


if __name__ == '__main__':
    unittest.main()
