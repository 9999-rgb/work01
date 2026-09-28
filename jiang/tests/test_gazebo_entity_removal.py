"""同名场景重建前必须等到 Gazebo 实际完成异步删除。"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control_gateway.gazebo_client import GazeboClient
from control_gateway.ros_node import ControlRequestError


class EntityRemovalTests(unittest.TestCase):
    def client(self, names, *, deleted=True):
        client = object.__new__(GazeboClient)
        client._delete_client = object()
        client._get_model_list_client = object()
        client._call = Mock(side_effect=[
            SimpleNamespace(success=deleted, status_message="model does not exist"),
            *[SimpleNamespace(success=True, model_names=value) for value in names],
        ])
        return client

    @patch("control_gateway.gazebo_client.time.sleep")
    def test_waits_for_stable_absence(self, _sleep):
        client = self.client([["scene"], [], ["scene"], [], []])
        client.delete_entity("scene")
        self.assertEqual(client._call.call_count, 6)

    @patch("control_gateway.gazebo_client.time.sleep")
    def test_missing_response_still_checks_pending_deletion(self, _sleep):
        client = self.client([[], []], deleted=False)
        client.delete_entity("scene", ignore_missing=True)
        self.assertEqual(client._call.call_count, 3)

    def test_timeout_does_not_authorize_respawn(self):
        client = self.client([])
        with self.assertRaisesRegex(ControlRequestError, "before timeout"):
            client.delete_entity("scene", timeout_sec=0)

    def test_failed_model_list_is_not_absence(self):
        client = self.client([])
        client._call.side_effect = [SimpleNamespace(success=True),
                                    SimpleNamespace(success=False)]
        with self.assertRaisesRegex(ControlRequestError, "verify entity removal"):
            client.delete_entity("scene")
