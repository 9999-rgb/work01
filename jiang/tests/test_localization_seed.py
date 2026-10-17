"""A known physical pose must remain local when AMCL is re-seeded."""
import math
from types import SimpleNamespace

from jiang.control_gateway.ros_node import RosControlNode


def test_measured_pose_seed_stays_inside_local_correction_neighbourhood():
    messages = []
    fake = SimpleNamespace(
        _validated_station_number=lambda value, _name, **_kwargs: float(value),
        _initial_pose_publisher=SimpleNamespace(publish=messages.append),
    )
    RosControlNode.publish_initial_pose(fake, -30.0, 5.8, .2, stamp_zero=True)
    message, = messages
    assert message.pose.pose.position.x == -30.0
    assert message.pose.pose.position.y == 5.8
    assert message.header.stamp.sec == 0
    assert message.header.frame_id == 'map'
    # Three-sigma spread must fit the 6 cm spawn-odom agreement gate. The old
    # 0.5 m sigma could place seeds outside either wall of this narrow aisle.
    for index in (0, 7):
        assert 0 < 3 * math.sqrt(message.pose.covariance[index]) <= .06
    assert 0 < 3 * math.sqrt(message.pose.covariance[35]) <= .05
