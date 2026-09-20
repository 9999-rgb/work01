#!/usr/bin/env python3
"""记录 Gazebo 实际 link 位姿，检查电缸滑动副是否发生非预期转动。"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from gazebo_msgs.msg import LinkStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=90)
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('drawer_motion_recorder')
    stream = args.out.open('w')
    last = 0.0

    def receive(message):
        nonlocal last
        now = time.time()
        if now-last < .04:
            return
        last = now
        poses = {}
        for name, pose in zip(message.name, message.pose):
            short = name.split('::')[-1]
            if any(s in short for s in ('two_cyl', 'three_cyl')) or short in ('b1','s2','s3','m1','body'):
                poses[short] = [getattr(pose.position,a) for a in 'xyz'] + [getattr(pose.orientation,a) for a in 'xyzw']
        angles = {}
        for name, pose in poses.items():
            if 'finger' not in name:
                continue
            base = poses.get(name.split('finger')[0]+'base')
            if base:
                dot = abs(sum(a*b for a,b in zip(base[3:],pose[3:])))
                angles[name] = math.degrees(2*math.acos(min(1.0,dot)))
        stream.write(json.dumps({'time':now,'poses':poses,'rod_rotation_deg':angles})+'\n')
        stream.flush()

    node.create_subscription(LinkStates,'/link_states',receive,qos_profile_sensor_data)
    end = time.monotonic()+args.seconds
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
