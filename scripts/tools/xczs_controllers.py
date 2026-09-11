#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""xczs 机器人控制器与关节映射（scripts/tools 下各现场工具共用）。

这些映射来自 URDF 的 ros2_control 配置，属于**模型结构**、不是场景契约，所以放在
工具层的共享模块里，而不是每个工具各写一份；场景相关的东西（杆长、杆的角色、接触点）
仍然从场景适配 YAML 读，不在这里。

同时提供 `SpinNode` 基类：rclpy 的 `Node.executor` 只存弱引用，必须用另一个强属性
保住 executor，否则它立刻被 GC、所有回调停摆——这个坑在四个工具里各踩过一次，
统一收在这里。

用法：
    from xczs_controllers import ARMS, ROD_SIDES, SpinNode, best_effort_qos

本模块只描述"谁归谁管"，不下发任何指令。
"""
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

WORLD_FRAME = "odom"
# 注意不是 /joint_states：本项目这份是 BEST_EFFORT 发布的，默认 RELIABLE 订阅收不到。
JOINT_STATES_TOPIC = "/xczs/joint_states"

# 双臂：MoveIt 的规划组名 / 末端 tip link / 控制器 action / 该控制器下 7 个关节。
ARMS = {
    "left": {
        "group": "left_arm",
        "tip": "l_arm_6",
        "action": "/xczs/left_arm_controller/follow_joint_trajectory",
        "joints": ["l_arm_%d_joint" % i for i in range(7)],
    },
    "right": {
        "group": "right_arm",
        "tip": "r_arm_6",
        "action": "/xczs/right_arm_controller/follow_joint_trajectory",
        "joints": ["r_arm_%d_joint" % i for i in range(7)],
    },
}

# 两侧电缸：左手两根、右手三根，各由一个控制器整组驱动。
ROD_SIDES = {
    "left": {
        "action": "/xczs/two_cylinder_controller/follow_joint_trajectory",
        "joints": ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint"],
    },
    "right": {
        "action": "/xczs/three_cylinder_controller/follow_joint_trajectory",
        "joints": ["r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
                   "r_three_cyl_finger3_joint"],
    },
}


def control_groups():
    """{action: [关节名]} —— 按控制器整组下发时用（一个控制器一条轨迹）。"""
    groups = {}
    for config in list(ARMS.values()) + list(ROD_SIDES.values()):
        groups[config["action"]] = list(config["joints"])
    return groups


def best_effort_qos(depth=10):
    """订阅 /xczs/joint_states 用的 QoS：发布端是 BEST_EFFORT。"""
    return QoSProfile(depth=depth, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      history=QoSHistoryPolicy.KEEP_LAST)


def wait_for(predicate, timeout, what):
    """轮询 predicate 直到为真，超时抛 RuntimeError。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    raise RuntimeError("timeout waiting for %s" % what)


class SpinNode(Node):
    """带常驻 spin 线程的 Node。

    用 MultiThreadedExecutor 是因为同一个节点里既要跑 action 客户端回调、又要收
    订阅消息，单线程 executor 会被长回调堵住。
    """

    def __init__(self, name, num_threads=4, **kwargs):
        super().__init__(name, **kwargs)
        # executor 必须显式带上**节点自己的 context**：默认构造会用全局
        # context，而 Web 任务层用的是私有 context（未初始化）——那样会在
        # GuardCondition 里炸成 `AttributeError: __enter__`。
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(
            num_threads=num_threads, context=self.context)
        self._spin_executor.add_node(self)
        self._spin_thread = threading.Thread(
            target=self._spin_executor.spin, daemon=True)
        self._spin_thread.start()

    def stop(self):
        if self._spin_thread.is_alive():
            self._spin_executor.shutdown()
            self._spin_thread.join(timeout=3.0)
