#!/usr/bin/env python3


from os import path
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2, MoveIt2Servo
from pymoveit2.robots import ur5
import tf2_ros
import math
import re
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import transforms3d as tf3d # type: ignore
import numpy as np
from std_msgs.msg import Bool
import yaml

from tf_transformations import quaternion_from_euler, euler_from_quaternion


from linkattacher_msgs.srv import AttachLink, DetachLink

def main():
    rclpy.init()
    node = Node("add_meshobj")
    floor = path.join(
        path.dirname(path.realpath(__file__)), "..", "assets", "floor.stl"
    )
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    for i in range(1, 4):
        moveit2.add_collision_mesh(
        filepath=floor,
        id="Floor",
        position=[2.1, 0.00, 0.1],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id="base_link",
        )
        
        time.sleep(0.5)
        moveit2.add_collision_mesh(
        filepath=floor,
        id="BackFloor",
        position=[-0.6, 0.00, 0.05],
        quat_xyzw=[0, 0.681639 ,0, 0.731689],
        frame_id="base_link",
        )
        time.sleep(0.5)
        
        print("Floor adding...")
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
