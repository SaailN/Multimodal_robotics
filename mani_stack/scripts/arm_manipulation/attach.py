#!/usr/bin/env python3

is_sim = True

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

if is_sim == True:
    from linkattacher_msgs.srv import AttachLink, DetachLink
else:
    from ur_msgs.srv import SetIO  # type: ignore
    from controller_manager_msgs.srv import SwitchController
def main():
    rclpy.init()
    node = Node("attach")
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2Servo = MoveIt2Servo(
        node=node, frame_id=ur5.base_link_name(), callback_group=callback_group
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    def controlGripper(status, box_name,node):
        if status == "ON":
            node.gripper_control = node.create_client(AttachLink, "/GripperMagnetON")
            node.req = AttachLink.Request()
        else:
            node.gripper_control = node.create_client(DetachLink, "/GripperMagnetOFF")
            node.req = DetachLink.Request()

        while not node.gripper_control.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("EEF service not available, waiting again...")

        node.req.model1_name = box_name
        node.req.link1_name = "link"
        node.req.model2_name = "ur5"
        node.req.link2_name = "wrist_3_link"
        print("ur5 ->", box_name)
        time.sleep(0.2)
        gripperServicefuture = node.gripper_control.call_async(node.req)
        rclpy.spin_until_future_complete(node, gripperServicefuture)
        print("Gripper Status: ", status, "has been requested")
    controlGripper("ON", "banana",node)
    
    
if __name__ == "__main__":
    main()