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
from mani_stack.srv import SendPose
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from linkattacher_msgs.srv import AttachLink, DetachLink
global servo_status
servo_status = 5
current_joint_states = [0, 0, 0, 0, 0, 0]

def main():
    rclpy.init()
    node = Node("moveToPoseNode")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    def getCurrentPose(useEuler=False):
        tempPose = [0, 0, 0]
        tempQuats = [0, 0, 0, 0]
        transform = tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
        tempPose[0] = round(transform.transform.translation.x, 7)
        tempPose[1] = round(transform.transform.translation.y, 7)
        tempPose[2] = round(transform.transform.translation.z, 7)
        tempQuats[0] = transform.transform.rotation.x
        tempQuats[1] = transform.transform.rotation.y
        tempQuats[2] = transform.transform.rotation.z
        tempQuats[3] = transform.transform.rotation.w
        if useEuler == True:
            tempQuats[0] = transform.transform.rotation.w
            tempQuats[1] = transform.transform.rotation.x
            tempQuats[2] = transform.transform.rotation.y
            tempQuats[3] = transform.transform.rotation.z
            tempQuats = tf3d.euler.quat2euler(tempQuats)
        return tempPose, tempQuats
    
    
    def moveToPoseService(Request,Response):
        
        node.get_logger().info("Request Arrived")
        postion = (Request.x,Request.y,Request.z)
        quaterion = (Request.xr,Request.yr,Request.zr,Request.wr)
        print("Postion: ",postion)
        moveit_callback_group = ReentrantCallbackGroup()
        moveitnode = Node("moveit2_node_pose")
        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=moveitnode,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=moveit_callback_group,
        )
        moveitexecutor = rclpy.executors.MultiThreadedExecutor(4)
        moveitexecutor.add_node(moveitnode)
        moveit_executor_thread = Thread(target=moveitexecutor.spin, daemon=True, args=())
        moveit_executor_thread.start()
        counter = 0
        while counter < 5:
            moveit2.move_to_pose(position=postion, quat_xyzw=quaterion, cartesian=False)
            statuts = moveit2.wait_until_executed()
            if statuts == True:
                break
            else:
                counter += 1            
                
        print("status",statuts)
        Response.success = True
        Response.message = "Success"
        print("done")
        node.get_logger().info("Request done with Succes")
        moveitnode.destroy_node()
        return Response
    dock_control_srv = node.create_service(SendPose, '/movetoPose', moveToPoseService, callback_group=callback_group)
    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)
    
if __name__ == '__main__':
    main()