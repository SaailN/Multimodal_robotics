#!/usr/bin/env python3


from os import path
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from mani_stack.srv import Manipulation


from mani_stack.srv import Manipulation



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
        # moveit2.add_collision_mesh(
        # filepath=floor,
        # id="Floor",
        # position=[2.1, 0.00, 0.1],
        # quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        # frame_id="base_link",
        # )
        # moveit2.add_collision_mesh(
        # filepath=floor,
        # id="Floor",
        # position=[2.1, 0.00, 0.1],
        # quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        # frame_id="base_link",
        # )
        
        time.sleep(0.5)
        moveit2.add_collision_mesh(
        filepath=floor,
        id="BackFloor",
        position=[-0.6, 0.00, 0.05],
        quat_xyzw=[0, 0.681639 ,0, 0.731689],
        frame_id="base_link",
        )
        time.sleep(0.5)
    def addmesh(req, res):
        z = req.z
        if req.function == "add":
            print("Adding mesh...")
            for i in range(1, 4):
                moveit2.add_collision_mesh(
                filepath=floor,
                id="Floor",
                position=[2.1, 0.00, z],
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                frame_id="base_link",
                )
            res.message = "Mesh added"
        elif req.function == "remove":
            moveit2.remove_collision_mesh(id="Floor")
            print("Removing mesh...")
            res.message = "Mesh removed"
        res.success = True
        return res
    node.create_service(Manipulation, '/addmeshobjectt', addmesh, callback_group=callback_group)
    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
