#!/usr/bin/env python3

import socket
import threading
import json
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
HEADER = 64
# SERVER = socket.gethostbyname(socket.gethostname())
SERVER = "192.168.118.158"
PORT = 5050
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"[STARTING] server is starting at {SERVER}:{PORT}")
server.bind(ADDR)

def handle_client(conn, addr):
    print(f"[NEW CONNECTION] {addr} connected.")

    connected = True
    while connected:
        msg_length = conn.recv(HEADER).decode(FORMAT)
        if msg_length:
            msg_length = int(msg_length)
            msg = conn.recv(msg_length).decode(FORMAT)
            # if msg == DISCONNECT_MESSAGE:
            #     connected = False
            #     conn.send("Conn Closing".encode(FORMAT))
            #     break
            data = json.loads(msg)
            
            try:
                rclpy.init()
            except:
                pass
            print(f"[{addr}] {data}")
            conn.send("Msg received".encode(FORMAT))
            postion = [float(data['x']), float(data['y']), float(data['z'])]
            quaternion = [0.5,0.5,0.5,0.5]
            
            node = Node("ListnerNode")
            print("Postion: ",postion)
            # Create callback group that allows execution of callbacks in parallel without restrictions
            callback_group = ReentrantCallbackGroup()
            node.SendPoseClient = node.create_client(SendPose, '/movetoPose')
            while not node.SendPoseClient.wait_for_service(timeout_sec=1.0):
                print(' SendPose Client service not available, waiting again...')
            node.SendPoseRequest = SendPose.Request()
            node.SendPoseRequest.x = postion[0]
            node.SendPoseRequest.y = postion[1]
            node.SendPoseRequest.z = postion[2]
            node.SendPoseRequest.xr = quaternion[0]
            node.SendPoseRequest.yr = quaternion[1]
            node.SendPoseRequest.zr = quaternion[2]
            node.SendPoseRequest.wr = quaternion[3]
            PoseComplete = node.SendPoseClient.call_async(node.SendPoseRequest)
            rclpy.spin_until_future_complete(node, PoseComplete)
            # print("Start Arn Manipulation")
            print("SendPose Client Response: ",PoseComplete.result())
            node.destroy_node()
            rclpy.shutdown()
            

    # conn.close()

def start_server():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()
        print(f"[ACTIVE CONNECTIONS] {threading.active_count() - 1}")

print("[STARTING] server is starting...")
start_server()



