#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
import sys
import cv2
import math
import tf2_ros
import transforms3d as tf3d # type: ignore
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage, Image
cameraWindow = None
import socket, cv2, pickle,struct

# Socket Create
server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_name  = socket.gethostname()
host_ip = '192.168.189.158'
print('HOST IP:',host_ip)
port = 9999
socket_address = (host_ip,port)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Socket Bind
server_socket.bind(socket_address)

# Socket Listen
server_socket.listen(5)
print("LISTENING AT:",socket_address)
image = None
def main():
    global image
    rclpy.init()
    node = rclpy.create_node("camera_publisher")
    def colorimagecb(data):
        try:
            global image
            image = node.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            pass
    node.color_cam_sub = node.create_subscription(
            Image, "/camera/color/image_raw", colorimagecb, 10
        )
    node.bridge = CvBridge()  
    while True:
        try:
            copy_image = image.copy()
            client_socket,addr = server_socket.accept()
            
            if client_socket:
                print('GOT CONNECTION FROM:',addr)
                while True:
                    a = pickle.dumps(image)
                    message = struct.pack("Q",len(a))+a
                    print(len(a))
                    client_socket.sendall(message)       
                    rclpy.spin_once(node) 
        except:
            pass
        rclpy.spin_once(node)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()