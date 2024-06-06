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

class camera_publish(Node):
    """
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    """

    def __init__(self):
       
        super().__init__("camera_publisher")  # registering node
        

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.colorimagecb, 10
        )
        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2  # rate of time to process image (seconds)
        self.bridge = CvBridge()  # initialise CvBridge object for image conversion
        
        self.timer = self.create_timer(
            image_processing_rate, self.process_image
        )  # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        self.cv_image = None  # colour raw image variable (from colorimagecb())
        

    

    def colorimagecb(self, data):
        """
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        """
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image = image.copy()
            
        except:
            pass
    def process_image(self):
        try:
            arucoImageWindow = self.cv_image
            
            client_socket,addr = server_socket.accept()
            print('GOT CONNECTION FROM:',addr)
            if client_socket:
                while True:
                    a = pickle.dumps(arucoImageWindow)
                    message = struct.pack("Q",len(a))+a
                    print(len(a))
                    client_socket.sendall(message)        
            
        except Exception as e:
            print(e)
            pass
def main():
    """
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    """
    rclpy.init(args=sys.argv)  # initialisation
    node = rclpy.create_node("publish_camera")  # creating ROS node
    node.get_logger().info("Node created: publish_camera")  # logging information
    camera_publish_class = camera_publish()  # creating a new object for class 'aruco_tf'
    rclpy.spin(camera_publish_class)  # spining on the object to make it alive in ROS 2 DDS
    
    camera_publish_class.destroy_node()  # destroy node after spin ends
    rclpy.shutdown()  # shutdown process


if __name__ == "__main__":
    """
    Description:    If the python interpreter is running that module (the source file) as the main program,
                    it sets the special __name__ variable to have a value “__main__”.
                    If this file is being imported from another module, __name__ will be set to the module's name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    """

    main()