#!/usr/bin/env python3




import rclpy
from std_msgs.msg import String
import sys
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage, Image
from mani_stack.srv import SendPose
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from mani_stack.srv import Coordinate
import tf2_ros
import time
from mani_stack.srv import Servo
import requests
import cv2
import matplotlib.pyplot as plt
from linkattacher_msgs.srv import AttachLink, DetachLink
################### GLOBAL VARIABLES #######################



##################### CLASS DEFINITION #######################


class tf(Node):
   

    def __init__(self):
      

        super().__init__("tf_publisher")  # registering node
        print(" tf publisher node registered")

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.colorimagecb, 10
        )
        self.depth_cam_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depthimagecb, 10
        )
         
        self.timer = self.create_timer(
            0.2, self.process_image
        )  # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None  # colour raw image variable (from colorimagecb())
        self.depth_image = None  # depth image variable (from depthimagecb())
        self.bridge = CvBridge() 
        self.check =None
    def depthimagecb(self, data):
        """
        Description:    Callback function for aligned depth camera topic.
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        """

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        # 	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################
        try:
            image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.depth_image = image.copy()
        except:
            pass

    def colorimagecb(self, data):
        """
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        """

        ############ ADD YOUR CODE HERE ############

        # 	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image = image.copy()
        except:
            pass
    
    
    def process_image(self):
        
        try:
            self.cv_image = self.cv_image.copy()
            
            cv2.imshow("color",self.cv_image)
            cv2.waitKey(1)
        except:
            pass
       

def main():
    """
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    """
    rclpy.init(args=sys.argv)  # initialisation
    node = rclpy.create_node("tf_process")  # creating ROS node
    node.get_logger().info("Node created:  tf process")  # logging information
    tfClass = tf()  # creating a new object for class 'aruco_tf'
    rclpy.spin(tfClass)  # spining on the object to make it alive in ROS 2 DDS
    tfClass.destroy_node()  # destroy node after spin ends
    rclpy.shutdown()  # shutdown process


if __name__ == "__main__":
    """
    Description:    If the python interpreter is running that module (the source file) as the main program,
                    it sets the special __name__ variable to have a value “__main__”.
                    If this file is being imported from another module, __name__ will be set to the module's name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    """

    main()