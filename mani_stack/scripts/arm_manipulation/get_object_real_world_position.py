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
from scipy.spatial.transform import Rotation as R
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from mani_stack.srv import Coordinate
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from threading import Thread
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import time
global x,y,z
x,y,z = 0.0 ,0.0,0.0
arucoImageWindow = None

def main():
    global rgb_rgb_image,depth_image
    rclpy.init()
    node = rclpy.create_node("camera_publisher")
    callback_group = ReentrantCallbackGroup()
    executor = rclpy.executors.MultiThreadedExecutor(5)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    tf_buffer = (
            tf2_ros.buffer.Buffer()
        )  # buffer time used for listening transforms
    listener = tf2_ros.TransformListener(tf_buffer,node)
    br = tf2_ros.TransformBroadcaster(node)
    tf_static_broadcaster = StaticTransformBroadcaster(node)
    #############################subscription functions##########################
    def colorrgb_imagecb(data):
        try:
            global rgb_image
            rgb_image = node.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            pass
    def depthimagecb(data):
        try:
            global depth_image
            depth_image_pass = node.bridge.imgmsg_to_cv2(data, "passthrough")
            depth_image = depth_image_pass.copy()
        except:
            pass
    def getCurrentPose(useEuler=False):
        print("Getting Current Pose")
        tempPose = [0, 0, 0]
        tempQuats = [0, 0, 0, 0]
        try:
            transform = tf_buffer.lookup_transform("base_link", "orange", rclpy.time.Time())
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
        except Exception as e:
            node.get_logger().info("Error in getting current pose: " + str(e))
        print("Current Pose: ", tempPose, tempQuats)
        return tempPose, tempQuats
    def sendTransforms():
        transformStamped = TransformStamped()
        transformStamped.header.stamp = node.get_clock().now().to_msg()
        transformStamped.header.frame_id = "camera_link"
        transformStamped.child_frame_id = "orange"
        transformStamped.transform.translation.x = z   
        transformStamped.transform.translation.y = x    
        transformStamped.transform.translation.z = y
        transformStamped.transform.rotation.x = 0.0 
        transformStamped.transform.rotation.y = 0.0
        transformStamped.transform.rotation.z = 0.0
        transformStamped.transform.rotation.w = 1.0
        # br.sendTransform(transformStamped)
        tf_static_broadcaster.sendTransform(transformStamped)
        print("Transform sent")
    ############################subscription####################################
    node.bridge = CvBridge()  
    node.color_cam_sub = node.create_subscription(
            Image, "/camera/color/rgb_image_raw", colorrgb_imagecb, 10
        )
    node.depth_cam_sub = node.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", depthimagecb, 10
        )
    
    node.timer = node.create_timer(
            0.2, sendTransforms
        )
    
    
    def processImage(Request,Response):
        global depth_image,x,y,z
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        imageX,imageY=Request.x,Request.y
        print(imageX," ",imageY)
        depth_distance = depth_image[int(imageY)][int(imageX)] / 1000
        realX = depth_distance * (sizeCamX - imageX - centerCamX) / focalX
        realY = depth_distance * (sizeCamY - imageY - centerCamY) / focalY
        realZ = depth_distance
        if Request.function != "image":
            x,y,z = Request.x,Request.y,Request.z
        else:
            x,y,z = realX,realY,realZ
        for i in  range(20):

            pose=getCurrentPose()[0]
            time.sleep(0.1)
            
        Response.x,Response.y,Response.z = float(pose[0]),float(pose[1]),float(pose[2])
        Response.success = True
        Response.message = "Success"
        node.get_logger().info("Request done with Succes")
        return Response
    real_world_coordinate_srv = node.create_service(Coordinate, '/coords', processImage, callback_group=callback_group)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    