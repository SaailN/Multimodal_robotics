#!/usr/bin/env python3


from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
import uvicorn
from os import path
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from mani_stack.srv import Manipulation , Coordinate
from linkattacher_msgs.srv import AttachLink, DetachLink
import threading


app = FastAPI()
meshcallbackgroup = ReentrantCallbackGroup()
class armcontroller(Node):
    def __init__(self):
        super().__init__("armController")  # registering node
        print(" armController node registered")
        #########client subscription##############
        self.armControlService = self.create_client(Manipulation, '/manipulationService', callback_group=ReentrantCallbackGroup())
        self.getCoordService = self.create_client(Coordinate, '/coords', callback_group=ReentrantCallbackGroup())
        self.mesh = self.create_client(Manipulation, '/addmeshobjectt', callback_group=meshcallbackgroup)
        while not self.armControlService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('armControlClientservice not available, waiting again...')
        
        while not self.mesh.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mesh not available, waiting again...')
            
        while not self.mesh.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mesh not available, waiting again...')
            
        while not self.getCoordService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('getCoordService not available, waiting again...')
        self.objectName = "object"
        app.mount("/static", StaticFiles(directory="/home/jalan/dev/ros2ws/src/23-Multimodal-Genrative-AI-for-Robotics-application/mani_stack/static"), name="static")
        app.mount("/static", StaticFiles(directory="/home/jalan/dev/ros2ws/src/23-Multimodal-Genrative-AI-for-Robotics-application/mani_stack/static"), name="static")

        @app.get("/")
        async def root():
            # self.armControlRequest = Manipulation.Request()
            # self.armControlRequest.x  = 0.35 
            # self.armControlRequest.y  = 0.1
            # self.armControlRequest.z  = 0.68
            # self.armControlRequest.xr = 1.0
            # self.armControlRequest.yr = 0.0
            # self.armControlRequest.zr = 0.0
            # self.armControlRequest.wr = 0.0
            # self.armControlRequest.function = "Pose"
            # armControlResponse = self.armControlService.call_async(self.armControlRequest)
            # rclpy.spin_until_future_complete(self, armControlResponse)
            # return {"message": armControlResponse.result().message, "success": armControlResponse.result().success}
            return {"message": "Hello World"}

        

        @app.get("/pick_object")
        async def pick_object(x: float, y: float,object: str):
            self.objectName = object
            self.get_logger().info(self.objectName )
            self.get_logger().info(self.objectName )
            # pick object using x (0-1) and y(0-1) and return a bool 
            self.getCoordRequest = Coordinate.Request()
            self.getCoordRequest.x = x
            self.getCoordRequest.y = y
            self.getCoordRequest.function = "image"
            for i in range(2):
                getCoordResponse = self.getCoordService.call_async(self.getCoordRequest)
                rclpy.spin_until_future_complete(self, getCoordResponse)
                time.sleep(0.2)
            ####################################add mesh################################
            self.meshRequest = Manipulation.Request()
            self.meshRequest.z = z-0.1
            self.meshRequest.function = "add"
            meshResponse = self.mesh.call_async(self.meshRequest)
            rclpy.spin_until_future_complete(self, meshResponse)
            ####################################add mesh################################
            # #####################################JOINT TO pre pose object#################
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.function = "Joint"
            self.armControlRequest.goal = "prePose"
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)
            
            ##########################GETS COORDINATES FROM CAMERA#####################
            x,y,z = getCoordResponse.result().x, getCoordResponse.result().y, getCoordResponse.result().z
            print("x,y,z",x,y,z)
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.x  = x 
            self.armControlRequest.y  = y
            self.armControlRequest.z  = z+0.2
            self.armControlRequest.xr = 0.71
            self.armControlRequest.yr = 0.70
            self.armControlRequest.zr = 0.02
            self.armControlRequest.wr = 0.02
            self.armControlRequest.function = "Pose"
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)
            
            
            ###################################MOVE TO PICK POSITION##################
            time.sleep(1.0)
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.x  = x 
            self.armControlRequest.y  = y
            self.armControlRequest.z  = z
            self.armControlRequest.function = "Servo"
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)
            #####################################Call LINKK ATTACHER#####################
            self.controlGripper("ON", self.objectName)
            time.sleep(1.0)
            print(self.objectName, "has been picked")
            ######################################SERVO TO PICK POSITION#################
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.x  = x 
            self.armControlRequest.y  = y
            self.armControlRequest.z  = z+0.2
            self.armControlRequest.function = "Servo"
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)
            ######################################SERVO TO PICK POSITION#################
            
            self.meshRequest = Manipulation.Request()
            self.meshRequest.z = z-0.1
            self.meshRequest.function = "remove"
            meshResponse = self.mesh.call_async(self.meshRequest)
            rclpy.spin_until_future_complete(self, meshResponse)
            #####################################remove mesh################################
            
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.function = "Joint"
            self.armControlRequest.goal = "home"
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)
            # #####################################JOINT TO HOME POSITION#################
            
           
            return {
                "success": armControlResponse.result().success,
                "message": armControlResponse.result().message
            }

        @app.get("/place_object")
        async def place_object(x: float, y: float,object: str):
            self.objectName = object
            # pick object using x (0-1) and y(0-1) and return a bool 
            self.getCoordRequest = Coordinate.Request()
            self.getCoordRequest.x = x
            self.getCoordRequest.y = y
            for i in range(3):
                getCoordResponse = self.getCoordService.call_async(self.getCoordRequest)
                rclpy.spin_until_future_complete(self, getCoordResponse)
                time.sleep(0.5)
            ##########################GETS COORDINATES FROM CAMERA#####################
            x,y,z = getCoordResponse.result().x, getCoordResponse.result().y, getCoordResponse.result().z
            print("x,y,z",x,y,z+0.2)
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.x  = x 
            self.armControlRequest.y  = y
            self.armControlRequest.z  = z+0.2
            self.armControlRequest.xr = 1.0
            self.armControlRequest.yr = 0.0
            self.armControlRequest.zr = 0.0
            self.armControlRequest.wr = 0.0
            self.armControlRequest.function = "Pose"
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)

            #####################################Call LINKK ATTACHER#####################
            self.controlGripper("OFF", self.objectName)
            
            ######################################SERVO TO PICK POSITION#################
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.function = "Joint"
            self.armControlRequest.goal = "home"
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)
            ######################################JOINT TO HOME POSITION#################
            return {
                "success": armControlResponse.result().success,
                "message": armControlResponse.result().message
            }
        @app.get("/move_to")
        async def move_to(location: str):
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.function = "Nav2"
            self.armControlRequest.goal = location
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)
            return {
                "success": armControlResponse.result().success,
                "message": armControlResponse.result().message
            }

    def controlGripper(self,status, objectname):
        @app.get("/move_to")
        async def move_to(location: str):
            self.armControlRequest = Manipulation.Request()
            self.armControlRequest.function = "Nav2"
            self.armControlRequest.goal = location
            armControlResponse = self.armControlService.call_async(self.armControlRequest)
            rclpy.spin_until_future_complete(self, armControlResponse)
            return {
                "success": armControlResponse.result().success,
                "message": armControlResponse.result().message
            }

    def controlGripper(self,status, objectname):
                    if status == "ON":
                        self.gripper_control = self.create_client(AttachLink, "/GripperMagnetON")
                        self.req = AttachLink.Request()
                    else:
                        self.gripper_control = self.create_client(DetachLink, "/GripperMagnetOFF")
                        self.req = DetachLink.Request()

                    while not self.gripper_control.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info("EEF service not available, waiting again...")
                    self.get_logger().info("Magnet On/Off service available")
                    self.req.model1_name = objectname
                    self.get_logger().info("Magnet On/Off service available")
                    self.req.model1_name = objectname
                    self.req.link1_name = "link"
                    self.req.model2_name = "ebot"
                    self.req.model2_name = "ebot"
                    self.req.link2_name = "wrist_3_link"
                    print("ur5 ->", objectname)
                    
                    print("ur5 ->", objectname)
                    
                    time.sleep(0.2)
                    gripperServicefuture = self.gripper_control.call_async(self.req)
                    rclpy.spin_until_future_complete(self, gripperServicefuture)
                    print("Gripper Status: ", status, "has been requested")
        
def main(args=None):
    rclpy.init()
    armcontrollerClass = armcontroller()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(armcontrollerClass)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    uvicorn.run(app, port=8000, log_level='warning')
    rclpy.shutdown()

if __name__ == '__main__':
    main()