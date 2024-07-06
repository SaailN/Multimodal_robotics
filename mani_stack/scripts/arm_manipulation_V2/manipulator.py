#!/usr/bin/env python3
from os import path
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2, MoveIt2Servo
from pymoveit2.robots import ur5
import tf2_ros
import math
from std_msgs.msg import Int8
import transforms3d as tf3d # type: ignore
from mani_stack.srv import Manipulation
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped,Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
from sensor_msgs_py import point_cloud2

global servo_status
servo_status = 0
current_joint_states = [0, 0, 0, 0, 0, 0]
###################globaal variables###################
global positionToGO,yaw,Xdept, Ydept , Zdept ,Zflag
positionToGO = {
        'initalPose':{'xyz': [5.44, 1.76, 0.0], 'quaternions': [0.0, 0.0, -0.7, 0.7], 'XYoffsets': [0.0, 0.0],'Yaw':0,'Zero':True},
        'bedroom':{'xyz': [-7.25, -1.19, 0.0], 'quaternions': [ 0.0, 0.0, 0.8939967, -0.4480736 ], 'XYoffsets': [0.0, 0.0],'Yaw':180,'Zero':False},
        'kitchen':{'xyz': [7.79, -3.51, 0.0], 'quaternions': [0.0, 0.0, 0.0, 1.0], 'XYoffsets': [0.0, 0.0],'Yaw':0,'Zero':True},
        }
Z=None
Zflag=False
def main():
    rclpy.init()
    node  = Node("manipulator_node")
    PoseNode = Node("pose_node")
    imuNode = Node("imu_nodeHELp")
    getZNode = Node("getZNode")
    callback_group = ReentrantCallbackGroup()
    PoseCallbackGroup = ReentrantCallbackGroup()
    # Create MoveIt 2 interface
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor.add_node(PoseNode)
    executor.add_node(imuNode)
    executor.add_node(getZNode)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    navigator = BasicNavigator()
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, PoseNode)
    
    def getGoalPoseStamped(goal):
        global positionToGO
        Goal = positionToGO[goal]
        goalPose = PoseStamped()
        goalPose.header.frame_id = 'map'
        goalPose.header.stamp = navigator.get_clock().now().to_msg()
        goalPose.pose.position.x = Goal['xyz'][0]
        goalPose.pose.position.y = Goal['xyz'][1]
        goalPose.pose.position.z = Goal['xyz'][2]
        goalPose.pose.orientation.x = Goal['quaternions'][0]
        goalPose.pose.orientation.y = Goal['quaternions'][1]
        goalPose.pose.orientation.z = Goal['quaternions'][2]
        goalPose.pose.orientation.w = Goal['quaternions'][3]
        print(goalPose)
        return goalPose  
    def computeAngle(setPoint, Input):
        error = Input - setPoint                                         
        output = 0.12 * error
        
        if(output > 1.0):
            output = 1.0
        elif(output < 0.2 and output > 0.0):
            output = 0.2
        elif(output < -1.0):
            output = -1.0
        elif(output > -0.2 and output < 0.0):
            output = -0.2         
        print("Input",Input,"setPoint",setPoint,"error",error,"output",output)
        return output*-1.0
    def normalize_angle(angle,isZero = False):
        """Normalizes an angle to the range [-π, π].
    
        Args:
            angle: A float representing the angle in radians.

        Returns:
            A float representing the normalized angle in radians.
        """
        if isZero:
            return angle
        if angle<0:
            angle = angle + 360
        return angle
    def getCurrentPose(useEuler=False):
        print("Getting Current Pose")
        tempPose = [0, 0, 0]
        tempQuats = [0, 0, 0, 0]
        try:
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
        except Exception as e:
            node.get_logger().info("Error in getting current pose: " + str(e))
        print("Current Pose: ", tempPose, tempQuats)
        return tempPose, tempQuats
    def imu_callback(msg):
        global yaw
        quaternion_array = msg.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        yaw = math.degrees(yaw)
        yaw = round(yaw,2)
        # print(yaw)
    moveit_callback_group = ReentrantCallbackGroup()
    ServoCallbackGroup = ReentrantCallbackGroup()
    moveitnode = Node("moveit2_node_pose")
    servoNode = Node("moveit2_node_servo")
    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=moveitnode,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=moveit_callback_group,
    )
    moveit2Servo = MoveIt2Servo(
            node=servoNode, frame_id=ur5.base_link_name(), callback_group=ServoCallbackGroup
        )
    moveitexecutor = rclpy.executors.MultiThreadedExecutor(1)
    moveitexecutor.add_node(moveitnode)
    moveitexecutor.add_node(servoNode)
    moveit_executor_thread = Thread(target=moveitexecutor.spin, daemon=True, args=())
    moveit_executor_thread.start()
    def moveToPoseService(position, quaternion):
        node.get_logger().info("Request Arrived")
        postion =   position
        quaterion = quaternion
        print("Postion: ",postion)
        
        counter = 0
        PreviousPose = getCurrentPose()[0]
        PreviousPose = [round(PreviousPose[0], 2), round(PreviousPose[1], 2), round(PreviousPose[2], 2)]
        while counter < 5:
            moveit2.move_to_pose(position=postion, quat_xyzw=quaterion, cartesian=False)
            
            statuts = moveit2.wait_until_executed()
            time.sleep(0.5)
            currentPose = getCurrentPose()[0]
            currentPose = [round(currentPose[0], 2), round(currentPose[1], 2), round(currentPose[2], 2)]
            statut = (PreviousPose != currentPose)
            print("Statut: ", statut)
            statuts = statuts or statut
            if statuts == True:
                break
            else:
                counter += 1 
        
        if statuts == True:
            return True
        else:
            return False

    def servoService(position):
        # traget = Request.id
        TargetPose = position
        
        
        
        # Spin the node in background thread(s)
        twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        def servo_status_updater(msg):
            global servo_status
            servo_status = msg.data
        moveitnode.odom_sub = moveitnode.create_subscription(
            Int8, "/servo_node/status", servo_status_updater, 10
        )
        def moveWithServo(linear_speed, angular_speed):
            twist_msg = TwistStamped()
            twist_msg.header.frame_id = ur5.base_link_name()
            twist_msg.header.stamp = moveitnode.get_clock().now().to_msg()
            twist_msg.twist.linear.x = linear_speed[0]
            twist_msg.twist.linear.y = linear_speed[1]
            twist_msg.twist.linear.z = linear_speed[2]
            twist_msg.twist.angular.x = angular_speed[0]
            twist_msg.twist.angular.y = angular_speed[1]
            twist_msg.twist.angular.z = angular_speed[2]
            twist_pub.publish(twist_msg)
        def checkSphericalTolerance(currentPose, targetPose, tolerance):
            currentTolerance = math.sqrt(
                (currentPose[0] - targetPose[0]) ** 2
                + (currentPose[1] - targetPose[1]) ** 2
                + (currentPose[2] - targetPose[2]) ** 2
            )
            return True if currentTolerance <= tolerance else False, currentTolerance
        
        sphericalToleranceAchieved = False
        
        while sphericalToleranceAchieved == False:
                currentPose = getCurrentPose()[0]
                sphericalToleranceAchieved, magnitude = checkSphericalTolerance(currentPose, TargetPose, 0.04)
                magnitude *= 3
                vx, vy, vz = (
                    (TargetPose[0] - currentPose[0]) / magnitude,
                    (TargetPose[1] - currentPose[1]) / magnitude,
                    (TargetPose[2] - currentPose[2]) / magnitude,
                )
                moveWithServo([vx,vy, vz], [0.0, 0.0, 0.0])
                # print("Vx:", vx, "Vy:", vy, "Vz:", vz)
                
                print("Current Pose: ", currentPose, "Target Pose: ", TargetPose)
                print("Vx:", vx, "Vy:", vy, "Vz:", vz)
                time.sleep(0.01)
                
                if servo_status > 0:
                    print("Exited While Loop due to Servo Error", servo_status)
                    break
        
        moveWithServo([0.0,0.0,0.0], [0.0, 0.0, 0.0])
        
        print("sphereical Tolerance Achieved", sphericalToleranceAchieved)
        
        return True
    
    def jointService(goal):
        homePoseStates = [0.0,-2.79,1.47,-1.1,-1.57,3.15]
        basketjointStates = [-1.07,-1.21,1.08,-1.67,-1.63,3.15]
        jointNode = Node("jointNode")
        
        goaljoints = [0.0,0.0,0.0,0.0,0.0,0.0]
        if goal == "home" : 
            goaljoints=homePoseStates 
        elif goal == "basket":
            goaljoints=basketjointStates
        counter = 0
        PreviousPose = getCurrentPose()[0]
        PreviousPose = [round(PreviousPose[0], 2), round(PreviousPose[1], 2), round(PreviousPose[2], 2)]
        print("cureent pose: ", PreviousPose)
        while counter < 5:
                moveit2.move_to_configuration(goaljoints)
                statuts = moveit2.wait_until_executed()
                time.sleep(0.5)
                currentPose = getCurrentPose()[0]
                currentPose = [round(currentPose[0], 2), round(currentPose[1], 2), round(currentPose[2], 2)]
                statut = (PreviousPose != currentPose)
                print("Statut: ", statut)
                statuts = statuts or statut
                if statuts == True:
                    break
                else:
                    counter += 1 
        return True
    def moveToGoal(goal):
        global positionToGO,yaw
        def moveBot(linearSpeedX,angularSpeed):
            twist = Twist()
            twist.linear.x = linearSpeedX
            twist.angular.z = angularSpeed
            imuNode.speedPub.publish(twist)
        goalPose = getGoalPoseStamped(goal)
        navigator.goToPose(goalPose)
        while not navigator.isTaskComplete():
            PreviousPose = getCurrentPose()[0]
            time.sleep(0.1)
        yawRotation = False
        isZero = positionToGO[goal]['Zero']
        while (yawRotation == False):
            angle=computeAngle(int(normalize_angle(positionToGO[goal]['Yaw'],isZero)),int(normalize_angle(yaw,isZero)))
            moveBot(0.0,angle)
            yawRotation = True if(int(normalize_angle(positionToGO[goal]['Yaw'],isZero)) == int(normalize_angle(yaw,isZero))) else False
            time.sleep(0.01)
        navigator.clearAllCostmaps()
        return True
    def distance(p1, p2):
        """Calculates the Euclidean distance between two points."""
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2) ** 0.5

    def convert_pointcloud2_to_xyz_array(msg):
        """Converts a PointCloud2 message to an array of dictionaries containing {x, y, z} coordinates."""
        points = []
        min_x = float('inf')
        x,y,z = 0,0,0
        pose = [0.40 , 0.06 ,0.39]
        i=0
        print(len(point_cloud2.read_points(msg, skip_nans=True)))
        for point in point_cloud2.read_points(msg, skip_nans=True):  # Skip NaN points
            x_value = distance(pose,point)  # Access the x-coordinate
            if x_value < min_x:
                min_x = x_value
                x,y,z = float(point[0]),float(point[1]),float(point[2])
        print(z)
        return x,y,z
    def point_cloud_callback(msg):
        global Xdept, Ydept , Zdept ,Zflag
        print("Point Cloud Callback")
        Zflag = False
        start_time = time.time()
        Xdept, Ydept , Zdept = convert_pointcloud2_to_xyz_array(msg)
        total_time = time.time() - start_time
        print("Time: ",total_time)
        Zflag = True
        # print("Min Distance: ",Z)
    def ManipuationControl(Request, Response):
        if Request.function == "Pose":
            print(" Pose Request Arrived")
            position = (Request.x , Request.y, Request.z)
            quaternion = (Request.xr, Request.yr, Request.zr, Request.wr)
            Response.success = moveToPoseService(position, quaternion)
            Response.message = "Pose Request Completed"
        elif Request.function == "Servo":
            position = (Request.x , Request.y, Request.z)
            print(" Servo Request Arrived")
            Response.success = servoService(position)
            Response.message = "Pose Request Completed"
        elif Request.function == "Joint":
            print(" Joint Request Arrived")
            goal = Request.goal
            Response.success = jointService(goal)
            Response.message = "Pose Request Completed"
        elif Request.function == "Nav2":
            goal = Request.goal
            print("Nav2 Request Arrived")
            Response.success = moveToGoal(goal)
            Response.message = "Pose Request Completed"
        elif Request.function == "GetZ":
            global Xdept, Ydept , Zdept ,Zflag
            print("GetZ Request Arrived")
            Response.success = True
            Response.message = "Pose Request Completed"
            while Zflag == False:
                time.sleep(0.01)
                print("Waiting for Z")
            print("Z: ",Zdept)
            Response.x, Response.y, Response.z = float(Xdept), float(Ydept) , float(Zdept)
        return Response
    navigator.setInitialPose(getGoalPoseStamped('initalPose'))
    navigator.waitUntilNav2Active()
    imuNode.speedPub = imuNode.create_publisher(Twist, '/cmd_vel', 10)
    imuNode.imu_sub = imuNode.create_subscription(Imu, '/imu', imu_callback, 10, callback_group=PoseCallbackGroup)
    movetopose_control_srv = node.create_service(Manipulation, '/manipulationService', ManipuationControl, callback_group=callback_group)
    rclpy.spin(node)
    rclpy.spin(imuNode)
    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()