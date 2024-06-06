# <?xml version="1.0" ?>

# <launch>
#   <arg name="gui" default="True" />
#   <param name="use_gui" value="$(arg gui)"/>
  
#   <param name="robot_description" command="$(find xacro)/xacro $(find robotq_85)/urdf/robotiq_arg2f_85_model.xacro" />  
  
#   <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
#   <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />

#   <node name="rviz" pkg="rviz" type="rviz" args="-d $(find robotq_85)/visualize.rviz" required="true" />
# </launch>
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file
def generate_launch_description():
    robot_state_publisher = Node(
            package="robot_state_publisher", 
            executable="spawner",
            arguments=['joint_state_broadcaster'],
            output="screen")
    joint_state_publisher = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen")
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])
