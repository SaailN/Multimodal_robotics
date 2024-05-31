#!/usr/bin/python3
# -*- coding: utf-8 -*-


import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ebot_description').find('ebot_description')

    xacro_file_ebot = os.path.join(pkg_share, 'models/','ebot/', 'ebot_description.xacro')
    assert os.path.exists(xacro_file_ebot), "The box_bot.xacro doesnt exist in "+str(xacro_file_ebot)
    robot_description_config_ebot = xacro.process_file(xacro_file_ebot)
    robot_description_ebot = robot_description_config_ebot.toxml()


    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ebot_description'), 'launch', 'start_eysip_demo.py'),
        )
    )
    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments = ["1.6", "-2.4", "-0.8", "3.14", "0", "0", "world", "odom"],
        output='screen')
    

    spawn_arm = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='ur5_spawner',
    	executable='spawn_entity.py',
        arguments=['-entity', 'ur5', '-topic', 'robot_description_ur5', '-x', '1.6', '-y', '-2.4', '-z', '0.58', '-Y', '3.14'],
        output='screen')

                                                 
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
       
        static_transform,
        spawn_arm
    ])
