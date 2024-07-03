from launch.actions import ExecuteProcess
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
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)


from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml
def generate_launch_description():
    # start_perception = Node(
    # package='mani_stack',
    # executable='task1ab-perception',
    # )
    # start_docking = Node(
    # package='ebot_docking',
    # executable='ebot_docking_boilerplate',
    # )
    # start_navigation = Node(
    # package='ebot_docking',
    # executable='task2b',
    # )
    
    start_perception = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack perceptionFast.py ',
        ]],
        shell=True
    )

    start_move_to_goal = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack manipulator.py',
        ]],
        shell=True
    )
    start_addMesh = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack add_mesh.py',
        ]],
        shell=True
    )
    star_getCoords = ExecuteProcess(
    cmd=[['ros2 run mani_stack get_object_real_world_position.py']],
    shell=True
    )
    start_fastapi = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack FastApi.py',
        ]],
        shell=True
    )
    
    
    return LaunchDescription([
        start_perception,
        # start_move_to_goal,
        star_getCoords,
        start_fastapi,
        # start_addMesh
    ])