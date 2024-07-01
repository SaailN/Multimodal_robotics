from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ebot", package_name="ebot_ur5").to_moveit_configs()
    return generate_demo_launch(moveit_config)
