config = {
    "robot": "UR5",  # not using
    "robot_apis": ["place_object", "pick_object", "move_to"], # available actions in robotic space
    "utils_apis": ["detect_objects", "say"], # available actions in utils space
    "full_view": False, # does camera have full view of the environment?
    "scene": "dynamic", # does the scene change over time?
    "scene_desc": "The scene has fruits and baskets placed in different locations. The robot can pick only 1 object at a time.",
    "suffix": "Write code using the provided APIs to perform the task. No comments.", # suffix for prompt
    "error_handling": True, # does the task require error handling?
    "imports": False, # does the task require imports?
    "code_output": True, # does the task require code output?
    "prompt_output_file": "prompt.txt", # file to store prompt
    "code_output_file": "output.py", # file to store code output
    "ros_server": "http://localhost:8000", # ROS server
    "paligemma_server": "https://redbird-mutual-hardly.ngrok-free.app/image/", # Paligemma server  for object detection
}


