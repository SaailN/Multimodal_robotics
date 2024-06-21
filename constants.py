config = {
    "robot": "UR5",  # not using
    "robot_apis": ["place_object", "pick_object"],
    "utils_apis": ["detect_objects", "say"],
    "full_view": False,
    "scene": "dynamic",
    "scene_desc": "The scene has fruits (stacked) and baskets, whose coordinates we do not know. The robot can pick only 1 object at a time.",
    "suffix": "Write code using the provided APIs to perform the task. No comments.",
    "error_handling": True,
    "imports": False,
    "code_output": True,
    "one_shot_file": "one_shot.txt",
    "prompt_output_file": "prompt.txt",
    "code_output_file": "output.py",
    "ros_server": "https://boxer-loyal-unicorn.ngrok-free.app",
}


