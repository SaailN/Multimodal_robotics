def get_camera_prompt(view):
    return (
        "The camera may not see the entire scene at once."
        if not view
        else "The camera sees the entire scene at once."
    )


def get_scene_prompt(scene):
    return "The scene is dynamic." if scene == "dynamic" else "The scene is static."


def get_error_prompt(error):
    return "Error handling is required." if error else "Error handling is not required."


def get_imports_prompt(imports):
    return "Import statements required" if imports else "No import statements required."


def get_code_output_prompt(code_output):
    return "Code output is required." if code_output else "Code output is not required."
