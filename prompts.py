# This file contains the tempalate text which correspond to different configurations in the constants.py file

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


# sample task and response for one shot learning

recorded_task = "Pick the orange from the bedroom and place in the red basket in the kitchen"

recorded_response= """
def complete_task():
    if move_to('bedroom'):
        objects = detect_objects(['orange'])
        if objects:
            object_location = objects[0]['location']
            if pick_object(object_location['x'], object_location['y'], 'orange'):
                say('Orange picked.')
            else:
                say('Failed to pick orange.')
        else:
            say('Orange not found.') 
    else:
        say('Failed to move to bedroom.')

    if move_to('kitchen'):
        objects = detect_objects(['red basket'])
        if objects:
            object_location = objects[0]['location']
            if place_object(object_location['x'], object_location['y'], 'orange'):
                say('Orange placed in red basket.')
            else:
                say('Failed to place orange in red basket.')
        else:
            say('Red basket not found.')
    else:
        say('Failed to move to kitchen.')

        
complete_task()
"""