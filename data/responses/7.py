def complete_task():
    if move_to('bedroom'):
        say('Looking for orange in bedroom.')
        objects = detect_objects(['orange'])
        if objects:
            object_location = objects[0]['location']
            if pick_object(object_location['x'], object_location['y'], 'orange'):
                say('Orange picked from bedroom.')
                return
            else:
                say('Failed to pick orange from bedroom.')
        else:
            say('Orange not found in bedroom.')

    say('Moving to kitchen to look for the orange.')
    if move_to('kitchen'):
        objects = detect_objects(['orange'])
        if objects:
            object_location = objects[0]['location']
            if pick_object(object_location['x'], object_location['y'], 'orange'):
                say('Orange picked from kitchen.')
                return
            else:
                say('Failed to pick orange from kitchen.')
        else:
            say('Orange not found in kitchen.')
        
complete_task()