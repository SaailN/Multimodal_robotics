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