def complete_task():
    objects = detect_objects(['orange'])
    if objects:
        object_location = objects[0]['location']
        if pick_object(object_location['x'], object_location['y'], 'orange'):
            say('Orange picked.')
        else:
            say('Failed to pick orange.')
    else:
        say('Orange not found.') 

complete_task()