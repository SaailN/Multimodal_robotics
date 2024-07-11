def complete_task():
    found = False
    for location in ['kitchen', 'bedroom']:
        if move_to(location):
            objects = detect_objects(['orange'])
            if objects:
                object_location = objects[0]['location']
                if pick_object(object_location['x'], object_location['y'], 'orange'):
                    say('Orange picked.')
                    found = True
                    break
                else:
                    say('Failed to pick orange.')
            else:
                say('Orange not found in ' + location)
        else:
            say('Failed to move to ' + location)
    
    if not found:
        say('Orange not found in kitchen or bedroom.')
        return
    
    if move_to('another room'):
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
        say('Failed to move to another room.')

complete_task()
```