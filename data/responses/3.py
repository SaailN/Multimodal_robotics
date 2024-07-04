def complete_task():
    if move_to('bedroom'):
        objects = detect_objects(['orange'])
        if objects:
            object_location = objects[0]['location']
            if pick_object(object_location['x'], object_location['y'], 'orange'):
                say('Orange picked.')
                if move_to('kitchen'):
                    objects = detect_objects(['yellow basket'])
                    if objects:
                        object_location = objects[0]['location']
                        if place_object(object_location['x'], object_location['y'], 'orange'):
                            say('Orange placed in yellow basket.')
                        else:
                            say('Failed to place orange in yellow basket.')
                    else:
                        say('Yellow basket not found.')
                else:
                    say('Failed to move to kitchen.')
            else:
                say('Failed to pick orange.')
        else:
            say('Orange not found.') 
    else:
        say('Failed to move to bedroom.')
        
    if move_to('kitchen'):
        objects = detect_objects(['banana'])
        if objects:
            object_location = objects[0]['location']
            if pick_object(object_location['x'], object_location['y'], 'banana'):
                say('Banana picked.')
                if move_to('bedroom'):
                    objects = detect_objects(['red basket'])
                    if objects:
                        object_location = objects[0]['location']
                        if place_object(object_location['x'], object_location['y'], 'banana'):
                            say('Banana placed in red basket.')
                        else:
                            say('Failed to place banana in red basket.')
                    else:
                        say('Red basket not found.')
                else:
                    say('Failed to move to bedroom.')
            else:
                say('Failed to pick banana.')
        else:
            say('Banana not found.') 
    else:
        say('Failed to move to kitchen.')

complete_task()
