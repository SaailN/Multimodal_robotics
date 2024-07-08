def complete_task():
    oranges_moved = 0
    while oranges_moved < 2:
        if move_to('bedroom'):
            objects = detect_objects(['orange'])
            if objects:
                object_location = objects[0]['location']
                if pick_object(object_location['x'], object_location['y'], 'orange'):
                    say('Orange picked.')
                    if move_to('kitchen'):
                        objects = detect_objects(['green basket'])
                        if objects:
                            object_location = objects[0]['location']
                            if place_object(object_location['x'], object_location['y'], 'orange'):
                                say('Orange placed in green basket.')
                                oranges_moved += 1
                            else:
                                say('Failed to place orange in green basket.')
                        else:
                            say('Green basket not found.')
                    else:
                        say('Failed to move to kitchen.')
                else:
                    say('Failed to pick orange.')
            else:
                say('Orange not found.') 
        else:
            say('Failed to move to bedroom.')
    say('Task completed.')

complete_task()