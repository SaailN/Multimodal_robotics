def complete_task():
    if not move_to('kitchen'):
        say('Failed to move to kitchen.')
        return

    while True:
        objects = detect_objects(['apple'])
        if not objects:
            say('No more apples found.')
            break
        object_location = objects[0]['location']
        if pick_object(object_location['x'], object_location['y'], 'apple'):
            say('Apple picked.')
            objects = detect_objects(['red basket'])
            if objects:
                basket_location = objects[0]['location']
                if place_object(basket_location['x'], basket_location['y'], 'apple'):
                    say('Apple placed in red basket.')
                else:
                    say('Failed to place apple in red basket.')
            else:
                say('Red basket not found.')
        else:
            say('Failed to pick apple.')

    while True:
        objects = detect_objects(['banana'])
        if not objects:
            say('No more bananas found.')
            break
        object_location = objects[0]['location']
        if pick_object(object_location['x'], object_location['y'], 'banana'):
            say('Banana picked.')
            objects = detect_objects(['yellow basket'])
            if objects:
                basket_location = objects[0]['location']
                if place_object(basket_location['x'], basket_location['y'], 'banana'):
                    say('Banana placed in yellow basket.')
                else:
                    say('Failed to place banana in yellow basket.')
            else:
                say('Yellow basket not found.')
        else:
            say('Failed to pick banana.')

complete_task()