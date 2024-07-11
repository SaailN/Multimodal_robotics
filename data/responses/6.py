def complete_task():
    if move_to('kitchen'):
        say('Reached kitchen, waiting for a minute.')
        sleep(60) 
    else:
        say('Failed to move to kitchen.')
    
    if not move_to('bedroom'):
        say('Failed to move to bedroom.')        

complete_task()