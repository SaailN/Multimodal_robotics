def pick_object(x, y) -> bool:
    """
    Name: pick_object
    Signature: pick_object(x: float, y: float) -> bool
    Input: x, y coordinates of object to pick
    Output: True if object is picked, False otherwise
    """
    picked = True

    if "error":
        picked = False
    return picked


def place_object(x, y) -> bool:
    """
    Name: place_object
    Signature: place_object(x: float, y: float) -> bool
    Input: x, y coordinates of object to place
    Output: True if object is placed, False otherwise
    """
    # requests call to place
    if "successful":
        return True
    else:
        return False
