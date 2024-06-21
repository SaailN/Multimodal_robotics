import requests
import constants 

def pick_object(x, y) -> bool:
    """
    Name: pick_object
    Signature: pick_object(x: float, y: float) -> bool
    Input: x, y coordinates of object to pick
    Output: True if object is picked, False otherwise
    """
    picked = requests.get(constants.config["ros_server"] + "/pick_object", params={"x": x, "y": y}).json()
    return picked

def place_object(x, y) -> bool:
    """
    Name: place_object
    Signature: place_object(x: float, y: float) -> bool
    Input: x, y coordinates of object to place
    Output: True if object is placed, False otherwise
    """
    # requests call to place
    placed = requests.get(constants.config["ros_server"] + "/place_object", params={"x": x, "y": y}).json()
    return placed
    

if __name__ == "__main__":
    # Test your code here
    result = pick_object(0.45678, 0.5)
    print(result)