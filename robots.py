import requests
import constants 
from rich import print

def pick_object(x, y, object) -> bool:
    """
    Name: pick_object
    Signature: pick_object(x: float, y: float, object: str) -> bool
    Input: x, y coordinates of object to pick, and name of the object being picked
    Output: True if object is picked, False otherwise
    """
    print("[yellow]Picking object")
    x = float(x)
    y = float(y)
    picked = requests.get(constants.config["ros_server"] + "/pick_object", params={"x": x, "y": y, "object": object}, timeout=1000)
    picked = picked.json()
    return picked["success"]

def place_object(x, y, object) -> bool:
    """
    Name: place_object
    Signature: place_object(x: float, y: float, object: str) -> bool
    Input: x, y coordinates of object to place, and the name of the object being placed
    Output: True if object is placed, False otherwise
    """
    # requests call to place
    print("[red]Placing object")
    x = float(x)
    y = float(y)
    placed = requests.get(constants.config["ros_server"] + "/place_object", params={"x": x, "y": y, "object": object}, timeout=1000)
    placed = placed.json()

    return placed["success"]
    

if __name__ == "__main__":
    # Test your code here
    result = pick_object(469, 321, "orange")
    print("not yet")
    print(result)