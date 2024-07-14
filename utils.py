import os 
import warnings 
warnings.filterwarnings("ignore")
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
import requests 
import constants 
import torch
import numpy as np
from PIL import Image
from rich import print 
from transformers import AutoProcessor, Owlv2ForObjectDetection
from transformers.utils.constants import OPENAI_CLIP_MEAN, OPENAI_CLIP_STD
from robots import *
import matplotlib.pyplot as plt
import time
import cv2

# contains utils documentation

def say(text):
    """
    Name: say
    Signature: say(text: str) -> None
    Input: text to say
    Output: None
    """
    # say something
    print(f"[bold]Robot says:[/bold] {text}")


def detect_objects(objects=[]):
    """
    Name: detect_objects
    Description: Returns the coordinates of the objects queried by the input parameter objects
    Signature: detect_objects(objects: list) -> dict
    Input: None the coordinates of the objects queried by the input parameter objects
    Signature: detect_objects(objects: list) -> dict
    Input: None
    Output dictionary of the format
    {
        'object': 'object_name',
        'location': {"x": 0.0, "y": 0.0}
    }
    """
    # get image  from url
    # Assuming constants.config["ros_server"] is properly defined
    print("[red]Fetching image...")
    time.sleep(5)
    image_url = constants.config["ros_server"] + "/static/image.jpg"

    while True:
        try:
            image = requests.get(image_url, stream=True, timeout=10)
            image = np.asarray(Image.open(image.raw))
            break
        except:
            continue
    print("[green]Image fetched successfully.")

    _, buffer = cv2.imencode('.jpg', image)
    image_bytes = buffer.tobytes()

    files = {'file': (image_bytes)}
    # print("donee")
    results = requests.post(constants.config["paligemma_server"], files = files, params = {"objects": objects})
    # print("dine")
    results = results.json()
    results = results['image']
    print("[green]Prediction done.")
    # results = results[0]
    # scores, labels, boxes = results["scores"], results["labels"], results["boxes"]
    labels = []
    boxes = []
    for obj in results:
        labels.append(obj['object'])
        boxes.append(obj['location'])
    # print(type(image))
    image_x, image_y, _ = image.shape
    # plot image 
    # print(boxes)
    # print("[green]Plotting image with bounding boxes.")
    fig, ax = plt.subplots()
    ax.imshow(image)
    plt.yticks(range(0, 800, 20))
    # Draw bounding boxes
    for box in boxes:
        ymin, xmin, ymax, xmax = box
        rect = plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False, edgecolor='red', linewidth=2)
        ax.add_patch(rect)

    # Save the image with bounding boxes
    plt.savefig('output.jpg')
    plt.close(fig)
    output = []
    # print(image_x, image_y)
    for i in range(len(labels)):
        output.append({
            "object": labels[i],
            "location": {
                "x": ((boxes[i][1] + boxes[i][3]) / 2),
                "y": ((boxes[i][0] + boxes[i][2]) / 2)
            }
        })
    return output

if __name__ == "__main__":
    results = detect_objects(["red basket", "orange"])
    print(results)
    print("[yellow]Sending pick_object request...")
    obj_in_hand = None
    for obj in results:
        if obj["object"] == "orange":
            picked = pick_object(obj["location"]["x"], obj["location"]["y"], obj["object"])
            print(picked)
            obj_in_hand = obj["object"]
    print("[yellow]Sending place_object request...")
    for obj in results:
        if obj["object"] == "red basket":
            placed = place_object(obj["location"]["x"], obj["location"]["y"], obj_in_hand)
            print(placed)
    


