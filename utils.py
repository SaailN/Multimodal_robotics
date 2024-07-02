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


processor = AutoProcessor.from_pretrained("google/owlv2-base-patch16-ensemble")
model = Owlv2ForObjectDetection.from_pretrained("google/owlv2-base-patch16-ensemble")

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
    image_url = constants.config["ros_server"] + "/static/image.jpg"

    while True:
        try:
            image = requests.get(image_url, stream=True, timeout=10)
            image = Image.open(image.raw)
            break
        except:
            continue


    print("[green]Image fetched successfully.")

    inputs = processor(text=objects, images=image, return_tensors="pt")

    with torch.no_grad():
        outputs = model(**inputs)    

    print("[green]Starting object detection.")
    def get_preprocessed_image(pixel_values):
        pixel_values = pixel_values.squeeze().numpy()
        unnormalized_image = (pixel_values * np.array(OPENAI_CLIP_STD)[:, None, None]) + np.array(OPENAI_CLIP_MEAN)[:, None, None]
        unnormalized_image = (unnormalized_image * 255).astype(np.uint8)
        unnormalized_image = np.moveaxis(unnormalized_image, 0, -1)
        unnormalized_image = Image.fromarray(unnormalized_image)
        return unnormalized_image

    unnormalized_image = get_preprocessed_image(inputs.pixel_values)
    target_sizes = torch.Tensor([unnormalized_image.size[::-1]])
    results = processor.post_process_object_detection(
    outputs=outputs, threshold=0.2, target_sizes=target_sizes
)   
    print("[green]Prediction done.")
    results = results[0]
    scores, labels, boxes = results["scores"], results["labels"], results["boxes"]
    image_x, image_y = unnormalized_image.size
    # plot image 
    # print(boxes)
    # print("[green]Plotting image with bounding boxes.")
    fig, ax = plt.subplots()
    ax.imshow(unnormalized_image)
    plt.yticks(range(0, 800, 20))
    # Draw bounding boxes
    for box in boxes:
        xmin, ymin, xmax, ymax = box.tolist()
        rect = plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False, edgecolor='red', linewidth=2)
        ax.add_patch(rect)

    # Save the image with bounding boxes
    plt.savefig('output.jpg')
    plt.close(fig)
    output = []
    # print(image_x, image_y)
    for i in range(len(scores)):
        output.append({
            "object": objects[labels[i]],
            "location": {
                "x": round(1280 * ((boxes[i][0] + (boxes[i][2] - boxes[i][0]) / 2) / image_x).item()),
                "y": round(720 * (((boxes[i][1] +    ((boxes[i][3] - boxes[i][1]) / 2))) / 540).item())
            }
        })
    print(output)
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
    


