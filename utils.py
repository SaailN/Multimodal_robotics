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
    image_url = constants.config["ros_server"] + "/static/maniimage.jpg"

    # Fetch the image
    image = requests.get(image_url, stream=True, timeout=10)

    # Check if the request was successful
    if image.status_code == 200:
        image = Image.open(image.raw)
    else:
        return []
    print("[green]Image fetched successfully.")

    inputs = processor(text=objects, images=image, return_tensors="pt")

    with torch.no_grad():
        outputs = model(**inputs)    

    def get_preprocessed_image(pixel_values):
        pixel_values = pixel_values.squeeze().numpy()
        unnormalized_image = (pixel_values * np.array(OPENAI_CLIP_STD)[:, None, None]) + np.array(OPENAI_CLIP_MEAN)[:, None, None]
        unnormalized_image = (unnormalized_image * 255).astype(np.uint8)
        unnormalized_image = np.moveaxis(unnormalized_image, 0, -1)
        unnormalized_image = Image.fromarray(unnormalized_image)
        return unnormalized_image

    unnormalized_image = get_preprocessed_image(inputs.pixel_values)
    target_sizes = torch.Tensor([unnormalized_image.size[::-1]])
    print("[red]Post processing...")
    results = processor.post_process_object_detection(
    outputs=outputs, threshold=0.2, target_sizes=target_sizes
)   
    print("[green]Prediction done.")
    results = results[0]
    scores, labels, boxes = results["scores"], results["labels"], results["boxes"]
    image_x, image_y = unnormalized_image.size
    output = []

    for i in range(len(scores)):
        output.append({
            "object": objects[labels[i]],
            "location": {
                "x": round(1280 * ((boxes[i][0] + (boxes[i][2] - boxes[i][0]) / 2) / image_x).item()),
                "y": round(720 - 720 * ((boxes[i][1] + (boxes[i][3] - boxes[i][1]) / 2) / image_y).item())
            }
        })
    picked = requests.get(constants.config["ros_server"] + "/pick_object", params={"x": output[0]["location"]["x"], "y": output[0]["location"]["y"]}).json()

    return output

if __name__ == "__main__":
    results = detect_objects(["orange"])
    print(results)


