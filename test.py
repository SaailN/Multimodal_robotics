
import cv2 
import requests
import constants 
from PIL import Image
import numpy as np
from transformers.utils.constants import OPENAI_CLIP_MEAN, OPENAI_CLIP_STD

url = 'https://redbird-mutual-hardly.ngrok-free.app/image/'

# image_url = constants.config["ros_server"] + "/static/image.jpg"
image_url = "basketeer.jpeg"
# while True:
#     try:
#         # image = requests.get(image_url, stream=True, timeout=10)
#         # image = Image.open(image.raw)
#         break
#     except:
#         continue

image = np.asarray(Image.open(image_url))
h,w,_ = image.shape
image = image[100:h-100, 200:w-200]
# def get_preprocessed_image(pixel_values):
#         pixel_values = pixel_values.squeeze()
#         unnormalized_image = (pixel_values * np.array(OPENAI_CLIP_STD)[:, None, None]) + np.array(OPENAI_CLIP_MEAN)[:, None, None]
#         unnormalized_image = (unnormalized_image * 255).astype(np.uint8)
#         unnormalized_image = np.moveaxis(unnormalized_image, 0, -1)
#         unnormalized_image = Image.fromarray(unnormalized_image)
#         return unnormalized_image

# unnormalized_image = get_preprocessed_image(image)
# target_sizes = torch.Tensor([unnormalized_image.size[::-1]])
# results = processor.post_process_object_detection(
# outputs=outputs, threshold=0.2, target_sizes=target_sizes)
_, buffer = cv2.imencode('.jpg', image)
image_bytes = buffer.tobytes()

files = {'file': (image_bytes)}

response = requests.post(url, files = files)
print(response.json())