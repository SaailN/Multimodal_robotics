from PIL import Image
from fastapi import Response
import json
import httpx 
import base64 
import os 

def compress_image(image_path, output_path, quality=10):
    image = Image.open(image_path)
    image.save(output_path, quality=quality)

def download_image(image_url):
    if not image_url.startswith("http"): # not a URL
            return None
    with httpx.stream("GET", image_url) as response:
        if response.status_code == 200:
            image = response.read()
            filepath = "dmp/" + image_url.split("/")[-1] + ".jpg"
            with open(filepath, "wb") as f:
                f.write(image)
            return filepath
        else:
            return None 
        
def get_base64(filepath):
    with open(filepath, "rb") as f:
        image = f.read()
        base64_data = base64.b64encode(image)
        base64_string = base64_data.decode('utf-8')
        return base64_string
    
def delete_image(filepath):
    os.remove(filepath)