#!/usr/bin/env python3
import requests
import cv2
import matplotlib.pyplot as plt
url = 'https://4ce4-103-21-125-76.ngrok-free.app/image/'

path = 'test2.png'
img = cv2.imread(path)
h, w, _ = img.shape
img = img[300: h - 100, 300: w - 300]
cv2.imshow('frame', img)
imageItem = {
    "image": img.tolist()
}
print(img.shape)

# item={"name" : "Abhigyan", "price" : 12}
# response = requests.post(url, json = item)
# print(response.json())
_, buffer = cv2.imencode('.jpg', img)
image_bytes = buffer.tobytes()

files = {'file': (image_bytes)}
response = requests.post(url, files=files)
print(response.json())
cv2.waitKey(0)
