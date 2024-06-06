import torch 

yolo_model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

img = ["https://www.shutterstock.com/image-photo/red-apple-isolated-on-white-600nw-1727544364.jpg"]

results = yolo_model(img)

results = results.pandas().xyxy[0]

print(results.iloc[0])
x_center, y_center = int((results.iloc[0]["xmin"] + results.iloc[0]["xmax"]) / 2), int((results.iloc[0]["ymin"] + results.iloc[0]["ymax"]) / 2)

print(x_center, y_center)
