from fastapi import FastAPI, Response, Request
import httpx
from contextlib import asynccontextmanager
from request_body import ChatQuery, InferQuery, InstructQuery
import json 
from constants import * 
import base64 
from utils import *
import os 
import torch

# required for async post requests from server
@asynccontextmanager
async def lifespan(app: FastAPI):
    app.requests_client = httpx.AsyncClient()
    yield
    await app.requests_client.aclose()

# create app 
app = FastAPI(lifespan=lifespan)

yolo_model = torch.load("yolov5s.pt")

# index
@app.get("/")
def index():
    return Response(
    content="API endpoint working. Built for e-YSIP.",
    status_code=200
) 

# command endpoint (x, y, z)
@app.post("/command")
async def command(request: Request, query: ChatQuery):
    json_obj = {
            "model": "llava",
            "prompt": """I will give you x, y, and z coordinates as three numbers in my query. Unless specified, 
                            the first number is x, second is y, third is z. If x, y,z are mentioned in the query in a different order, map the numbers
                            accordingly.
                            Please return a JSON object of the form {\"x\": <integer>, \"y\": <integer>, \"z\": <integer>}. 
                            Do not respond with any text, just the JSON object. My query starts now - """ + query.query,
            "stream": False,
            "format": "json"}
    requests_client = request.app.requests_client
    response = await requests_client.post(OLLAMA_URL, json=json_obj, timeout=None)
    response = json.loads(response.json()["response"])
    return_response = {
        "response": response
    }
    return Response(
        content=json.dumps(return_response),
        status_code=200
    )

# VLM interfacing - gives inference of images(s)
@app.post("/infer") 
async def infer(request: Request, query: InferQuery):
    requests_client = request.app.requests_client

    images = [] # contains base64 of all images
    
    for image_url in query.images:
        image_path = download_image(image_url)
        if not image_path:
            return Response(
                content="Error downloading image",
                status_code=400
            )
        compress_image(image_path, image_path, quality=10)
        base64_string = get_base64(image_path)
        images.append(base64_string)
        delete_image(image_path)
            
    json_obj = {
            "model": "llava",
            "prompt": query.query,
            "stream": False,
            "images": images}
    # print("Sending request to localhost")
    response = await requests_client.post(OLLAMA_URL, json=json_obj, timeout=None)
    response = response.json()["response"]
    return_response = {
        "response": response
    }
    return Response(
        content=json.dumps(return_response),
        status_code=200
    )

@app.get('/instruct')
async def instruct(request: Request, query: InstructQuery):
    image_url = query.image
    image_path = download_image(image_url)
    if not image_path:
        return Response(
            content="Error downloading image",
            status_code=400
        )
    
    im = Image.open(image_path)
    result = yolo_model([im])
    result = result.pandas().xyxy[0]

    llm_prompt = json.dumps(result["name"].to_dict())
    user_ask = query.query

    json_obj = {
            "model": "llava",
            "prompt": f"""The user has given the query {query.query}. Is it executable given the fact that the scene has the objects given by the following JSON
                        object? {llm_prompt}
                        Output 0 if no, 1 if yes.""",
            "stream": False,
            "format": "json"}
    
    requests_client = request.app.requests_client
    response = await requests_client.post(OLLAMA_URL, json=json_obj, timeout=None)
    print(response)