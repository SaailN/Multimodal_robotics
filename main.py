from fastapi import FastAPI, Response, Request
import httpx
from contextlib import asynccontextmanager
from request_body import ChatQuery, InferQuery
import json 
from constants import * 
import base64 


# required for async post requests from server
@asynccontextmanager
async def lifespan(app: FastAPI):
    app.requests_client = httpx.AsyncClient()
    yield
    await app.requests_client.aclose()

# create app 
app = FastAPI(lifespan=lifespan)

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
        if not image_url.startswith("http"): # not a URL
            return Response(
                content=json.dumps({"response": "Invalid URL of image"}),
                status_code=400
            )
        with httpx.stream("GET", image_url) as response:
            if response.status_code == 200:
                image = response.read()
                # print("Fetched image")
                base64_data = base64.b64encode(image)
                base64_string = base64_data.decode('utf-8')
                images.append(base64_string)
            else:
                return Response(
                    content=json.dumps({"response": "Invalid URL of image"}),
                    status_code=400
                )
        
    # print(images[0])
    
    json_obj = {
            "model": "llava",
            "prompt": query.query,
            "stream": False,
            "images": images}
    # print("Sending request to localhost")
    response = await requests_client.post(OLLAMA_URL, json=json_obj, timeout=None)
    response = json.loads(response.json()["response"])
    return_response = {
        "response": response
    }
    return Response(
        content=json.dumps(return_response),
        status_code=200
    )
