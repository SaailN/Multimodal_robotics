from fastapi import FastAPI, Response, Request
import httpx
from contextlib import asynccontextmanager
from request_body import ChatQuery
import json 

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
            "prompt": """I will give you x, y, and z coordinates as three numbers in my query. 
                            Please return a JSON object of the form {\"x\": <integer>, \"y\": <integer>, \"z\": <integer>}. 
                            Do not respond with any text, just the JSON object. My query starts now - """ + query.query,
            "stream": False,
            "format": "json"}
    url = "http://0.0.0.0:11434/api/generate"
    requests_client = request.app.requests_client
    response = await requests_client.post(url, json=json_obj, timeout=None)
    response = json.loads(response.json()["response"])
    return_response = {
        "response": response
    }
    return Response(
        content=json.dumps(return_response),
        status_code=200
    )
    


