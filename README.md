## API Endpoints

**Access at:** https://usable-frank-kitten.ngrok-free.app/

### Enable FastAPI server and connect to ngrok**

```fastapi run```

```ngrok http --domain=usable-frank-kitten.ngrok-free.app 8000```


### Routes:

1. /command - 

**Request** - 
class ChatQuery(BaseModel):
   query: str | None

**Response** - 

{"response": {"x": 0, "y": 0, "z": 0}}