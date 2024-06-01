from pydantic import BaseModel

# JSON body for the /chat endpoint
class ChatQuery(BaseModel):
   query: str | None

class InferQuery(BaseModel):
   query: str | None 
   images : list[str] | None