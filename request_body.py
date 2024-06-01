from pydantic import BaseModel

# JSON body for the /chat endpoint
class ChatQuery(BaseModel):
   query: str | None
