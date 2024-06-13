import os
import google.generativeai as genai
from dotenv import load_dotenv

def setup_llm_api():
    load_dotenv()
    genai.configure(api_key=os.environ['API_KEY'])
    model = genai.GenerativeModel('gemini-1.5-flash')

def generate_llm_response(prompt, write=""):
    model = genai.GenerativeModel('gemini-1.5-flash')
    response = model.generate_content(prompt)
    code = response.candidates[0].content.parts[0].text
    if write:
        with open(write, 'w') as f:
            f.write(code.split('\n', 1)[1].rsplit('\n', 1)[0])
    return response.candidates[0].content.parts[0].text
