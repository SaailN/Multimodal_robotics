from robots import *
from utils import *
from templates import api_docs, llm_prompt_full
import prompts
import llm_apis
from rich.console import Console
from constants import config
from robots import *
from utils import *
import wave
import pyaudio
import whisper
import sys

# Audio stream configuration
FORMAT = pyaudio.paInt16
CHANNELS = 1 if sys.platform == 'darwin' else 2
RATE = 16000
CHUNK = 1024
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True)
model = whisper.load_model("small")

# Language Model Program Class
class LMP:
    def __init__(self, config): # initialize with the class
        robot_doc = """"""
        for api in config["robot_apis"]:
            robot_doc += eval(api).__doc__
        utils_doc = """"""
        for api in config["utils_apis"]:
            utils_doc += eval(api).__doc__

        self.robot_doc = robot_doc
        self.utils_doc = utils_doc
        self.scene_desc = config["scene_desc"]
        self.suffix = config["suffix"]
        self.config = config

    def get_api_docs(self): # returns robots and utils documentation
        return_val = api_docs.substitute(
            {"robot_doc": self.robot_doc, "utils_doc": self.utils_doc}
        )
        return return_val

   
    def get_prompt(self, task): # returns the prompt for the task and writes it to a file

        prompt = llm_prompt_full.substitute(
            {
                "task": task,
                "scene_desc": self.scene_desc,
                "camera_view": prompts.get_camera_prompt(config["full_view"]),
                "scene_dynamic": prompts.get_scene_prompt(config["scene"]),
                "error_handling": prompts.get_error_prompt(config["error_handling"]),
                "imports": prompts.get_imports_prompt(config["imports"]),
                "code_output": prompts.get_code_output_prompt(config["code_output"]),
                "suffix": self.suffix,
                "api_docs": self.get_api_docs(),
                "recorded_task": prompts.recorded_task,
                "recorded_response": prompts.recorded_response
            }
        )
        with open(config["prompt_output_file"], "w") as f:
            f.write(prompt)
        return prompt


def main():
    console = Console() # initialize console for rich output

    console.print("[yellow]Initializing LLM API. [/yellow]")
    llm_apis.setup_llm_api()
    # console.control("\033[A\033[K")
    console.print("[green]LLM API initialized successfully. [/green]")

    lmp = LMP(config)
    # while True:
    #     task=console.input("Input task: \n")
    # task = "go to kitchen and pick up orange"
    # try:
    #     with wave.open('output.wav', 'wb') as wf:
    #         wf.setnchannels(CHANNELS)
    #         wf.setsampwidth(p.get_sample_size(FORMAT))
    #         wf.setframerate(RATE)

    #         console.print('[red]Recording...[/red]')
    #         while True:
    #             wf.writeframes(stream.read(CHUNK, exception_on_overflow=False))

    # except KeyboardInterrupt:
    #     stream.close()
    #     p.terminate()
    #     console.print("[red]Recording stopped[/red]")

    # task = model.transcribe('output.wav')["text"]
    # console.print("[yellow]Task: [/yellow]" + task)
    task = console.input("[yellow]Enter Task:") # text input for task
    prompt = lmp.get_prompt(task)

    console.print("[green]Prompt generated successfully. [/green]")
    console.print("[yellow]Sending prompt to LLM for code generation. [/yellow]")

    code = llm_apis.generate_llm_response(prompt, write=config["code_output_file"])
    console.print("[green]Code generated successfully. [/green]")

    console.print("[yellow]Printing code. [/yellow]")
    console.print(code)
    exec(code) # execute code


if __name__ == "__main__":
    main()
