from robots import *
from utils import *
from templates import api_docs, llm_prompt
import prompts
import llm_apis
from rich.console import Console


config = {
    "robot": "UR5",  # not using
    "robot_apis": ["place_object", "pick_object"],
    "utils_apis": ["detect_objects", "say"],
    "full_view": False,
    "scene": "dynamic",
    "scene_desc": "The scene has fruits (stacked) and baskets, whose coordinates we do not know. The robot can pick only 1 object at a time.",
    "suffix": "Write code using the provided APIs to perform the task. No comments.",
    "error_handling": True,
    "imports": False,
    "code_output": True,
    "prompt_output_file": "prompt.txt",
    "code_output_file": "output.py",
}


class LMP:
    def __init__(self, config):
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

    def get_api_docs(self):
        return_val = api_docs.substitute(
            {"robot_doc": self.robot_doc, "utils_doc": self.utils_doc}
        )
        return return_val

    def get_prompt(self, task):

        prompt = llm_prompt.substitute(
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
            }
        )
        with open(config["prompt_output_file"], "w") as f:
            f.write(prompt)
        return prompt


def main():
    console = Console()

    console.print("[yellow]Initializing LLM API. [/yellow]")
    llm_apis.setup_llm_api()
    # console.control("\033[A\033[K")
    console.print("[green]LLM API initialized successfully. [/green]")

    lmp = LMP(config)

    task = console.input("Input task: \n")
    prompt = lmp.get_prompt(task)

    console.print("[green]Prompt generated successfully. [/green]")
    console.print("[yellow]Sending prompt to LLM for code generation. [/yellow]")

    code = llm_apis.generate_llm_response(prompt, write=config["code_output_file"])
    console.print("[green]Code generated successfully. [/green]")

    console.print("[yellow]Printing code. [/yellow]")
    console.print(code)


if __name__ == "__main__":
    main()
