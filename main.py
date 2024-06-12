from robots import *
from utils import *
from templates import api_docs, llm_prompt
import prompts
config = {
    "robot": "UR5",  # not using
    "robot_apis": ["place_object", "pick_object"],
    "utils_apis": ["detect_objects", "say"],
    "full_view": False,
    "scene": "dynamic",
    "scene_desc": "The scene has fruits (stacked) and baskets, whose coordinates we do not know.",
    "suffix": "Write code using the provided APIs to perform the task. No comments.",
    "error_handling": True,
    "imports": False,
    "code_output": True,
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
        return_val = api_docs.substitute({
            "robot_doc": self.robot_doc,
            "utils_doc": self.utils_doc
        })
        return return_val

    def get_prompt(self, task, filename):
        
        prompt = llm_prompt.substitute({
            "task": task,
            "scene_desc": self.scene_desc,
            "camera_view": prompts.get_camera_prompt(config["full_view"]),
            "scene_dynamic": prompts.get_scene_prompt(config["scene"]),
            "error_handling": prompts.get_error_prompt(config["error_handling"]),
            "imports": prompts.get_imports_prompt(config["imports"]),
            "code_output": prompts.get_code_output_prompt(config["code_output"]),
            "suffix": self.suffix,
            "api_docs": self.get_api_docs()
        })
        with open(filename, "w") as f:
            f.write(prompt)


def main():
    lmp = LMP(config)
    task = "Pick 3 apples and place it in the basket."
    lmp.get_prompt(task, "prompt.txt")


if __name__ == "__main__":
    main()
