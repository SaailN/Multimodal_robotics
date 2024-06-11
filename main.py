from robots import *
from utils import *

config = {
    "robot": "UR5",  # not using
    "robot_apis": ["place_object", "pick_object"],
    "utils_apis": ["detect_objects", "say"],
    "full_view": False,
    "scene": "dynamic",
    "scene_desc": "The scene has fruits (stacked) and baskets, whose coordinates we do not know.",
    "suffix": "Write code using the provided APIs to perform the task. No comments.",
    "error_handling": False,
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

    def get_api_docs(self):
        docs = f"""
Robot APIs:
        {self.robot_doc}
Utils APIs:
        {self.utils_doc}
        """
        return docs

    def get_prompt(self, task, filename):
        camera_view = (
            "The camera may not see the entire scene at once."
            if not config["full_view"]
            else "The camera sees the entire scene at once."
        )
        scene_dynamic = (
            "The scene is dynamic. Objects may move over time."
            if config["scene"] == "dynamic"
            else "The scene is static."
        )
        error_handling = (
            "Error handling is required."
            if config["error_handling"]
            else "Error handling is not required."
        )
        imports = (
            "Import statements required"
            if config["imports"]
            else "No import statements required."
        )
        code_output = (
            "Code output is required."
            if config["code_output"]
            else "Code output is not required."
        )
        prompt = f"""These are the APIs defined.
        {self.get_api_docs()}

Task - {task}

{self.scene_desc}. {camera_view} {scene_dynamic}
{self.suffix}

{error_handling}
{imports}
{code_output}

        """
        with open(filename, "w") as f:
            f.write(prompt)


def main():
    lmp = LMP(config)
    task = "Pick 3 apples and place it in the basket."
    lmp.get_prompt(task, "prompt.txt")


if __name__ == "__main__":
    main()
