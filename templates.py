from string import Template

api_docs = Template(
    """
Robot APIs:
        $robot_doc
Utils APIs:
        $utils_doc
"""
)

llm_prompt_full = Template(
    """
PROMPT:
These are the APIs defined.
$api_doc

Task - $task

$scene_desc. $camera_view $scene_dynamic
$suffix

$error_handling
$imports
$code_output

RESPONSE:
"""
)

llm_prompt_task = Template(
    """
Task - $task

RESPONSE:
"""
)

