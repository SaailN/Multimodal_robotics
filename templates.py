from string import Template

api_docs = Template(
    """
Robot APIs:
        $robot_doc
Utils APIs:
        $utils_doc
"""
)

llm_prompt = Template(
    """
PROMPT:
These are the APIs defined.
$api_docs

Task - $task

$scene_desc. $camera_view $scene_dynamic
$suffix

$error_handling
$imports
$code_output"""
)
