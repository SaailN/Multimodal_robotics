## Code for using LLM to plan the tasks.

This repository contains the central framework, which using the available Robot APIs (pick/place) and utility APIs (object detection) prompts an LLM to generate code for executing a given task.

### Getting Started

1. Go to Gemini AI console and generate your free API key. 
2. Make a `.env` file and put the key in this format - 
`API_KEY=<YOUR_API_KEY>`
3. Install the requirements. `pip install -r requirements.txt`
4. Run main.py


### File-wise Documentation

- *main.py* is the file doing everything. It contains a configuration for tuning the LLM prompt.
- *robots.py* implements the robot APIs.
- *utils.py* implements the utility APIs.
- *templates.py* implements the templates for the prompts for LLMs.
- *prompts.py* implements the prompt-engineering techniques for the templates.
- *output.txt* and *output.py* will contain the generated prompt and code, respectively.

<!-- ### Things left to do..

- Implement Utils APIs - 
    1. Implement object detection and update its docstring in the specified format.
    2. Implement say() and add text-to-speech.
- Implement Robot APIs - 
    1. Implement picking object with a request-response model for getting task completion status.
    2. Implement place object similarly. -->