## ROSGPT

- Simple tool for executing actions in turtlebot
- Uses ontology based approach for converting unstrucutred human text to structured JSON objects
- Deserializes the JSON and publishes it to cmd_vel
- Very simple and limited use-case
- No fine-tuning done, only few shot learning

## ManipVQA

- Manip + Visual Question Answering
- ImageNet and PACO don't focus on granular information in images
- Datasets such as HANDAL and PhysObjects are available for robotics, they contain annotation for part marks, object attributes, and affordances
- Affordance defines the possibility of an agent to perform actions with an object.
- ManipVQA merges HANDAL, PhysObjects, PACO etc.
-  Referring Expression Comprehension (REC) involves the model receiving an image accompanied by a natural language description and subse
quently predicting the bounding box coordinates that delin
eate the specified target within the image. 
-  Conversely, REG prompts the model to produce a descriptive natural language statement about an area within an image, defined by pro
vided bounding box coordinates.
- Uses  SPHINX-1K  + LLaMA 2
- Given the necessity for both global and local visual grounding in robotic tasks, we integrate the visual encoder from CLIP [17] to extract local semantic features and the Q-Former [31] for summarizing visual features

## RT-1 

- Robotics Transformers 1
- Lots of details
- Essentially converts NLP to robotic tasks using a transformer architecture
- Uses FiLM EfficientNetB0
- Uses a lot of pre-collected data (130k episodes over 700 tasks, 7 skills and took 17 months to collect)
- Does pre-training with internet data

## RT - 2
- Robotics Transformers 2
- Tries to use VLMs and LLMs trained on internet data to generalise to robotics tasks and observe emergent behaviours
- trains a Visual Language Action (VLA) model
- Uses LLM + Visual Transformer 
- **Pre-trained on internet corpus**, then fine tuned and co-fine tuned again
- Same dataset as RT-1 used