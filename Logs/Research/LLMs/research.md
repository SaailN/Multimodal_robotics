# Code as Policies
Recent progress in natural language processing
shows that (LLMs) pretrained on Internetscale data exhibit out-of-the-box capabilities that can be applied to language-using robots e.g., planning a
sequence of steps from natural language instructions without additional model finetuning.

![bloc](<Screenshot 2024-06-07 162126.png>)


LLMs trained on code-completion have shown to be capable of synthesizing Python programs from docstrings. We find that
these models can be re-purposed to write robot policy code, given
natural language commands (formatted as comments)



Code-writing models can express a variety of arithmetic operations
as well as feedback loops grounded in language. They not only
generalize to new instructions, but having been trained on billions
of lines of code and comments, can also prescribe precise values
(e.g., velocities) to ambiguous descriptions ("faster" and "to the
left") depending on context – to elicit behavioral commonsense.



KEY POINTS: 

1. code as policies: a formulation
of using LLMs to write robot code,
2. a method for hierarchical
code-gen that improves state-of-the-art on both robotics and
standard code-gen problems with 39.8% P@1 on HumanEval
3. a new benchmark to evaluate future language models on
robotics code-gen problems
4. ablations that analyze how
CaP improves metrics of generalization and that it abides
by scaling laws – larger models perform better


Socratic Models uses visual language models (VLMs) to substitute perceptual information into the language prompts
that generate plans, and it uses language-conditioned policies e.g.,
for grasping. The following example illustrates the qualitative
differences between our approach versus the aforementioned prior
works.

 When tasked to "move the coke can a bit to the right":
![diff](<Screenshot 2024-06-07 151344.png>)

## RoboCodeGen
We introduce a new benchmark with 37 function generation problems with several key differences from previous code-gen benchmarks:
1. it is robotics-themed with questions
on spatial reasoning (e.g., find the closest point to a set of points),
2. geometric reasoning (e.g., check if one bounding box is contained
in another)


![table](<Screenshot 2024-06-07 162326.png>)


## Prompting Language Model Programs
Prompts to generate LMPs contain two elements:
1. Hints e.g., import statements that inform the LLM which APIs
are available and type hints on how to use those APIs.
2. Examples are instruction-to-code pairs that present few-shot
"demonstrations" of how natural language instructions should be
converted into code. 

## CaP: Pick & Place Policies for Table-Top Manipulation

The table-top manipulation domain tasks a UR5e robot arm
to pick and place various plastic toy objects on a table. The
arm is equipped with a suction gripper and an in-hand Intel
Realsense D435 camera. We provide perception APIs that detect
the presences of objects, their positions, and bounding boxes, via
MDETR . We also provide a scripted primitive that picks an
object and places it on a target position. Prompts are similar to
those from the last domain, except trajectory parsing is replaced
with position parsing

With unseen task attributes, CLIPort’s
performance degrades significantly, while LLM-based methods
retain similar performance. On unseen tasks and attributes, end-to-end systems like CLIPort struggle to generalize and CaP
outperforms LLM reasoning directly with language.

## CaP: Mobile Robot Navigation and Manipulation
In this domain, a robot with a mobile base and a 7 DoF arm is
tasked to perform navigation and manipulation tasks in real-world
kitchen. For perception, the LMPs are given object detection APIs
implemented via ViLD. For actions, the robot is given APIs to
navigate to locations and grasp objects via both names and coordinates