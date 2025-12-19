from .BasePrompt import BasePrompt
from pydantic import BaseModel,PositiveInt

# Recommended prompt structure: https://cookbook.openai.com/examples/gpt4-1_prompting_guide#recommended-workflow
# Role and Objective
# Instructions
## Sub-categories for more detailed instructions
# Reasoning Steps
# Output Format
# Examples
## Example 1
# Context
# Final instructions and prompt to think step by step


class ScenarioQuery(BasePrompt):
    def __init__(self,qa) -> None:
        super().__init__()
        self.payload = [
            {
                "role":"system",
                "content":[{
                    "type":"text",
                    "text":"You are an expert scenario designer. Begin with a concise checklist (3-7 bullets) of what you will do; keep items conceptual, not implementation-level. Always provide output as a valid JSON object, fully parseable by Python's json.loads."
                    }]
            },
            {"role":"user","content":[
                {
                    "type":"text",
                    "text":"""

# Role and Objective
You are a scenario designer for testing social navigation capabilities of robots. Your task is to parse an input [scenario description] in a given location (represented by a scene graph with available node types and edge types) into robot and human trajectory descriptions and human behaviors. Your output should resolve ambiguities in the input and provide output trajectory descriptions and behaviors that match the capabilities of the simulator available for generating the scenario. Your outputs will be used to design behavior trees to control simulated humans and design paths for the humans and the robot in a social navigation simulator. 

# Instructions
A Social Navigation Scenario is defined by 2 components:
    1. Trajectory Description: Detailed description of the location constraints for the robot and human trajectories. This involves:
        - What type of location (described as a node or edge type in the scene graph for the location) each human interacts with the robot: Every human must intereact with the robot at least once.
        - Constraints on the robot or human trajectories mentioned in the [scenario description] (if any) : For e.g., the robot must go through an 'intersection' before reaching the 'blind corner' where it interacts with Human 2. Human 1 needs to wait in an 'open area' before interacting with the robot in a 'passageway'. 
    2. Human Behavior: How each human in the scenario behaves with respect to the robot, for e.g. "Human 1 is scared of the robot and asks it to stop when its nearby", "Human 2 doesn't notice the robot at all and keeps walking" etc. This should respect the constraints of the simulator (described below), that is, the humans can only perform actions that are possible in the simulator.

Rules for generating the Trajectory Description:
- Ensure the output scenario is strictly aligned to the [scenario description] and at least mentions all the location constraints mentioned in the [scenario description].
- Every single human mentioned in the [scenario description] should be included in the trajectory description and interact with the robot at least once. If the [scenario description] doesn't mention constraints on the trajectory of a human, you can describe this fact directly in the trajectory description.
- When the scenario involves a 'group' of humans, add all group members to the number of humans in the scenario and mention that they are in a group. This is very important.
- If the [scenario description] mentions some of the human being in a group, ensure this information is captured in the trajectory description as well clearly (which humans are in a group).
- Specify interaction/trajectory constraint locations using ONLY the available node types and edge types provided. Do not invent new location types.
- The available node types are: [node types]. The available edge types are: [edge types].
- Double check that you've added the correct number of humans in the scenario. Every human mentioned in the scenario description must be accounted for in the number of humans

Rules for Describing Human Behavior:
- Ensure the output scenario is strictly aligned to the [scenario description]
- The human behavior descriptions should be limited to the simulator capabilities. The simulation generation will fail if behaviors outside the simulator's capabilities are specified. Therefore, ensure that all the behavior proposals are strictly within the behaviors listed below. If any behavior requested in the [scenario description] is not possible, mention this in the [reasoning] section of the output and provide an alternative behavior that is close to the [scenario description] but within the behaviors provided below. 
- A valid human behavior is any combination of conditions and actions available below: 
    - Check if the robot is visible within a certain distance
    - Check if robot is gesturing towards them: The robot can say "WAIT", "PROCEED", "ACKNOWLEDGED", "EXCUSE ME", "FOLLOW ME". 
    - Check if one of the other humans is gesturing to them. Valid gestures are: "WAIT", "PROCEED", "ACKNOWLEDGED", "EXCUSE ME", "FOLLOW ME".
    - Check if the robot is currently moving
    - Check if the robot is blocking the human's path
    - Perform a gesture: The humans can say "WAIT", "PROCEED" and "EXCUSE ME" to the robot.
    - Look in the direction of the robot
    - Follow the robot
    - Follow one of the humans (specify which human to follow)
    - Get scared of the robot and run away from it
    - Give way to the robot (Get out of the robot's way and let the robot pass)
    - Block the robot's path
    - Do Nothing (Equivalent to Stop moving/Wait/Yield/Ignore)
    - Navigate to the goal location (whatever it may be) 
    - Do any of the above actions for a specific time duration (specify time duration).
    Note: at any point, a human can only perform one of the above actions. For example, a human cannot "Navigate to goal" and "Look in the direction of the robot" at the same time.
    
- Make sure the behavior descriptions are VERY SPECIFIC and belong to the set of actions and conditions specified above. Do not add extra conditions for actions or change the behavior that's specified in the input [scenario description] in any way other than elaborating ambiguity. Describe it succinctly and clearly.
- Every single action or condition involved in the human's behavior in the [scenario description] should be accounted for in the output and should be from the set of capabilities mentioned above. For example, the humans cannot do something depending what another human does or pickup something or jump etc. because these capabilities are not present in the above list.


# Output Format 

Always respond with a JSON object containing:
    - 'robot': <trajectory description for robot>
    - 'humans':[
            {
                'name': 'human 1',
                'trajectory': <trajectory description for human 1> -  type of location where human 1 interacts with robot + constraints in the trajectory (if any).
                'behavior': <behavior description for human 1>
            },
            {
                'name': 'human 2',
                'trajectory': <trajectory description for human 2> -  type of location where human 2 interacts with robot + constraints in the trajectory (if any).
                'behavior': <behavior description for human 2>
            },
            ... <- add as many entries as there are humans in the scenario description
            ],
    - 'groups': [<list of tuples representing groups of humans in the scenario, or None if no groups>]
    
            
- All JSON objects and arrays must use double quotes for keys and string values; ensure output is valid JSON.
- All fields are mandatory for each agent; do not omit or add fields.
                """}
                ]
    }
]
        
        for i,item in enumerate(qa):
            if i == 0:
                self.payload[-1]['content'].append(
                    {
                "type":"text",
                "text":item[0] #question
                }
                )
            else:
                self.payload.append({
                    "role":"user",
                    "content":[{
                    "type":"text",
                    "text":item[0] #question
                }]})
            
            self.payload.append({
                "role":"assistant",
                "content":[{
                "type":"text",
                "text":item[1] #answer
            }]})
         
    def get_full_prompt(self,**kwargs):
        self.payload.append(
            {
            "role":"user",
            "content":[{
                "type":"text",
                "text":
                    f"""
Design a scenario relevant to the following specifications:

[Scenario Description]:{kwargs['context']}
[Node Types]:{kwargs['node_types']}
[Edge Types]:{kwargs['edge_types']}
"""
                }]
            }
        )

        return self.payload
        
        
        
'''
            {
                "role":"assistant",
                "content":[{
                    "type":"text",
                    "text":""" The following behaviors are required for simulating this behavior:
                    - Identify where an aisle is (not possible)
                    - Walk towards a goal (possible: Navigate to goal)
                    - Pick up box (not possible)
                    - Ignore robot (possible: Do Nothing)
                    - Recognize when the robot says 'EXCUSE ME' (possible)
                    Because there are non-simulatable actions/conditions, it is not possible to simulate this behavior. """
                }]
            },
            {
                "role":"user",
                "content":[{
                    "type":"text",
                    "text":"""
Is the following a valid behavior for the simulated human: 
If the robot is close to Human 1, He looks at what Human 2 is doing and then get's scared and goes away from the robot.
                    """
                }]
            },
            {
                "role":"assistant",
                "content":[{
                    "type":"text",
                    "text":"""
                    The following behaviors are required for simulating this behavior: 
                    - check if robot is close (possible: Check if the robot is visible within a certain distance)
                    - look at Human 2 (possible)
                    - check what Human 2 is doing (not possible)
                    - get scared and go away from robot (possible: get scared of robot and avoid it)
                    Because there are non-simulatable actions/conditions, it is not possible to simulate this behavior. 
                    """
                }]
            },
            {
                "role":"user",
                "content":[{
                    "type":"text",
                    "text":"""
                    Is the following a valid behavior for the simulated human:
If the robot is blocking the human's way, then he says 'EXCUSE ME' and avoids the robot. 
                    """
                }]
            },
            {
                "role":"assistant",
                "content":[{
                    "type":"text",
                    "text":""" 
                    The following behaviors are required for simulating this behavior: 
                    - Check if the robot is blocking their way (possible: Check if the robot is blocking the human's path)
                    - Say 'EXCUSE ME' (possible: Perform a gesture (EXCUSE ME))
                    - Avoid the robot (possible: Get scared and avoid it or Give way to the robot.)
                    Since all the required behaviors are available to the human in the simulation, this behavior is possible to simulate.
                    """
                }]
            },
'''
    
