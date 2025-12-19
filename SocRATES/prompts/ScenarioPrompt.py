from .BasePrompt import BasePrompt
from pydantic import BaseModel,PositiveInt
    
class PromptQuery(BasePrompt):
    def __init__(self) -> None:
        super().__init__()
        self.payload = [
            {
                "role":"system",
                "content":[{
                    "type":"text",
                    "text":"Provide outputs strictly in a paragraph form of at most 5 sentences."
                    }]
            },
            {"role":"user","content":[
                {
                    "type":"text",
                    "text":"""
You are an expert at crafting scenario descriptions for a simulation system. The simulation system converts your textual description into a simulation.
The simulation involves a single robot navigating to a goal while interacting with a variable number of humans at different locations in the environment. Each human has a specific behavior defined in a sentence using if-else constructs that make the human react to the robot in a specific manner
A simulation is defined by a tuple (N,L,{B1,B2,...BN}) where: 
 - N is the number of humans in the scenario. Each human interacts with the robot at some location 
 - L is the set of "interesting" locations that characterize the location where the scenario takes place.
 - Bi is the behavior of human i, which is a description of how the human behaves when the human interacts with the robot and other humans. It is an If-Then-Else description of the human's behavior and can be arbitrarily complex. 
    
Make sure the behavior descriptions are VERY SPECIFIC and realistic within the limited set of actions and conditions the simulator provides. 
Every single action or condition involved in the human's behavior, should be from the set of capabilities mentioned above. For example, the humans cannot do something depending what another human does or pickup something or jump etc. because these capabilities are not present in the above list.
r
YOUR SCENARIO DESCRIPTION SHOULD BE SUCCINT and should stick to a natural chained description of the scenario parameters. 

DO NOT PROVIDE ANY EXTRA INFORMATION, ADJECTIVES, QUALIFIERS, CONTEXT SENTENCES, EXPECTED ROBOT BEHAVIOR, DESCRIPTIONS THAT IS BEYOND THE LOCATION AND BEHAVIOR DESCRIPTIONS. STICK TO THE TASK AND BE BRIEF WHILE ENCAPSULATING ALL THE REQUIRED INFORMATION.

DO NOT USE 'OR' CONDITIONS FOR ACTIONS (Like IF condition Then Action 1 OR Action 2). 

Your input will be in the following format:
[N]: Number of humans in the scenario
[L1]: Type of location where the human 1 interacts with the robot
[L2]: Type of location where the human 2 interacts with the robot
...
[LN]: Type of location where the human N interacts with the robot
[Group IDs]: Comma separated group IDs for each human indicating which humans are in the same group and interact at the same location. a Group ID of -1 indicates the human is not part of any group. Note that humans in the same group should have the same interaction location (this will be ensured in the input). If there are humans as part of groups, make sure to describe this aspect in the scenario description.
[Human 1 Name]: Behavior Description
[Human 2 Name]: Behavior Description
...
[Human N Name]: Behavior Description

Your output should be in the following format: 
[Prompt]: <A SUCCINT prompt that Fully describes the scenario, the humans and their behaviors in a single paragraph>
"""}]}
]

    def get_full_prompt(self,**kwargs):
        behaviors = ""
        locations = ""
        groups = None
        if 'group_ids' in kwargs:
            groups = f"\n[Group IDs]: {kwargs['group_ids']}"
        for i in range(1,kwargs['num_humans']+1):
            locations += f"""\n[L{i}]: {kwargs['locations'][i]}"""
            # behaviors += f"""\n[Human {i}]: Actions: ({', '.join(kwargs['behaviors'][i]['actions'])}), Conditions: ({', '.join(kwargs['behaviors'][i]['conditions'])})"""
            behaviors += f"""\n[Human {i}]: {kwargs['behaviors'][i]}"""

        self.payload.append(
            {
            "role":"user",
            "content":[{
                "type":"text",
                "text":
                    f"""
Design a scenario relevant to the following specifications:

[N]:{kwargs['num_humans']}
\n{locations}\n
\n{behaviors}\n
"""}]
            }
        )
        if groups:
            self.payload[-1]['content'][0]['text'] += groups + "\n"
        
        return self.payload


"""
You are an expert at crafting scenario descriptions for a simulation system. The simulation system converts your textual description into a simulation.
The simulation involves a single robot navigating to a goal while interacting with a variable number of humans at different locations in the environment. Each human has a specific behavior defined in a sentence using if-else constructs that make the human react to the robot in a specific manner
A simulation is defined by a tuple (N,L,{B1,B2,...BN}) where: 
 - N is the number of humans in the scenario. Each human interacts with the robot at some location 
 - L is the set of "interesting" locations that characterize the location where the scenario takes place.
 - Bi is the behavior of human i, which is a description of how the human behaves when the human interacts with the robot and other humans. It is an If-Then-Else description of the human's behavior and can be arbitrarily complex. 
    
Make sure the behavior descriptions are VERY SPECIFIC and realistic within the limited set of actions and conditions the simulator provides. 
Every single action or condition involved in the human's behavior, should be from the set of capabilities mentioned above. For example, the humans cannot do something depending what another human does or pickup something or jump etc. because these capabilities are not present in the above list.

YOUR SCENARIO DESCRIPTION SHOULD BE SUCCINT and should stick to a natural chained description of the scenario parameters. 

DO NOT PROVIDE ANY EXTRA INFORMATION, ADJECTIVES, QUALIFIERS, CONTEXT SENTENCES, EXPECTED ROBOT BEHAVIOR, DESCRIPTIONS THAT IS BEYOND THE LOCATION AND BEHAVIOR DESCRIPTIONS. STICK TO THE TASK AND BE BRIEF WHILE ENCAPSULATING ALL THE REQUIRED INFORMATION.

You can translate the inputs into a prompt using simple templates. Some valid templates are given below, you can modify these to create more complex behaviors:
1. If (condition), then (action), else (action)
2. If (condition), then (action1), (action2) else (action3)
3. If (condition1), then (action1), else if (condition2), then (action2) else (action3)
4. For x seconds (action1), if (condition), then (action2) else (action3)
5. If (condition), then for x seconds (action1), else (action2)

DO NOT USE OR CONDITIONS FOR ACTIONS (Like IF condition Then Action 1 OR Action 2). 
YOU CAN ALSO CHANGE THE ORDER OF THE ACTIONS AND CONDITIONS TO CREATE MORE REALISTIC BEHAVIORS. THE INPUTS ARE ONLY THE SET OF CONDITIONS AND ACTIONS TO BE USED, NOT THE ORDER.

Your input will be in the following format:
[N]: Number of humans in the scenario
[L1]: Type of location where the human 1 interacts with the robot
[L2]: Type of location where the human 2 interacts with the robot
...
[LN]: Type of location where the human N interacts with the robot
[Human 1 Name]: (Action1, Action2, ..., ActionM),(Condition1, Condition2, ..., ConditionK) <- the actions and conditions that should be part of the behavior of human 1 
[Human 2 Name]: (Action1, Action2, ..., ActionM),(Condition1, Condition2, ..., ConditionK) <- the actions and conditions that should be part of the behavior of human 2
...
[Human N Name]: Actions: (Action1, Action2, ..., ActionM), Condition: (Condition1, Condition2, ..., ConditionK) <- the actions and conditions that should be part of the behavior of human N. You must combine these actions and conditions to create a realistic behavior description for the human.

Your output should be in the following format: 
[Prompt]: <A SUCCINT prompt that describes the scenario, the humans and their behaviors in a single paragraph>
    
"""