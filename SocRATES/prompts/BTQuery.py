from pydantic import BaseModel
class StructuredBTResponse(BaseModel):
    reasoning: str
    tree: str
    
from .BasePrompt import BasePrompt
class BTQuery(BasePrompt):
    def __init__(self) -> None:
        super().__init__()
        self.payload = [
        {"role": "system", "content": [{
         "type":"text",
         "text":f"""
        #Role and Objective
        You are an expert Behavior Tree Designer for a social navigation robotics simulator and an expert software engineer. Begin with a concise checklist (3-7 bullets) of what you will do; keep items conceptual, not implementation-level. Always provide output as a valid JSON object, fully parseable by Python's json.loads.
        You are diligent and tireless!
        You NEVER leave comments describing code without implementing it!
        You always COMPLETELY IMPLEMENT the needed code and do not leave placeholders or assume anything!
         """}]
        },         
        {"role": "user", 
     "content": [{
         "type":"text",
         "text":"""
Your job is to design a Behavior Tree using the BehaviorTree.CPP library in XML according to the [BEHAVIOR] described by the user, using only the existing actions and conditions. 
The tutorial below will explain how to design behavior trees in XML:
- ALL YOUR BEHAVIOR Trees MUST BE OUTPUT IN THIS XML FORMAT TO FILL IN THE TEMPLATE SHOWN BELOW (Fill ONLY the part indicated and leave the rest of the tree intact):
    <?xml version='1.0' encoding='utf-8'?>
    <root main_tree_to_execute='humanbehaviortree'>
        <include path='BTRegularNav.xml'/>
        <BehaviorTree ID='humanbehaviortree'>
            <Fallback name='behavior_fallback'>
                
                <!--- ONLY FILL IN THIS PART -->
            
                <Sequence name='RegNav'>
                    <SetBlackboard output_key='agentid' value='{id}' />
                    <SetBlackboard output_key='timestep' value='{dt}' />
                    <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
                </Sequence>
            </Fallback>
        </BehaviorTree>
    </root>
- <Sequence name="node_name"> and <Fallback name="node_name"> nodes dictate control flow in the behavior tree.
    - Sequence nodes execute their children in order from top to bottom and return Success only if ALL CHILD NODES returned SUCCESS. (like an AND gate).
        - Each sequence node can have multiple child nodes
        - Thus a sequence node describes a series of actions/conditions that are to be executed in order one after the other.
        - If even a single condition of a sequence node fails, the entire sequence node fails and the next sibling of the Sequence node is executed instead.
        
    - Fallback nodes execute their children in order from top to bottom and return Success if ANY CHILD NODE SUCCEEDS(like an OR gate). 
        - Each Fallback node must have AT LEAST 2 CHILD NODES.
        - Thus a fallback node describes multiple alternative strategies to achieve a goal and each strategy is activated only if the previous one fails. Success and failure is determined by conditioned within the fallback's children. 
        - If all nodes of a fallback node fail, the entire fallback node fails.
        - If even a single child of a fallback node succeeds, the entire fallback node succeeds.
    - Pay close attention to the differences between Sequence and Fallback nodes and where they should be used. Double check that you have used them correctly. 
        
- You will be ONLY BE FILLING IN the part of tree that governs what the human agent does when not regularly navigating to the goal. This could either be interactions with another human agent or interaction with the robot.  
- Each Sequence and Fallback TreeNodes have a single tag with attribute 'name'. E.g.:
    <Fallback name="SaySomething 1">
        <Sequence name = "SaySomething 2">
            ....
        </Sequence>
    </Fallback>
    
- How Behavior Trees are Executed:
    1. Order of execution of nodes:  Each node in the behavior tree is "ticked" (queried for execution) at every time step one by one depending on the conditions and actions from top to bottom. If the node returns success, the next node is ticked in order from top to bottom. If the node returns failure, the next node to be ticked depends on whether the parent node is a Sequence or Fallback node.
    2. Action nodes: Action nodes make the agent perform an action and always return success
    3. Condition nodes:  check for a condition and return success or failure depending on whether the condition is met.
    4. Sequence nodes:  executed from top to bottom until the first failure after which the entire sequence node returns failure. 
    - Thus in Sequence nodes, all nodes until the first failure are executed regardless of whether the entire sequence succeeds or fails.
    - If a single condition or action fails in a sequence node, the entire sequence node fails.
    4. Fallback nodes: executed from top to bottom until the first success after which the entire fallback node returns success.
    - If a node in a fallback node fails, the next child node of the fallback is executed.
    5. Sequence and Fallback nodes are all executed from top to bottom.
    
The following Action Nodes, Condition nodes and Decorators are available to use and can be composed into behavior trees to achieve the user's request. 
Note that you can only fill in values for node attributes with "??".

    - BT Conditions
        - <IsRobotVisible agent_id="{id}" distance="??" /> : returns success if robot is in line of sight within ?? metres. 
        - <IsRobotNearby agent_id="{id}" distance="??" /> : returns success if robot is within ?? metres of the agent (regardless of line of sight).            
        - <RobotSays agent_id="{id}" message="??"/> :  returns success if robot is saying ??. The possible inputs and their meanings are: 0 (No gesture), 1("WAIT"), 2("PROCEED"),3("ACKNOWLEDGED"),4("EXCUSE ME"),5("FOLLOW ME"). 
        - <HumanSays agent_id="{id}" target_id="??" message="??"/> :  returns success if human with the id = target_id is saying ??. The possible inputs and their meanings are: 0 (No gesture), 1("WAIT"), 2("PROCEED"),3("ACKNOWLEDGED"),4("EXCUSE ME"),5("FOLLOW ME"). This is the same as the number of the human - 1. For example, Human_1 has id = 0 and Human_2 has id = 1. target_id should be an integer-string only like "1".
        - <RobotMoved agent_id="{id}"/>: returns success if the robot is currently moving. 
        - <IsRobotBlocking agent_id="{id}" distance="??"/>: returns success if the robot is blocking the agent's path and within ?? meters.
         - <TimeExpiredCondition seconds="??" ts="{dt}", only_once="??"): Set a timer for ?? seconds and returns Success if ?? seconds have passed since the first time this node was ticked.
            -> How to use this node:     
                1. Do Something for ?? seconds and one time only: Inverter + only_once = True
                <Sequence name='DoSomethingForTime'>
                    <Inverter>
                        <TimeExpiredCondition seconds=?? ts='{dt}' only_once='True'/>
                    </Inverter>
                    <DoSomethingAction agent_id='{id}' ... />
                </Sequence>
                
                2. Delay DoSomething by ?? seconds (since the first tick of this node) before Doing Something and one time only: No inverter + only_once = True
                <Sequence name='DoSomethingAfterTime'>
                    <TimeExpiredCondition seconds='??' ts='{dt}' only_once='True'/>
                    <DoSomethingAction agent_id='{id}' ... />
                </Sequence>
                
                3. Do something every ?? seconds constantly: No inverter + only_once = False
                    <Sequence name='DoNothingEveryTime'>
                        <TimeExpiredCondition seconds='??' ts='{dt}' only_once='False'/>
                        <DoNothingAction agent_id='{id}' ... />
                    </Sequence>
                    
                4: Stop doing something every ?? seconds constantly: Inverter + only_once = False
                    <Sequence name='DoSomethingEveryTime'>
                        <Inverter>
                            <TimeExpiredCondition seconds='??' ts='{dt}' only_once='False'/>
                        </Inverter>
                        <DoSomethingAction agent_id='{id}' ... />
                    </Sequence>
                    
                Examples:
                -> timed sequential behavior: If condition is true, do something for 2 seconds then do something else:
                    <Condition agent_id='{id}'/>
                    <Fallback name='TimedBehaviorFallback'>
                        <Sequence name='DoSomethingfor2Seconds'>
                            <Inverter>
                                <TimeExpiredCondition seconds='2.0' ts='{dt}' only_once='True'/>
                            </Inverter>
                            <DoSomething agent_id='{id}' ... />
                        </Sequence>
                        <DoSomethingElse agent_id='{id}' time_step='{dt}'/>
                    </Fallback>
                    - for the first 2 seconds when the condition is true, the human will DoSomething. After 2 seconds, the DoSomething node will stop executing and the DoSomethingElse node will start executing since the TimeExpiredCondition will return success after 2 seconds have expired.
                    
                -> delayed behavior: If robot is nearby, after 3 second delay, make a gesture and give way to robot:
                    <Sequence name='NoticeAndGiveWay'>
                        <IsRobotNearby agent_id='{id}' distance='2.0'/>
                        <TimeExpiredCondition seconds='3.0' ts='{dt}' only_once='True'/>
                        <MakeGesture agent_id='{id}' message='2'/>
                        <GiveWaytoRobot agent_id='{id}' time_step='{dt}'/>
                    </Sequence>
                    
                -> constant behavior: If robot is nearby, every 1 second, make a gesture and give way to robot:
                    <Sequence name='NoticeAndGiveWay'>
                        <IsRobotNearby agent_id='{id}' distance='2.0'/>
                        <Inverter>
                            <TimeExpiredCondition seconds='1.0' ts='{dt}' only_once='False'/>
                        </Inverter>
                        <MakeGesture agent_id='{id}' message='2'/>
                        <GiveWaytoRobot agent_id='{id}' time_step='{dt}'/>
                    </Sequence>
                -> make the human wait: If Robot is nearby, freeze for 5 seconds then run away:
                    <Sequence name='WaitUntilRobotGoesAway'>
                        <IsRobotNearby agent_id='{id}' distance='2.0'/>
                        <Fallback name='WaitUntilTimeExpires'>
                            <Sequence name='WaitFor5Seconds'>
                                <Inverter>
                                    <TimeExpiredCondition seconds='5.0' ts='{dt}' only_once='False'/>
                                </Inverter>
                            </Sequence>
                            <RunAwayFromRobot agent_id='{id}' time_step='{dt}'/>
                    </Sequence>
            - Each TimeExpiredCondition node is independent and the time measured is from the first time that particular node is ticked. Keep this is mind when using multiple timeexpiredconditions. The time is NOT global across all nodes, it is relative to the first time the node is ticked.    

    - Note: In the simulator, for distance, 0.5 is considered very close, 1.0 is considered close, 2.0 is considered moderate distance and 5.0 is considered far
    - BT Actions:
        (a) Communication Actions:
            - <MakeGesture agent_id="{id}" message="??"/>: Makes the agent perform a gesture. Choices are: [0 (No gesture), 1("WAIT"), 2("PROCEED"), 3("EXCUSE ME")]. 
        (b) Movement Actions:
            - <LookAtRobot agent_id="{id}" /> : Makes the agent look in the direction of the robot
            - <FollowRobot agent_id="{id}" time_step="{dt}"/>: Makes the agent follow the robot
            - <FollowHuman agent_id="{id}" target_id="??" time_step="{dt}"/>: Makes the agent follow the human with a specific target_id (= ??). This is the same as the number of the human - 1. For example, Human_1 has id = 0 and Human_2 has id = 1. target_id should be an integer-string only like "1".
            - <RunAwayFromRobot agent_id="{id}" time_step="{dt}"/>: Makes the agent run away from the robot in a scared manner
            - <GiveWaytoRobot agent_id="{id}" time_step="{dt}"/>: Makes the agent give way to the robot.
            - <BlockRobot agent_id="{id}" time_step="{dt}"/>: Makes the agent move in front of the robot and block it
    
    - BT Subtree for Regular Navigation:
        - You can use the RegularNavTree subtree to make the agent navigate normally to its goal. In order to do this, just add the following snippet DIRECTLY WITH NO CHANGES where required. When using this subtree, DO NOT CHANGE ANYTHING IN THIS SNIPPET and add it as-is:
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
   
    - Rule 1: 2 Movement actions cannot be part of the same sequence node. This is because all movement actions perform conflicting actions. Use Fallback nodes to choose between multiple movement actions based on conditions.
        - For example, the following sequence is INVALID:
        <Sequence name='invalid_movement_sequence'>
            <FollowRobot agent_id="{id}" time_step="{dt}"/>
            <BlockRobot agent_id="{id}" time_step="{dt}"/>
        </Sequence>
        - Instead, you can use a Fallback node to choose between multiple movement actions based on conditions. For example:
            <Fallback name='movement_fallback'>
                <Sequence name='follow_robot_sequence'>
                    <IsRobotVisible agent_id="{id}" distance="2.0"/>
                    <FollowRobot agent_id="{id}" time_step="{dt}"/>
                </Sequence>
                <Sequence name='block_robot_sequence'>
                    <IsRobotNearby agent_id="{id}" distance="1.0"/>
                    <BlockRobot agent_id="{id}" time_step="{dt}"/>
                </Sequence>
            </Fallback>
    - Rule 2: Whenever using a movement action node, it should always be the last node within a parent sequence or fallback. Whenever a movement node is to be used, it should always be the last node of a sequence/fallback. No nodes should follow a movement action node within a parent sequence.
        
    - BT Decorators:
        - Inverter: An inverter block inverts the output of its children (If children output failure, inverter outputs success and vice-versa).
        - For example, to make the human only move if human agent with id 2 is not saying "WAIT":
            <?xml version='1.0' encoding='utf-8'?>
            <root main_tree_to_execute='humanbehaviortree'>
                <include path='BTRegularNav.xml'/>
                <BehaviorTree ID='humanbehaviortree'>
                    <Fallback name='behavior_fallback'>
                    
                        <Sequence name = 'sub_behavior'>  
                            <Inverter>
                                <HumanSays agent_id="{id}" target_id="2" message="1"/>
                            </Inverter>
                        </Sequence>
                    
                        <Sequence name='RegNav'>
                            <SetBlackboard output_key='agentid' value='{id}' />
                            <SetBlackboard output_key='timestep' value='{dt}' />
                            <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
                        </Sequence>
                    </Fallback>
                </BehaviorTree>
            </root>

    - Note that generally, conditions should be followed by actions. It is rarely the case that an action is followed by a condition.
         - Example:
         1. Make the agent follow the robot perpetually:
            <Sequence name='follow_robot_perpetually'>
                <FollowRobot agent_id="{id}" time_step="{dt}"/>
            </Sequence>
        2. Make the agent follow the robot for 10 seconds once:
            <Sequence name='follow_robot_once'>
                <Inverter>
                    <TimeExpiredCondition seconds="10.0" ts="{dt}" only_once="True" />
                </Inverter>
                <FollowRobot agent_id="{id}" time_step="{dt}"/>
            </Sequence>
        3. Make the agent follow the robot until its no longer visible:
            <Sequence name='follow_robot_perpetually'>
                <IsRobotVisible agent_id="{id}" distance="2.0"/>
                <FollowRobot agent_id="{id}" time_step="{dt}"/>
            </Sequence>

- Think carefully and design the tree. Do NOT directly translate the behavior from english to XML word by word. For example, if the behavior asks for "if robot is visible wait until it is not visible", you should NOT directly translate it to:
        <Sequence name='wait_until_robot_not_visible'>
            <IsRobotVisible agent_id="{id}" distance="2.0"/>
            <Inverter>
                <IsRobotVisible agent_id="{id}" distance="2.0"/>
            </Inverter>
        </Sequence>
        This is INCORRECT because this Sequence will ALWAYS FAIL no matter what since both nodes cannot be true at the same time. Instead, you can just use 1 condition so that the human effectively waits while the robot is visible:
        <Sequence name='wait_while_robot_visible'>
            <IsRobotVisible agent_id="{id}" distance="2.0"/>
        </Sequence>
- After designing the tree, simulate it step by step to check if it achieves the desired behavior.

- Along with the XML behavior tree and the reasoning outputs, you should also output a 'tree description' which is a direct translation of the XML behavior tree that you generated into pseudocode in the form of if-else condition. This will be used by users who are unfamiliar with behavior trees to understand the behavior you designed and provide feedback if needed. This should be created after you create the XML behavior tree.
    
Now answer the following question:

Behavior: gesture "WAIT" when the robot is visible within 1m but not stop moving during this time.

Return output in the following json format:
{   
    'Reasoning': <Reasoning behind answer>
    'Tree': <XML CODE>,
    'Tree Description': <Direct Translation of your Behavior Tree to pseudocode>
    
}"""}]},
        {
            "role": "assistant", 
            "content": [
                {
                    "type":"text",
                    "text":"""
                    {
    'Reasoning': "To create a behavior tree where a human gestures "WAIT," you need to use the `MakeGesture` action node with the message corresponding to the "WAIT" gesture. The message code for "WAIT" is 1. Since the human should not stop moving, the regular nav subtree is added in the first sequence as well
In this behavior tree:
- `<MakeGesture agent_id="{id}" message="1"/>` makes the agent perform the "WAIT" gesture (message code 1).
",
    'Tree': "
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
            
            <Sequence name = 'sub_behavior'>
                <IsRobotVisible agent_id="{id}" distance="1.0"/>
                <MakeGesture agent_id="{id}" message="1"/>
                <Sequence name='RegNav'>
                    <SetBlackboard output_key='agentid' value='{id}' />
                    <SetBlackboard output_key='timestep' value='{dt}' />
                    <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
                </Sequence>    
            </Sequence>
            
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>
",
'Tree Description': "
        if robot visible within 1m:
            make gesture 'WAIT'
            navigate normally to goal
        else:
            navigate normally to goal
    "
}"""
                }
            ]
        },
        {
            "role":"user",
            "content":[
                {
                    "type":"text",
                    "text":"""During the first 50 secs, if the the robot is visible, gesture 'WAIT' while navigating to the goal. what should be the corresponding xml code?
Return output in the following json format:
{   
    'Reasoning': <Reasoning behind answer>,
    'Tree': <XML CODE>,
    'Tree Description': <Direct Translation of your Behavior Tree to pseudocode>
    
}"""
                }
            ]
        },
        {
            "role":"assistant",
            "content":[
                {
                    "type":"text",
                    "text":"""
                    {
                    'Reasoning':"To create a behavior tree where a human checks if the robot is visible and gestures 'WAIT' during the first 50 seconds, you can use the `TimeExpiredCondition` with only_once = True to create the waiting period and the `IsRobotVisible` condition to check the robot's visibility and the `MakeGesture` action node with the message corresponding to the "WAIT" gesture. The message code for "WAIT" is 1. Since the human should continue navigating to the goal, the regular nav subtree is added in the fallback sequence as well.
In this behavior tree:
- Inverted `<TimeExpiredCondition seconds="50" ts="{dt}" only_once="true" />` creates a condition for 50 seconds.
- `<IsRobotVisible agent_id="{id}" distance="1.0"/>` checks if the robot is visible within 1m.
- `<MakeGesture agent_id="{id}" message="1"/>` makes the agent perform the "WAIT" gesture (message code 1)
This sequence ensures that the agent waits for 50 seconds before checking if the robot is visible and gestures 'WAIT' within the specified distance.
"
                    'Tree':"
<?xml version='1.0' encoding='utf-8'?>
    <root main_tree_to_execute='humanbehaviortree'>
        <include path='BTRegularNav.xml'/>
        <BehaviorTree ID='humanbehaviortree'>
            <Fallback name='behavior_fallback'>
            
                <Sequence name = 'sub_behavior'>
                    <Inverter>
                        <TimeExpiredCondition seconds="50" ts="{dt}" only_once="true" />
                    </Inverter>
                    <IsRobotVisible agent_id="{id}" distance="1.0"/>
                    <MakeGesture agent_id="{id}" message="1"/>      
                    <Sequence name='RegNav'>
                        <SetBlackboard output_key='agentid' value='{id}' />
                        <SetBlackboard output_key='timestep' value='{dt}' />
                        <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
                    </Sequence> 
                </Sequence>
                
                <Sequence name='RegNav'>
                    <SetBlackboard output_key='agentid' value='{id}' />
                    <SetBlackboard output_key='timestep' value='{dt}' />
                    <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
                </Sequence>
            </Fallback>
    </BehaviorTree>
</root>
                ",
                'Tree Description':"
    If 50s is Not expired (once) and robot visible within 1m:
            make gesture 'WAIT'
            navigate normally to goal
    else:
        navigate normally to goal
        
                
                "}"""
                }
            ]
        },
        {
            "role":"user",
            "content":[
                {
                    "type":"text",
                    "text":"""
Behavior: move only if the robot is saying "PROCEED".
"""
                }
            ]
        },
        {
            "role":"assistant",
            "content":[
                {
                    "type":"text",
                    "text":"""{
                        
                        'Reasoning':To create a behavior tree where a human moves only if the robot is saying "Proceed" you need to use the `RobotSays` condition node with the message corresponding to the "Proceed" gesture. 
                        The message code for "Proceed" is 2. To make the human stop moving unless the robot says Proceed, we can use the Inverter to reverse the logic.
In th behavior tree:
- `<RobotSays agent_id="{id}" message="2"/>` checks if the robot is saying "Proceed" (message code 2).

                        'Tree':"
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
            
            <Sequence name = 'sub_behavior'>
                <Inverter>
                    <RobotSays agent_id="{id}" message="2"/>
                </Inverter>
            </Sequence>
            
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>",      
"Tree Description":"
        if robot is NOT saying 'PROCEED': 
            Do Nothing
        else:
            navigate normally to goal
"}
"""   
                }
            ]
        },
        {
            "role":"user",
            "content":[{
                "type":"text",
                "text":"""
            Behavior: Human rushes towards the robot if the robot is visible after waiting for 5 seconds and continuously blocks the robot for 40s and asks it to WAIT.
            """
            }]
        },
        {
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{
    'Reasoning': "First, the human should wait for 5s before triggering the blocking and gesturing behavior. Then, run the BlockRobot node for 40s if the robot is visible with MakeGesture and falling back to regularNav when the robot is not visible.",
    'Tree':""
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
        
            <Sequence name='sub_behavior'>
                <IsRobotVisible agent_id="{id}" distance="4.0" />
                <Fallback name='WaitAndBlockFallback'>
                    <Sequence name='WaitBeforeBlocking'>
                        <Inverter>
                            <TimeExpiredCondition seconds="5.0" ts="{dt}" only_once="True" />
                        </Inverter>
                    </Sequence>
                    <Sequence name='GestureAndBlockRobotSequence'>
                        <!-- Then make gesture and block for 40s -->
                        <MakeGesture agent_id="{id}" message="1"/>
                        <Inverter>
                            <TimeExpiredCondition seconds="40.0" ts="{dt}" only_once="True" />
                        </Inverter>
                        <BlockRobot agent_id="{id}" time_step="{dt}" />
                    </Sequence>
                </Fallback>
            </Sequence>
        
            <Sequence name="RegNav">
                <MakeGesture agent_id="{id}" message="0"/>
                <SetBlackboard output_key="agentid" value="{id}" />
                <SetBlackboard output_key="timestep" value="{dt}" />
                <SubTree ID="RegularNavTree" id="agentid" dt="timestep" /> <!-- Using Subtree -->
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>",
},
"Tree Description":"
        if robot visible:
            If 5s is Not expired (once):
                Do Nothing
            else:
                make gesture 'WAIT
                if 40s is Not expired (once):
                    block robot
                else:
                    make gesture 'No Gesture'
                    navigate normally to goal
            else:
                make gesture 'No Gesture'
                navigate normally to goal
        else:
            make gesture 'No Gesture
            navigate normally to goal'"
            """}]
        },     
        {
            "role":"user",
            "content":[{
                "type":"text",
                "text":"""
Behavior: If the robot is visible at the intersection and its moving, the human will wait for the robot to say 'PROCEED' and then continue towards their goal normally, treating the robot as a normal obstacle afterward.
"""
            }]
        },
        {
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{
                    
                    'Reasoning':"The human waits for the robot to say 'PROCEED' (gesture code 2) "
              'when visible at the intersection and when it is moving, and then continues towards '
              'their goal using RegularNav.",
                    'Tree':
"<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
            
            <Sequence name='sub_behavior'>
                <IsRobotVisible agent_id="{id}" distance="4.0" />
                <RobotMoved agent_id='{id}'/>
                <Inverter>
                    <RobotSays agent_id="{id}" message="2" />
                </Inverter>
            </Sequence>
            
            <Sequence name="RegNav">
                <MakeGesture agent_id="{id}" message="0"/>
                <SetBlackboard output_key="agentid" value="{id}"/>
                <SetBlackboard output_key="timestep" value="{dt}"/>
                <SubTree ID="RegularNavTree" id="agentid", dt="timestep" /> <!-- Using Subtree -->
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>,
'Tree Description':"
    if robot is visible and robot moves and robot is NOT saying 'PROCEED':
        Do Nothing
    else:
        make gesture 'No Gesture'
        navigate normally to goal
"} """
            }]
        },{
            "role":"user",
            "content":[{
                "type":"text",
                "text":""" 
Behavior: When the robot is visible, the human follows the robot for 5 seconds, then blocks it for 5 seconds and then continues normal navigation to the goal.
"""
            }]
        },{
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{{
        'Reasoning': "To make the human follow and block the robot we need to use sequence of timeexpired conditions.",
        'Tree': "
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
            <Sequence name='sub_behavior'>
                <IsRobotVisible agent_id="{id}" distance="4.0" />
                <Sequence name='Follow'>
                    <Inverter>
                        <TimeExpiredCondition seconds="5.0" ts="{dt}" only_once="True" />
                    </Inverter>
                    <FollowRobot agent_id="{id}" time_step="{dt}" />
                </Sequence>
                <Sequence name='Block'>
                    <Inverter>
                        <TimeExpiredCondition seconds="5.0" ts="{dt}" only_once="True" />
                    </Inverter>
                    <BlockRobot agent_id="{id}" time_step="{dt}" />
                </Sequence>
            </Sequence>
            <Sequence name="RegNav">
                <SetBlackboard output_key="agentid" value="{id}"/>
                <SetBlackboard output_key="timestep" value="{dt}"/>
                <SubTree ID="RegularNavTree" id="agentid",dt="timestep" /> <!-- Using Subtree -->
            </Sequence>
        
        </Fallback>
    </BehaviorTree>
</root>",
'Tree Description': "
    if robot visible and 5s is Not expired (once):
        follow robot
        if 5s is Not expired (once):
            block robot
        else:
            navigate normally to goal
    else:
        navigate normally to goal
"
} """
            }]
        },
        {
            "role":"user",
            "content":[{
                "type":"text",
                "text":"""
Behavior: If the robot is visible at the intersection and its moving, the human will wait for 5 seconds and then continue towards their goal normally, treating the robot as a normal obstacle afterward. If the robot blocks the human's path during this time, the human will look at the robot.
"""
            }]
        },
        {
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{
                    
                    'Reasoning':"The human waits for the robot for 5 seconds if its visible and moving. This can be achieved with TimeExpiredCondition without inverter, IsRobotVisible and RobotMoved condition. After 5 seconds, the human continues normal navigation (The TimeExpiredCondition will return success after 5 seconds).
              If the robot blocks the human, human looks at the robot.",
                    'Tree':
"<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
            
            <Sequence name='sub_behavior'>
                <IsRobotVisible agent_id="{id}" distance="4.0" />
                <RobotMoved agent_id='{id}'/>
                <!-- Wait for 5 seconds -->
                <Fallback name='WaitFallback'>
                    <Sequence name='DoNothingFor5Seconds'>
                        <Inverter>
                            <TimeExpiredCondition seconds="5.0" ts="{dt}" only_once="True" />
                        </Inverter>
                    </Sequence>
                
                    <Sequence name='LookAtRobotIfBlocked'>
                        <IsRobotBlocking agent_id="{id}" distance="1.0"/>
                        <LookAtRobot agent_id="{id}" />
                    </Sequence>
                    
            </Sequence>
            <Sequence name="RegNav">
                <MakeGesture agent_id="{id}" message="0"/>
                <SetBlackboard output_key="agentid" value="{id}"/>
                <SetBlackboard output_key="timestep" value="{dt}"/>
                <SubTree ID="RegularNavTree" id="agentid", dt="timestep" /> <!-- Using Subtree -->
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>,
'Tree Description':"
    if robot is visible and robot moves:
        if 5s is expired (once):
            if robot is blocking within 1m:
                look at robot
            else:
                make gesture 'No Gesture'
                navigate normally to goal
        else:
            Do Nothing
    else:
        make gesture 'No Gesture'
        navigate normally to goal
"} """
            }]
        },
        {
            "role":"user",
            "content":[{
                "type":"text",
                "text":"""
Behavior: If robot is visible and is saying Excuse Me, the human will wait for 3 seconds. If robot is not visible, is nearby and says Excuse Me, human will look at robot and then run away getting scared.
"""
            }]
        },
        {
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{
                    
                    'Reasoning':"If IsRobotVisible and RobotSays Excuse Me, wait for 3s and then continue normal navigation. If not IsRobotVisible and IsRobotNearby and RobotSays Excuse Me, look at robot for a short period (2s) and then run away.",
                    'Tree':
"<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
            <!-- First behavior: If robot is visible and saying Excuse Me then Wait -->
            <Sequence name='sub_behavior'>
                <IsRobotVisible agent_id="{id}" distance="4.0" />
                <RobotSays agent_id="{id}" message="4"/>
                <!-- Wait for 3 seconds -->
                <Fallback name='WaitFor3SecondsFallback'>
                    <Sequence name='DoNothingFor3Seconds'>
                        <Inverter>
                            <TimeExpiredCondition seconds="3.0" ts="{dt}" only_once="True" />
                        </Inverter>
                    </Sequence>    
                    <Sequence name="MoveAfterWaiting">
                        <SetBlackboard output_key="agentid" value="{id}"/>
                        <SetBlackboard output_key="timestep" value="{dt}"/>
                        <SubTree ID="RegularNavTree" id="agentid", dt="timestep" /> <!-- Using Subtree -->
                    </Sequence>
            </Sequence>
            
            <!-- Second behavior: If robot is NOT visible, is nearby and saying Excuse Me then Look for a few seconds and Run Away -->
            <Sequence name='sub_behavior_2'>
                <Inverter>
                    <IsRobotVisible agent_id="{id}" distance="4.0" />
                </Inverter>
                <IsRobotNearby agent_id="{id}" distance="2.0" />
                <RobotSays agent_id="{id}" message="4"/>
                <Fallback name='LookAndRunFallback'>
                    <Sequence name='LookAtRobotFor2Seconds'>
                        <Inverter>
                            <TimeExpiredCondition seconds="2.0" ts="{dt}" only_once="True" />
                        </Inverter>
                        <LookAtRobot agent_id="{id}" />
                    </Sequence>
                    <RunAwayFromRobot agent_id="{id}" time_step="{dt}" />
                </Fallback>
            </Sequence>
            
            <Sequence name="RegNav">
                <MakeGesture agent_id="{id}" message="0"/>
                <SetBlackboard output_key="agentid" value="{id}"/>
                <SetBlackboard output_key="timestep" value="{dt}"/>
                <SubTree ID="RegularNavTree" id="agentid", dt="timestep" /> <!-- Using Subtree -->
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>,
'Tree Description':"
    if robot is visible and robot says 'EXCUSE ME':
        if 3s is expired (once):
            if robot is blocking within 1m:
                look at robot
            else:
                navigate normally to goal
        else:
            Do Nothing
    else:
        make gesture 'No Gesture'
        navigate normally to goal
"} """
            }]
        }]
        
        
        new_behavior_query = {
            "role":"user",
            "content":[{
                "type":"text",
                "text":"""Now, create a behavior tree for the following behavior:

Behavior: <BEHAVIOR>

Remember to Return an answer in JSON format shown below:"""+"""
{   
    'Reasoning': <reasoning behind tree design>,
    'Tree': <XML Behavior Tree ONLY>,
    'Tree Description': <Description of the behavior generated by this tree in pseudocode>
}"""
        }]
    }           
        self.payload.append(new_behavior_query)
            
    def get_full_prompt(self,**kwargs):
         full_prompt = self.payload
         full_prompt[-1]["content"][0]['text'] = full_prompt[-1]["content"][0]['text'].replace("<BEHAVIOR>",kwargs['behavior'])
         return full_prompt

'''
        {
            "role":"user",
            "content":[
                {
                    "type":"text",
                    "text":"""Given below is Behavior Tree for a human:
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
           
            <Sequence name='sub_behavior'>
                <IsRobotVisible agent_id='{id}' distance='4.0'/>
                <RobotSays agent_id='{id}' message='2'/>
                <MakeGesture agent_id='{id}' message='0'/>
            </Sequence>
            
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>
The required behavior for this tree is as follows:
When the robot is visible, the human will wait for the robot to say 'PROCEED' and only then continue towards their goal normally, treating the robot as a normal obstacle afterward. If the robot is not visible, the human will continue navigating normally.
There are some errors in the above XML code to model the above behavior. List the errors and correct them.
Return output in the following json format:
{
        'Reasoning': <Reasoning behind answer>
        'Tree': <LIST OF CORRECTIONS>,
        'Tree Description': <Direct Translation of your Behavior Tree to pseudocode>
}"""
                }
            ]
        },
        {
            "role":"assistant",
            "content":[
                {
                    "type":"text",
                    "text":"""{
        'Reasoning': "The provided behavior tree describes a human agent's behavior in a simulated environment. Here's a detailed explanation of the behavior corresponding to the tree:
### Explanation
#### Main Tree Execution
The main tree to execute is specified as `humanbehaviortree`.
#### Included File
The tree includes another behavior tree defined in `BTRegularNav.xml`. This file contains the behavior for regular navigation using the `RegularNavTree` ID.
#### Fallback Node (`behavior_fallback`)
The root node of the `humanbehaviortree` is a fallback node named `behavior_fallback`. A fallback node executes its children from top to bottom and returns `SUCCESS` if any child node returns `SUCCESS`. If all children return `FAILURE`, then the fallback node returns `FAILURE`.
#### First Sequence Node (`sub_behavior`)
The first child of the fallback node is a sequence node named `sub_behavior`. A sequence node executes its children from top to bottom and returns `FAILURE` if any child node returns `FAILURE`. If all children return `SUCCESS`, then the sequence node returns `SUCCESS`.
1. **`IsRobotVisible` Condition**
    - Checks if the robot is visible within a distance of 4.0 units from the human agent.
    - If the robot is not visible, this condition returns `FAILURE`, causing the sequence to fail and the fallback to proceed to the next child (RegNav Sequence).
2. **`RobotSays` Condition**
    - Checks if the robot is saying "PROCEED" (message code 2).
    - If the robot is not saying "PROCEED", this condition returns `FAILURE`, causing the sequence to fail and the fallback to proceed to the next child.
3. **`MakeGesture` Action**
    - Makes the human agent stop any gesture (message code 0).
    - If this action is successful, the sequence returns `SUCCESS`, causing the fallback node to return `SUCCESS` as well.
#### Second Sequence Node (`RegNav`)
If the first sequence (`sub_behavior`) fails, the fallback node proceeds to execute the second sequence node named `RegNav`.
### Summary
The human agent's behavior can be summarized as follows:
1. The agent first checks if the robot is visible within a 4.0 unit distance.
2. If the robot is visible and saying "PROCEED," the agent stops any gestures.
4. If either the robot is not visible or the robot is not saying "PROCEED" the agent proceeds with its regular navigation behavior defined in RegNav Sequence."
        'ANSWER': "### Errors in the Behavior Tree corresponding to the Given Behavior
1. The above behavior tree doesn't do RegularNav if the RobotSays "PROCEED" condition is satisfied and instead just calls the MakeGesture action, the human would just stop in place and not make any gestures or move if the robot is visible and is saying Proceed. This is a bug. The human should continue navigating to their goal if the robot is visible and says proceed, and the human should stop moving after the robot is visible unless the robot says proceed. Inverter node can be used to stop the human from moving when the robot is visible and not saying proceed. This will keep the execution stuck on the first sequence until the robot says proceed.

### Corrected Tree
Below is one possible corrected behavior tree to implement the given behavior exactly.
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='humanbehaviortree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='humanbehaviortree'>
        <Fallback name='behavior_fallback'>
            
            <Sequence name='sub_behavior'>
                <IsRobotVisible agent_id='{id}' distance='4.0'/>
                <Inverter>
                    <RobotSays agent_id='{id}' message='2'/>
                </Inverter>
            </Sequence>
            
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>"
}"""
                }
            ]
        },
'''