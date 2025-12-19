import base64
import json
from os import error
import tiktoken
import math
import io
import re
import PIL.Image as Image
from io import BytesIO
from collections.abc import Mapping
import networkx as nx
from PIL import Image as PILImage
import random
from termcolor import colored
import pprint
import xml.etree.ElementTree as ET
from termcolor import cprint
from pydantic import BaseModel,StrictInt,PositiveInt

class SimpleResponse(BaseModel):
    response: str

class HumanTraj(BaseModel):
    name: str
    groupid: StrictInt
    trajectory: list[str]
    interaction_point: str

class SimpleHumanTraj(BaseModel):
    name: str
    trajectory: list[str]

class SimpleTrajectories(BaseModel):
    robot: str
    humans: list[SimpleHumanTraj]
    
class Trajectories(BaseModel):
    robot: list[str]
    humans: list[HumanTraj]
    
class StructuredTrajResponse(BaseModel):
    reasoning: str
    trajectories: Trajectories

class StructuredBTResponse(BaseModel):
    reasoning: str
    tree: str
    tree_description:str
    
class Behavior(BaseModel):
    name: str
    behavior: str

class Human(BaseModel):
    name: str
    behavior: str
    trajectory: str
    
class StructuredScenarioResponse(BaseModel):
    robot: str
    humans: list[Human]
    groups: list[list[str]]

class StructuredResponse(BaseModel):
    scenario: StructuredScenarioResponse
    behavior: list[StructuredBTResponse]
    trajectories: Trajectories

eprint = lambda x:cprint(x,'red') #error
iprint = lambda x:cprint(x,'yellow') #user input
rprint = lambda x:cprint(x,'green') #result
lprint = lambda x:cprint(x,'blue') #log

def parse_questions_answers(file_path):
    questions_answers = []

    # Open the file and read its contents
    with open(file_path, 'r') as file:
        content = file.read()

    # Split the content by the separator "*****"
    qa_pairs = content.split("*****")

    # Iterate over each QA pair
    for qa in qa_pairs:
        if qa.strip():  # Ensure the string is not empty
            lines = qa.strip().splitlines()

            # Initialize variables to hold the question and answer
            question_lines = []
            answer_lines = []
            current_section = None

            # Parse each line to separate question and answer
            for line in lines:
                if line.startswith("Q:"):
                    current_section = "question"
                    question_lines.append(line[2:].strip())
                elif line.startswith("A:"):
                    current_section = "answer"
                    answer_lines.append(line[2:].strip())
                else:
                    # Append to the current section (either question or answer)
                    if current_section == "question":
                        question_lines.append(line.strip())
                    elif current_section == "answer":
                        answer_lines.append(line.strip())

            # Join the lines to form complete question and answer texts
            question = "\n".join(question_lines).strip()
            answer = "\n".join(answer_lines).strip()

            if question and answer:
                questions_answers.append((question, answer))

    return questions_answers

def _validate_attribute_value(attr_name, attr_value, expected_type):
    """
    Validate an attribute value against its expected type from node_library.

    Args:
        attr_name: Name of the attribute
        attr_value: Actual value from the XML
        expected_type: Expected type/value from node_library (e.g., "string", "bool", "int", "float", "{id}", "{dt}")

    Returns:
        (is_valid, error_message)
    """
    # Check for literal values like "{id}" or "{dt}"
    if expected_type.startswith("{") and expected_type.endswith("}"):
        if attr_value != expected_type:
            return False, f"Attribute '{attr_name}' must have literal value '{expected_type}', got '{attr_value}'"
        return True, None

    # Check for type-based validation
    if expected_type == "string":
        # Any non-empty string is valid
        if not attr_value:
            return False, f"Attribute '{attr_name}' cannot be empty (expected string)"
        return True, None

    elif expected_type == "bool":
        if attr_value.lower() not in ["true", "false"]:
            return False, f"Attribute '{attr_name}' must be 'true' or 'false', got '{attr_value}'"
        return True, None

    elif expected_type == "int":
        try:
            int(attr_value)
            return True, None
        except ValueError:
            return False, f"Attribute '{attr_name}' must be an integer, got '{attr_value}'"

    elif expected_type == "float":
        try:
            float(attr_value)
            return True, None
        except ValueError:
            return False, f"Attribute '{attr_name}' must be a float, got '{attr_value}'"

    # Unknown type
    return False, f"Unknown type '{expected_type}' for attribute '{attr_name}'"

def validate_bt(tree,node_library,debug=False):
    '''
    Rules:
    Tree Structure:
      - There must be a main_tree_to_execute attribute in the root node which has the value equal to the behavior tree node to be executed.
      - There must be exactly one BehaviorTree node in the XML.
      - The BehaviorTree node must have exactly one child node.
      - There must be a SubTree node with ID 'RegularNavTree' for regular navigation.
      - The file BTRegularNav.xml must be included in the XML.
    Nodes and Attributes:
      - All nodes used in the tree must be present in the provided node_library.
      - Each node must have the correct attributes as specified in the node_library.
      - Fallback control nodes must have at least 2 children.
      - Sequence control nodes must have at least 1 child.
      - SetBlackBoard nodes must be in pairs (at least 1) (one for setting agentid and one for setting dt) and only for regular navigation.
      - When regular navigation is used, the entire subtree node in the following form must be used:
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
    -> no two movement nodes must be direct siblings in a sequence parent node.
    -> A node and its inversion cannot be siblings. (always failure)
    '''

    # Check that main_tree_to_execute attribute exists
    if 'main_tree_to_execute' not in tree.attrib:
        error_string = "The root node must have a 'main_tree_to_execute' attribute"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False, error_string

    #Ensure the main tree to execute is in the tree
    if len(tree.findall(f".//*[@ID='{tree.attrib['main_tree_to_execute']}']"))!=1:
        error_string = f"Main tree to execute {tree.attrib['main_tree_to_execute']} is not present or is present more than once. There can be only 1 main_tree_to_execute from the root node and it must be present in the tree as well"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False,error_string

    if len(tree.findall('.//BehaviorTree'))!=1:
        error_string = "There can be only 1 BehaviorTree node. You've added None or multiple"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False,error_string

    # Verify that main_tree_to_execute references the BehaviorTree node
    behavior_tree_node = tree.find('.//BehaviorTree')
    if behavior_tree_node.get('ID') != tree.attrib['main_tree_to_execute']:
        error_string = f"main_tree_to_execute attribute must reference the BehaviorTree node's ID. Expected '{behavior_tree_node.get('ID')}', got '{tree.attrib['main_tree_to_execute']}'"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False, error_string

    if len(tree.find('BehaviorTree'))!=1:
        error_string = "Behavior Tree node can have only a single child"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False,error_string

    # Find and validate RegNav subtrees by looking for SubTree nodes with ID='RegularNavTree'
    # Then validate their parent Sequence structure
    regnav_sequences = []
    regular_nav_subtrees = tree.findall(".//SubTree[@ID='RegularNavTree']")

    # Must have at least one RegularNavTree subtree
    if len(regular_nav_subtrees) == 0:
        error_string = "You haven't added RegNav subtree. Remember to add the blackboards and include the required file (for agentid and dt)"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False, error_string

    # Find parent sequences for each RegularNavTree subtree
    for subtree_node in regular_nav_subtrees:
        # Find the parent of this SubTree node
        parent = None
        for elem in tree.iter():
            if subtree_node in list(elem):
                parent = elem
                break

        if parent is None:
            error_string = "Could not find parent for RegularNavTree SubTree node"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

        # Parent must be a Sequence node
        if parent.tag != 'Sequence':
            error_string = f"RegularNavTree SubTree must be inside a Sequence node, found it in '{parent.tag}'"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

        regnav_sequences.append(parent)

    # Validate each RegNav sequence structure
    for regnav_seq in regnav_sequences:
        # Must have exactly 3 children: 2 SetBlackboard + 1 SubTree
        if len(regnav_seq) != 3:
            error_string = """Sequence containing RegularNavTree must have exactly 3 children (2 SetBlackboard + 1 SubTree). It should be:
            <Sequence>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>"""
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

        children = list(regnav_seq)

        # First child must be SetBlackboard with output_key='agentid' and value='{id}'
        if children[0].tag != 'SetBlackboard':
            error_string = "First child of RegNav sequence must be SetBlackboard for agentid"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

        if children[0].attrib != {'output_key': 'agentid', 'value': '{id}'}:
            error_string = "First SetBlackboard must have output_key='agentid' and value='{id}'"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

        # Second child must be SetBlackboard with output_key='timestep' and value='{dt}'
        if children[1].tag != 'SetBlackboard':
            error_string = "Second child of RegNav sequence must be SetBlackboard for timestep"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

        if children[1].attrib != {'output_key': 'timestep', 'value': '{dt}'}:
            error_string = "Second SetBlackboard must have output_key='timestep' and value='{dt}'"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

        # Third child must be SubTree with specific attributes
        if children[2].tag != 'SubTree':
            error_string = "Third child of RegNav sequence must be SubTree"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

        if children[2].attrib != {'ID': 'RegularNavTree', 'id': 'agentid', 'dt': 'timestep'}:
            error_string = "SubTree must have ID='RegularNavTree', id='agentid', and dt='timestep'"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

    # Check that BTRegularNav.xml is included
    included_files = tree.findall('.//include')
    if len(included_files) != 1:
        error_string = "File for regularnav is not included. Add it after the root element: <include path='BTRegularNav.xml'/>"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False, error_string

    if included_files[0].attrib != {'path': 'BTRegularNav.xml'}:
        error_string = "Path of the included file for regularnav is wrong. The correct way is: <include path='BTRegularNav.xml'/>"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False, error_string

    # Collect all SetBlackboard and SubTree nodes that are inside RegNav sequences
    nodes_in_regnav = set()
    for regnav_seq in regnav_sequences:
        for child in regnav_seq:
            nodes_in_regnav.add(child)

    # Check that SetBlackboard and SubTree nodes only appear within RegNav sequences
    all_setblackboard = tree.findall('.//SetBlackboard')
    for node in all_setblackboard:
        if node not in nodes_in_regnav:
            error_string = "SetBlackboard nodes can only be used within RegNav sequences. Found SetBlackboard outside RegNav."
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

    all_subtree = tree.findall('.//SubTree')
    for node in all_subtree:
        if node not in nodes_in_regnav:
            error_string = "SubTree nodes can only be used within RegNav sequences. Found SubTree outside RegNav."
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

    error_string = None
    ignore_nodes = ['SetBlackboard','SubTree','include','root','BehaviorTree']
    for elem in tree.iter():
        if elem.tag not in node_library and elem.tag not in ignore_nodes:
            error_string = f'{elem.tag} not in node_library'
            break

        # Skip validation for SetBlackboard and SubTree as they're already validated above
        if elem.tag in ignore_nodes:
            continue

        #check if all required attributes are present
        if len(node_library[elem.tag])!=len(elem.attrib.keys()):
            error_string = f"Node '{elem.tag}' has incorrect number of attributes. Expected {list(node_library[elem.tag].keys())}, got {list(elem.attrib.keys())}"
            break

        for attr_name in node_library[elem.tag]:
            if attr_name not in elem.attrib:
                error_string = f"Node '{elem.tag}' is missing required attribute '{attr_name}'"
                break

        if error_string:
            break

        #check for incorrect attribute names and validate attribute values
        for k,v in elem.attrib.items():
            if k not in node_library[elem.tag]:
                error_string = f"Attribute '{k}' not valid for node '{elem.tag}'. Valid attributes are: {list(node_library[elem.tag].keys())}"
                break

            # Validate attribute value against expected type
            expected_type = node_library[elem.tag][k]
            is_valid, validation_error = _validate_attribute_value(k, v, expected_type)
            if not is_valid:
                error_string = f"Node '{elem.tag}': {validation_error}"
                break

        if error_string:
            break

        #Sequence control node must have at least 1 child
        if elem.tag == 'Sequence' and len(elem)<1:
            error_string = "That's an incorrect BT! A Sequence control node must have at least 1 child"
            break

        #Fallback control node must have at least 2 children
        if elem.tag == 'Fallback' and len(elem)<2:
            error_string = "That's an incorrect BT! A Fallback control node must have at least 2 children"
            break

    # Check that no two movement nodes are direct siblings in a sequence parent node
    # Define movement action nodes based on the node library
    movement_nodes = {'LookAtRobot', 'FollowRobot', 'FollowHuman', 'RunAwayFromRobot',
                      'GiveWaytoRobot', 'BlockRobot'}

    # Check all Sequence nodes for any movement node siblings (not just consecutive)
    for sequence in tree.iter('Sequence'):
        children = list(sequence)

        # Collect all movement children in this sequence
        movement_children = []
        for child in children:
            is_movement = (
                child.tag in movement_nodes or
                (child.tag == 'Sequence' and child in regnav_sequences)
            )
            if is_movement:
                movement_children.append(child)

        # If there are 2 or more movement nodes as siblings, return error
        if len(movement_children) >= 2:
            error_string = f"Multiple movement nodes ({', '.join([f'{child.tag}' for child in movement_children])}) cannot be siblings in the same Sequence node. Movement actions perform conflicting actions and must be separated by conditions or wrapped in a Fallback node to choose between them."
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False, error_string

    # Check that a node and its inversion cannot be siblings
    # This would always result in failure as one would always return the opposite of the other
    for elem in tree.iter():
        children = list(elem)
        if len(children) < 2:
            continue

        # Get all Inverter nodes among the children
        inverter_children = [child for child in children if child.tag == 'Inverter']

        # For each Inverter, check if its inverted node exists as a sibling
        for inverter in inverter_children:
            # Inverter should have exactly one child
            if len(inverter) != 1:
                continue

            inverted_node = list(inverter)[0]

            # Check if any sibling matches the inverted node
            for sibling in children:
                if sibling == inverter:
                    continue

                # Check if sibling has the same tag and attributes as the inverted node
                if sibling.tag == inverted_node.tag:
                    error_string = f"A node '{sibling.tag}' and its inversion cannot be siblings under the same parent '{elem.tag}'. This would always result in failure as they produce opposite results."
                    if debug:
                        eprint("Retrying Behavior Generation..")
                        eprint(error_string)
                    return False, error_string

    if error_string:
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False,error_string

    return True, None

def get_img_from_path(img_path):
    with open(img_path, "rb") as image_file:
        encoded_string = base64.b64encode(image_file.read())
    return encoded_string.decode("utf-8")

def filter_scene_graph(scene_graph, node_type_to_remove):
    #for simple serialized dict graphs
    G = {'nodes':[],'links':[]}
    filtered_nodes = []
    # Add nodes to the graph if they are not of the type to remove
    for node in scene_graph['nodes']:
        if node['type'] != node_type_to_remove:
            G['nodes'].append(node)
            filtered_nodes.append(str(node['id']))
    
    # Add edges to the graph, only if both nodes in an edge are still in the graph
    for edge in scene_graph['links']:
        node1, node2 = edge.split('<->')
        if node1 in filtered_nodes and node2 in filtered_nodes:
           G['links'].append(edge)
    
    # Create the new scene graph structure to return
    return G

def load_imgs_for_prompt(img_path):
    dims = PILImage.open(img_path).size
    img_cost = calculate_image_token_cost(dims)
    print(f'{dims}:{img_cost}')
    with open(img_path, "rb") as image_file:
        encoded_img = base64.b64encode(image_file.read()).decode('utf-8')
    return encoded_img,img_cost
        
#https://github.com/openai/tiktoken/issues/250
def calculate_image_token_cost(dims, detail="auto"):
    # Constants
    LOW_DETAIL_COST = 85
    HIGH_DETAIL_COST_PER_TILE = 170
    ADDITIONAL_COST = 85

    if detail == "auto":
        # assume high detail for now
        detail = "high"

    if detail == "low":
        # Low detail images have a fixed cost
        return LOW_DETAIL_COST
    
    elif detail == "high":
        # Calculate token cost for high detail images
        width, height = dims
        # Check if resizing is needed to fit within a 2048 x 2048 square
        if max(width, height) > 2048:
            # Resize the image to fit within a 2048 x 2048 square
            ratio = 2048 / max(width, height)
            width = int(width * ratio)
            height = int(height * ratio)
        # Further scale down to 768px on the shortest side
        if min(width, height) > 768:
            ratio = 768 / min(width, height)
            width = int(width * ratio)
            height = int(height * ratio)
        # Calculate the number of 512px squares
        num_squares = math.ceil(width / 512) * math.ceil(height / 512)
        # Calculate the total token cost
        total_cost = num_squares * HIGH_DETAIL_COST_PER_TILE + ADDITIONAL_COST
        return total_cost
    else:
        # Invalid detail_option
        raise ValueError("Invalid value for detail parameter. Use 'low' or 'high'.")

def pretty_print_conversation(messages):
    role_to_color = {
        "system": "red",
        "user": "green",
        "assistant": "blue",
        "function": "magenta",
    }
    
    for message in messages:
        if message["role"] == "system":
            print(colored(f"system: {message['content']}\n", role_to_color[message["role"]]))
        elif message["role"] == "user":
            print(colored(f"user: {message['content']}\n", role_to_color[message["role"]]))
        elif message["role"] == "assistant" and message.get("function_call"):
            print(colored(f"assistant: {message['function_call']}\n", role_to_color[message["role"]]))
        elif message["role"] == "assistant" and not message.get("function_call"):
            print(colored(f"assistant: {message['content']}\n", role_to_color[message["role"]]))
        elif message["role"] == "function":
            print(colored(f"function ({message['name']}): {message['content']}\n", role_to_color[message["role"]]))

def get_image_bytes_from_url(image_url: str) -> bytes:
    im = Image.open(image_url)
    buf = io.BytesIO()
    im.save(buf, format="JPEG")
    return buf.getvalue()

def load_image_from_url(image_url: str) -> Image:
    image_bytes = get_image_bytes_from_url(image_url)
    return Image.from_bytes(image_bytes)    
    

def num_tokens_from_messages(message: Mapping[str, object], model: str) -> int:
    """
    Calculate the number of tokens required to encode a message.
    Args:
        message (Mapping): The message to encode, in a dictionary-like object.
        model (str): The name of the model to use for encoding.
    Returns:
        int: The total number of tokens required to encode the message.
    Example:
        message = {'role': 'user', 'content': 'Hello, how are you?'}
        model = 'gpt-3.5-turbo'
        num_tokens_from_messages(message, model)
        output: 11
    """

    encoding = tiktoken.encoding_for_model(model)
    num_tokens = 2  # For "role" and "content" keys
    for value in message.values():
        if isinstance(value, list):
            for item in value:
                num_tokens += len(encoding.encode(item["type"]))
                if item["type"] == "text":
                    print("detected text")
                    num_tokens += len(encoding.encode(item["text"]))
                elif item["type"] == "image_url":
                    print(f"""detected {item["image_url"]["detail"]} detail image""")
                    num_tokens += calculate_image_token_cost(item["image_url"]["url"], item["image_url"]["detail"])

        elif isinstance(value, str):
            num_tokens += len(encoding.encode(value))
        else:
            raise ValueError(f"Could not encode unsupported message value type: {type(value)}")
    return num_tokens

def get_image_dims(image):
    if re.match(r"data:image\/\w+;base64", image):
        image = re.sub(r"data:image\/\w+;base64,", "", image)
        image = Image.open(BytesIO(base64.b64decode(image)))
        return image.size
    else:
        raise ValueError("Image must be a base64 string.")
    
def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def createHuman(agents_file,bt_file,params):
    #Edit Agents.yaml
    yaml_agent = f"""
    agent{params['id']}:
      id: {params['id']}
      skin: 0
      behavior: {params['behavior']}
      group_id: {params['group_id']}
      max_vel: 1.5
      radius: 0.4
      init_pose:
        x: {params['x']}
        y: {params['y']}
        z: 1.250000
        h: {params['h']}
      goal_radius: 0.3
      cyclic_goals: {params['cyclic_goals']}
      goals:
    """
    #add goals
    for i,goal in enumerate(params['goals']):
        yaml_agent += f"""
                    - g{i}"""
    
    for i,goal in enumerate(params['goals']):
        yaml_agent+=f"""
                g{i}:
                    x:{goal['x']}
                    y:{goal['y']}
                    h:{goal['h']}"""
    
    with open(agents_file,"a") as f:
        f.write(yaml_agent)
    #Edit BT file
    with open(bt_file,'w') as f:
        f.write(params['bt'])


