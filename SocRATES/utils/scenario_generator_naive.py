from utils.query_handler_naive import QueryHandler
import os
from utils import utils
from utils.utils import *
import json,yaml,copy
from utils.scene_graph import SceneGraph
import xml.etree.ElementTree as ET
import pprint
import tenacity
import numpy as np
from collections import defaultdict
import shutil
class ScenarioGenerator:
    '''
        Helper class for generating scenarios.
    '''
    def __init__(self,config):
        self.qh = QueryHandler(config)
        self.config = config
        self.debug = config['debug']
        with open(self.config['location']['scene_graph_file'],'r') as f:
            self.scene_graph_json = json.load(f)
        self.counter = config['counter']    
        self.scgraph = SceneGraph(self.scene_graph_json)
        self.encoded_map_img = utils.encode_image(self.config['location']['annotated_map_img_file'])
        self.node_library = self.config['node_library']
        
        self.node_types = []
        self.edge_types = []
        for node in self.scgraph.get_parent_nodes():
            if self.scgraph.graph.nodes[node]['type'] not in self.node_types:
                self.node_types.append(self.scgraph.graph.nodes[node]['type']) 
        for edge in self.scgraph.graph.edges:
            if self.scgraph.graph.edges[edge]['type'] not in self.edge_types:
                self.edge_types.append(self.scgraph.graph.edges[edge]['type'])     
        
        with open(self.config['location']['map_params_file'],'r') as f:
            self.map_params = yaml.safe_load(f)
    
    def generate_scenario(self):
        '''
            Generates a scenario with hunavsim components using the query handler
        '''
        try:
            for attempt in tenacity.Retrying(retry_error_callback=lambda x: eprint(x)):
                #for _ in range(4):
                with attempt:
                    ######################## SCENARIO GENERATION #####################
                    #GENERATE SCENARIO
                    while True:
                        scenario = self.qh.query_scenario(
                                    context = self.config['context'],
                                    task = self.config['task'],
                                    rough_scenario = self.config['rough_scenario'],
                                    location_description = self.config['location']['description'],
                                    encoded_img = self.encoded_map_img ,   
                                    scene_graph = json.dumps(self.scene_graph_json),
                                    node_types = self.node_types,
                                    edge_types = self.edge_types          
                                    ) 
                        if scenario == None:
                            raise Exception           
                        
                        behaviors = {}
                        for i,bt in enumerate(scenario.behavior):
                            #check XML
                            try:
                                test_xml = ET.fromstring(bt.tree)  
                            except Exception as e:
                                print(e)
                            #check BT
                            bt_valid, errors = utils.validate_bt(test_xml,self.node_library,False)
                            behaviors[f'human {i}'] = scenario.behavior[i].tree
                        
                        
                        trq_response_structured = scenario.trajectories
                        all_trajectories_valid = True 
                        trajectories =  trq_response_structured.trajectories
                        robot_traj = trajectories.robot
                        human_traj = trajectories.humans
                        all_human_traj = [h.trajectory for h in human_traj]
                        #test trajectory correctness
                        scgraph = SceneGraph(self.scene_graph_json)
                        valid_trajectories = False
                        for v in [robot_traj] + all_human_traj:
                            traj_valid,errors = scgraph.isvalidtrajectory(v)
                            if not traj_valid: #requery LLM with error message
                                all_trajectories_valid = False

                        if all_trajectories_valid:
                            rhmeet = True
                            for human in human_traj:
                                if not((human.interaction_point in human.trajectory) and (human.interaction_point in robot_traj)):
                                    rhmeet = False
                                    break
                            
                            if rhmeet:
                                valid_trajectories = True
                        
                              
                        #reasoning = scenario['reasoning']    
                        file_path = os.path.join(self.config['paths']['save_dir'],self.config['experiment_name']+'_'+'response_traj.json')
                        try:
                            with open(file_path,'r') as f:
                                file_load = json.load(f)
                        except:
                            with open(file_path,'w') as f:
                                json.dump({},f)
                                file_load = {}
                                    
                        file_load[self.counter] = {
                            'counter': self.counter,
                            'scenario':scenario.scenario.json(),

                            'trajectories':scenario.trajectories.trajectories.json(),
                            'behavior_trees':json.dumps(behaviors),
                            'bt_valid': bt_valid,
                            'traj_valid':valid_trajectories
                            }
                        with open(file_path,'w') as f:
                            json.dump(file_load,f, indent=2)
                        break
        except tenacity.RetryError as error:
            eprint(f"Unable to generate scenario, please rerun script: {error}")
            exit()
            
        return
