from utils.query_handler import QueryHandler
import os
import numpy as np
from utils import utils
from utils.utils import *
import json,yaml,copy
from utils.scene_graph import SceneGraph
import pprint
import tenacity
import numpy as np
from collections import defaultdict
import shutil,time
class ScenarioGenerator:
    '''
        Helper class for generating scenarios.
    '''
    def __init__(self,config):
        self.qh = QueryHandler(config)
        self.config = config
        # print(os.path.join(self.config['paths']['save_dir'],self.config['experiment_name']+'_'+'response_traj.json'))
        self.debug = config['debug']
        with open(self.config['location']['scene_graph_file'],'r') as f:
            self.scene_graph_json = json.load(f)
            
        self.scgraph = SceneGraph(self.scene_graph_json)
        self.encoded_map_img = utils.encode_image(self.config['location']['annotated_map_img_file'])
        self.node_library = self.config['node_library']
        self.num_humans = 0
        self.node_types = []
        self.edge_types = []
        self.retry_count = config['retry_count']
        self.user_feedback = config['user_feedback']
        assert self.user_feedback in [False, 'retry_local','retry_global']
        for node in self.scgraph.get_parent_nodes():
            if self.scgraph.graph.nodes[node]['type'] not in self.node_types:
                self.node_types.append(self.scgraph.graph.nodes[node]['type']) 
        for edge in self.scgraph.graph.edges:
            if self.scgraph.graph.edges[edge]['type'] not in self.edge_types:
                self.edge_types.append(self.scgraph.graph.edges[edge]['type'])     
        
        with open(self.config['location']['map_params_file'],'r') as f:
            self.map_params = yaml.safe_load(f)
        self.time_taken = {
                'scenario_generation':0,
                'trajectory_generation':0,
                'bt_generation':0,
                'inputs':0,
                'total':0
            }
        self.edits = {
            'trajectory':[],
            'bt':defaultdict(lambda:[])
        }
        
    def get_scenario_desc(self):
        if self.config['use_handcrafted_scenario']:
            return self.config['handcrafted_scenario']['scenario']
        else:
            return self.qh.query_scenario(
                context = self.config['context'],
                node_types = ', '.join(self.node_types),
                edge_types = ', '.join(self.edge_types)
            )
  
    def get_trajectories(self,scenario_desc,num_humans,edits=None,prev_output=None):
        parsed_description = ''
        parsed_description += f"Robot: {scenario_desc['robot']}\n"
        num_humans = 0
        for human in scenario_desc['humans']:
            human_name = human['name']
            behavior = human['behavior']
            parsed_description += f"{human_name}: {behavior}\n"
            num_humans += 1
        
        parsed_description += f"Groups: {scenario_desc['groups']}\n"
        
        if self.config['use_handcrafted_scenario']:
            return self.config['handcrafted_scenario']['groupids'],self.config['handcrafted_scenario']['trajectories']
        
        else:
            if edits!=None:
                return self.qh.edit_traj(
                scene_graph_json=self.scene_graph_json,
                node_types=self.node_types,
                edge_types=self.edge_types,
                encoded_img=self.encoded_map_img,
                scenario_description=parsed_description,
                num_humans = num_humans,
                edits = edits,
                prev_output = prev_output
                )
            else:
                return self.qh.query_traj(
                scene_graph_json=self.scene_graph_json,
                node_types=self.node_types,
                edge_types=self.edge_types,
                encoded_img=self.encoded_map_img,
                scenario_description=parsed_description,
                num_humans = num_humans,
                )

    def generate_scenario(self):
        '''
            Generates a scenario with hunavsim components using the query handler
        '''
        try:
            total_time_start = time.time()
            edit_time_scenario = 0
            edit_time_traj = 0
            edit_time_bt = 0
            #overall generation retry loop
            for attempt in tenacity.Retrying(retry_error_callback=lambda x: eprint(x)):
                #for _ in range(4):
                with attempt:
                    ######################## SCENARIO GENERATION #####################
                    scenario_gen_time_start = time.time()
                    if self.config['load_scenario_response']:
                        #LOAD SCENARIO
                        lprint('loading saved scenario proposal')
                        with open(self.config['scenario_file'],'r') as f:
                            scenario_parsed = json.load(f)
                            if isinstance(scenario_parsed, str):
                                scenario_parsed = eval(scenario_parsed)
                    else:
                        #GENERATE SCENARIO
                        while True: #allow retrying due to user feedback
                            scenario = self.get_scenario_desc()
                            if scenario == None:
                                raise Exception
                            
                            rprint("SCENARIO DESCRIPTION GENERATED BY LLM:")
                            rprint("Scenario:")
                            print(f"Robot:{scenario.robot}")
                            for h in scenario.humans:
                                print(f"{h.name}: \n \t behavior: {h.behavior}\n\t trajectory: {h.trajectory}\n")
                            print(f"Groups: {scenario.groups}")
                            '''
                            # rprint("Behavior Description:")
                            # for human in scenario['humanbehavior']:
                            #     print(f"{b.name}: {b.behavior}")
                            #     behaviors_parsed[b.name.lower().replace(' ','')] = b.behavior
                            
                            if self.debug:
                                rprint("Reasoning:")
                                lprint(f"{scenario.reasoning.scenario_importance}")
                                rprint("Simulating humans:")
                                lprint(scenario.reasoning.simulating_humans)
                            '''
                            continue_choice = True
                            if self.user_feedback == False:
                                edit_time_scenario_start = time.time()
                                while True:
                                    iprint("CONTINUE WITH THIS SCENARIO(Y) OR RETRY(N)?:")
                                    continue_choice = input().lower().strip()
                                    if continue_choice!='y' and continue_choice!='n':
                                        eprint("invalid input")
                                        continue
                                    else:
                                        if continue_choice=='y':
                                            continue_choice = True
                                        else:
                                            continue_choice = False                           
                                        break
                                edit_time_scenario += time.time() - edit_time_scenario_start
                                
                            if continue_choice:
                                scenario_parsed = scenario.json()
                                #reasoning = scenario['reasoning']    
                                with open(self.config['scenario_file'],'w') as f:
                                    json.dump(scenario_parsed,f)
                                scenario_parsed = eval(scenario_parsed)
                                break
                    scenario_gen_time_end = time.time()
                    scenario_gen_time = scenario_gen_time_end - scenario_gen_time_start - edit_time_scenario
                    self.time_taken['scenario_generation'] += scenario_gen_time       
                    
                    trajectoryQ_inputs = {
                            'robot':scenario_parsed['robot'],
                            'groups':scenario_parsed['groups'],
                            'humans':[]
                        }
                    for human in scenario_parsed['humans']:
                        trajectoryQ_inputs['humans'].append({human['name']: human['trajectory']})
    
                    behaviorQ_input = {b['name']: b['behavior'] for b in scenario_parsed['humans']}
                    
                    ######################### TRAJECTORY GENERATION #####################
                    local_retry_count = 0
                    trajectory_gen_time_start = time.time()
                    trajectories_parsed = None
                    if self.config['load_trajectory_response']:
                        #LOAD TRAJECTORY
                        lprint('loading saved trajectory')
                        with open(self.config['trajectory_file'],'r') as f:
                            traj = json.load(f)   
                            if isinstance(traj, str):
                                traj = eval(traj) 
                        groupids = {}
                        interaction_points_parsed = {}
                        trajectories_parsed = {
                            'robot': traj['robot']
                        }
                        for human in traj['humans']:
                            groupids[human['name']] = human['groupid']
                            trajectories_parsed[human['name']] = human['trajectory']
                            interaction_points_parsed[human['name']] = human['interaction_point']
                    else:
                        edits = None
                        while local_retry_count<self.retry_count:
                            #GENERATE TRAJECTORY
                            if edits == None:
                                trajectories = self.get_trajectories(scenario_parsed,self.num_humans)
                            else:
                                trajectories = self.get_trajectories(scenario_parsed,self.num_humans,edits=edits,prev_output=trajectories)
                                                            
                            if trajectories == None:
                                eprint("Invalid Trajectories generated:")
                                edits=None
                                retry_choice=None
                                while True: # allow retrying due to user feedback
                                    
                                    if self.user_feedback == 'retry_local':
                                        print('Retrying Trajectory Generation')
                                        retry_choice = '1'
                                        local_retry_count+=1
                                    
                                    elif self.user_feedback == 'retry_global':
                                        print('Retrying scenario generation')
                                        retry_choice = '2'

                                    else:
                                        edit_time_traj_start = time.time()
                                        iprint("Retry Trajectory Regeneration (1) or Regenerate Scenario(2)?:")
                                        retry_choice = input().lower().strip()
                                        edit_time_traj += time.time() - edit_time_traj_start
                                    
                                    if retry_choice!='1' and retry_choice!='2':
                                        eprint("invalid input, please press (1) or (2)")
                                        continue
                                    else:
                                        break
                                if retry_choice=='2':
                                    raise Exception
                                else:
                                    continue
                            rprint("GENERATED TRAJECTORIES:")
                            rprint(f"Robot Trajectory: ")
                            print(trajectories.robot)
                            rprint(f"Human trajectories: ")
                            for t in trajectories.humans:
                                print(f"{t.name}(groupid: {t.groupid}): {t.trajectory}, interaction_point:{t.interaction_point}")
                            
                            continue_choice = True
                            edits = None
                            if self.user_feedback == False:
                                while True:
                                    edit_time_traj_start = time.time()
                                    iprint("CONTINUE WITH PROPOSED TRAJECTORIES(Y) or RETRY(N) or SUGGEST EDITS(E)?:")
                                    continue_choice = input().lower().strip()
                                    edit_time_traj += time.time() - edit_time_traj_start
                                    if continue_choice!='y' and continue_choice!='n' and continue_choice!='e':
                                        eprint("invalid input")
                                        continue
                                    else:
                                        if continue_choice=='y':
                                            continue_choice = True
                                            edits = None
                                        elif continue_choice == 'n':
                                            continue_choice = False                           
                                            edits = None
                                        else:
                                            #edit the response and send back to LLM
                                            iprint("HOW WOULD YOU LIKE TO CHANGE THE TRAJECTORIES? (Enter text):")
                                            edit_time_traj_start = time.time()
                                            edits = input().lower().strip()
                                            edit_time_traj += time.time() - edit_time_traj_start
                                            self.edits['trajectory'].append(edits)
                                        break                   
                                                         
                            if continue_choice and edits==None:
                                trajectories_parsed = {}
                                interaction_points_parsed = {}
                                groupids = {}
                                for traj in trajectories.humans:
                                    trajectories_parsed[traj.name.lower().replace(' ','')] = traj.trajectory
                                    groupids[traj.name.lower().replace(' ','')] = traj.groupid 
                                    interaction_points_parsed[traj.name.lower().replace(' ','')] = traj.interaction_point
                                
                                trajectories_parsed['robot'] = trajectories.robot
                                with open(self.config['trajectory_file'],'w') as f:
                                    json.dump(trajectories.json(), f, indent=2)
                                break
                        if trajectories_parsed == None:
                            return None,None,None,None,None,{'failure_reason':"Failed to generate trajectories"}
                    trajectory_gen_time_end = time.time()
                    trajectory_gen_time = trajectory_gen_time_end - trajectory_gen_time_start - edit_time_traj
                    self.time_taken['trajectory_generation'] += trajectory_gen_time
                    
                    ######################## BT GENERATION #####################
                    local_retry_count = 0
                    bt_gen_time_start = time.time()
                    if self.config['load_bt_response']:
                        #LOAD BT
                        lprint('loading saved behaviors')
                        with open(self.config['bt_file'],'r') as f:
                            behavior_trees_parsed = json.load(f)
                            if isinstance(behavior_trees_parsed, str):
                                behavior_trees_parsed = eval(behavior_trees_parsed)
                    else:
                        while True:
                            #GENERATE BT
                            lprint("Generating Behavior Trees")
                            behavior_trees_parsed = {}
                            for name, behav in behaviorQ_input.items():
                                edits = None
                                behavior_response=None
                                success_bt_generation = False
                                while local_retry_count<self.retry_count:
                                    lprint(f"Generating Tree for {name}...")
                                    behavior_response =  self.qh.query_bt(
                                        behavior_description=behav,
                                        node_library=self.node_library,
                                        edits = edits,
                                        prev_bt=behavior_response
                                    )
                                    if behavior_response == None:
                                        # if self.debug:
                                        retry_choice=None
                                        while True:
                                            
                                            if self.user_feedback == 'retry_local':
                                                print('Retrying Behavior generation')
                                                retry_choice = '1'
                                                local_retry_count+=1
                                            
                                            elif self.user_feedback == 'retry_global':
                                                print('Retrying scenario generation')
                                                retry_choice = '2'
                                            
                                            else:
                                                edit_time_bt_start = time.time()
                                                iprint("Retry Behavior generation (1) or Regenerate Scenario(2)?:")
                                                retry_choice = input().lower().strip()
                                                edit_time_bt += time.time() - edit_time_bt_start

                                            if retry_choice!='1' and retry_choice!='2':
                                                eprint("invalid input, please press (1) or (2)")
                                                continue
                                            else:
                                                break
                                        if retry_choice=='2':
                                            raise Exception
                                        else:
                                            continue
                                    behavior_trees_parsed[name] = behavior_response.tree
                                    rprint(f"Generated Behavior Tree for {name}")
                                    
                                    continue_choice = True
                                    edits = None
                                    if self.user_feedback == False:
                                        while True:
                                            iprint("CONTINUE WITH THIS BEHAVIOR(Y) OR RETRY(N) or SUGGEST EDITS(E)?:")
                                            edit_time_bt_start = time.time()
                                            continue_choice = input().lower().strip()
                                            edit_time_bt += time.time() - edit_time_bt_start
                                            if continue_choice!='y' and continue_choice!='n' and continue_choice!='e':
                                                eprint("invalid input")
                                                continue
                                            else:
                                                if continue_choice=='y':
                                                    continue_choice = True
                                                    edits = None
                                                elif continue_choice == 'n':
                                                    continue_choice = False
                                                    edits = None
                                                else:
                                                    #edit the response and send back to LLM
                                                    iprint("HOW WOULD YOU LIKE TO CHANGE THE BEHAVIOR? (Enter text):")
                                                    edit_time_bt_start = time.time()
                                                    edits = input().lower().strip()
                                                    edit_time_bt += time.time() - edit_time_bt_start
                                                    self.edits['bt'][name].append(edits)
                                                    
                                                break
                                    
                                    if continue_choice and edits == None:
                                        success_bt_generation = True
                                        break                            
                                if not success_bt_generation: # exceeded retry count
                                    return None,None,None,None,None,{'failure_reason':"Failed to generate behavior tree for "+name}
                            with open(self.config['bt_file'],'w') as f:
                                json.dump(behavior_trees_parsed,f)
                            break
                    bt_gen_time_end = time.time()
                    bt_gen_time = bt_gen_time_end - bt_gen_time_start - edit_time_bt
                    self.time_taken['bt_generation'] += bt_gen_time
                    
                    #save generated scenario
                    #if not self.config['load_scenario_response'] and not self.config['load_trajectory_response'] and not self.config['load_bt_response']:
                    with open(os.path.join(self.config['paths']['save_dir'],self.config['experiment_name']+'_'+'response_traj.json'),'w') as f:
                        json.dump({
                            'scenario':scenario_parsed,
                            'groupids':groupids,
                            'trajectories':trajectories_parsed,
                            'interaction_points':interaction_points_parsed,
                            'behavior_trees':behavior_trees_parsed
                            },f)
            self.time_taken['total'] += time.time()-total_time_start - edit_time_scenario - edit_time_traj - edit_time_bt
        except tenacity.RetryError as error:
            eprint(f"Unable to generate scenario, please rerun script: {error}")
            return None,None,None,None,None,None
        
        
        outputs = {
            'token_usage':self.qh.token_usage,
            'num_queries':self.qh.num_queries,
            'time_taken': self.time_taken,
            'edits': self.edits
        }

                
        return scenario_parsed, groupids, trajectories_parsed, interaction_points_parsed, behavior_trees_parsed,outputs
    
    def instantiate_simulator(self,file_paths,groupids,trajectories,behaviors_trees,interaction_points):
        '''
        Instantiates hunavsim with trajectories and behaviors   
    
        '''
        # write trajectories
        agents_yaml = {'hunav_loader': {'ros__parameters': {'map': self.config['location']['name'],
                        'publish_people': True,
                        'agents': []}}}
        blank_human = {'id': None,
            'skin': 0,
            'behavior': 0,
            'group_id': -1,
            'max_vel': 1.0,
            'radius': 0.4,
            'init_pose': {'x': None, 'y': None, 'z': 1.25, 'h': 0.0},
            'goal_radius': 0.5,
            'cyclic_goals': False,
            'goals': [],
            }
        spawn_radius = 1.5
        agents = {} 
        # transform from scene graph to world trajectories
        world_trajectories = {}        
        for k,v in trajectories.items():
            world_trajectories[k] = []
            for l in v:
                world_trajectories[k].append(self.pix2world(self.scgraph.graph.nodes[l]['pos']))
        
        invert_group_ids = defaultdict(lambda:[])
        for k,v in groupids.items():
            invert_group_ids[v].append(k)
        angles = defaultdict(lambda: np.random.uniform(0,np.pi*2.0))
        for k,v in invert_group_ids.items():
            if k!=-1:
                for i,human in enumerate(v):
                    angles[human] = i*np.pi*2.0/len(v)
        #gather all peds trajectories
        i = 0
        skin = np.random.choice([0,1,2],len(trajectories))
        for name,trajectory in trajectories.items():
            if name!='robot':
                agents_yaml['hunav_loader']['ros__parameters']['agents'].append(name)
                agents[name] = copy.deepcopy(blank_human)
                agents[name]['skin'] = int(skin[i])
                agents[name]['id'] = i
                agents[name]['behavior'] = 7+i
                agents[name]['group_id'] = groupids[name]
                for j,g in enumerate(world_trajectories[name]):
                    if j == 0: #starting position of human
                        r = np.random.uniform(0.0,spawn_radius)
                        agents[name]['init_pose'] = {
                            'x':float(g[0] + r*np.cos(angles[name])),
                            'y':float(g[1] + r*np.sin(angles[name])),
                            'z':1.25,
                            'h': float(angles[name]+np.pi)#float(theta),
                        }
                        if len(world_trajectories[name]) <= 1:
                            agents[name]['goals'].append(f'g{1}')
                            agents[name][f'g{1}'] = {
                                'x':g[0],
                                'y':g[1],
                                'h':1.25
                            }
                    else:
                        agents[name]['goals'].append(f'g{j}')
                        agents[name][f'g{j}'] = {
                            'x':g[0],
                            'y':g[1],
                            'h':1.25
                        }
                i+=1
                
                if i>=len(behaviors_trees.keys()):
                    break
        agents_yaml['hunav_loader']['ros__parameters'].update(agents)
        
        #write trajectories in hunav_sim        
        with open(os.path.join(file_paths['hunav_sim_dir'],file_paths['agents_file']),'w') as f:
            yaml.dump(agents_yaml,f)

        #write waypoints for the robot
        robot_poses = {
            'initial_pose':{},
            'waypoints':[]
        }
        spawn_radius_robot = 0.5
        for j,g in enumerate(world_trajectories['robot']):
                # add some noise to robot start position and waypoints
                r = np.random.uniform(0.0,spawn_radius_robot)
                theta = np.random.uniform(0,2*np.pi)
                g[0] += float(r*np.cos(theta))
                g[1] += float(r*np.sin(theta))
                if j == 0:
                    robot_poses['initial_pose'] = {
                        'x':g[0],
                        'y':g[1],
                        'yaw':0.0,
                    }
                else:
                    robot_poses['waypoints'].append({
                        'position':{
                        'x':g[0],
                        'y':g[1],
                        'yaw':0.0
                        }
                    })
        
        with open(os.path.join(file_paths['hunav_gazebo_wrapper_dir'],'config','robot_poses.yaml'),'w') as f:
            yaml.dump(robot_poses,f)
        
        with open(os.path.join(file_paths['hunav_gazebo_wrapper_dir'],'config','trajectory.json'),'w') as f:
            json.dump(
                {
                    "groupids":groupids,
                    "trajectories":trajectories,
                    "interaction_points":interaction_points
                },
                f
            )
        
        with open(os.path.join(file_paths['hunav_gazebo_wrapper_dir'],'config','launch_params.yaml'),'w') as f:
            yaml.dump({
                'robot_poses':os.path.join(file_paths['hunav_gazebo_wrapper_dir'],'config','robot_poses.yaml'),
                'scenario':os.path.join(file_paths['hunav_gazebo_wrapper_dir'],'config','trajectory.json'),
                'world_name':self.config['location']['name']
                },f)
        
        #write behavior files for hunavsim
        i = 0
        for human, behav in behaviors_trees.items():
            with open(os.path.join(file_paths['hunav_sim_dir'],file_paths['bt_dir'],f'LLMBT_{i}.xml'),'w') as f:
                f.write(behav)
            i+=1
        print("Wrote trajectories and behaviors to simulation files")

    def pix2world(self,px):
        return [
            px[0]*self.map_params['resolution'] + self.map_params['origin'][0],
            -1.0*(px[1]*self.map_params['resolution'] + self.map_params['origin'][1])
        ]