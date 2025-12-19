#!/usr/bin/env python3
import rclpy,os
import rclpy.logging
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Int32
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Pose
from nav2_simple_commander.robot_navigator import TaskResult
from std_msgs.msg import String
from hunav_msgs.msg import Agent,Agents
from hunav_msgs.msg import PauseNavs,PauseNav
from rclpy.qos import QoSProfile
from collections import defaultdict
import json,yaml
import numpy as np
import signal
import sys
from tabulate import tabulate
def is_near(pose1,pose2,thresh = 1.0):
    if isinstance(pose1,Pose):
        pose_1 = {'x':pose1.position.x,'y':pose1.position.y}
    else:
        pose_1 = pose1
    if isinstance(pose2,Pose):
        pose_2 = {'x':pose2.position.x,'y':pose2.position.y}
    else:
        pose_2 = pose2
        
    dist = np.sqrt((pose_1['x']-pose_2['x'])**2+(pose_1['y']-pose_2['y'])**2)
    #print(f"Distance:{dist}")
    if dist<thresh:
        return True
    return False

class ScenarioManager(Node):
    '''
        This node tracks the robot position, and pauses and unpauses human 
        navigation to make it more likely for a scenario to happen. 
        Default behavior:
            Pause the human navigation one-goal before the node where the human and 
            the robot are supposed to meet, until the robot arrives at the node preceding the interaction node
    '''
    def __init__(self,scenario_file,robot_poses_file):
        super().__init__('timing_manager')
        self.publisher_ = self.create_publisher(PauseNavs, 'pause_nav', 10)
        qos = QoSProfile(depth=10)
        self.robot_sub = self.create_subscription(Agent,'/robot_states',self.robotStateCallback,qos)
        self.task_status_sub = self.create_subscription(Int32,'/task_status',self.taskStatusCallback,qos)
        self.navigation_status_sub = self.create_subscription(Int32,'/navigation_status',self.navigationStatusCallback,qos)
        self.human_sub = self.create_subscription(Agents,'/human_states',self.humanStateCallback,qos)
        self.human_behavior_pub = self.create_publisher(String,'/human_behavior_state',10)
        self.scenario_finished_pub = self.create_publisher(Int32,'/scenario_finished',10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.met_robot = {}
        self.robot_current_goal_index = 0 #which goal is the robot currently heading towards?
        self.robot_position = None
        self.record_agents = defaultdict(lambda:[])
        self.declare_parameter('save_file','temp.json')
        self.save_file = self.get_parameter('save_file').get_parameter_value().string_value
        with open(scenario_file,'r') as f:
            scenario = json.load(f)
            
        scenario_trajectories = scenario['trajectories']
        self.robot_trajectory = scenario_trajectories['robot']#[1:]
        self.interaction_points = scenario['interaction_points']
        self.trajectories = scenario_trajectories
        with open(robot_poses_file,'r') as f:
            wrg = yaml.safe_load(f)
        self.world_robot_goals = wrg['waypoints'] #waypoints do not include the initial_pose
        
        self.human_goals = defaultdict(lambda:[])
        for agent_name,traj in scenario_trajectories.items():
            goals = traj.copy()
            if agent_name!='robot':
                # if len(traj)!=1:
                #     goals = goals[1:]
                self.human_goals[agent_name] = goals
                print(f"Human {agent_name} has {len(goals)} goals")
        self.pause_nav = {} # which humans have to be navigation-paused
        self.pause_msg = PauseNavs()          
        self._stopping = False
        self._shutdown_timer = None
        
    def taskStatusCallback(self,msg):
        # if nav2 finishes, terminate the scenario
        if msg.data>0:
            if msg.data == TaskResult.FAILED.value:
                self.get_logger().error('Navigation failed')
            elif msg.data == TaskResult.CANCELED.value:
                self.get_logger().warn('Navigation canceled')
            elif msg.data == TaskResult.SUCCEEDED.value:
                self.get_logger().info('Navigation succeeded')
            
            agent_trajectories = {}
            for agent,poses in self.record_agents.items():
                agent_trajectories[agent] = np.vstack(poses)
                self.get_logger().info(f"Saved trajectory for agent: {agent} of length {agent_trajectories[agent].shape}")
            np.savez(self.save_file,data=agent_trajectories)
            self.get_logger().info('Task completed, shutting down gracefully...')
            self._stopping = True
            self.scenario_finished_pub.publish(Int32(data=1))
            # self._shutdown_timer = self.create_timer(0.001, self._shutdown_callback)
            #self.destroy_node()
            #rclpy.shutdown()
        self.scenario_finished_pub.publish(Int32(data=0))
    
    def _shutdown_callback(self):
        if self._shutdown_timer:
            self.destroy_timer(self._shutdown_timer)
            self._shutdown_timer = None

    def navigationStatusCallback(self,msg):
        '''
        which waypoint (index)is the robot currently heading towards
        '''
        self.robot_current_goal_index = msg.data
    
    def robotStateCallback(self,msg):
        '''
        Track which waypoint the robot is currently heading towards
        '''
        # if self.robot_current_goal_index==len(self.robot_trajectory)-1:
        #     self.get_logger().info("Robot has finished navigation")
        # else:
        #     current_goal = self.world_robot_goals[self.robot_current_goal_index]['position']
        #     if is_near(current_goal,{'x':msg.position.position.x,'y':msg.position.position.y},1.5):
        #         self.robot_current_goal_index+=1 #this might exceed the length of the list so check in human_state_callback
        self.robot_position = msg.position
        
    def humanStateCallback(self,msg): 
        '''
        self.pause_nav = DefaultDict(False)
        Track which waypoint the human is currently heading to 
        For each human: 
            if the human NOT has met the robot already
                if the point corresponding to human_robot_first_intersection for this human is the next goal
                    If this point is not the next goal for the robot
                        self.pause_nav[human.id] = True
        '''
        behaviors = String()
        human_rows = []
        if self.robot_position and not is_near(self.robot_position,self.world_robot_goals[-1]['position'],1.0): #if recieved robot position and robot hasn't finished navigation
            self.pause_msg.agents = []
            self.record_agents['robot'].append((self.robot_position.position.x, self.robot_position.position.y))
            for human in msg.agents:
                behaviors.data += f"Human {human.name} behavior_state: {human.behavior_state}\n"
                if len(human.goals) == 0:
                    # human has reached goal already, no point in managing
                    self.pause_msg.agents.append(PauseNav(id = human.id, pause_nav = False))
                    human_rows.append([ #'Human ID','Paused','Current Goal Node','Interaction Node'
                    human.id,
                    self.pause_nav.get(human.id, None),
                    self.human_goals[human.name][0],
                    self.interaction_points[human.name]
                    ])
                    self.record_agents[human.name].append((human.position.position.x, human.position.position.y,human.behavior_state,human.gesture))
                    continue
                
                goal_index = len(self.human_goals[human.name])-len(human.goals)
                assert goal_index>=0 
                human_rows.append([ #'Human ID','Paused','Current Goal Node','Interaction Node'
                    human.id,
                    self.pause_nav.get(human.id, None),
                    self.human_goals[human.name][goal_index] if goal_index<len(self.human_goals[human.name]) else 'N/A',
                    self.interaction_points[human.name] if human.name in self.interaction_points else 'N/A'
                ])
                self.record_agents[human.name].append((human.position.position.x, human.position.position.y,human.behavior_state,human.gesture))
                
                #last goal reached or no goal for human
                # if len(human.goals)==0 or (len(human.goals) == 1 and is_near(human.position,human.goals[0],human.goal_radius)): 
                #     print(f"Human {human.id} has reached their final goal ({len(self.human_goals)}).")
                #     continue
                
                #human hasn't reached the final goal yet
                self.pause_msg.agents.append(PauseNav(id = human.id, pause_nav = False))
                self.pause_nav[human.id] = False
                try:
                    # lstr1 = f"Next goal for agent {human.id}: {self.human_goals[human.name][goal_index]}, IP: {self.interaction_points[human.name]}"
                    next_goal = self.human_goals[human.name][goal_index]
                    #print(lstr1)
                except IndexError as e:
                    print("IndexError encountered:****")
                    print(e)
                    print(human.id)
                    print(self.human_goals[human.name])
                    print(human.goals)
                    print(goal_index)
                    print(self.interaction_points[human.name])
                    print("*******************")
                
                #only manipulate the human's movement for RegularNav and when robot is not nearby
                if human.behavior_state == 0 and (not (is_near(human.position,self.robot_position))): 
                    #self.get_logger().log(f"Can manage {human.name}'s behavior",LoggingSeverity.INFO,throttle_duration_sec = 2.0)
                    if (self.interaction_points[human.name] == self.human_goals[human.name][goal_index]) or (self.interaction_points[human.name] == self.trajectories[human.name][0]): #is the human's next goal the interaction node, or the human is already at the interaction node at spawn.                        
                        #is the robot's next goal the interaction node?
                        if self.robot_trajectory[self.robot_current_goal_index+1] != self.interaction_points[human.name]:#
                            #has the robot already passed the interaction point (assume we check the first occurence of the interaction point)?
                            if (self.robot_current_goal_index+1)<self.robot_trajectory.index(self.interaction_points[human.name]):
                                self.pause_msg.agents[-1].pause_nav = True #wait for the robot
                                self.pause_nav[human.id] = True                
        else:
            self.get_logger().warn("Haven't received robot position yet")
            
        robot_row = ['Robot','N/A',self.robot_trajectory[self.robot_current_goal_index+1], 'N/A']
        table = [['Human ID','Paused','Current Goal Node','Interaction Node'],*human_rows,robot_row]
        self.get_logger().log(tabulate(table,headers = 'firstrow',tablefmt='fancy_grid'),LoggingSeverity.INFO,throttle_duration_sec = 2.0)
        self.human_behavior_pub.publish(behaviors)
    
    def timer_callback(self):
        self.publisher_.publish(self.pause_msg)
        #self.get_logger().info('Publishing')

def main(args=None):
    rclpy.init(args=args)
    with open(os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'config','launch_params.yaml'),'rb') as f:
        config_file = yaml.safe_load(f)
        
    scenario_file = config_file['scenario']
    robot_poses_file = config_file['robot_poses']
    scenario_manager = ScenarioManager(scenario_file,robot_poses_file)  

    try:
        while rclpy.ok() and not scenario_manager._stopping:
            rclpy.spin_once(scenario_manager)
    finally:
        try:
            scenario_manager.destroy_timer(scenario_manager.timer)
        except Exception:
            pass
        print("****Shutting down Scenario Manager****")
        if scenario_manager.handle:
            scenario_manager.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()