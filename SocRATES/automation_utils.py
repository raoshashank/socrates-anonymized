import os
import subprocess
from omegaconf import OmegaConf
import time,json
from utils.scenario_generator import ScenarioGenerator
from termcolor import cprint
from topic_watch import TopicWatcher
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

def generate_scenario(full_conf, skip_simulation=False):
    """Generate scenario using existing map (skipping map annotation)"""
    cprint("Generating scenario from existing map...", "blue")
    outputs = {}
    file_paths = OmegaConf.merge(full_conf['hunav_sim'],full_conf['paths'])
    # Change to scenario generation directory
    try:
        #import the configs from config.yaml and inputs.yaml            
        cprint("****************** Social Navigation Scenario Generator***********************", "cyan", attrs=["bold"])
        full_conf['debug'] = True
        sg = ScenarioGenerator(full_conf)
        scenario, groupids, trajectories, interaction_points, behavior_trees,outputs = sg.generate_scenario()
        if trajectories and behavior_trees:
            if not skip_simulation:
                sg.instantiate_simulator(
                    file_paths= file_paths,
                    groupids= groupids,
                    trajectories= trajectories,
                    behaviors_trees= behavior_trees,
                    interaction_points = interaction_points
                )
            outputs['scenario'] = scenario
            outputs['groupids'] = groupids
            outputs['trajectories'] = trajectories
            outputs['interaction_points'] = interaction_points
            outputs['behavior_trees'] = behavior_trees
            
            cprint("✓ Scenario generation completed successfully.", "green")
        else:
            cprint("✗ Scenario generation failed.", "red")
            return outputs
        
    except Exception as e:
        cprint(f"✗ Exception during scenario generation: {e}", "red") 
        outputs['failure_reason'] = str(e)
    return outputs

def build_colcon_ws(colcon_ws_path):
    cprint("Building the workspace...", "blue")
    build_cmd = ["colcon", "build", "--packages-select","hunav_gazebo_wrapper", "hunav_sim","hunav_agent_manager"]
    build_result = subprocess.run(build_cmd, cwd=colcon_ws_path, capture_output=True, text=True)

    if build_result.returncode != 0:
        cprint(f"✗ Build failed with error: {build_result.stderr}", "red")
        return False
    else:
        cprint("✓ Build completed successfully", "green")

    # Source the workspace
    cprint("Sourcing the workspace...", "blue")
    source_cmd = ["bash", "-c", f"source {os.path.join(colcon_ws_path, 'install', 'setup.bash')}"]
    source_result = subprocess.run(source_cmd, capture_output=True, text=True)

    if source_result.returncode != 0:
        cprint(f"✗ Sourcing failed with error: {source_result.stderr}", "red")
        return False
    else:
        cprint("✓ Workspace sourced successfully", "green")
    
    return True

def launch_scenario(experiment_dir, watcher):
    
    #run scenario
    video_name = os.path.join(experiment_dir,f'output.avi')
    save_file_name = os.path.join(experiment_dir,f'simulation_results.npz')
    scenario_cmd = ["ros2", "launch", "hunav_gazebo_wrapper", "scenario_nav2_lino.launch.py", f"video_name:={video_name}",f"save_file:={save_file_name}"]
    scenario_env = os.environ.copy()
    scenario_env["ROS_DOMAIN_ID"] = "0"

    log_path = os.path.join(experiment_dir, "test.log")
    
    log_fh = open(log_path, "w")
    scenario_process = subprocess.Popen(scenario_cmd, 
                                        stdout=log_fh, 
                                        stderr=subprocess.STDOUT, 
                                        text=True, 
                                        env=scenario_env, 
                                        bufsize=1)
    
    # Monitor stdout for success message
    launch_success_detected = False
    navigation_success_detected = False

    max_timeout_seconds = 30  # 30 seconds timeout
    navigation_timeout_seconds = 60*10
    
    #check if scenario launched correctly    
    try:
        cprint("Waiting for /human_states and /robot_description topic to receive messages...", "blue")
        watcher.wait_for("/human_states", lambda msg: len(msg.get('agents', [])) > 0, timeout_seconds=max_timeout_seconds)
        watcher.wait_for("/robot_description", lambda msg: len(msg)>0, timeout_seconds=max_timeout_seconds)
        cprint("✓ Topics are active - scenario is ready!", "green", attrs=["bold"])
        launch_success_detected = True
    except TimeoutError as e:
        cprint(f"⚠ Timeout waiting for /human_states topic to receive messages", "yellow")
        cprint(e)
        scenario_process.kill()

    #start ros2 bag recording
    bag_name = os.path.join(experiment_dir, "simulation_recording")
    #check if nav2 launched correctly
    nav2_launch_timeout_seconds = 450
    try:
        cprint("Waiting for /plan topic to receive messages...", "blue")    
        watcher.wait_for("/task_status", lambda msg: msg is not None, timeout_seconds=nav2_launch_timeout_seconds)
    except TimeoutError:
        cprint(f"⚠ Timeout waiting for /task_status topic to receive messages - Nav2 may not have launched correctly", "yellow")
        scenario_process.kill()
        return launch_success_detected, navigation_success_detected

    #wait for navigation to finish
    if launch_success_detected:
        try:
            cprint("Waiting for /scenario_finished topic to indicate task has completed...", "yellow")
            watcher.wait_for("/scenario_finished",lambda msg: msg['data']==1, timeout_seconds=navigation_timeout_seconds) #10 mins timeout
            navigation_success_detected = True
        except TimeoutError:
            cprint(f"⚠ Timeout waiting for /task_status topic to indicate task completion", "yellow")
            scenario_process.kill()
    
    return launch_success_detected, navigation_success_detected

def run_nav2_node(nav2_params_file,watcher):
    #os.chdir(colcon_ws_directory)    
    cprint("Starting nav2 waypoint follower...", "blue")
    nav2_cmd = ["ros2", "launch", "hunav_gazebo_wrapper", "nav2_follow_waypoints.launch.py", f"params_file:={nav2_params_file}"]
    # nav2_result = subprocess.run(nav2_cmd, capture_output=True, text=True, env=env)
    # success = False
    # if nav2_result.returncode != 0:
    #     cprint(f"✗ Nav2 failed with error: {nav2_result.stdout}, {nav2_result.stderr}", "red")
    # else:
    #     cprint("✓ Nav2 ran successfully", "green")
    #     success = True
    scenario_env = os.environ.copy()
    scenario_env["ROS_DOMAIN_ID"] = "0"
    
    nav2_process = subprocess.Popen(nav2_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=scenario_env, bufsize=1, universal_newlines=True)
    
    # Monitor stdout for success message
    max_timeout = 3600 * 10  # 10 mins timeout
    '''
    while timeout_counter < max_timeout:
        if nav2_process.poll() is not None:
            # Process has terminated
            _, stderr = nav2_process.communicate()
            cprint(f"⚠ Nav2 process terminated early. Stderr: {stderr}", "yellow")
            break
        
        try:
            line = nav2_process.stdout.readline()
            if line:
                cprint(f"Nav2 output: {line.strip()}", "cyan")
                if "Waypoint follower node has exited" in line:
                    cprint("✓ Success marker detected - Waypoint following finished!", "green", attrs=["bold"])
                    success_detected = True
                    break
            else:
                time.sleep(1)
                timeout_counter += 1
        except:
            time.sleep(1)
            timeout_counter += 1
    '''
    
    #check if nav2 launched correctly
    cprint("Waiting for /plan topic to receive messages...", "blue")    
    try:
        watcher.wait_for("/plan", lambda msg: len(msg) > 0, timeout=2*3600) 
        watcher.wait_for("/task_status", lambda msg: len(msg) > 0, timeout=2*3600) 
    except TimeoutError:
        cprint(f"⚠ Timeout waiting for /plan topic to receive messages - Nav2 may not have launched correctly", "yellow")
        return False, "Timeout waiting for /plan topic to receive messages - Nav2 may not have launched correctly"

    i = 0
    while True:
        #print(f"{'.'*(i%7)}",end=" ")
        i+=1
        task_status = watcher.get_last("/task_status").get('data',0)
        print(f"Current task status: {task_status}",end="\r")
        if task_status>0:
            if task_status == 1:
                cprint("✓ Task status indicates success!", "green", attrs=["bold"])
                failure_reason = ""
                break
            elif task_status in [0,2,3]:
                failure_reason = "Task failed"
                break
    
    cprint("✓ /task_status topic indicates completion", "green", attrs=["bold"])
    return True, failure_reason

def nuke_ros():
    cprint("Killing all ROS processes...", "magenta")
    killros3 = [
        "killall", "-9",
        "ros2", "_ros2_daemon", "rviz2", "gzserver",
        "robot_state_publisher", "gzclient",
        "controller_server", "lifecycle_manager"
    ]
    killros2 = ["bash", "-c", "ps aux | grep ros | grep -v grep | awk '{print $2}' | xargs kill -9"]
    subprocess.run(killros3, capture_output=True)
    subprocess.run(killros2, capture_output=True)
        
        
def run_socrates(
    inputs,
    colcon_ws_directory,
    # video_name,
    # save_file_name,
    experiment_dir,
    skip_generation = False,
    load = False,
    skip_simulation = False,
    max_retries = 5
    ):
    
    #scenario generation (optional)-> scenario launch -> nav2run -> record video -> kill ros
    outputs = {}
    config = OmegaConf.load('config.yaml')  
    full_conf = OmegaConf.merge(config, inputs)  
    output_file_path = os.path.join(experiment_dir,'scenario_generation_outputs.json')
    #1. Generate scenario 
    if not skip_generation: 
        # config['user_feedback'] = 'retry_local'    
        outputs = generate_scenario(full_conf,skip_simulation)
        
        with open(os.path.join(experiment_dir, 'scenario_generation_outputs.json'), 'w') as f:
                json.dump(outputs, f, indent=4)
        
        if 'failure_reason' in outputs:
            cprint(f"✗ Scenario generation failed. Skipping this trial.", "red")
            return False, outputs
    else:
        cprint("⏭️  Skipping scenario generation, using existing scenario...", "yellow")
        if load:
            try:
                print(f"Loading scenario generation outputs from {os.path.relpath(output_file_path)}")
                outputs = json.load(open(output_file_path,'r'))
                file_paths = OmegaConf.merge(full_conf['hunav_sim'],full_conf['paths'])
                sg = ScenarioGenerator(full_conf)
                sg.instantiate_simulator(
                    file_paths= file_paths,
                    groupids= outputs['groupids'],
                    trajectories= outputs['trajectories'],
                    behaviors_trees= outputs['behavior_trees'],
                    interaction_points = outputs['interaction_points']
                )
            except Exception as e:
                cprint(f"✗ Scenario generation outputs not found for loading. Skipping this trial.", "red")
                cprint(e)
                outputs['failure_reason'] = "Scenario generation outputs not found for loading"
                return False, outputs
    
    # Add delay to ensure scenario files are properly written
    time.sleep(2)

    #2. Build colcon_ws
    if not build_colcon_ws(colcon_ws_directory):
        cprint(f"✗ Colcon workspace build failed. Skipping this trial.", "red")
        outputs['failure_reason'] = "Colcon workspace build failed"
        return False, outputs
    
    if not skip_simulation:
        topics = {
            "/human_states":"hunav_msgs/msg/Agents", # simulation launched correctly
            "/robot_description":"std_msgs/msg/String", # robot spawned correctly
            "/plan":"nav_msgs/msg/Path", # nav2 running 
            "/robot_states":"hunav_msgs/msg/Agent", # robot state
            "/task_status":"std_msgs/msg/Int32",
            "/scenario_finished":"std_msgs/msg/Int32",
            "/cmd_vel":"geometry_msgs/msg/Twist", # task state
            }
        qos_profiles = {}
        for k,v in topics.items():
            qos_profiles[k] = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        
        #3. Simulate Scenario
        success_launch, success_nav2 = False, False
        failure_reason = ""
        watch = TopicWatcher(topics,qos_profiles)
        watch.start()
        for attempt in range(max_retries):
            cprint(f"🚀 Attempting scenario launch (attempt {attempt + 1}/{max_retries})...", "yellow")        
            #3. Launch Scenario and nav2
            success_launch, success_nav2 = launch_scenario(experiment_dir, watch)
            cprint(f"Simulation Finished!", "yellow")
            nuke_ros() 
            if success_launch:
                break
            
        if not success_launch:
            cprint(f"✗ All {max_retries} attempts failed. Exiting.", "red", attrs=["bold"])
            outputs['failure_reason'] = failure_reason
            return False, outputs
        else:
            cprint(f"✓ Scenario launched successfully!", "green", attrs=["bold"])
        
        if success_nav2:
            cprint(f"✓ Nav2 completed successfully!", "green", attrs=["bold"])
        else:
            cprint(f"✗ Nav2 failed to complete.", "red", attrs=["bold"])
            outputs['failure_reason'] = failure_reason
            
        
        watch.stop()
        # Kill all ROS nodes
    
    return True, outputs