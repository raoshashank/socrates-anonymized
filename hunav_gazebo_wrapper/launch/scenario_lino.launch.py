from os import path
import os
from os import environ
from os import pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, ExecuteProcess, Shutdown, 
                            RegisterEventHandler, TimerAction, LogInfo, EmitEvent)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, 
                            PythonExpression, EnvironmentVariable,Command)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events.process import SignalProcess
from signal import SIGINT, SIGTERM, SIGKILL
import yaml
import numpy as np
from IPython import embed
camera_positions = {    
    'arena_nus_com1_building': {
        'x': -15,
        'y': -3,
        'z': 40.0,
        'yaw': np.pi/2.0
    },
    'small_warehouse': {
        'x': 0,
        'y': 0,
        'z': 15.0,
        'yaw': np.pi
    },
    'hospital': {
        'x': 0,
        'y': -12.5,
        'z': 30.0,
        'yaw': np.pi
    }
}
remove_objects = {
    'hospital': ["scrub","wheelchair","unit_box"]
}


def _boolean_command(arg: str) -> PythonExpression:
    # Expands to "--<arg>" if <arg> is "true", otherwise expands to "" (empty)
    return PythonExpression(['"--', arg, '" if "', LaunchConfiguration(arg), '" == "true" else ""'])


# Create world file with video recording camera
def remove_objects_with_name(world_content, name_substring):
    """
    Remove all models from the world content that contain the specified substring in their name.
    
    Args:
      world_content (str): The original SDF world content as a string.
      name_substring (str): Substring to match in model names for removal.
    
    Returns:
      str: Modified world content with specified models removed.
    """
    lines = world_content.splitlines()
    modified_lines = []
    skip_mode = False
    for line in lines:
        if '<model name=' in line and name_substring.lower() in line.lower():
            skip_mode = True
            print(f"REMOVING MODEL: {line.strip()}")
        if not skip_mode:
            modified_lines.append(line)
        if '</model>' in line and skip_mode:
            skip_mode = False
    return '\n'.join(modified_lines)


def add_camera_to_world(world_content, camera_name, pose_x=0, pose_y=0, pose_z=8, 
                roll=0, pitch=1.5708, yaw=0, width=1920, height=1080, 
                horizontal_fov=1.3963, update_rate=30, namespace="video_camera"):
      """
      Add a camera to existing world SDF content.
      
      Args:
        world_content (str): Existing SDF world content
        camera_name (str): Name for the camera model
        pose_x, pose_y, pose_z (float): Camera position
        roll, pitch, yaw (float): Camera orientation
        width, height (int): Image dimensions
        horizontal_fov (float): Camera field of view
        update_rate (int): Camera update rate
        namespace (str): ROS namespace for the camera
      
      Returns:
        str: Modified world content with added camera
      """
      camera_model = f'''
      <model name="{camera_name}">
        <pose>{pose_x} {pose_y} {pose_z} {roll} {pitch} {yaw}</pose>
        <static>true</static>
        <link name="{camera_name}_link">
        <sensor name="{camera_name}_sensor" type="camera">
          <camera>
          <horizontal_fov>{horizontal_fov}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>{update_rate}</update_rate>
          <visualize>true</visualize>
          <plugin name="{camera_name}_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>{namespace}</namespace>
          </ros>
          <camera_name>{camera_name}</camera_name>
          <frame_name>{camera_name}_link_optical</frame_name>
          </plugin>
        </sensor>
        </link>
      </model>'''
      
      # Find the closing </world> tag and insert camera before it
      closing_tag = '</world>'
      insertion_point = world_content.rfind(closing_tag)
      
      if insertion_point != -1:
        print("INSERTED CAMERA!!!!!"*6)    
        return world_content[:insertion_point] + camera_model + '\n    ' + world_content[insertion_point:]
      else:
        # If no closing tag found, append to end
        return world_content + camera_model

def generate_launch_description():
    ld = LaunchDescription()
    with open(os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'config','launch_params.yaml'),'rb') as f:
        config = yaml.safe_load(f)
    #env vars
    model, plugin, media = GazeboRosPaths.get_paths()
    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']
    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'models',
    ])
    set_env_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), my_gazebo_models]
    )
    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH', 
        value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH'), my_gazebo_models]
    )
    set_env_gazebo_plugin = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', 
        value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH'), plugin]
    )
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)
    
    # Gazebo
    hunav_gazebo_wrapper_pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    robot_params_file =  os.path.join(hunav_gazebo_wrapper_pkg_dir,'config','robot_poses.yaml')   
    
    robot_name = 'linorobot2'    
    #linorobot
    #robot_description = Command(['xacro ',  PathJoinSubstitution([FindPackageShare("linorobot2_description"), "urdf/robots", "4wd.urdf.xacro"])])
    
    with open(robot_params_file,'r') as f:
        robot_poses = yaml.safe_load(f)
    
    initial_pose = robot_poses['initial_pose']
    
    # spawn_robot_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', robot_name,
    #         '-file',robot_sdf,
    #         #'-topic', 'robot_description', 
    #         '-x', str(initial_pose['x']),
    #         '-y', str(initial_pose['y']),
    #         '-z','0.1',
    #         '-Y', str(initial_pose['yaw']),
    #         ],
    #     output='screen',
    # )
    
    ##################HuNavSim##############
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('hunav_agent_manager'),'config','custom_agents.yaml'])]) 
    ld.add_action(hunav_loader_node)
    
    base_world_file_path = os.path.join(hunav_gazebo_wrapper_pkg_dir,'worlds',config['world_name'],'worlds',config['world_name']+'.world')
    with open(base_world_file_path, 'r') as f:
        world_content = f.read()

    if config['world_name'] in remove_objects:
        for name_substring in remove_objects[config['world_name']]:
            print(f"Removing objects with name containing: {name_substring}")
            world_content = remove_objects_with_name(world_content, name_substring)
            
    camera_pose = camera_positions.get(config['world_name'], camera_positions['small_warehouse'])
    world_content = add_camera_to_world(world_content, "top_down_camera",pose_x = camera_pose['x'],pose_y = camera_pose['y'],
                                        pose_z=camera_pose['z'], yaw = camera_pose['yaw'],pitch=1.5708)
    
    # elif config['world_name'] == 
    base_world_file_path = os.path.join(hunav_gazebo_wrapper_pkg_dir,'worlds',config['world_name'],'worlds',config['world_name']+'_camera.world')
    with open(base_world_file_path, 'w') as f:
        f.write(world_content)

    #generate world
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        #output='screen',
        parameters=[{'base_world':base_world_file_path},
        {'use_gazebo_obs': True},
        {'update_rate': 1000.0},
        {'robot_name': robot_name},
        {'global_frame_to_publish': 'map'},
        {'use_navgoal_to_start': False},
        {'navgoal_topic':'/plan'},
        {'ignore_models': 'ground_plane'}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )
           
    generate_world_process = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )
    ld.add_action(generate_world_process)
    
    
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(hunav_manager_node)

    
    ##############Gazebo##############
    
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Set "true" to increase messages written to terminal.'
    )
    ld.add_action(declare_arg_verbose)

    gzserver_cmd = [
    'gzserver',
    os.path.join(
        hunav_gazebo_wrapper_pkg_dir, 'worlds',
        config['world_name'], 'worlds', 'generatedWorld.world'  # check this path!
    ),
    _boolean_command('verbose'),   # expands to "--verbose" or "" (OK for argv)
    '-s', 'libgazebo_ros_init.so',
    '-s', 'libgazebo_ros_factory.so',
    '-s', 'libgazebo_ros_state.so',
    '--ros-args',
    '--params-file', os.path.join(hunav_gazebo_wrapper_pkg_dir, 'config', 'params.yaml'),
]

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd,
        output='screen',
        shell=False              # <- direct signals to the real process
        #sigterm_timeout='2',
        #sigkill_timeout='2',
    )
    gzclient_cmd = [
        # use_nvidia_gpu,
        'gzclient',
        _boolean_command('verbose'), ' ',
    ]
    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd,
        #output='screen',
        shell=True,
        #on_exit=Shutdown(),
    )
    gazebo_launch_process = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[gzserver_process, gzclient_process],
                )
            ]
        )
    )
    ld.add_action(gazebo_launch_process)
    #######################################
    #spawn robot    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]  
    ld.add_action(DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'))
    
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('linorobot2_gazebo'),
                                                       'launch','gazebo.launch.py')),
            launch_arguments={'spawn_x':str(initial_pose['x']),
                              'spawn_y':str(initial_pose['y']),
                              'ros-arguments':'--ros-args --log-level ERROR'}.items()
    ))
    
    #Launch Scenario Manager
    scenario_manager_node = Node(
        package='hunav_gazebo_wrapper',
        executable='scenario_manager.py',
        output='screen',
        ros_arguments=[
            "--log-level",
            "info"
            #PythonExpression(['"node_test:=" + "', "debug", '"']),
        ],
        parameters=[{'save_file': LaunchConfiguration('save_file',default='scenario_output.npz')}]
    )
    # ld.add_action(scenario_manager_node)
    
    # Launch Nav2 and waypoint follower
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),
    #                                                    'launch','nav2_follow_waypoints.launch.py'))
    #     ))
    
    
    # Video Recorder Node - triggered by /plan topic
    video_recorder_node = Node(
        package='hunav_gazebo_wrapper',
        executable='video_recorder',
        name='conditional_video_recorder',
        output='screen',
        parameters=[
            {'filename': LaunchConfiguration('video_name', default='output.avi')},
            {'fps': 30.0},
            {'codec': 'MP4V'},
            {'trigger_topic': '/plan'},
            {'robot_poses_file': 'robot_poses.yaml'},
        ],
        remappings=[
            ('image', '/video_camera/top_down_camera/image_raw'),
        ],
        arguments=['--ros-args', '--log-level', 'ERROR']
    )
    # ld.add_action(video_recorder_node)
    
    # Kill all nodes when scenario_manager dies
    # scenario_manager_exit_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=scenario_manager_node,
    #         on_exit=[
    #         LogInfo(msg='Scenario manager exited (fallback): signalling gzserver and shutting down.'),
    #         EmitEvent(event=SignalProcess(signal_number=SIGINT,  process_matcher=gzserver_process)),
    #         TimerAction(period=0.5, actions=[
    #             EmitEvent(event=SignalProcess(signal_number=SIGTERM, process_matcher=gzserver_process)),
    #         ]),
    #         TimerAction(period=2.0, actions=[
    #             EmitEvent(event=SignalProcess(signal_number=SIGKILL, process_matcher=gzserver_process)),
    #         ]),
    #         TimerAction(period=2.2, actions=[Shutdown()]),
    #         ]
    #     )
    # )
    # ld.add_action(scenario_manager_exit_handler)
    
    return ld