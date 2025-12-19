from os import path
import os
from os import environ
from os import pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, ExecuteProcess, Shutdown, 
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, 
                            PythonExpression, EnvironmentVariable,Command)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
import yaml
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd

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
    
    robot_name = 'burger'
    robot_sdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models','turtlebot3_burger','model.sdf')
    robot_urdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'urdf','turtlebot3_burger.urdf')
    
    with open(robot_urdf, 'r') as infp:
        robot_description  = infp.read()
    
    with open(robot_params_file,'r') as f:
        robot_poses = yaml.safe_load(f)
    
    initial_pose = robot_poses['initial_pose']
    
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name,
            '-file',robot_sdf,
            #'-topic', 'robot_description', 
            '-x', str(initial_pose['x']),
            '-y', str(initial_pose['y']),
            '-z','0.1',
            '-Y', str(initial_pose['yaw']),
            ],
        output='screen',
    )
    ##################HuNavSim##############
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('hunav_agent_manager'),'config','custom_agents.yaml'])]) 
    ld.add_action(hunav_loader_node)
    
    
    #generate world
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        #output='screen',
        parameters=[{'base_world':os.path.join(hunav_gazebo_wrapper_pkg_dir,'worlds',config['world_name'],'worlds',config['world_name']+'.world')},
        {'use_gazebo_obs': True},
        {'update_rate': 1000.0},
        {'robot_name': robot_name},
        {'global_frame_to_publish': 'map'},
        {'use_navgoal_to_start': True},
        {'navgoal_topic':'plan'},
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
        # use_nvidia_gpu,
        'gzserver ',
        '--pause ',
         os.path.join(hunav_gazebo_wrapper_pkg_dir,'worlds',config['world_name'],'worlds','generatedWorld.world'), 
        _boolean_command('verbose'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        '-s ', 'libgazebo_ros_state.so',
        '--ros-args',
        '--params-file', os.path.join(hunav_gazebo_wrapper_pkg_dir, 'config', 'params.yaml') ,
    ]

    gzclient_cmd = [
        # use_nvidia_gpu,
        'gzclient',
        _boolean_command('verbose'), ' ',
    ]

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd,
        #output='screen',
        shell=True,
        on_exit=Shutdown(),
    )

    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd,
        #output='screen',
        shell=True,
        on_exit=Shutdown(),
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
    
    
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='true'),
                     'robot_description': robot_description}],
        remappings=remappings)
    ld.add_action(robot_state_publisher_cmd)
    
    robot_spawn_process = RegisterEventHandler(
        OnProcessStart(
            target_action=gzclient_process,
            on_start=[
                LogInfo(msg='Gazebo launched, bringing robot after 2 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[spawn_robot_cmd],
                )
            ]
        )
    )
    ld.add_action(robot_spawn_process)

    
    #Launch Scenario Manager
    scenario_manager_node = Node(
        package='hunav_gazebo_wrapper',
        executable='scenario_manager.py',
        output='screen',
        ros_arguments=[
            "--log-level",
            PythonExpression(['"node_test:=" + "', "info", '"']),
        ]
    )
    ld.add_action(scenario_manager_node)
    return ld