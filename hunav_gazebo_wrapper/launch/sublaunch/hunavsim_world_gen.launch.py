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
                            PythonExpression, EnvironmentVariable)
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
    
    
    gazebo_params_file = os.path.join(get_package_share_directory('hunav_gazebo_wrapper'), 'config', 'params.yaml')
    ##human details
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_agent_manager'),
        'config',
       'custom_agents.yaml'
    ])
    
    ###world to launch
    ld.add_action(
        DeclareLaunchArgument('base_world',
                              default_value='',
                              description='Base world to use')
    )
   
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        #output='screen',
        parameters=[agent_conf_file]
    ) 
    ld.add_action(hunav_loader_node)
    
    ld.add_action(
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description='Name of the robot'
        )
    )
    
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        #output='screen',
        parameters=[{'base_world':LaunchConfiguration('base_world')},
        {'use_gazebo_obs': True},
        {'update_rate': 1000.0},
        {'robot_name': LaunchConfiguration('robot_name')},
        {'global_frame_to_publish': 'map'},
        {'use_navgoal_to_start': False},
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
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Set "true" to increase messages written to terminal.'
    )
    ld.add_action(declare_arg_verbose)
    ld.add_action(generate_world_process)
    #### Launch gazebo with world
    gzserver_cmd = [
        # use_nvidia_gpu,
        'gzserver ',
        '--pause ',
         LaunchConfiguration('world_path'), 
        _boolean_command('verbose'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        '-s ', 'libgazebo_ros_state.so',
        '--ros-args',
        '--params-file', gazebo_params_file ,
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
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[gzserver_process,gzclient_process],
                )
            ]
        )
    ))
    
    return ld