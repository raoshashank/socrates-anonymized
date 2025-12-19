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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, 
                            PythonExpression, EnvironmentVariable)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    with open(os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'config','launch_params.yaml'),'rb') as f:
        config = yaml.safe_load(f)
        
    rviz_config_file = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'false',    
                          'rviz_config': rviz_config_file,
                          'log_level':'error'}.items())
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'nav2_params','mppi.yaml'),
        description='params file path'
    ))
    ld.add_action(rviz_cmd)
    ld.add_action(DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'worlds',config['world_name'],'map','map.yaml'),
        description='map file'
    ))
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch','bringup_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'false',
                          'slam':'False',
                          'map':LaunchConfiguration('map_file'),
                          'use_sim_time':'true',
                          'params_file': LaunchConfiguration('params_file'),
                          'autostart':'true',
                          'use_composition':'True',
                          'emulate_tty': 'True',
                          'use_respawn':'False',
                          'log_level':'error'}.items())
    ld.add_action(bringup_cmd)
    
    waypoint_follower_node = Node(
        package='hunav_gazebo_wrapper',
        executable='waypointfollow.py',
        #output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR'],        
    )
    ld.add_action(waypoint_follower_node)
    waypoint_follower_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=waypoint_follower_node,
            on_exit=[LogInfo(msg="Waypoint follower node has exited, shutting down the launch.")]
        )
    )
    ld.add_action(waypoint_follower_exit_handler)
    return ld