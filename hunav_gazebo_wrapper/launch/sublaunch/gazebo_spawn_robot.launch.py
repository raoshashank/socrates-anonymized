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

def generate_launch_description():
    ld = LaunchDescription()
    hunav_gazebo_wrapper_pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    ld.add_action(DeclareLaunchArgument('robot_description',default_value='',description=''))
    ld.add_action(DeclareLaunchArgument('robot_model',default_value='',description=''))
    ld.add_action(DeclareLaunchArgument('robot_waypoints_file',default_value='',description=''))
    ld.add_action(DeclareLaunchArgument('robot_name',default_value='',description=''))    
    ld.add_action(DeclareLaunchArgument('world_path',default_value='',description='Which world file to launch'))
    ld.add_action(DeclareLaunchArgument('initial_pose_x',default_value='',description='initial_pose of the robot as array'))
    ld.add_action(DeclareLaunchArgument('initial_pose_y',default_value='',description='initial_pose of the robot as array'))
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]  
    
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': LaunchConfiguration('robot_description')}],
        remappings=remappings)
    
    ld.add_action(robot_state_publisher_cmd)    
    
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', LaunchConfiguration('robot_name'),
            '-file',LaunchConfiguration('robot_model'),
            '-x', LaunchConfiguration('initial_pose_x'),
            '-y', LaunchConfiguration('initial_pose_y'),
            '-z', '0.1',
            '-Y', '0.0',
            ],
        output='screen',
    )
    ld.add_action(spawn_robot_cmd)

    return ld