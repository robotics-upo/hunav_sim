
from os import path
from os import environ
from os import pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGS = [
    DeclareLaunchArgument(
        'configuration_file', default_value='agents.yaml',
        description='Specify configuration file name in the cofig directory'
    ),
    DeclareLaunchArgument(
        'gazebo_world_file', default_value='empty_cafe.world',
        description='Specify the name of the base Gazebo world to be populated with pedestrians'
    )
]

def generate_launch_description():

    world_file_name = LaunchConfiguration('gazebo_world_file')
    conf_file_name = LaunchConfiguration('configuration_file')
    # agent configuration file
    conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_agent_manager'),
        'config',
        conf_file_name
    ])


    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[conf_file]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin direclty. So we do not need to 
    # indicate the path
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_plugin',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file_name}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ordered_launch_event = RegisterEventHandler(
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


    ld = LaunchDescription(ARGS)

    # Load the sim configuration file as ROS2 params
    ld.add_action(hunav_loader_node)
    # Read the sim configuration params from the hunav_loader node
    #ld.add_action(hunav_gazebo_worldgen_node)
    ld.add_action(ordered_launch_event)
    return ld
    

    




