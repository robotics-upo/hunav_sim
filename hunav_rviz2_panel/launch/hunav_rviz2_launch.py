import os

from click import argument
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import (OnProcessStart)

def generate_launch_description():

    bringup_dir = get_package_share_directory('hunav_rviz2_panel')

    ARGUMENTS = [ 
        DeclareLaunchArgument('sim_time', default_value='true',description='flag to use sim time'),
        DeclareLaunchArgument('map', default_value=os.path.join(bringup_dir, 'maps', 'map_cafe2.yaml'),description='Full path to map yaml file to load'),
        DeclareLaunchArgument('autostart', default_value='true',description='Automatically startup the nav2 stack'),
    ]

    sim_time = LaunchConfiguration('sim_time')
    map_yaml_file = LaunchConfiguration('map')   

    rviz_conf = PathJoinSubstitution(
        [FindPackageShare("hunav_rviz2_panel"), "launch", "rviz2.rviz"])

    launch_description = LaunchDescription(ARGUMENTS)

    rviz = Node(
        package='rviz2',
        name='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': sim_time}],
        arguments=['-d', rviz_conf]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        #parameters=[configured_params],
        parameters=[{'use_sim_time': sim_time}, 
                    {'yaml_filename': map_yaml_file}
                    ]
        #remappings=remappings
    )

    lyfecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]            
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz,
            on_start=[
                LogInfo(msg='HunNavRVizPanel started, launching map_server after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[map_server, lyfecycle],
                )
            ]
        )
    )

    launch_description.add_action(rviz)
    launch_description.add_action(ordered_launch_event)
    return launch_description
