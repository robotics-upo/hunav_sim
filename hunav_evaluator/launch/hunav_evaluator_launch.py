
from os import path
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGS = [
    # Two modes:
    # 1- (DISABLED) The user start/stop the recording through the
    #    the service /hunav_trigger_recording
    # 2- (DEFAULT) The recording start/stop process is automatic.
    #    It starts when the first topic is received.
    #    It stops when a certain time pass without receiving data.
    # DeclareLaunchArgument(
    #     'mode', default_value='2',
    #     description='Specify working mode (1 or 2)'
    # ),
    DeclareLaunchArgument(
        'frequency', default_value='1.0',
        description='Specify the frecuency of data capture. (0.0 data is captured as the same freq than data is published)'
    )
]

def generate_launch_description():

    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[
            #{'use_sim_time': True},
            #{'mode': LaunchConfiguration('mode')},
            {'frequency': LaunchConfiguration('frequency')}
        ]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ld = LaunchDescription(ARGS)
    ld.add_action(hunav_evaluator_node)
   
    return ld
    

    