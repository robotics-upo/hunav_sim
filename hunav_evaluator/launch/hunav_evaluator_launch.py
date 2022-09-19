
from os import path
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the cofig directory'
    )
]

def generate_launch_description():

    metrics_file_name = LaunchConfiguration('metrics_file')
    # agent configuration file
    metrics_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        metrics_file_name
    ])

    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ld = LaunchDescription(ARGS)
    ld.add_action(hunav_evaluator_node)
   
    return ld
    

    