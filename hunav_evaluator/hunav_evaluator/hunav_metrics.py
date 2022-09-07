import numpy as np
import sys

import rclpy
from hunav_msgs.msg import Agents
from hunav_msgs.msg import Agent


def total_time(agents, robot):
    secs = (robot[len(robot)-1].header.stamp - robot[0].header.stamp).sec


#def robot_path_length(agents, robot):




# def compute_metrics(agents, robot):
#     if(tt_enable):
#         secs = total_time(agents, robot)
    
#     if(rpl_enable):
#         length = robot_path_length(agents, robot)

        