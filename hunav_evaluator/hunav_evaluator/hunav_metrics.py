from importlib.resources import path
from tokenize import group
import numpy as np
import math
import sys
import rclpy
from hunav_msgs.msg import Agents
from hunav_msgs.msg import Agent
from geometry_msgs.msg import Pose

# Teaching Robot Navigation Behaviors to Optimal RRT Planners
# Noé Pérez-Higueras, Fernando Caballero & Luis Merino

def euclidean_distance(pose, pose1):
    return math.sqrt((pose.position.x - pose1.position.x)**2 + (pose.position.y - pose1.position.y)**2)

def get_group_center(agents, robot, step, group_id, distance):
    poses = list()
    x = 0
    y = 0

    for agent in agents:
        for i in range(len(agent.agents)):
            if agent.agents[i].group_id == group_id:
                pose = Pose()
                pose.position.x = agent.agents[i].position.position.x + (distance * math.cos(agent.agents[i].yaw))
                pose.position.y = agent.agents[i].position.position.y + (distance * math.sin(agent.agents[i].yaw))
                poses.append(pose)
        
    for pose in poses:
        x = x + pose.position.x
        y = y + pose.position.y
    
    if len(poses) != 0:
        x = x/len(poses)
        y = y/len(poses)

    group_center = Pose()
    group_center.position.x = float(x)
    group_center.position.y = float(y)

    return group_center

def indicator_function(norm, k):
    if k == "intimate":
        if norm < 0.45:
            return 1
        else:
            return 0
    elif k == "personal":
        if norm >= 0.45 and norm < 1.2:
            return 1
        else:
            return 0
    elif k == "social":
        if norm >= 1.2 and norm < 3.6:
            return 1
        else:
            return 0
    elif k == "public":
        if norm >= 3.6:
            return 1
        else:
            return 0
    else:
        return 0

def total_time(agents, robot):
    t2 = rclpy.time.Time.from_msg(agents[len(agents)-1].header.stamp)
    t1 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    secs = (t2 - t1).to_msg().sec #.nanoseconds() #/1e9
    print('\nTime_to_reach_goal computed: %.2f secs' % secs)
    return secs

def robot_path_length(agents, robot):
    path_length = 0
    for i in range(len(robot)-1):
        path_length += euclidean_distance(robot[i+1].position, robot[i].position)# - robot[i+1].radius - robot[i].radius
    print('\nPath_length computed: %.2f m' % path_length)
    return path_length

def cumulative_heading_changes(agents, robot):
    chc = 0
    for i in range(len(robot) - 1):
        norm = normalize_angle(robot[i].yaw - robot[i+1].yaw)
        if norm < 0.0:
            norm *= -1
        chc += norm

    print('Cumulative_heading_changes: %.2f rads' % chc)
    return chc

def normalize_angle(ang):
    while (ang <= -math.pi): 
      ang += 2 * math.pi
    while (ang > math.pi):
      ang -= 2 * math.pi
    return ang

def avg_closest_person(agents, robot):
    avg_dist = 0
    for i in range(len(robot)):
        min_dist = 10000 
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius
            if(d < min_dist):
                min_dist = d
        if(len(agents[i].agents)>0):
            avg_dist += min_dist

    print('Average_closest_person: %.2f' % avg_dist)
    return avg_dist/len(robot)


def minimum_distance_to_people(agents, robot):
    min_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agents:
            min_distance.append(euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius) 
    
    min_dist = min(min_distance)
    
    print('Minimum_distance_to_people: %.2f' % min_dist)

    return min_dist

def maximum_distance_to_people(agents, robot):
    max_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agents:
            max_distance.append(euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius)
    
    max_dist = max(max_distance)
    
    print('Maximum_distance_to_people: %.2f' % max_dist)

    return max_dist

def personal_space_intrusions(agents, robot):
    # Choose k (intimate, personal, social, public)
    k = "intimate"
    space_intrusions = 0

    for i in range(len(robot)):
        min_dist = 10000
        
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius
            if d < min_dist:
                min_dist = d

        indicator = indicator_function(min_dist, k)
        if indicator == 1:
            space_intrusions += 1

    n = 1/len(robot)
    space_intrusions *= n
    percentage = space_intrusions * 100

    print('Personal_space_intrusions: %.2f' % percentage + "% " + "K: %s" % k)
        
    return percentage, k

def interaction_space_intrusions(agents, robot):
    # Choose k (intimate, personal, social, public)
    k = "intimate"
    # Choose group_id
    group_id = -1
    # Choose distance 
    distance = 1.5

    space_intrusions = 0
    min_dist = 10000
    for i in range(len(robot)):
        people_group = get_group_center(agents, robot[i], i, group_id, distance)
        d = euclidean_distance(robot[i].position, people_group)
        if d < min_dist:
            min_dist = d
            indicator = indicator_function(min_dist, k)
            if indicator == 1:
                space_intrusions += 1
        

    n = 1/len(robot)

    space_intrusions *= n
    percentage = space_intrusions * 100

    print('Interaction_space_intrusions: %.2f' % percentage + "% " + "K: %s" % k)
        
    return percentage

# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez


# The metrics Robot on Person Personal Distance Violation, Person on Robot Personal Distance Violation, Intimate Distance Violation and
# Person on Robot Intimate Distance Violation have already been implemented in the Personal_space_intrusions function.
# Instead of returning the number of times, it returns a percentage of distance violation.

def robot_on_person_collision(agents, robot):

    collision_count = 0
    
    for i in range(len(robot)):
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius
            if d <= 0.05:
                collision_count += 1
    
    print('Robot_on_person_collision: %i' % collision_count)

    return collision_count


def person_on_robot_collision(agents, robot):
    
    collision = 0

    for i in range(len(agents)):
        for agent in agents[i].agents:
            for r in robot:
                d = euclidean_distance(r.position, agent.position) - r.radius - agent.radius 
                if d <= 0.05:
                    collision += 1

    return collision

def time_not_moving(agents, robot):
    
    avg_time = total_time(agents, robot)/len(agents)

    count = 0
    for r in robot:
        if(r.linear_vel < 0.01 and abs(r.angular_vel < 0.02)):
            count=count+1
    time_stopped = avg_time*count
            
    return time_stopped

#ToDo
def path_irregularity(agents, robot):
    pass

#ToDo
def path_efficiency(agents, robot):
    pass

metrics = {
    'time_to_reach_goal': total_time,
    'path_length': robot_path_length,
    'cumulative_heading_changes': cumulative_heading_changes,
    'avg_distance_to_closest_person': avg_closest_person,
    'minimum_distance_to_people': minimum_distance_to_people,
    'personal_space_intrusions': personal_space_intrusions,
    'intimate_space_intrusions': interaction_space_intrusions,
    'robot_on_person_collision': robot_on_person_collision,
    'person_on_robot_collision': person_on_robot_collision,
    'time_not_moving': time_not_moving
}


# def compute_metrics(agents, robot):
#     if(tt_enable):
#         secs = total_time(agents, robot)
    
#     if(rpl_enable):
#         length = robot_path_length(agents, robot)

