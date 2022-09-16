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
    return math.sqrt(abs((pose.position.x - pose1.position.x)**2 + (pose.position.y - pose1.position.y)**2))

def closest_person(agents, robot, step):
    closest_distance = euclidean_distance(robot.goals[step], agents[0].goals[0])
    current_closest = agents[0]

    for i in range(len(agents)):
        for j in range(len(agents[i].goals)):
            distance = euclidean_distance(robot.goals[step], agents[i].goals[j])
            if distance < closest_distance:
                closest_distance = distance
                current_closest = agents[i]

    return current_closest

def closest_initial_pose(agents, robot):
    closest_distance = euclidean_distance(robot.position, agents[0].position)
    current_closest = agents[0]

    for i in range(len(agents)):
        distance = euclidean_distance(robot.position, agents[i].position)
        if distance < closest_distance:
            closest_distance = distance
            current_closest = agents[i]
    
    return current_closest

def get_group_center(agents, robot, step, group_id, distance):
    poses = list()
    x = 0
    y = 0

    for i in range(len(agents)):
       if agents[i].group_id == group_id:
            if step < len(agents[i].goals):
                pose = Pose()
                pose.position.x = agents[i].goals[step].position.x + (distance * math.cos(agents[i].yaw))
                pose.position.y = agents[i].goals[step].position.y + (distance * math.sin(agents[i].yaw))
                poses.append(pose)
    
    for pose in poses:
        x = x + pose.position.x
        y = y + pose.position.y
    
    x = x/len(poses)
    y = y/len(poses)

    group_center = Pose()
    group_center.position.x = x
    group_center.position.y = y

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
    secs = (robot[len(robot)-1].header.stamp - robot[0].header.stamp).sec
    return secs

def robot_path_length(agents, robot):
    path_length = 0
    goals_n = 0

    for i in range(len(robot)):
        
        path_length = path_length + math.sqrt(abs((robot[i].position.position.x - robot[i].goals[goals_n].position.x)**2 + (robot[i].position.position.y - robot[i].goals[goals_n].position.y)**2))
        
        for j in range(len(robot[i].goals) - 1):    
            path_length = path_length + math.sqrt(abs((robot[i].goals[j].position.x - robot[i].goals[j+1].position.x)**2 + (robot[i].goals[j].position.y - robot[i].goals[j+1].position.y)**2))
            
    return path_length

def cumulative_heading_changes(agents, robot):
    chc = 0
    for i in range(len(robot) - 1):
        chc = chc + (robot[i].yaw - robot[i+1].yaw)
    return chc

# I don't know if initial position is needed, I think it is.
def avg_closest_person(agents, robot):
    avg_closest = 0
    robot_goals_num = len(robot[0].goals)

    # If initial_pose is needed add:
    #agent = closest_initial_pose(agents, robot[0])
    #avg_closest = avg_closest + euclidean_distance(robot[0].position, agent.position)

    for i in range(robot_goals_num):
        agent = closest_person(agents, robot[0], i)
        avg_closest = avg_closest + euclidean_distance(robot[0].goals[i], agent.goals[i])
    
    n = 1/robot_goals_num  # +1 in case of initial position

    avg_closest = avg_closest * n

    return avg_closest

def min_distance_to_people(agents, robot):
    min_distance = list()

    #If initial_pose is needed add:
    #agent = closest_initial_pose(agents, robot[0])
    #min_distance.append(euclidean_distance(robot[0].position, agent.position))

    for i in range(len(robot[0].goals)):
        agent = closest_person(agents, robot[0], i)
        min_distance.append(euclidean_distance(robot[0].goals[i], agent.goals[i]))
    
    return min(min_distance)

def max_distance_to_people(agents, robot):
    max_distance = list()
    current_agent = 0

    #If initial_pose is needed add:
    #agent = closest_initial_pose(agents, robot[0])
    #max_distance.append(euclidean_distance(robot[0].position, agent.position))

    for i in range(len(robot[0].goals)):
        agent = closest_person(agents, robot[0], i)
        max_distance.append(euclidean_distance(robot[0].goals[i], agent.goals[i]))
        current_agent = current_agent + 1
    
    return max(max_distance)

def personal_space_intrusions(agents, robot):
    # Choose k (intimate, personal, social, public)
    k = "intimate"
    space_intrusions = 0

    # If initial_pose is needed add:
    #agent = closest_initial_pose(agents, robot[0])
    #norm = euclidean_distance(robot[0].position, agent.position)
    #indicator = indicator_function(norm, k)

    #if indicator == 1:
    #    space_intrusions = space_intrusions + 1

    for j in range(len(robot[0].goals)):
        closest = closest_person(agents, robot[0], j)
        
        if j < len(closest.goals):
            norm = euclidean_distance(robot[0].goals[j], closest.goals[j])
            indicator = indicator_function(norm, k)
            if indicator == 1:
                space_intrusions = space_intrusions + 1

    n = 1/(len(robot[0].goals))
    space_intrusions = space_intrusions * n
    percentage = space_intrusions * 100
        
    return percentage

def interaction_space_intrusions(agents, robot):
    # Choose k (intimate, personal, social, public)
    k = "intimate"
    # Choose group_id
    group_id = -1
    # Choose distance 
    distance = 1.5

    space_intrusions = 0
    for j in range(len(robot[0].goals)):
        people_group = get_group_center(agents, robot[0], j, group_id, distance)
        norm = euclidean_distance(robot[0].goals[j], people_group)
        indicator = indicator_function(norm, k)
        if indicator == 1:
            space_intrusions = space_intrusions + 1

    n = 1/(len(robot[0].goals))
    space_intrusions = space_intrusions * n
    percentage = space_intrusions * 100
        
    return percentage

# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez



def main():

    robot = Agent()
    robot.id = 0
    robot.type = Agent.ROBOT
    robot.behavior_state = Agent.BEH_NO_ACTIVE
    robot.skin = 1
    robot.behavior = 3
    robot.group_id = -1
    robot.desired_velocity = 1.5
    robot.radius = 0.4
    robot.position.position.x = 0.0
    robot.position.position.y = 0.0
    robot.position.position.z = 1.250000
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 1.0
    pose.position.z = 1.250000
    
    pose1 = Pose()
    pose1.position.x = 2.0
    pose1.position.y = 2.0
    pose1.position.z = 1.250000

    pose7 = Pose()
    pose7.position.x = 5.0
    pose7.position.y = 5.0
    pose7.position.z = 1.250000

    robot.goals.append(pose)
    robot.goals.append(pose1)
    robot.goals.append(pose7)

    agent = Agent()
    agent.id = 0
    agent.type = Agent.PERSON
    agent.behavior_state = Agent.BEH_NO_ACTIVE
    agent.skin = 1
    agent.behavior = 3
    agent.group_id = -1
    agent.desired_velocity = 1.5
    agent.radius = 0.4
    agent.position.position.x = -2.819170
    agent.position.position.y = 1.610966
    agent.position.position.z = 1.250000

    pose2 = Pose()
    pose2.position.x = 1.141170
    pose2.position.y = 1.010966
    pose2.position.z = 1.250000
    
    pose3 = Pose()
    pose3.position.x = 2.132697
    pose3.position.y = 2.040503
    pose3.position.z = 1.250000

    pose6 = Pose()
    pose6.position.x = 7.3455
    pose6.position.y = 6.4566
    pose6.position.z = 1.25000

    agent.goals.append(pose2)
    agent.goals.append(pose3)
    agent.goals.append(pose6)

    agent1 = Agent()
    agent1.id = 1
    agent1.type = Agent.PERSON
    agent1.behavior_state = Agent.BEH_NO_ACTIVE
    agent1.skin = 1
    agent1.behavior = 3
    agent1.group_id = -1
    agent1.desired_velocity = 1.5
    agent1.radius = 0.4
    agent1.position.position.x = -5.441170
    agent1.position.position.y = 3.110966
    agent1.position.position.z = 1.25000012

    pose4 = Pose()
    pose4.position.x = 1.1
    pose4.position.y = 1.0
    pose4.position.z = 1.250000
    
    pose5 = Pose()
    pose5.position.x = 2.332697
    pose5.position.y = 2.840503
    pose5.position.z = 1.250000

    agent1.goals.append(pose4)
    agent1.goals.append(pose5)

    robots = list()
    robots.append(robot)

    agents = list()
    agents.append(agent)
    agents.append(agent1)

    for i in range(len(robots[0].goals)):
        print("\nClosest current agent: " + str(closest_person(agents, robots[0], i)))

    print("Robot path length: " + str(robot_path_length(agents, robots)))
    print("\nCumulative heading changes: " + str(cumulative_heading_changes(agents, robots)))
    print("\nAverage distance to closest person: " + str(avg_closest_person(agents, robots)))
    print("\nMinimun distance to closest person: " + str(min_distance_to_people(agents, robots)))
    print("\nMaximun distance to people: " + str(max_distance_to_people(agents, robots)))
    print("\nPersonal space intrusions: " + str(personal_space_intrusions(agents, robots)) + "%")
    print("\nInteracion space intrusions: " + str(interaction_space_intrusions(agents, robots)) + "%")

if __name__ == "__main__":
    main()

# def compute_metrics(agents, robot):
#     if(tt_enable):
#         secs = total_time(agents, robot)
    
#     if(rpl_enable):
#         length = robot_path_length(agents, robot)

        