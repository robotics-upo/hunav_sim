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

# Robot does not have header and stamp attributes
def total_time(agents, robot):
    t2 = rclpy.time.Time.from_msg(agents[len(agents)-1].header.stamp)
    t1 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    secs = (t2 - t1).to_msg().sec #.nanoseconds() #/1e9
    print('\nTime_to_reach_goal computed: %.2f secs' % secs)
    return secs

def robot_path_length(agents, robot):
    path_length = 0
    for i in range(len(robot)-1):
        path_length += euclidean_distance(robot[i+1].position, robot[i].position)
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
            d = euclidean_distance(robot[i].position, agent.position)
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
            min_distance.append(euclidean_distance(robot[i].position, agent.position))
    
    min_dist = min(min_distance)
    
    print('Minimum_distance_to_people: %.2f' % min_dist)

    return min_dist

def maximum_distance_to_people(agents, robot):
    max_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agents:
            max_distance.append(euclidean_distance(robot[i].position, agent.position))
    
    max_dist = max(max_distance)
    
    print('Maximum_distance_to_people: %.2f' % max_dist)

    return max_dist

def personal_space_intrusions(agents, robot):
    # Choose k (intimate, personal, social, public)
    k = "social"
    space_intrusions = 0

    for i in range(len(robot)):
        min_dist = 10000
        
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position)
            if d < min_dist:
                indicator = indicator_function(d, k)
                if indicator == 1:
                    space_intrusions = space_intrusions + 1

    n = 1/(len(agents[0].agents))
    space_intrusions = space_intrusions * n
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
            indicator = indicator_function(d, k)
            if indicator == 1:
                space_intrusions = space_intrusions + 1

    n = 1/(len(robot)) 
    space_intrusions = space_intrusions * n
    percentage = space_intrusions * 100

    print('Interaction_space_intrusions: %.2f' % percentage + "% " + "K: %s" % k)
        
    return percentage

# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez

def robot_on_person_collision(agents, robot):

    collision_count = 0
    
    for i in range(len(robot)):
        for agent in agents[i].agents:
            if euclidean_distance(robot[i].position, agent.position) == 0:
                collision_count = collision_count + 1
    
    print('Robot_on_person_collision: %i' % collision_count)

    return collision_count


def person_on_robot_collision(agents, robot):
    
    collision_dict = dict()
    collision = 0

    for i in range(len(agents)):
        for agent in agents[i].agents:
            for r in robot:
                if euclidean_distance(r.position, agent.position) == 0:
                    collision = collision + 1
                    collision_dict[agent.id] = collision
            collision = 0

    return collision_dict

def main():

    robot = Agent()
    robot.id = 0
    robot.type = Agent.ROBOT
    robot.behavior_state = Agent.BEH_NO_ACTIVE
    robot.skin = 1
    robot.behavior = 3
    robot.group_id = 1
    robot.desired_velocity = 1.5
    robot.radius = 0.4
    robot.yaw = 3.2

    robot.position.position.x = 5.0
    robot.position.position.y = 6.0
    robot.position.position.z = 1.250000
    
    agent = Agent()
    agent.id = 0
    agent.type = Agent.PERSON
    agent.behavior_state = Agent.BEH_NO_ACTIVE
    agent.skin = 1
    agent.behavior = 3
    agent.group_id = -1
    agent.desired_velocity = 1.5
    agent.radius = 0.4
    agent.yaw = 7.0
    agent.position.position.x = 20.0
    agent.position.position.y = 6.0
    agent.position.position.z = 1.250000

    agent1 = Agent()
    agent1.id = 1
    agent1.type = Agent.PERSON
    agent1.behavior_state = Agent.BEH_NO_ACTIVE
    agent1.skin = 1
    agent1.behavior = 3
    agent1.group_id = -1
    agent1.desired_velocity = 1.5
    agent1.radius = 0.4
    agent1.yaw = 1.8
    agent1.position.position.x = 7.0
    agent1.position.position.y = 6.0
    agent1.position.position.z = 1.25000012

    robots = list()
    robots.append(robot)

    agents = Agents()
    agents.agents.append(agent)
    agents.agents.append(agent1)

    agents_list = list()
    agents_list.append(agents)


    #total_time(agents_list, robots)
    robot_path_length(agents_list, robots)
    cumulative_heading_changes(agents_list, robots)
    avg_closest_person(agents_list, robots)
    minimum_distance_to_people(agents_list, robots)
    maximum_distance_to_people(agents_list, robots)
    personal_space_intrusions(agents_list, robots)
    interaction_space_intrusions(agents_list, robots)
    robot_on_person_collision(agents_list, robots)
    
    person_collision = person_on_robot_collision(agents_list, robots)
    print('Person_on_robot_collision:')
    for key in person_collision:
        print("\tAgent: " + str(key) + " --> Collisions count: " + str(person_collision[key]))

if __name__ == "__main__":
    main()

# def compute_metrics(agents, robot):
#     if(tt_enable):
#         secs = total_time(agents, robot)
    
#     if(rpl_enable):
#         length = robot_path_length(agents, robot)

        