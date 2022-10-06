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


def get_group_center(agents_i, group_id, distance):
    
    group = []
    for agent in agents_i:
        if agent.group_id == group_id:
            pose = Pose()
            pose.position.x = agent.position.position.x + (distance * math.cos(agent.yaw))
            pose.position.y = agent.position.position.y + (distance * math.sin(agent.yaw))
            group.append(pose)

    interaction_center = Pose()    
    for p in group:
        interaction_center.position.x += p.position.x
        interaction_center.position.y += p.position.y
    
    interaction_center.position.x = float(interaction_center.position.x/len(group))
    interaction_center.position.y = float(interaction_center.position.y/len(group))
    return interaction_center



def indicator_function(norm, k):
    if k == 'intimate':
        if norm < 0.45:
            return 1
        else:
            return 0
    elif k == 'personal':
        if norm >= 0.45 and norm < 1.2:
            return 1
        else:
            return 0
    elif k == 'social':
        if norm >= 1.2 and norm < 3.6:
            return 1
        else:
            return 0
    elif k == 'public':
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
    print('Path_length computed: %.2f m' % path_length)
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

    avg_dist = avg_dist/len(robot)
    print('Average_closest_person: %.2f m' % avg_dist)
    return avg_dist


def minimum_distance_to_people(agents, robot):
    min_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agents:
            min_distance.append(euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius) 
    
    min_dist = min(min_distance)
    
    print('Minimum_distance_to_people: %.2f m' % min_dist)

    return min_dist

def maximum_distance_to_people(agents, robot):
    max_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agents:
            max_distance.append(euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius)
    
    max_dist = max(max_distance)
    
    print('Maximum_distance_to_people: %.2f m' % max_dist)

    return max_dist



def space_intrusions(agents, robot, k):
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

    space_intrusions = space_intrusions / len(robot)
    percentage = space_intrusions * 100.0

    return percentage


def intimate_space_intrusions(agents, robot):
    percentage =  space_intrusions(agents, robot, 'intimate')
    print('Intimate_space_intrusions: %.2f %% of the total time' % percentage)
    return percentage

def personal_space_intrusions(agents, robot):
    percentage =  space_intrusions(agents, robot, 'personal')
    print('Personal_space_intrusions: %.2f %% of the total time' % percentage)
    return percentage
    
def social_space_intrusions(agents, robot):
    percentage =  space_intrusions(agents, robot, 'social')
    print('Social_space_intrusions: %.2f %% of the total time' % percentage)
    return percentage



def detect_groups(agents):
    group_ids = []
    for a in agents[0].agents:
        if(a.group_id != -1 and ((a.group_id in group_ids) == False)):
            group_ids.append(a.group_id)

    return group_ids


def group_space_intrusions(agents, robot, k):
    group_ids = detect_groups(agents)
    if(len(group_ids)==0):
        return 0.0

    d=1.5
    space_intrusions = 0
    for i in range(len(robot)):
        min_dist = 10000
        for id in group_ids:
            group_center = get_group_center(agents[i].agents, id, d)
            dist = euclidean_distance(robot[i].position, group_center.position) - robot[i].radius
            if dist < min_dist:
                min_dist = dist
        indicator = indicator_function(min_dist, k)
        if indicator == 1:
            space_intrusions += 1

    space_intrusions = space_intrusions / len(robot)
    percentage = space_intrusions * 100.0

    return percentage


def group_intimate_space_intrusions(agents, robot):
    percentage =  group_space_intrusions(agents, robot, 'intimate')
    print('Group_intimate_space_intrusions: %.2f %% of the total time' % percentage)
    return percentage

def group_personal_space_intrusions(agents, robot):
    percentage =  group_space_intrusions(agents, robot, 'personal')
    print('Group_personal_space_intrusions: %.2f %% of the total time' % percentage)
    return percentage

def group_social_space_intrusions(agents, robot):
    percentage =  group_space_intrusions(agents, robot, 'social')
    print('Group_social_space_intrusions: %.2f %% of the total time' % percentage)
    return percentage           


# def interaction_space_intrusions(agents, robot):
#     # Choose k (intimate, personal, social, public)
#     k = "intimate"
#     # Choose group_id
#     group_id = -1
#     # Choose distance 
#     distance = 1.5

#     space_intrusions = 0
#     min_dist = 10000
#     for i in range(len(robot)):
#         people_group = get_group_center(agents, robot[i], i, group_id, distance)
#         d = euclidean_distance(robot[i].position, people_group)
#         if d < min_dist:
#             min_dist = d
#             indicator = indicator_function(min_dist, k)
#             if indicator == 1:
#                 space_intrusions += 1
        

#     n = 1/len(robot)

#     space_intrusions *= n
#     percentage = space_intrusions * 100

#     print('Interaction_space_intrusions: %.2f' % percentage + "% " + "K: %s" % k)
        
#     return percentage



# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez

# The metrics Robot on Person Personal Distance Violation, Person on Robot Personal Distance Violation, Intimate Distance Violation and
# Person on Robot Intimate Distance Violation have already been implemented in the Personal_space_intrusions function.
# Instead of returning the number of times, it returns a percentage of distance violation.

def collisions(agents, robot):
    robot_collision = 0
    person_collision = 0

    for i in range(len(robot)):
        for agent in agents[i].agents:

            if euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius < 0.05:
                # Robot's angle
                nrx = (robot[i].position.position.x - agent.position.position.x) * math.cos(agent.yaw) + (robot[i].position.position.y - agent.position.position.y) * math.sin(agent.yaw)
                nry = -(robot[i].position.position.x - agent.position.position.x) * math.sin(agent.yaw) + (robot[i].position.position.y - agent.position.position.y) * math.cos(agent.yaw)
                alpha = math.atan2(nry, nrx)

                # Agent's angle
                nrx = (agent.position.position.x - robot[i].position.position.x) * math.cos(robot[i].yaw) + (agent.position.position.y - robot[i].position.position.y) * math.sin(robot[i].yaw)
                nry = -(agent.position.position.x - robot[i].position.position.x) * math.sin(robot[i].yaw) + (agent.position.position.y - robot[i].position.position.y) * math.cos(robot[i].yaw)
                alpha2 = math.atan2(nrx, nry)

                if alpha < alpha2 and robot[i].linear_vel > agent.linear_vel:
                    robot_collision += 1
                elif alpha > alpha2 and robot[i].linear_vel < agent.linear_vel:
                    person_collision += 1
                elif alpha < alpha2 and robot[i].linear_vel < agent.linear_vel:
                    #person_collision += 1
                    robot_collision += 1
                elif alpha > alpha2 and robot[i].linear_vel > agent.linear_vel:
                    #robot_collision += 1
                    person_collision += 1
                elif alpha == alpha2 and robot[i].linear_vel == agent.linear_vel:
                    robot_collision += 1
                    person_collision += 1

    return robot_collision, person_collision

def robot_on_person_collision(agents, robot):

    collision = collisions(agents, robot)

    return collision[0]

def person_on_robot_collision(agents, robot):
    
    collision = collisions(agents, robot)

    return collision[1]

def time_not_moving(agents, robot):
    
    time_step = total_time(agents, robot)/len(agents)

    count = 0
    for r in robot:
        if(r.linear_vel < 0.001 and abs(r.angular_vel < 0.01)):
            count=count+1
    time_stopped = time_step*count
    print('Time stopped: %i secs' % time_stopped)
    return time_stopped


def goal_reached(agents, robot):
    if(len(robot[-1].goals)):
        for g in robot[-1].goals:
            d = euclidean_distance(robot[-1].position, g) - robot[-1].goal_radius
            if d<0.0:
                return True
    return False
   

def final_goal_distance(agents, robot):
    min_dist = 10000
    if(len(robot[-1].goals)):
        for g in robot[-1].goals:
            d = euclidean_distance(robot[-1].position, g)
            if d<min_dist:
                min_dist = d
        return d
    else:
        return min_dist


def minimum_goal_distance(agents, robot):
    min_dist = 10000
    for r in robot:
        if(len(r.goals)):
            for g in r.goals:
                d = euclidean_distance(r.position, g)
                if d<min_dist:
                    min_dist = d
    return min_dist

#ToDo
def path_irregularity(agents, robot):
    pass

#ToDo
def path_efficiency(agents, robot):
    pass


# Evaluation of Socially-Aware Robot Navigation
# Yuxiang Gao * and Chien-Ming Huang
# Department of Computer Science, The Johns Hopkins University, Baltimore, MD, United States

# TODO: 
# Average Displacement Error --> Trajectory needed
# Final Displacement Error --> Trajectory needed
# Asymmetric Dynamic Time Warping --> Trajectory needed
# Topological Complexity --> Path needed
# Path irregularity and Path efficiency are similar to the ones in SEAN paper .
# Personal space and o/p/r-space metrics are similar to the ones in Teaching Robot Navigation Behaviors to Optimal RRT Planners paper.
# Social Force Model (SFM)
# Extended Social Force Model.




metrics = {
    # N. Perez-Higueras, F. Caballero, and L. Merino, “Teaching Robot Nav-
    # igation Behaviors to Optimal RRT Planners,” International Journal of
    # Social Robotics, vol. 10, no. 2, pp. 235–249, 2018.
    'time_to_reach_goal': total_time,
    'path_length': robot_path_length,
    'cumulative_heading_changes': cumulative_heading_changes,
    'avg_distance_to_closest_person': avg_closest_person,
    'minimum_distance_to_people': minimum_distance_to_people,
    'maximum_distance_to_people': maximum_distance_to_people,
    'intimate_space_intrusions': intimate_space_intrusions,
    'personal_space_intrusions': personal_space_intrusions,
    'social_space_intrusions': social_space_intrusions,
    'group_intimate_space_intrusions': group_intimate_space_intrusions,
    'group_personal_space_intrusions': group_personal_space_intrusions,
    'group_social_space_intrusions': group_social_space_intrusions,
    # N. Tsoi, A. Xiang, P. Yu, S. S. Sohn, G. Schwartz, S. Ramesh,
    # M. Hussein, A. W. Gupta, M. Kapadia, and M. V ́azquez, “Sean 2.0:
    # Formalizing and generating social situations for robot navigation,”
    # IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 11 047–
    # 11 054, 2022
    #   - 'Total Path Length' (meters): similar to 'path_length'
    #   - 'Path Irregularity': (radians): total rotations in the robot's 
    #       traveled path greater than the total rotations in the search-based 
    #       path from the starting pose.
    #   - 'Path Efficiency': (meters): ratio between robot's traveled path and 
    #       geodesic distance of the search-based path from the starting pose.

    # true when the robot's final pose is within a specified distance of the goal. 
    # The final distance threshold is easily adjustable by the user, but defaults 
    # to 1.2m.
    'completed': goal_reached,
    #(meters): the closest the robot passes to the target position.
    'minimum_distance_to_target': minimum_goal_distance,  
    #(meters): distance between the last robot position and the target position.
    'final_distance_to_target': final_goal_distance, 
    #   - 'Robot on Person Personal Distance Violation': number of times a robot 
    # approaches a person within the personal distance of the robot.
    # Similar to 'personal_space_intrusions'
    #   - 'Person on Robot Personal Distance Violation': number of times a person 
    # approaches the robot within the personal distance of the robot.
    #   - 'Intimate Distance Violation': number of times the robot approached within 
    # the intimate distance of a person.
    #   - 'Person on Robot Intimate Distance Violation': number of times a person 
    # approaches the robot within the intimate distance of the robot.
    'robot_on_person_collision': robot_on_person_collision,
    'person_on_robot_collision': person_on_robot_collision,
    'time_not_moving': time_not_moving,
    # TODO:'static_obstacle_collision': static_obs_collision,
    # number of times the robot collides with a static obstacle.
}




