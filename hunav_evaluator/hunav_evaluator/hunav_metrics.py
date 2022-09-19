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

# Robot does not have header and stamp attributes
def total_time(agents, robot):
    t2 = rclpy.time.Time.from_msg(agents[len(agents)-1].header.stamp)
    t1 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    secs = (t2 - t1).to_msg().sec #.nanoseconds() #/1e9
    print('time_to_reach_goal computed: %.2f secs' % secs)
    return secs

def robot_path_length(agents, robot):
    path_length = 0
    for i in range(len(robot)-1):
        path_length += euclidean_distance(robot[i+1], robot[i])
    print('path_length computed: %.2f m' % path_length)
    return path_length


def cumulative_heading_changes(agents, robot):
    chc = 0
    for i in range(len(robot) - 1):
        norm = normalize_angle(robot[i].yaw - robot[i+1].yaw)
        if norm < 0.0:
            norm *= -1
        chc += norm

    print('cumulative_heading_changes: %.2f rads' % chc)
    return chc


def normalize_angle(ang):
    while (ang <= -math.pi): 
      ang += 2 * math.pi
    while (ang > math.pi):
      ang -= 2 * math.pi
    return ang

def avg_closest_person(agents, robot):
    avg_closest = 0
    robot_goals_num = len(robot[0].goals)

    # If initial_pose is needed add:
    #agent = closest_initial_pose(agents, robot[0])
    #avg_closest = avg_closest + euclidean_distance(robot[0].position, agent.position)

    for i in range(robot_goals_num):
        agent = closest_person(agents, robot[0], i)
        if i < len(agent.goals):
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
        if i < len(agent.goals):
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
        if i < len(agent.goals):
            max_distance.append(euclidean_distance(robot[0].goals[i], agent.goals[i]))
    
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

    n = 1/(len(robot[0].goals)) # + 1 in case of initial position
    space_intrusions = space_intrusions * n
    percentage = space_intrusions * 100
        
    return percentage, k

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

    n = 1/(len(robot[0].goals)) # +1 in case of initial position
    space_intrusions = space_intrusions * n
    percentage = space_intrusions * 100
        
    return percentage

# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez

def robot_on_person_collision(agents, robot):

    collision_count = 0
    
    for i in range(len(robot)):
        # If initial pose needed:
        agent = closest_initial_pose(agents, robot[i])
        if euclidean_distance(robot[i].position, agent.position) == 0:
            collision_count = collision_count + 1

        for j in range(len(robot[i].goals)):
            agent = closest_person(agents, robot[i], j)
            if j < len(agent.goals):
                if euclidean_distance(robot[i].goals[j], agent.goals[j]) == 0:
                    collision_count = collision_count + 1
    
    return collision_count


def person_on_robot_collision(agents, robot):
    
    collision_dict = dict()
    collision = 0

    for i in range(len(agents)):
        # If initial pose needed:
        if euclidean_distance(robot[0].position, agents[i].position) == 0:
            collision = collision + 1
            collision_dict[agents[i].id] = collision

        for j in range(len(agents[i].goals)):
            if j < len(robot[0].goals):
                if euclidean_distance(robot[0].goals[j], agents[i].goals[j]) == 0:
                    collision = collision + 1
                    collision_dict[agents[i].id] = collision
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

    robot.position.position.x = 0.0
    robot.position.position.y = 0.0
    robot.position.position.z = 1.250000
    
    pose_robot = Pose()
    pose_robot.position.x = 1.0
    pose_robot.position.y = 1.0
    pose_robot.position.z = 1.250000

    pose_robot_1 = Pose()
    pose_robot_1.position.x = 2.0
    pose_robot_1.position.y = 2.0
    pose_robot_1.position.z = 1.250000

    pose_robot_2 = Pose()
    pose_robot_2.position.x = 3.5
    pose_robot_2.position.y = 3.5
    pose_robot_2.position.z = 1.250000

    robot.goals.append(pose_robot)
    robot.goals.append(pose_robot_1)
    robot.goals.append(pose_robot_2)


    agent = Agent()
    agent.id = 0
    agent.type = Agent.PERSON
    agent.behavior_state = Agent.BEH_NO_ACTIVE
    agent.skin = 1
    agent.behavior = 3
    agent.group_id = -1
    agent.desired_velocity = 1.5
    agent.radius = 0.4
    agent.yaw = 2.2
    agent.position.position.x = 0.0
    agent.position.position.y = 0.0
    agent.position.position.z = 1.250000

    pose_agent = Pose()
    pose_agent.position.x = 5.6
    pose_agent.position.y = 3.1
    pose_agent.position.z = 1.250000
    

    agent.goals.append(pose_agent)

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
    agent1.position.position.x = 0.0
    agent1.position.position.y = 0.0
    agent1.position.position.z = 1.25000012

    pose_agent_1 = Pose()
    pose_agent_1.position.x = 3.0
    pose_agent_1.position.y = 4.0
    pose_agent_1.position.z = 1.250000

    pose_agent_1_1 = Pose()
    pose_agent_1_1.position.x = 2.0
    pose_agent_1_1.position.y = 2.4
    pose_agent_1_1.position.z = 1.250000

    pose_agent_1_2 = Pose()
    pose_agent_1_2.position.x = 3.5
    pose_agent_1_2.position.y = 3.5
    pose_agent_1_2.position.z = 1.250000

    agent1.goals.append(pose_agent_1)
    agent1.goals.append(pose_agent_1_1)
    agent1.goals.append(pose_agent_1_2)

    robots = list()
    robots.append(robot)

    agents = list()
    agents.append(agent)
    agents.append(agent1)

    # Metrics from Teaching Robot Navigation Behaviors to Optimal RRT Plannersx
    print("\nMetrics from # Teaching Robot Navigation Behaviors to Optimal RRT Planners\n")
    print("\nRobot path length: " + str(robot_path_length(agents, robots)))
    print("\nCumulative heading changes: " + str(cumulative_heading_changes(agents, robots)))
    print("\nAverage distance to closest person: " + str(avg_closest_person(agents, robots)))
    print("\nMinimun distance to closest person: " + str(min_distance_to_people(agents, robots)))
    print("\nMaximun distance to people: " + str(max_distance_to_people(agents, robots)))
    p_space = personal_space_intrusions(agents, robots)
    print("\nPersonal space intrusions: " + str(p_space[0]) + "%" + " k: " + str(p_space[1]))
    print("\nInteracion space intrusions: " + str(interaction_space_intrusions(agents, robots)) + "%")

    # Metrics from SEAN 2.0
    print("\n\nMetrics from # SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation\n")
    print("\nRobot on Person Collision: " + str(robot_on_person_collision(agents, robots)))
    print("\nPerson on Robot Collision:")
    person_collision = person_on_robot_collision(agents, robots)
    for key in person_collision:
        print("\n\tAgent: " + str(key) + " --> Collisions count: " + str(person_collision[key]))

if __name__ == "__main__":
    main()

# def compute_metrics(agents, robot):
#     if(tt_enable):
#         secs = total_time(agents, robot)
    
#     if(rpl_enable):
#         length = robot_path_length(agents, robot)

        