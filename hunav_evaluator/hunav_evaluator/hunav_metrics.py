import numpy as np
import math
import rclpy
from hunav_msgs.msg import Agents
from hunav_msgs.msg import Agent
from geometry_msgs.msg import Pose
from hunav_evaluator.sfm import SFM
from typing import List, Tuple
from angles import normalize_angle, shortest_angular_distance
from enum import Enum

# Teaching Robot Navigation Behaviors to Optimal RRT Planners
# Noé Pérez-Higueras, Fernando Caballero & Luis Merino


class SpaceType(Enum):
    INTIMATE = 0
    PERSONAL = 1
    SOCIAL = 2
    PUBLIC = 3


def indicator_function_space(norm: float, space_type: SpaceType) -> bool:
    """
    Determines if the distance norm falls within the specified space type.

    Args:
        norm (float): The distance norm to evaluate.
        space_type (SpaceType): The type of space to check against.

    Returns:
        bool: True if the norm is within the specified space type, False otherwise.
    """
    match space_type:
        case SpaceType.INTIMATE:
            return norm < 0.45
        case SpaceType.PERSONAL:
            return 0.45 <= norm < 1.2
        case SpaceType.SOCIAL:
            return 1.2 <= norm < 3.6
        case SpaceType.PUBLIC:
            return norm >= 3.6
    return False


def euclidean_distance(pose: Pose, pose1: Pose):
    return math.sqrt(
        (pose.position.x - pose1.position.x) ** 2
        + (pose.position.y - pose1.position.y) ** 2
    )


def agent_distance(agent: Agent, agent1: Agent):
    return (
        euclidean_distance(agent.position, agent1.position)
        - agent.radius
        - agent1.radius
    )


def get_group_center(agents_i: List[Agent], group_id: int, distance: float) -> Pose:
    """
    Computes the center of a group of agents at a specified distance from their current positions.
    Args:
        agents_i (List[Agent]): List of agents in the current time step.
        group_id (int): The group ID to filter agents.
        distance (float): The distance from the agent's position to compute the center.
    Returns:
        Pose: The computed center pose of the group at the specified distance.
    """
    if len(agents_i) == 0:
        raise ValueError("No agents found in the provided list.")
    group = []
    for agent in agents_i:
        if agent.group_id == group_id:
            pose = Pose()
            pose.position.x = agent.position.position.x + (
                distance * math.cos(agent.yaw)
            )
            pose.position.y = agent.position.position.y + (
                distance * math.sin(agent.yaw)
            )
            group.append(pose)

    interaction_center = Pose()
    for p in group:
        interaction_center.position.x += p.position.x
        interaction_center.position.y += p.position.y

    interaction_center.position.x = float(interaction_center.position.x / len(group))
    interaction_center.position.y = float(interaction_center.position.y / len(group))
    return interaction_center


def get_time_stamps(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Extracts the time stamps from the agents and computes the time difference
    from the first agent's time stamp.
    Args:
        agents (List[Agents]): List of Agents messages containing the time stamps.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list of time differences in seconds from the first agent's time stamp.
    """
    time_list = []
    t0 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    for a in agents:
        t = rclpy.time.Time.from_msg(a.header.stamp)
        dur = (t - t0).to_msg()
        s = float(dur.sec + dur.nanosec / 1e9)
        time_list.append(s)
    return time_list


def total_time(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Computes the total time taken to reach the goal by the robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the time stamps.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the total time in seconds.
    """
    t2 = rclpy.time.Time.from_msg(agents[len(agents) - 1].header.stamp)
    t1 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    dur = t2 - t1
    secs = dur.nanoseconds / 1e9
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Time_to_reach_goal computed: {secs:.2f} secs"
    )
    return [secs]


def robot_path_length(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Computes the total path length traveled by the robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the robot's state.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the total path length in meters.
    """
    path_length = 0.0
    for i in range(len(robot) - 1):
        path_length += euclidean_distance(robot[i + 1].position, robot[i].position)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Path_length computed: {path_length:.2f} m"
    )
    return [path_length]


def cumulative_heading_changes(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Computes the cumulative heading changes of the robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the robot's state.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the cumulative heading changes in radians.
    """
    chc_list = [0.0]
    chc = 0
    for i in range(len(robot) - 1):
        norm = abs(shortest_angular_distance(robot[i].yaw, robot[i + 1].yaw))
        chc += norm
        chc_list.append(norm)

    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Cumulative_heading_changes: {chc:.2f} rads"
    )
    return [chc, chc_list]


def avg_closest_person(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Computes the average distance to the closest person for each robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the average distance to the closest person
        and a list of minimum distances for each robot.
    """
    min_dist_list = []
    avg_dist = 0
    for i in range(len(robot)):
        min_dist = np.inf
        for agent in agents[i].agents:
            d = agent_distance(robot[i], agent)
            min_dist = max(min(min_dist, d), 0.0)  # Ensure non-negative distance
        if len(agents[i].agents) > 0:
            avg_dist += min_dist
            min_dist_list.append(min_dist)

    avg_dist = avg_dist / len(robot)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Average_closest_person: {avg_dist:.2f} m"
    )
    return [avg_dist, min_dist_list]


def minimum_distance_to_people(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Computes the minimum distance to any person for each robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the minimum distance to any person for each robot.
    """
    min_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agents:
            d = agent_distance(robot[i], agent)
            min_distance.append(max(d, 0.0))  # Ensure non-negative distance
    min_dist = min(min_distance)

    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Minimum_distance_to_people: {min_dist:.2f} m"
    )
    return [min_dist]


def maximum_distance_to_people(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Computes the maximum distance to any person for each robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the maximum distance to any person for each robot.
    """
    max_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agents:
            max_distance.append(
                euclidean_distance(robot[i].position, agent.position)
                - robot[i].radius
                - agent.radius
            )

    max_dist = max(max_distance)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Maximum_distance_to_people: {max_dist:.2f} m"
    )

    return [max_dist]


def space_intrusions(
    agents: List[Agents], robot: List[Agent], k: SpaceType
) -> Tuple[float, List[int]]:
    """
    This function calculates the percentage of time the robot intrudes into
    the intimate, personal, or social space of other agents.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
        k (SpaceType): The type of space to check for intrusions (intimate, personal, social).
    Returns:
        Tuple[float, List[int]]: A tuple containing the percentage of space intrusions
        and a list indicating which robots had intrusions (1 for intrusion, 0 for no intrusion).
    """

    space_intrusions = 0
    space_intrusions_list = [0] * len(robot)

    for i in range(len(robot)):
        min_dist = np.inf
        for agent in agents[i].agents:
            d = agent_distance(robot[i], agent)
            min_dist = max(min(min_dist, d), 0.0)  # Ensure non-negative distance
        indicator = indicator_function_space(min_dist, k)
        if indicator:
            space_intrusions += 1
            space_intrusions_list[i] = 1

    space_intrusions = space_intrusions / len(space_intrusions_list)
    percentage = space_intrusions * 100.0

    return percentage, space_intrusions_list


def intimate_space_intrusions(
    agents: List[Agents], robot: List[Agent]
) -> Tuple[float, List[int]]:
    """
    Computes the percentage of time the robot intrudes into the intimate space of other agents.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the percentage of intimate space intrusions
        and a list indicating which robots had intrusions (1 for intrusion, 0 for no intrusion).
    """
    percentage, slist = space_intrusions(agents, robot, SpaceType.INTIMATE)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Intimate_space_intrusions: {percentage:.2f} % of the total time"
    )
    return [percentage, slist]


def personal_space_intrusions(
    agents: List[Agents], robot: List[Agent]
) -> Tuple[float, List[int]]:
    """
    Computes the percentage of time the robot intrudes into the personal space of other agents.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the percentage of personal space intrusions
        and a list indicating which robots had intrusions (1 for intrusion, 0 for no intrusion).
    """
    percentage, slist = space_intrusions(agents, robot, SpaceType.PERSONAL)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Personal_space_intrusions: {percentage:.2f} % of the total time"
    )
    return [percentage, slist]


def social_space_intrusions(
    agents: List[Agents], robot: List[Agent]
) -> Tuple[float, List[int]]:
    percentage, slist = space_intrusions(agents, robot, SpaceType.SOCIAL)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Social_space_intrusions: {percentage:.2f} % of the total time"
    )
    return [percentage, slist]


def detect_groups(agents: List[Agents]) -> List[int]:
    """
    Detects unique group IDs from the agents and returns a list of those IDs.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
    Returns:
        List[int]: A list of unique group IDs found in the agents.
    """
    group_ids = []
    for a in agents[0].agents:
        if a.group_id != -1 and ((a.group_id in group_ids) == False):
            group_ids.append(a.group_id)

    return group_ids


def group_space_intrusions(
    agents: List[Agents], robot: List[Agent], k: SpaceType
) -> Tuple[float, List[int]]:
    """
    Computes the percentage of time the robot intrudes into the intimate, personal, or social space
    of groups of agents.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
        k (SpaceType): The type of space to check for intrusions (intimate, personal, social).
    Returns:
        Tuple[float, List[int]]: A tuple containing the percentage of space intrusions
        and a list indicating which robots had intrusions (1 for intrusion, 0 for no intrusion).
    """
    group_ids = detect_groups(agents)
    if len(group_ids) == 0:
        return [0.0]

    d = 1.5
    space_intrusions = 0
    group_list = [0] * len(robot)
    for i in range(len(robot)):
        min_dist = np.inf
        for id in group_ids:
            group_center = get_group_center(agents[i].agents, id, d)
            dist = (
                euclidean_distance(robot[i].position, group_center)
                - robot[i].radius
            )
            min_dist = max(min(min_dist, dist), 0.0)  # Ensure non-negative distance
        indicator = indicator_function_space(min_dist, k)
        if indicator:
            space_intrusions += 1
            group_list[i] = 1

    space_intrusions = space_intrusions / len(group_list)
    percentage = space_intrusions * 100.0

    return [percentage, group_list]


def group_intimate_space_intrusions(
    agents: List[Agents], robot: List[Agent]
) -> List[float]:
    """Computes the percentage of time the robot intrudes into the intimate space of groups of agents.

    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the percentage of intimate space intrusions
        and a list indicating which robots had intrusions (1 for intrusion, 0 for no intrusion).
    """
    r = group_space_intrusions(agents, robot, SpaceType.INTIMATE)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Group_intimate_space_intrusions: {r[0]:.2f} % of the total time"
    )
    return r


def group_personal_space_intrusions(
    agents: List[Agents], robot: List[Agent]
) -> List[float]:
    """Computes the percentage of time the robot intrudes into the personal space of groups of agents.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the percentage of personal space intrusions
        and a list indicating which robots had intrusions (1 for intrusion, 0 for no intrusion).
    """
    r = group_space_intrusions(agents, robot, SpaceType.PERSONAL)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Group_personal_space_intrusions: {r[0]:.2f} % of the total time"
    )
    return r


def group_social_space_intrusions(
    agents: List[Agents], robot: List[Agent]
) -> List[float]:
    """Computes the percentage of time the robot intrudes into the social space of groups of agents.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the percentage of social space intrusions
        and a list indicating which robots had intrusions (1 for intrusion, 0 for no intrusion).
    """
    r = group_space_intrusions(agents, robot, SpaceType.SOCIAL)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Group_social_space_intrusions: {r[0]:.2f} % of the total time"
    )
    return r


# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez

# The metrics Robot on Person Personal Distance Violation, Person on Robot Personal Distance Violation, Intimate Distance Violation and
# Person on Robot Intimate Distance Violation have already been implemented in the Personal_space_intrusions function.
# Instead of returning the number of times, it returns a percentage of distance violation.


def collisions(agents: List[Agents], robot: List[Agent]) -> List[int]:
    """Computes the number of collisions between robots and persons based on their positions and velocities.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the number of robot collisions, person collisions,
        and lists indicating which robots and persons had collisions (1 for collision, 0 for no collision).
    """
    robot_coll_list = [0] * len(robot)
    person_coll_list = [0] * len(robot)
    robot_collisions = 0
    person_collisions = 0

    for i in range(len(robot)):
        for agent in agents[i].agents:

            if (
                euclidean_distance(robot[i].position, agent.position)
                - robot[i].radius
                - agent.radius
                < 0.02
            ):

                # Robot's angle
                nrx = (
                    robot[i].position.position.x - agent.position.position.x
                ) * math.cos(agent.yaw) + (
                    robot[i].position.position.y - agent.position.position.y
                ) * math.sin(
                    agent.yaw
                )
                nry = -(
                    robot[i].position.position.x - agent.position.position.x
                ) * math.sin(agent.yaw) + (
                    robot[i].position.position.y - agent.position.position.y
                ) * math.cos(
                    agent.yaw
                )
                alpha = math.atan2(nry, nrx)

                # Agent's angle
                nrx = (
                    agent.position.position.x - robot[i].position.position.x
                ) * math.cos(robot[i].yaw) + (
                    agent.position.position.y - robot[i].position.position.y
                ) * math.sin(
                    robot[i].yaw
                )
                nry = -(
                    agent.position.position.x - robot[i].position.position.x
                ) * math.sin(robot[i].yaw) + (
                    agent.position.position.y - robot[i].position.position.y
                ) * math.cos(
                    robot[i].yaw
                )
                alpha2 = math.atan2(nrx, nry)

                if abs(alpha) < abs(alpha2) and robot[i].linear_vel > agent.linear_vel:
                    robot_collisions += 1
                    robot_coll_list[i] = 1
                elif (
                    abs(alpha) > abs(alpha2) and robot[i].linear_vel < agent.linear_vel
                ):
                    person_collisions += 1
                    person_coll_list[i] = 1
                elif (
                    abs(alpha) < abs(alpha2) and robot[i].linear_vel < agent.linear_vel
                ):
                    # person_collision += 1
                    robot_collisions += 1
                    robot_coll_list[i] = 1
                elif (
                    abs(alpha) > abs(alpha2) and robot[i].linear_vel > agent.linear_vel
                ):
                    # robot_collision += 1
                    person_collisions += 1
                    person_coll_list[i] = 1
                elif (
                    abs(alpha) == abs(alpha2)
                    and robot[i].linear_vel == agent.linear_vel
                ):
                    robot_collisions += 1
                    person_collisions += 1
                    robot_coll_list[i] = 1
                    person_coll_list[i] = 1

    return robot_collisions, person_collisions, robot_coll_list, person_coll_list


def robot_on_person_collision(agents: List[Agents], robot: List[Agent]) -> List[int]:
    """Computes the number of collisions between the robot and persons.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the number of robot collisions with persons and a list indicating which robots had collisions (1 for collision, 0 for no collision).
    """
    collision = collisions(agents, robot)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Robot_on_person_collision: {collision[0]} times"
    )
    return [collision[0], collision[2]]


def person_on_robot_collision(agents: List[Agents], robot: List[Agent]) -> List[int]:
    """Computes the number of collisions between persons and the robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[int]: A list containing the number of person collisions with the robot and a list indicating which robots had collisions (1 for collision, 0 for no collision).
    """

    collision = collisions(agents, robot)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Person_on_robot_collision: {collision[1]} times"
    )
    return [collision[1], collision[3]]


def time_not_moving(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the total time the robot is not moving based on its linear and angular velocities.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the total time the robot is not moving in seconds and a list indicating which robots are not moving (1 for not moving, 0 for moving).
    """

    not_moving = [0] * len(robot)
    time_step = total_time(agents, robot)[0] / len(agents)

    count = 0
    for index, r in enumerate(robot):
        if r.linear_vel < 0.01 and abs(r.angular_vel < 0.02):
            count = count + 1
            not_moving[index] = 1
    time_stopped = time_step * count
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Time_not_moving: {time_stopped:.2f} secs"
    )
    return [time_stopped, not_moving]


def goal_reached(agents: List[Agents], robot: List[Agent]) -> List[bool]:
    """Checks if the robot has reached its goal based on the distance to the goal.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
        Returns:
        List[bool]: A list containing a boolean indicating whether the robot has reached its goal.
    """
    mind = 0.3
    if len(robot[-1].goals):
        for g in robot[-1].goals:
            dist = euclidean_distance(robot[-1].position, g) - robot[-1].goal_radius
            if dist < mind:
                return [True]
    return [False]


def final_goal_distance(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Computes the distance from the robot's final position to its goal.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the minimum distance to the goal for the robot.
    """
    min_dist = np.inf
    if len(robot[-1].goals):
        for g in robot[-1].goals:
            dist = euclidean_distance(robot[-1].position, g)
            min_dist = max(min(min_dist, dist), 0.0)
    return [min_dist]


def minimum_goal_distance(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the minimum distance from the robot's position to any of its goals.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the minimum distance to any goal for the robot.
    """
    min_dist = np.inf
    for r in robot:
        if len(r.goals):
            for g in r.goals:
                dist = euclidean_distance(r.position, g)
                min_dist = max(min(min_dist, dist), 0.0)  # Ensure non-negative distance
    return [min_dist]


# SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
# ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA, AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University


def avg_robot_linear_speed(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the average robot linear speed based on the robot's linear velocities.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the average robot speed and a list of individual speeds.
    """
    speed_list = []
    speed = 0
    for r in robot:
        speed_list.append(r.linear_vel)
        speed += r.linear_vel

    speed = speed / len(robot)

    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Average_robot_speed: {speed:.2f} m/s"
    )
    return [speed, speed_list]


def avg_robot_angular_speed(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the average robot angular speed based on the robot's angular velocities.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the average robot angular speed and a list of individual angular speeds.
    """
    speed_list = []
    speed = 0
    for r in robot:
        speed_list.append(r.angular_vel)
        speed += np.abs(r.angular_vel)

    speed = speed / len(robot)

    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Average_robot_angular_speed: {speed:.2f} rad/s"
    )
    return [speed, speed_list]


def avg_acceleration(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the average robot acceleration based on the robot's linear velocities.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the average robot acceleration and a list of individual acceleration values.
    """
    acceleration = 0
    acceleration_list = [0.0]
    for i in range(len(robot) - 1):
        dv = robot[i + 1].linear_vel - robot[i].linear_vel
        tf = rclpy.time.Time.from_msg(agents[i + 1].header.stamp)
        ti = rclpy.time.Time.from_msg(agents[i].header.stamp)
        dt = (tf - ti).nanoseconds / 1e9
        if dt != 0.0:
            accel = dv / dt
            acceleration += np.abs(accel)
            acceleration_list.append(accel)
        else:
            acceleration_list.append(0.0)

    acceleration = acceleration / len(robot)

    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Average_robot_acceleration: {acceleration:.5f} m/s^2"
    )
    return [acceleration, acceleration_list]


def avg_overacceleration(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the average robot jerk (over-acceleration) based on the robot's linear velocities.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the average robot jerk (over-acceleration) and a list of individual jerk values.
    """
    jerk = 0
    jerk_list = [0.0]
    for i in range(len(robot) - 1):
        dv = robot[i + 1].linear_vel - robot[i].linear_vel
        tf = rclpy.time.Time.from_msg(agents[i + 1].header.stamp)
        ti = rclpy.time.Time.from_msg(agents[i].header.stamp)
        dur = (tf - ti).to_msg()
        dt = float(dur.sec + dur.nanosec / 1e9)
        if dt != 0.0:
            acceleration = dv / dt
            jerk += np.abs(acceleration / dt)
            jerk_list.append(acceleration / dt)
        else:
            jerk_list.append(0.0)

    jerk = jerk / len(robot)

    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Average_robot_jerk(over_acceleration): {jerk:.5f} m/s^3"
    )
    return [jerk, jerk_list]


# Learning a Group-Aware Policy for Robot Navigation
# Kapil Katyal ∗1,2 , Yuxiang Gao ∗2 , Jared Markowitz 1 , Sara Pohland 3 , Corban Rivera 1 , I-Jeng Wang 1 , Chien-Ming Huang 2


def avg_pedestrian_velocity(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the average speed of pedestrians in the environment.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the average pedestrian speed and a list of individual pedestrian speeds.
    """
    speed = 0
    speed_list = []
    for i in range(len(agents)):
        speed2 = 0.0
        for agent in agents[i].agents:
            speed += agent.linear_vel
            speed2 += agent.linear_vel
        if len(agents[i].agents) > 0:
            speed_list.append(speed2 / len(agents[i].agents))
        else:
            speed_list.append(0.0)

    speed = speed / (len(agents) * len(agents[0].agents))

    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Average_Pedestrian_speed: {speed:.2f} m/s"
    )
    return [speed, speed_list]


def avg_closest_pedestrian_velocity(agents, robot):
    speed = 0
    speed_list = []
    for i in range(len(robot)):
        min_dist = 10000
        closest = Agent()
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position)
            if d < min_dist:
                min_dist = d
                closest = agent
                if min_dist < 0.0:
                    min_dist = 0.0

        speed += closest.linear_vel
        speed_list.append(closest.linear_vel)
    if len(robot) > 0:
        speed = speed / len(robot)
    rclpy.logging.get_logger("hunav_evaluator").debug(
        f"Average_Closest_Pedestrian_speed: {speed:.2f} m/s"
    )
    return [speed, speed_list]


def avg_pedestrian_angle(agents, robot):
    pass


# metrics based on social force model


# cumulative modulus of the social force provoked
# by the robot in the agents
def social_force_on_agents(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the cumulative modulus of the social force provoked by the robot in the agents.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the cumulative social force and a list of individual social forces for each robot.
    """
    sfm = SFM()
    sf = 0.0
    sf_list = []
    for agts, rb in zip(agents, robot):
        f = sfm.modulusSocialForce2(rb, agts)
        sf += f
        sf_list.append(f)
    return [sf, sf_list]


# cumulative modulus of the social force provoked
# by the agents in the robot
def social_force_on_robot(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the cumulative modulus of the social force provoked by the agents in the robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the cumulative social force and a list of individual social forces for each robot.
    """
    sfm = SFM()
    sf = 0.0
    sf_list = []
    for agts, rb in zip(agents, robot):
        f = sfm.modulusSocialForce(rb, agts)
        sf += f
        sf_list.append(f)
    return [sf, sf_list]


# cumulative social work employed in this planner:
# https://github.com/robotics-upo/social_force_window_planner
def social_work(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the cumulative social work employed in the planner.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the cumulative social work and a list of individual social work values for each robot.
    """
    sfm = SFM()
    sw = 0.0
    sw_list = []
    for agts, rb in zip(agents, robot):
        f = sfm.computeSocialWork(rb, agts)
        sw += f
        sw_list.append(f)
    return [sw, sw_list]


# cumulative obstacle force on the agents
def obstacle_force_on_agents(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """Computes the cumulative obstacle force on the agents.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the cumulative obstacle force on the agents
        and a list of individual obstacle forces for each agent.
    """
    sfm = SFM()
    of = 0.0
    of_list = []
    for agts in agents:
        avg = 0.0
        for a in agts.agents:
            avg += np.linalg.norm(sfm.computeObstacleForce(a))
        avg = avg / len(agts.agents)
        of += avg
        of_list.append(avg)
    return [of, of_list]


# cumulative obstacle force on the robot
def obstacle_force_on_robot(agents: List[Agents], robot: List[Agent]) -> List[float]:
    """
    Computes the cumulative obstacle force on the robot.
    Args:
        agents (List[Agents]): List of Agents messages containing the agents' states.
        robot (List[Agent]): List of Agent messages representing the robot's state.
    Returns:
        List[float]: A list containing the cumulative obstacle force on the robot
        and a list of individual obstacle forces for each robot.
    """
    sfm = SFM()
    of = 0.0
    of_list = []
    for r in robot:
        f = np.linalg.norm(sfm.computeObstacleForce(r))
        of += f
        of_list.append(f)
    return [of, of_list]



# P. T. Singamaneni, A. Favier, and R. Alami, “Towards benchmarking
# human-aware social robot navigation: A new perspective and metrics,”
# in 2023 32nd IEEE International Conference on Robot and Human
# Interactive Communication (RO-MAN), 2023, pp. 914–921.
# Danger costs: Fear and Panic
# Surprise cost: Visibility, Shock, React
def danger_fear_cost(agents: List[Agents], robot: List[Agent]) -> List[int]:
    fc = 0.0
    fc_list = np.zeros(len(agents[0].agents))
    for agts, rb in zip(agents, robot):
        robot_pos = np.array([rb.position.position.x, rb.position.position.y])
        #robot_vel = rb.linear_vel * np.array([math.cos(rb.yaw), math.sin(rb.yaw)])
        robot_vel = np.array([rb.velocity.linear.x, rb.velocity.linear.y])
        rr=rb.radius
        for idx, agent in enumerate(agts.agents):
            human_pos = np.array([agent.position.position.x, agent.position.position.y])
            human_vel = np.array([agent.velocity.linear.x, agent.velocity.linear.y])
            f = cost_fear(robot_pos, robot_vel, human_pos, human_vel, rh=agent.radius, rr=rr)
            fc += f
            fc_list[idx]+=f

    return [fc, fc_list.tolist()]
    

def cost_fear(robot_pos, robot_vel, human_pos, human_vel, rh=0.3, rr=0.3):
    """
    Calcula el costo de miedo (Cost_fear) según la ecuación (1) del paper.
    
    Parámetros:
        robot_pos : np.array (2,)
            Posición [x, y] del robot
        robot_vel : np.array (2,)
            Velocidad [vx, vy] del robot
        human_pos : np.array (2,)
            Posición [x, y] del humano
        human_vel : np.array (2,)
            Velocidad [vx, vy] del humano
        rh : float
            Radio circunscrito del humano
        rr : float
            Radio circunscrito del robot
    
    Retorna:
        float : valor del costo de miedo (>= 0)
    """
    R = rh + rr
    Prh = human_pos - robot_pos
    Vrel = robot_vel - human_vel
    drheff = np.linalg.norm(Prh)
    
    PV = np.dot(Prh, Vrel)
    
    if PV > 0 and drheff > 0:
        return np.linalg.norm(Vrel) / drheff
    else:
        return 0.0


def danger_panic_cost(agents: List[Agents], robot: List[Agent]) -> List[int]:
    pc = 0.0
    pc_list = np.zeros(len(agents[0].agents))
    for agts, rb in zip(agents, robot):
        robot_pos = np.array([rb.position.position.x, rb.position.position.y])
        #robot_vel = rb.linear_vel * np.array([math.cos(rb.yaw), math.sin(rb.yaw)])
        robot_vel = np.array([rb.velocity.linear.x, rb.velocity.linear.y])
        rr=rb.radius
        for idx, agent in enumerate(agts.agents):
            human_pos = np.array([agent.position.position.x, agent.position.position.y])
            human_vel = np.array([agent.velocity.linear.x, agent.velocity.linear.y])
            p = cost_panic(robot_pos, robot_vel, human_pos, human_vel, rh=agent.radius, rr=rr)
            pc += p
            pc_list[idx]+=p

    return [pc, pc_list.tolist()]


def cost_panic(robot_pos, robot_vel, human_pos, human_vel, rh=0.3, rr=0.3):
    """
    Calcula el costo de pánico (Cost_panic) según la ecuación (2) del paper.
    
    Parámetros:
        robot_pos : np.array (2,)
            Posición [x, y] del robot
        robot_vel : np.array (2,)
            Velocidad [vx, vy] del robot
        human_pos : np.array (2,)
            Posición [x, y] del humano
        human_vel : np.array (2,)
            Velocidad [vx, vy] del humano
        rh : float
            Radio circunscrito del humano
        rr : float
            Radio circunscrito del robot
    
    Retorna:
        float : valor del costo de pánico (>= 0)
    """
    R = rh + rr
    Prh = human_pos - robot_pos
    Vrel = robot_vel - human_vel
    
    PV = np.dot(Prh, Vrel)
    if PV <= 0:
        return 0.0
    
    norm_Prh = np.linalg.norm(Prh)
    norm_Vrel = np.linalg.norm(Vrel)
    if norm_Prh == 0 or norm_Vrel == 0:
        return 0.0
    
    cos_theta = np.dot(Prh, Vrel) / (norm_Prh * norm_Vrel)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)  # Evitar errores numéricos
    theta = np.arccos(cos_theta)
    
    d_perp = norm_Prh * np.sin(theta)
    
    if d_perp > R and np.abs(np.sin(theta)) > 1e-6:
        return norm_Vrel / ((d_perp - R) * np.abs(np.sin(theta)))
    else:
        return 0.0



# ---------- World ↔ Grid coordinate transforms ----------
def world_to_grid(world_pos, occ_grid):
    """
    Convert world coordinates (meters) to grid indices (ints).
    
    Parameters:
        world_pos : (x, y) in meters
        occ_grid : nav_msgs/OccupancyGrid
    
    Returns:
        (i, j) tuple in grid coordinates
    """
    x, y = world_pos
    res = occ_grid.info.resolution
    x0 = occ_grid.info.origin.position.x
    y0 = occ_grid.info.origin.position.y
    
    i = int((x - x0) / res)
    j = int((y - y0) / res)
    return (i, j)


def grid_to_world(grid_idx, occ_grid):
    """
    Convert grid indices to world coordinates (meters).
    """
    i, j = grid_idx
    res = occ_grid.info.resolution
    x0 = occ_grid.info.origin.position.x
    y0 = occ_grid.info.origin.position.y
    
    x = i * res + x0 + res / 2.0
    y = j * res + y0 + res / 2.0
    return (x, y)

# ---------- Line of sight check (Bresenham) ----------
def line_of_sight(human_pos, robot_pos, occupancy_grid):
    """
    Check if the robot is visible from the human using Bresenham ray tracing.
    
    Parameters:
        human_pos : tuple (x, y) in grid coordinates (ints)
        robot_pos : tuple (x, y) in grid coordinates (ints)
        occupancy_grid : 2D np.array (0=free, 1=obstacle)
    
    Returns:
        bool : True if LoS is free, False if occluded
    """
    x0, y0 = human_pos
    x1, y1 = robot_pos

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        if occupancy_grid[y, x] == 1:
            return False  # obstacle blocks LoS
        if (x, y) == (x1, y1):
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return True


def is_visible(robot_pos, human_pos, human_dir, occ_grid):
    # --- check FoV ---
    vis_angle = cost_visibility(robot_pos, human_pos, human_dir)
    if vis_angle == 0.0:
        return False  # outside FoV
    # --- check LoS ---
    # --- check line of sight ---
    human_grid = world_to_grid(human_pos, occ_grid)
    robot_grid = world_to_grid(robot_pos, occ_grid)
    
    # Convert OccupancyGrid to numpy array for line_of_sight check
    width = occ_grid.info.width
    height = occ_grid.info.height
    data = np.array(occ_grid.data).reshape((height, width))
    # Convert to binary: 0=free, 1=occupied (treat unknown as occupied)
    grid_np = np.where(data < 0, 1, np.where(data > 50, 1, 0))
    
    if not line_of_sight(human_grid, robot_grid, grid_np):
        return False  # occluded
    return True  # visible


def cost_visibility(robot_pos, human_pos, human_dir,
                    theta_fov=120.0, d_proxemics=0.45):
    """
    Cost of visibility (eq. 3 in the paper).
    
    Parameters:
        robot_pos : np.array (2,) -> [x_r, y_r]
        human_pos : np.array (2,) -> [x_h, y_h]
        human_dir : np.array (2,) -> unit vector of human orientation
        theta_fov : float -> human FoV angle in degrees
        d_proxemics : float -> proxemics distance (>0.45m)
    """
    Prh = robot_pos - human_pos
    dhreff = np.linalg.norm(Prh)
    if dhreff <= 0:
        return 0.0
    
    u_Prh = Prh / dhreff
    cos_theta = np.dot(human_dir / np.linalg.norm(human_dir), u_Prh)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)  # rad
    
    if theta > np.deg2rad(theta_fov / 2.0):
        return 0.0  # outside FoV
    
    alpha = d_proxemics / (np.deg2rad(theta_fov) / 2.0)
    return alpha * (theta / dhreff)

def surprise_visibility_cost(agents: List[Agents], robot: List[Agent], grid) -> List[int]:
    vc = 0.0
    vc_list = np.zeros(len(agents[0].agents))
    for agts, rb in zip(agents, robot):
        robot_pos = np.array([rb.position.position.x, rb.position.position.y])
        for idx, agent in enumerate(agts.agents):
            human_pos = np.array([agent.position.position.x, agent.position.position.y])
            human_dir = np.array([math.cos(agent.yaw), math.sin(agent.yaw)])
            if is_visible(robot_pos, human_pos, human_dir, grid):
                v = cost_visibility(robot_pos, human_pos, human_dir)
            else:
                v = 0.0
            vc += v
            vc_list[idx]+=v

    return [vc, vc_list.tolist()]



def cost_shock(robot_pos, human_pos, t,
               trecognise=0.150, treact=0.600, d_proxemics=0.45):
    """"
    Parameters:
        robot_pos : np.array (2,)
            Position [x, y] of the robot
        human_pos : np.array (2,)
            Position [x, y] of the human
        dhreff : float
            Effective distance between human and robot
        t : float
            Timeo since the robot entered the human's FoV (seconds)
        trecognise : float
            Recognition time (~0.150 s)
        treact : float
            Reaction time (~0.600 s)
        d_proxemics : float
            Distancia proxemics distance (>0.45m)
    Return:
        float : shock cost
    """
    dhreff = np.linalg.norm(robot_pos - human_pos)
    if dhreff <= 0:
        return 0.0
    gamma = treact / trecognise
    SR = t / treact if 0 < t < treact else 1.0
    return max(d_proxemics / dhreff * (1 - gamma * SR), 0)


def surprise_shock_cost(agents: List[Agents], robot: List[Agent], grid) -> List[int]:
    sc = 0.0
    t1 = rclpy.time.Time.from_msg(agents[1].header.stamp)
    t2 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    dt = (t2 - t1).nanoseconds / 1e9  # Diferencia en segundos (float)
    sc_list = np.zeros(len(agents[0].agents))
    time_visible = np.zeros(len(agents[0].agents))
    for agts, rb in zip(agents, robot):
        robot_pos = np.array([rb.position.position.x, rb.position.position.y])
        for idx, agent in enumerate(agts.agents):
            human_pos = np.array([agent.position.position.x, agent.position.position.y])
            human_dir = np.array([math.cos(agent.yaw), math.sin(agent.yaw)])
            if is_visible(robot_pos, human_pos, human_dir, grid):
                time_visible[idx] += dt  
                s = cost_shock(robot_pos, human_pos, t=time_visible[idx])
            else:
                s = 0.0
                time_visible[idx] = 0.0
            sc += s
            sc_list[idx]+=s

    return [sc, sc_list.tolist()]


def cost_react(robot_pos, human_pos, t, treact=0.600, d_proxemics=0.45):
    """    
    Parameters:
        robot_pos : np.array (2,)
            Position [x, y] of the robot
        human_pos : np.array (2,)
            Position [x, y] of the human
        t : float
            Time since the robot entered the human's FoV (seconds)
        treact : float
            Reaction time (~0.600 s)
        d_proxemics : float
            Proxemics distance (>0.45m)
    
    Returns:
        float : reaction cost
    """
    dhreff = np.linalg.norm(robot_pos - human_pos)
    SR = t / treact if 0 < t < treact else 1.0
    return d_proxemics / dhreff * (1 - SR)



def surprise_react_cost(agents: List[Agents], robot: List[Agent], grid) -> List[int]:
    rc = 0.0
    t1 = rclpy.time.Time.from_msg(agents[1].header.stamp)
    t2 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    dt = (t2 - t1).nanoseconds / 1e9  # Diferencia en segundos (float)
    rc_list = np.zeros(len(agents[0].agents))
    time_visible = np.zeros(len(agents[0].agents))
    for agts, rb in zip(agents, robot):
        robot_pos = np.array([rb.position.position.x, rb.position.position.y])
        for idx, agent in enumerate(agts.agents):
            human_pos = np.array([agent.position.position.x, agent.position.position.y])
            human_dir = np.array([math.cos(agent.yaw), math.sin(agent.yaw)])
            if is_visible(robot_pos, human_pos, human_dir, grid):
                time_visible[idx] += dt  
                r = cost_react(robot_pos, human_pos, t=time_visible[idx])
            else:
                r = 0.0
                time_visible[idx] = 0.0
            rc += r
            rc_list[idx]+=r

    return [rc, rc_list.tolist()]



# TODO
def path_irregularity(agents, robot):
    pass


# TODO
def path_efficiency(agents, robot):
    pass


# TODO
def static_obs_collision(agents, robot):
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


metrics = {
    # N. Perez-Higueras, F. Caballero, and L. Merino, “Teaching Robot Nav-
    # igation Behaviors to Optimal RRT Planners,” International Journal of
    # Social Robotics, vol. 10, no. 2, pp. 235–249, 2018.
    "time_to_reach_goal": total_time,
    "path_length": robot_path_length,
    "cumulative_heading_changes": cumulative_heading_changes,
    "avg_distance_to_closest_person": avg_closest_person,
    "minimum_distance_to_people": minimum_distance_to_people,
    "maximum_distance_to_people": maximum_distance_to_people,
    "intimate_space_intrusions": intimate_space_intrusions,
    "personal_space_intrusions": personal_space_intrusions,
    "social_space_intrusions": social_space_intrusions,
    "group_intimate_space_intrusions": group_intimate_space_intrusions,
    "group_personal_space_intrusions": group_personal_space_intrusions,
    "group_social_space_intrusions": group_social_space_intrusions,
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
    "completed": goal_reached,
    # (meters): the closest the robot passes to the target position.
    "minimum_distance_to_target": minimum_goal_distance,
    # (meters): distance between the last robot position and the target position.
    "final_distance_to_target": final_goal_distance,
    #   - 'Robot on Person Personal Distance Violation': number of times a robot
    # approaches a person within the personal distance of the robot.
    # Similar to 'personal_space_intrusions'
    #   - 'Person on Robot Personal Distance Violation': number of times a person
    # approaches the robot within the personal distance of the robot.
    #   - 'Intimate Distance Violation': number of times the robot approached within
    # the intimate distance of a person.
    #   - 'Person on Robot Intimate Distance Violation': number of times a person
    # approaches the robot within the intimate distance of the robot.
    "robot_on_person_collision": robot_on_person_collision,
    "person_on_robot_collision": person_on_robot_collision,
    "time_not_moving": time_not_moving,
    # TODO: 'static_obstacle_collision': static_obs_collision,
    # number of times the robot collides with a static obstacle.
    # SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
    # ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA, AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University
    "avg_robot_linear_speed": avg_robot_linear_speed,
    "avg_robot_angular_speed": avg_robot_angular_speed,
    "avg_acceleration": avg_acceleration,
    "avg_overacceleration": avg_overacceleration,
    # Learning a Group-Aware Policy for Robot Navigation
    # Kapil Katyal ∗1,2 , Yuxiang Gao ∗2 , Jared Markowitz 1 , Sara Pohland 3 , Corban Rivera 1 , I-Jeng Wang 1 , Chien-Ming Huang 2
    "avg_pedestrian_velocity": avg_pedestrian_velocity,
    "avg_closest_pedestrian_velocity": avg_closest_pedestrian_velocity,
    # metrics based on Social Force Model employed in different papers
    "social_force_on_agents": social_force_on_agents,
    "social_force_on_robot": social_force_on_robot,
    "social_work": social_work,
    "obstacle_force_on_robot": obstacle_force_on_robot,
    "obstacle_force_on_agents": obstacle_force_on_agents,
    # P. T. Singamaneni, A. Favier, and R. Alami, “Towards benchmarking
    # human-aware social robot navigation: A new perspective and metrics,”
    # in 2023 32nd IEEE International Conference on Robot and Human
    # Interactive Communication (RO-MAN), 2023, pp. 914–921.
    # Danger costs: Fear and Panic
    # Surprise cost: Visibility, Shock, React
    "danger_fear_cost": danger_fear_cost,
    "danger_panic_cost": danger_panic_cost,
    "surprise_visibility_cost": surprise_visibility_cost,
    "surprise_shock_cost": surprise_shock_cost,
    "surprise_react_cost": surprise_react_cost,
}
