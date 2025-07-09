import numpy as np
from hunav_msgs.msg import Agents
from hunav_msgs.msg import Agent
from angles import shortest_angular_distance
from typing import Tuple


class SFM:
    def __init__(self):

        self.forceFactorDesired = 2.0  # Factor for the desired force
        self.forceFactorObstacle = 10  # Factor for the obstacle force
        self.forceSigmaObstacle = 0.2  # Sigma for the obstacle force
        self.forceFactorSocial = 2.1  # Factor for the social force
        self.forceFactorGroupGaze = 3.0  # Factor for the group gaze force
        self.forceFactorGroupCoherence = 2.0  # Factor for the group coherence force
        self.forceFactorGroupRepulsion = 1.0  # Factor for the group repulsion force
        self.lambda_ = 2.0  # Lambda for the social force
        self.gamma = 0.35  # Gamma for the social force
        self.n = 2.0  # n for the social force
        self.nPrime = 3.0  # n' for the social force
        self.relaxationTime = 0.5  # Relaxation time for the desired force

    def retrievePosAndVel(self, agent: Agent) -> Tuple[np.ndarray, np.ndarray]:
        """
        It retrieves the position and velocity of the agent
        and returns them as numpy arrays.
        Args:
            agent (Agent): The agent from which to retrieve position and velocity.
        Returns:
            Tuple[np.ndarray, np.ndarray]: A tuple containing the position and velocity as numpy arrays.
        """
        agentpos = np.array(
            [agent.position.position.x, agent.position.position.y], dtype=np.float32
        )
        agentvel = np.array(
            [agent.velocity.linear.x, agent.velocity.linear.y], dtype=np.float32
        )
        return agentpos, agentvel

    def computeDesiredForce(self, agent: Agent) -> np.ndarray:
        """
        It computes the desired force for the agent
        based on its position, velocity, and goal.
        If the agent has no goals or the goal is near, it will apply a force
        to slow down the agent based on its current velocity.

        Args:
            agent (Agent): The agent for which to compute the desired force.
        """

        desiredForce = np.array([0.0, 0.0], dtype=np.float32)
        agentpos, agentvel = self.retrievePosAndVel(agent)
        if len(agent.goals) == 0:
            desiredForce = -agentvel / self.relaxationTime
            return desiredForce

        agentgoal = np.array([agent.goals[0].position.x, agent.goals[0].position.y])
        dist = np.linalg.norm(agentgoal - agentpos)

        if dist < agent.goal_radius:
            desiredForce = -agentvel / self.relaxationTime
            return desiredForce

        desiredDirection = (agentgoal - agentpos) / dist

        desiredForce = (
            self.forceFactorDesired
            * (desiredDirection * agent.desiredVelocity - agentvel)
            / self.relaxationTime
        )
        return desiredForce

    def computeObstacleForce(self, agent: Agent) -> np.ndarray:
        """
        It computes the obstacle force for the agent
        based on its position and the closest obstacles.
        If the agent has no closest obstacles, it returns a zero vector.
        Args:
            agent (Agent): The agent for which to compute the obstacle force.
        Returns:
            np.ndarray: The computed obstacle force vector.
        """

        obstacleForce = np.array([0.0, 0.0], dtype=np.float32)
        agentpos, _ = self.retrievePosAndVel(agent)
        if len(agent.closest_obs) > 0:
            for obs in agent.closest_obs:
                agentobs = np.array([obs.x, obs.y])
                dist = np.linalg.norm(agentpos - agentobs)
                dirvec = (agentpos - agentobs) / dist
                dist = dist - agent.radius
                obstacleForce += (
                    self.forceFactorObstacle
                    * np.exp(-dist / self.forceSigmaObstacle)
                    * dirvec
                )

            obstacleForce = obstacleForce / len(agent.closest_obs)
            return obstacleForce

        return obstacleForce

    def computeSocialForceTwoAgents(self, agent1: Agent, agent2: Agent) -> np.ndarray:
        """
        It computes the social force between two agents.
        It is the force that the first agent exerts on the second agent
        to influence its behavior in a social context.
        The social force is computed based on the positions and velocities of the agents,
        as well as the parameters lambda_, gamma, n, and nPrime.
        Args:
            agent1 (Agent): The first agent.
            agent2 (Agent): The second agent.
        """
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        agent1pos, agent1vel = self.retrievePosAndVel(agent1)
        agent2pos, agent2vel = self.retrievePosAndVel(agent2)
        diff = agent1pos - agent2pos
        diffDirection = diff / np.linalg.norm(agent1pos - agent2pos)

        velDiff = agent2vel - agent1vel
        interactionVector = self.lambda_ * velDiff + diffDirection
        interactionLength = np.linalg.norm(interactionVector)
        interactionDirection = interactionVector / interactionLength
        theta = shortest_angular_distance(
            np.arctan2(diffDirection[1], diffDirection[0]),
            np.arctan2(interactionDirection[1], interactionDirection[0]),
        )
        B = self.gamma * interactionLength

        forceVelocityAmount = -np.exp(
            -np.linalg.norm(agent1pos - agent2pos) / B - (self.nPrime * B * theta) ** 2
        )
        forceAngleAmount = -np.sign(theta) * -np.exp(
            -np.linalg.norm(agent1pos - agent2pos) / B - (self.n * B * theta) ** 2
        )
        forceVelocity = forceVelocityAmount * interactionDirection
        interactionDirection_leftNormal = np.array(
            [-interactionDirection[1], interactionDirection[0]]
        )
        forceAngle = forceAngleAmount * interactionDirection_leftNormal
        socialForce = self.forceFactorSocial * (forceVelocity + forceAngle)
        return socialForce

    def computeSocialForce(self, agent: Agent, agents: Agents) -> np.ndarray:
        """
        It computes the social force that the agents exert on a specific agent.
        Args:
            agent (Agent): The agent for which to compute the social force.
            agents (Agents): The collection of all agents in the environment.
        Returns:
            np.ndarray: The computed social force vector for the agent.
        """
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        for a in agents.agents:
            if a.id == agent.id:
                continue
            # force provoked by agent 'a' in agent 'agent'
            socialForce += self.computeSocialForceTwoAgents(a, agent)
        return socialForce

    def modulusSocialForce(self, agent: Agent, agents: Agents) -> float:
        """
        It computes the cumulative modulus of the social force exerted by the agents
        on a specific agent.
        Args:
            agent (Agent): The agent for which to compute the cumulative social force modulus.
            agents (Agents): The collection of all agents in the environment.
        Returns:
            float: The cumulative modulus of the social force exerted on the agent.
        """
        socialForceMod = 0.0
        for a in agents.agents:
            if a.id == agent.id:
                continue
            # force provoked by agent 'a' in agent 'agent'
            socialForceMod += np.linalg.norm(self.computeSocialForceTwoAgents(a, agent))
        return socialForceMod

    def computeSocialForce2(self, agent: Agent, agents: Agents) -> np.ndarray:
        """
        It computes the social force that the agent exerts on the rest of agents.
        Args:
            agent (Agent): The agent for which to compute the social force.
            agents (Agents): The collection of all agents in the environment.
        Returns:
            np.ndarray: The computed social force vector for the agent.
        """
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        for a in agents.agents:
            if a.id == agent.id:
                continue
            # force provoked by agent 'agent' in agent 'a'
            socialForce += self.computeSocialForceTwoAgents(agent, a)
        return socialForce

    def modulusSocialForce2(self, agent, agents):
        """
        It computes the cumulative modulus of the social force exerted by a specific agent
        on the rest of the agents.
        Args:
            agent (Agent): The agent for which to compute the cumulative social force modulus.
            agents (Agents): The collection of all agents in the environment.
        Returns:
            float: The cumulative modulus of the social force exerted by the agent on the rest of the agents.
        """
        socialForceMod = 0.0
        for a in agents.agents:
            if a.id == agent.id:
                continue
            # force provoked by agent 'agent' in agent 'a'
            socialForceMod += np.linalg.norm(self.computeSocialForceTwoAgents(agent, a))
        return socialForceMod

    def computeSocialWork(self, agent_robot, agents):
        """
        It computes the social work done by the robot and the social work provoked
        by the robot in the other agents.
        Args:
            agent_robot (Agent): The robot agent for which to compute the social work.
            agents (Agents): The collection of all agents in the environment.
        Returns:
            float: The total social work done by the robot and the social work provoked in other agents.
        """

        # compute social work of the robot
        socForceRobot = self.computeSocialForce(agent_robot, agents)
        obsForceRobot = self.computeObstacleForce(agent_robot)
        wr = np.linalg.norm(socForceRobot) + np.linalg.norm(obsForceRobot)

        # compute the social work provoked by the
        # robot in the other agents
        wa = 0.0
        for a in agents.agents:
            if a.id == agent_robot.id:
                continue
            # force provoked by agent robot in agent 'a'
            wa += np.linalg.norm(self.computeSocialForceTwoAgents(agent_robot, a))

        return wr + wa
