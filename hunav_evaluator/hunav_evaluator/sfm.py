
import numpy as np
import math
import sys
from hunav_msgs.msg import Agents
from hunav_msgs.msg import Agent
from geometry_msgs.msg import Pose


class SFM():
    def __init__(self):

        self.forceFactorDesired = 2.0
        self.forceFactorObstacle = 10
        self.forceSigmaObstacle = 0.2
        self.forceFactorSocial = 2.1
        self.forceFactorGroupGaze = 3.0
        self.forceFactorGroupCoherence = 2.0
        self.forceFactorGroupRepulsion = 1.0
        self.lambda_ = 2.0
        self.gamma = 0.35
        self.n = 2.0
        self.nPrime = 3.0
        self.relaxationTime = 0.5


    def computeDesiredForce(self, agent):
        
        desiredForce = np.array([0.0, 0.0], dtype=np.float32)
        agentvel = np.array([agent.velocity.linear.x, agent.velocity.linear.y])
        if len(agent.goals) == 0:
            desiredForce = -agentvel / self.relaxationTime 
            return desiredForce

        agentpos = np.array([agent.position.position.x, agent.position.position.y])
        agentgoal = np.array([agent.goals[0].position.x, agent.goals[0].position.y])
        dist = np.linalg.norm(agentgoal - agentpos)
        desiredDirection = (agentgoal - agentpos) / dist

        if (dist < agent.goal_radius):
            desiredForce = -agentvel / self.relaxationTime 
            return desiredForce

        desiredForce = self.forceFactorDesired * (desiredDirection * agent.desiredVelocity - agentvel) / self.relaxationTime    
        return desiredForce
    


    def computeObstacleForce(self, agent):

        obstacleForce = np.array([0.0, 0.0], dtype=np.float32)
        agentpos = np.array([agent.position.position.x, agent.position.position.y])
        if len(agent.closest_obs)>0:
            for obs in agent.closest_obs:
                agentobs = np.array([obs.x, obs.y])
                dist = np.linalg.norm(agentpos - agentobs)
                dirvec = (agentpos - agentobs)/dist
                dist = dist - agent.radius
                obstacleForce += self.forceFactorObstacle * np.exp(-dist / self.forceSigmaObstacle) * dirvec

            obstacleForce = obstacleForce / len(agent.closest_obs)
            return obstacleForce
        
        return obstacleForce 
    

    def computeSocialForceTwoAgents(self, agent1, agent2):
        '''
        It computes the social force
        provoked by the agent1 in the
        agent2 
        '''
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        agent2pos = np.array([agent2.position.position.x, agent2.position.position.y])
        agent1pos = np.array([agent1.position.position.x, agent1.position.position.y])
        diff = agent1pos - agent2pos
        diffDirection = diff/np.linalg.norm(agent1pos - agent2pos)

        agent2vel = np.array([agent2.velocity.linear.x, agent2.velocity.linear.y])
        agent1vel = np.array([agent1.velocity.linear.x, agent1.velocity.linear.y])
        velDiff = agent2vel - agent1vel
        interactionVector = self.lambda_ * velDiff + diffDirection
        interactionLength = np.linalg.norm(interactionVector)
        interactionDirection = interactionVector / interactionLength
        theta = math.atan2(diffDirection[1], diffDirection[0]) - math.atan2(interactionDirection[1], interactionDirection[2])
        theta = self.normalize_angle(theta)
        B = self.gamma * interactionLength
        forceVelocityAmount = -math.exp(-np.linalg.norm(agent1pos - agent2pos) / B - (self.nPrime * B * theta)**2)
        forceAngleAmount = -np.sign(theta) * math.exp(-np.linalg.norm(agent1pos - agent2pos) / B - (self.n * B * theta)**2)
        forceVelocity = forceVelocityAmount * interactionDirection
        interactionDirection_leftNormal = np.array([-interactionDirection[1], interactionDirection[0]])
        forceAngle = forceAngleAmount * interactionDirection_leftNormal
        socialForce = self.forceFactorSocial * (forceVelocity + forceAngle)
        return socialForce


    def computeSocialForce(self, agent, agents):
        '''
        It computes the social force
        provoked by the agents in the
        agent 
        '''
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        for a in agents:
            if a.id == agent.id:
                continue
            # force provoked by agent 'a' in agent 'agent'
            socialForce += self.computeSocialForceTwoAgents(a, agent)
        return socialForce 


    def modulusSocialForce(self, agent, agents):
        '''
        It computes the cumulative modulus of 
        the social force provoked by the agents in the agent 
        '''
        socialForceMod = 0.0
        for a in agents:
            if a.id == agent.id:
                continue
            # force provoked by agent 'a' in agent 'agent'
            socialForceMod += np.linalg.norm(self.computeSocialForceTwoAgents(a, agent))
        return socialForceMod 


    def computeSocialForce2(self, agent, agents):
        '''
        It computes the social force
        provoked by the agent in the
        rest of agents 
        '''
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        for a in agents:
            if a.id == agent.id:
                continue
            # force provoked by agent 'agent' in agent 'a'
            socialForce += self.computeSocialForceTwoAgents(agent, a)
        return socialForce 
    

    def modulusSocialForce2(self, agent, agents):
        '''
        It computes the cumulative modulus 
        of the social force provoked by the agent 
        in the rest of agents 
        '''
        socialForceMod = 0.0
        for a in agents:
            if a.id == agent.id:
                continue
            # force provoked by agent 'agent' in agent 'a'
            socialForceMod += np.linalg.norm(self.computeSocialForceTwoAgents(agent, a))
        return socialForceMod 


    def computeSocialWork(self, agent_robot, agents):

        # compute social work of the robot
        socForceRobot = self.computeSocialForce(agent_robot, agents)
        obsForceRobot = self.computeObstacleForce(agent_robot)
        wr = np.linalg.norm(socForceRobot) + np.linalg.norm(obsForceRobot)

        # compute the social work provoked by the
        # robot in the other agents
        wa = 0.0
        for a in agents:
            if a.id == agent_robot.id:
                continue
            # force provoked by agent robot in agent 'a'
            wa += np.linalg.norm(self.computeSocialForceTwoAgents(agent_robot, a))

        return wr + wa

    def normalize_angle(self, theta):
        # theta should be in the range [-pi, pi]
        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi
        return theta