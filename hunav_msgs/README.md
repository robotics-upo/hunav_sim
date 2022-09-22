# HuNav Msgs

A ROS2 package to use Agent messages.

**Tested in ROS2 foxy**

## Description

This package contains messages and services for Agents.

Two types of message are used:
    
* ```Agent.msg``` is used to create an agent. It contains the type of agent (Person, Robot or other), behaviors, initial pose, goals, and more information relative to an agent.

* ```Agents.msg``` contains a std_msgs/Header and a list of Agents.

Services:

Three types of services are used:

* ```ComputeAgent.srv``` 
* ```ComputeAgents.srv```
* ```MoveAgent.srv```