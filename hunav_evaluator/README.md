# Hunav Evaluator

A ROS2 python package to compute different navigation metrics about the simulations performed with the HuNav Simulator.

**Tested in ROS2 Foxy** 


## Dependencies

* numpy
* hunav_msgs


## Description

This package subscribes to a set of topic published by the Hunav Agent Manager:

    - ```/human_states``` this topic publishes messages of type *hunav_msgs/msg/Agents* with the status of the human agents for each time step.
    - ```/robot_states``` this topic publishes messages of type *hunav_msgs/msg/Agent* with the status of the robot for each time step.
    
This information is stored and then is employed to compute a set of metrics related to the social robot navigation. 

The recording process is automatic. 

It starts when the first message in the subscribed topics is received.
The recording stops when a certain time passes without receiving data through the topics.


## Parameters

* ```frequency```. Indicate the frequency of capturing the data (it must be slower than data publishing). If the value is set to zero, the data is captured at the same frequency than it is published.


## TODO:

* Create a yaml file to select the metrics and frequency.
* Add a service to start/stop the recording so the user can decide when to record.
* Program a visual interface (probably a RViz panel) to configure the evaluation and the metrics to be computed.
* Think about an easy way to add new metrics by the user.


