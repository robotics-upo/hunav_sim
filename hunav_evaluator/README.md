# Hunav Evaluator

A ROS2 python package to compute different navigation metrics about the simulations performed with the HuNav Simulator.

**Tested in ROS2 Foxy** 


## Dependencies

* numpy
* hunav_msgs


## Description

This package subscribes to a set of topic published by the Hunav Agent Manager:

* ```/human_states``` this topic publishes messages of type *hunav_msgs/msg/Agents* with the status of the human agents for each time step.
* ```/robot_states``` this topic publishes messages of type *hunav_msgs/msg/Agent* with the status of the robot for each time step.
    
This information is stored and then is employed to compute a set of metrics related to the social robot navigation. 

The recording process is automatic. 

It starts when the first message in the subscribed topics is received.
The recording stops when a certain time passes without receiving data through the topics.

## Add new metrics

Metrics can be found in the ```hunav_metrics.py``` file. 

Each function takes two parameters:

* ```robot``` a list where all information recorded about the social robot is stored.
* ```agents``` a list where all information recorded about the human agents are stored.

To add a new metric, add a function to the hunav_metrics.py file.

After it has been added, the function's name must be added to the *metrics* global variable.

Once the ```hunav_metrics.py``` file has been updated, the function's name has to be added to the ```metrics.yaml``` file.


## Parameters

* ```frequency```. Indicate the frequency of capturing the data (it must be slower than data publishing). If the value is set to zero, the data is captured at the same frequency than it is published.


## TODO:

* Program a visual interface (probably a RViz panel) to configure the evaluation and the metrics to be computed.


