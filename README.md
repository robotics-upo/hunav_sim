# Human Navigation behavior Simulator (HuNavSim)

A controller of human navigation behaviors for Robotics based on ROS2.

**Tested in ROS2 Foxy** 

The simulated people are affected by the obstacles and other people using the [Social Force Model](https://github.com/robotics-upo/lightsfm).
Besides, some human reactions to the presence of robots have been included.


## Dependencies

* Yo must download and install the Social Force Model library. Follow the instructions here: lightsfm https://github.com/robotics-upo/lightsfm


## Features

* Realistic human navigation behavior based on an adapted Social Force Model and its extension for groups of people.

* A core of human navigation behaviors which is independent of any simulator. The core can be used in different simulators by means of a wrapper.

* A wrapper for Gazebo Simulator (tested in Gazebo 11) is provided here: XXXXXXX

* The simulator core is programmed under the new ROS2 framework (tested in Foxy distro).

* Despite following the same human navigation model (SFM), the regular navigation behavior for each human agent is different by means of random initialization, or user specification, of some parameters of the model (within reasonable fixed boundaries). That provides a richer human navigation behavior closer to the real world.

* A set of realistic human navigation reactions to the presence of a robot is provided:

    * *regular*: the human treat the robot like another human.
    * *impassive*: the human deal with the robot like a static obstacle.
    * *surprised*: when the human sees the robot, he/she stops walking and starts to look at the robot.
    * *curious*: the human abandons the current navigation goal for a while and starts to approach the robot slowly.
    * *scared*: the human tries to stay far from the robot.
    * *threatening*: the human tries to block the path of the robot by walking in front of it.

* The navigation behavior defined by the user for each human agent is led by a configurable behavior tree.

* Finally, a set of metrics for social navigation evaluation are provided (ongoing work). This set includes the metrics found in the literature plus some other ones. Moreover, the metrics computed can be easily configured and extended by the user.



## Steps to use HuNavSim with a robotic simulator

The navigation behavior of the human agents spawned in a regular physics simulator can be controlled by HuNavSim. Therefore, HuNavSim can be "connected" to a popular simulators used in Robotics like Gazebo, Webots or Unity.

To do so, we must programme a ROS2 wrapper of the particular simulator. At each simulation step, the wrapper must collect the current state of the human agents and the robot (positions and velocities), and send them to HuNavSim. The HuNavSim controller will compute and return the next state of the agents. Finally, the wrapper must update the agents' state in the simulation.

That communication with HuNavSim can be easily done through different ROS2 services available. These services use the messages of the package hunav_msgs:

* *"/compute_agents"*. The request must contain the current state of all the human agents (hunav_msgs/Agents message) and the robot (hunav_msgs/Agent message). HuNavSim will fill the response with the updated state of the agents (hunav_msgs/Agents message).

* *"/compute_agent"*. The request must contain the id of the desired human agent (integer value). The response contains the updated state of the indicated agent. 

* *"/move_agent"*. The request includes the states of the human agents, the robot and id of the agent that must be computed. The response of HuNavSim is the udpated state of the indicated agent. 


![](https://github.com/robotics-upo/hunav_sim/blob/master/images/HuNavSim.png)


A Gazebo (v11) wrapper is provided in: XXXXXXX
 


## Configuration

(yaml) TODO

ros parameters and ROS topic publications...


## Example run

TODO


