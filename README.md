# Human Navigation behavior Simulator (HuNavSim)

A controller of human navigation behaviors for Robotics based on ROS2.

**This is a work in progress version**

**Tested in ROS2 Humble**

The simulated people are affected by the obstacles and other people using the [Social Force Model](https://github.com/robotics-upo/lightsfm).
Besides, a set of human reactions to the presence of robots have been included.

If you use this simulator in your work, please cite:

N. Pérez-Higueras, R. Otero, F. Caballero and L. Merino, "HuNavSim: A ROS 2 Human Navigation Simulator for Benchmarking Human-Aware Robot Navigation," in IEEE Robotics and Automation Letters, doi: 10.1109/LRA.2023.3316072.

Bibtex:
```
@ARTICLE{PerezRal2023,
  author={Pérez-Higueras, Noé and Otero, Roberto and Caballero, Fernando and Merino, Luis},
  journal={IEEE Robotics and Automation Letters}, 
  title={HuNavSim: A ROS 2 Human Navigation Simulator for Benchmarking Human-Aware Robot Navigation}, 
  year={2023},
  month={September},
  volume={},
  number={},
  pages={1-8},
  issn={2377-3766},
  doi={10.1109/LRA.2023.3316072}}
```
A pre-print version of the accepted paper is available [here](https://arxiv.org/abs/2305.01303). 


## Dependencies

* You must download and install the Social Force Model library. Follow the instructions here: https://github.com/robotics-upo/lightsfm
* The ros people_msgs are also required. At the moment of this development, people_msgs were not still available to be installed from the apt ros-humble package server. You can get the package from here: https://github.com/wg-perception/people/tree/ros2. Please, copy it and put it in your workspace.
* The ROS2 packages *nav2-behavior-tree* and *behaviortree_cpp_v3* are also needed.
  ```sh
  sudo apt install ros-humble-nav2-behavior-tree ros-humble-behaviortree-cpp-v3
  ```

## Features

* Realistic human navigation behavior based on an adapted Social Force Model and its extension for groups of people.

* A core of human navigation behaviors which is independent of any simulator. The core can be used in different simulators by means of a wrapper.

* A wrapper for Gazebo Simulator (tested in Gazebo 11) is provided here: https://github.com/robotics-upo/hunav_gazebo_wrapper

* The simulator core is programmed under the new ROS2 framework (tested in Humble distro).

* A GUI based on a RViz2 panel is employed to easily configure the human agents. For instructions, please check the [hunav_rviz2_panel](https://github.com/robotics-upo/hunav_sim/tree/humble/hunav_rviz2_panel) 

* A set of realistic human navigation reactions to the presence of a robot is provided:

    * *regular*: the human treat the robot like another human.
    * *impassive*: the human deal with the robot like a static obstacle.
    * *surprised*: when the human sees the robot, he/she stops walking and starts to look at the robot.
    * *curious*: the human abandons the current navigation goal for a while and starts to approach the robot slowly.
    * *scared*: the human tries to stay far from the robot.
    * *threatening*: the human tries to block the path of the robot by walking in front of it.

* The navigation behavior defined by the user for each human agent is led by a configurable behavior tree.

* Finally, a set of metrics for social navigation evaluation are provided. This set includes the metrics found in the literature plus some other ones. Moreover, the metrics computed can be easily configured and extended by the user. Further information is provided in the [hunav_evaluator](https://github.com/robotics-upo/hunav_sim/tree/humble/hunav_evaluator)



## Steps to use HuNavSim with a robotic simulator

The navigation behavior of the human agents spawned in a regular physics simulator can be controlled by HuNavSim. Therefore, HuNavSim can be "connected" to a popular simulators used in Robotics like Gazebo, Webots, Morse or Unity.

To do so, we must programme a ROS2 wrapper of the particular simulator. At each simulation step, the wrapper must collect the current state of the human agents and the robot (positions and velocities), and send them to HuNavSim. The HuNavSim controller will compute and return the next state of the agents. Finally, the wrapper must update the agents' state in the simulation.

That communication with HuNavSim can be easily done through different ROS2 services available. These services use the messages of the package hunav_msgs:

* *"/compute_agents"*. The request must contain the current state of all the human agents (hunav_msgs/Agents message) and the robot (hunav_msgs/Agent message). HuNavSim will fill the response with the updated state of the agents (hunav_msgs/Agents message).

* *"/compute_agent"*. The request must contain the id of the desired human agent (integer value). The response contains the updated state of the indicated agent. 

* *"/move_agent"*. The request includes the states of the human agents, the robot and id of the agent that must be computed. The response of HuNavSim is the udpated state of the indicated agent. 

Moreover, the initial configuration parameters of the agents can be read from the *"/hunav_loader"* ROS2 node. This node loads the agents data from the yaml file *'agents.yaml'* located in the *config* directory of the package *hunav_agent_manager*. Then, the parameters can be retreived through the ROS2 service */hunav_loader/get_parameters*. 


![](https://github.com/robotics-upo/hunav_sim/blob/humble/images/HuNavSim.png)


A Gazebo (v11) wrapper is provided in: https://github.com/robotics-upo/hunav_gazebo_wrapper
 

## Configuration

The user must define the desired number and properties of hunav agents. This is done through the file agents.yaml. The user can edit this file directly, o can create it through a GUI, check the [hunav_rviz2_panel](https://github.com/robotics-upo/hunav_sim/tree/humble/hunav_rviz2_panel).   

An example snippet of a agents.yaml file with two agents can be seen next:

```yaml
hunav_loader:
  ros__parameters:
    map: cafe
    publish_people: true
    agents:
      - agent1
      - agent2
    agent1:
      id: 1
      skin: 2
      behavior: 5
      group_id: -1
      max_vel: 1.5
      radius: 0.4
      init_pose:
        x: -3.973340
        y: -8.576801
        z: 1.250000
        h: 0.0
      goal_radius: 0.3
      cyclic_goals: true
      goals:
        - g0
        - g1
        - g2
      g0:
        x: -3.133759
        y: -4.166653
        h: 1.250000
      g1:
        x: 0.997901
        y: -4.131655
        h: 1.250000
      g2:
        x: -0.227549
        y: -10.187146
        h: 1.250000
    agent2:
      id: 2
      skin: 3
      behavior: 6
      group_id: -1
      max_vel: 1.5
      radius: 0.4
      init_pose:
        x: 2.924233
        y: 5.007970
        z: 1.250000
        h: 0.0
      goal_radius: 0.3
      cyclic_goals: true
      goals:
        - g0
        - g1
      g0:
        x: -2.644067
        y: 2.066231
        h: 1.250000
      g1:
        x: -1.663169
        y: -3.291318
        h: 1.250000
```

### Global Parameters

As global parameters the user must indicate:

* ```map```. Name of the map (ROS 2d map used by the ROS map_server) that corresponds to the scenario that is going to be loaded in the base simulator (Gazebo, Webots or similar). **Curretly this parameter is not being used**. 
* ```publish_people```. Boolean to indicate whether the hunav agents must be also published in the topic /people using the message *People* of the *people_msgs*. This can be useful if other nodes need to receive information about the people around the robot. In order to have the complete data of an 2D navigation agent, the agent's yaw (*radians*) has been included in the field *position.z* and the angular velocity (*rads/sec*) in the field *velocity.z*.   
* ```agents```. This is a list with the names of the agents to be spawned. 


### Agent parameters

The user must provide the following data under the identification name of each agent. The names must match the names list indicated in the parameter *agents*:

* ```id```. Integer value that must be unique for each agent.
* ```skin```. Integer value to indicate the 3d model to represent the hunav agent. Currently, it is used for the 3d models spawned in Gazebo only. It must be in the range [0-4].
* ```behavior```. Integer value to identify one of the six available navigation behaviors:
  * **1** - Regular
  * **2** - Impassive
  * **3** - Surprised
  * **4** - Scared
  * **5** - Curious
  * **6** - Threatening
* ```group_id```. Integer value to identy a walking group. It must be shared by the members of the same group. Value *-1* indicates the agent is not walking in group.  
* ```max_vel```. The maximum velocity of the agent in m/s.
* ```radius```. Radius in meters of the circunference that circumbscribes the agent's footprint.
* ```init_pose```. It contains the coordinates in meters of the agent's initial position in the scenario (*x*, *y* and *z*) and also the heading in radians. 
* ```goal_radius```. Radius in meters of the goal. This value is employed to decide when the agent has reached a goal.
* ```cyclic_goals```. Boolean to indicate whether the final goal has been reached, the agent must be begin the goal list again. 
* ```goals```. List with the identifiers (strings) of the agent's goals. For each goal, the user must provide under the related identifier the 2D position of the goal and a heading.  
  

### Metrics parameters  

The user can also configure the set of metrics to be computed. Check the [hunav_evaluator](https://github.com/robotics-upo/hunav_sim/tree/humble/hunav_evaluator) to know how.


## Example run

Some example launch to run the HuNavSim with Gazebo can be found in the documentation of the [hunav_gazebo_wrapper](https://github.com/robotics-upo/hunav_gazebo_wrapper) 


## TODOs

* Augmenting the number navigation reactions of the agents.
* Including configurable small variations of the Social Force Model weights to enrich the variety of the agents navigation.
* Programming another RViz Panel to select the metrics to be computed.
* Completing the set of metrics included.  

## Acknowledgements

This work is partially supported by Programa Operativo FEDER Andalucia 2014-2020, Consejeria de Economía, Conocimiento y Universidades (DeepBot, PY20\_00817) and the project NHoA (PLEC2021-007868) and NORDIC (TED2021-132476B-I00), funded by MCIN/AEI/10.13039/501100011033 and the European Union NextGenerationEU/PRTR.

<!---![](https://github.com/robotics-upo/hunav_sim/blob/master/images/image.png | width=100) -->
<img src="https://github.com/robotics-upo/hunav_sim/blob/humble/images/image.png" width="400">

