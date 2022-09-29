# HuNav RViz2 Panel

A ROS2 C++ package to generate human agents to use in the Human Navigation behavior Simulator (HuNavSim).

**Tested in ROS2 Foxy**

# Dependencies

* ros-foxy-nav2-map-server
* ros-foxy-nav2-lifecycle-manager

## Description

This package provides a RViz panel which allows to create hunav agents and save them in a Yaml configuration file. 
This file will be used by the Human Navigation behavior Simulator (HuNavSim) in order to spawn humans with different characteristics.

A previous 2D map of the navigation scenario is required. We will use nav2-map-server to load the map and visualize it in RViz (see the launch file **hunav_rviz2_launch.py**).

Apart of creating human agents, it also gives the possibility to open a Yaml file that has already been generated.

## Steps to use HuNav RViz2 Panel

A launch file for testing is provided to launch the panel:
```sh
ros2 launch hunav_rviz2_panel hunav_rviz2_launch.py
```

Some example maps related to the café simulated scenario are provided in the maps directory.

After launching the system we will see:

![](https://github.com/robotics-upo/hunav_sim/blob/master/hunav_rviz2_panel/images/RVizPanelGlobal.png)

The hunav panel provide two options:

* To open the base configuration file, agents.yaml
* To create and configure the hunav agents from scratch.

The first option allows to open a Yaml file that has already been generated. It is stored in the install directory of the ROS2 workspace:
```sh
/install/hunav_agent_manager/share/hunav_agent_manager/config/agents.yaml
```
Example:
![](https://github.com/robotics-upo/hunav_sim/blob/master/hunav_rviz2_panel/images/AgentsGenerated.png)

As result, we will visualize on the map, the initial position and goals for each agent indicated in the yaml file (as can be seen in the image above). In next iterations, we will allow the user to modify the agents features and to store the new changes.  

The second option allows to generate a new set of hunav agents. Each agent must have a name, behavior, skin, initial pose and goals as can be seen in the following image:

![](https://github.com/robotics-upo/hunav_sim/blob/master/hunav_rviz2_panel/images/HumanAgentWindow.png)

Hunav agents' initial pose and goals are stored by using the HuNav RViz tool. To do so, click the "Set initial pose" button, and then, from the tool panel, select HunavGoals.

Once HunavGoals is active, move your mouse to the desired position on the map, and click on it. This will publish a MarkerArray (Cylinder) on ```/hunav_agent``` topic, and cylinder marker will show on the map.

Follow the same procedure for the agents' goals. These goals are published as a MarkerArray (Squares) on ```/hunav_goals``` topic.

Example:

![](https://github.com/robotics-upo/hunav_sim/blob/master/hunav_rviz2_panel/images/AgentCreation.gif)

## TODO

* In this moment, initial pose and goals windows don't update point after receiving a point (Close and open the window to update it). We are fixing it.
* Allow to modify Yaml file while is being shown.