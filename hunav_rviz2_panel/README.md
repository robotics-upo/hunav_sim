# HuNav RViz2 Panel

A ROS2 C++ package to generate human agents to use in the HuNav Simulator.

**Tested in ROS2 Foxy**

# Dependencies

* ros-foxy-nav2-map-server
* ros-foxy-nav2-lifecycle-manager

## Description

This package provides a RViz panel which allows to create human agents and store them in a Yaml file that will be used by the HuNav Simulator.

This package makes use of nav2-map-server to load the map that is going to be used.

Apart of creating human agents, it also gives the possibility to open a Yaml file that has already been generated.

## Steps to use HuNav RViz2 Panel

A launch file for testing is provided to launch the panel:
```sh
ros2 launch hunav_rviz2_panel hunav_rviz2_launch.py
```

The map that is being used is a generated map. It's stored in the maps directory of this repository.

Afterwards, the following will be shown:

![](https://github.com/robotics-upo/hunav_sim/hunav_rviz2_panel/images/RVizPanelGlobal.png)

Once it has been launched, two options are available:

* Open yaml file
* Create agents

The first option allows to open a Yaml file that has already been generated. It is stored in the install directory of the ROS2 workspace:
```sh
/install/hunav_agent_manager/share/hunav_agent_manager/config/agents.yaml
```

The second option allows to generate human agents. Each human agent has a name, behavior, skin, initial pose and goals as can be seen in the following image:

![](https://github.com/robotics-upo/hunav_sim/hunav_rviz2_panel/images/HumanAgentwindow.png)

Human agent's initial pose and goals are stored by using the HuNav RViz tool. To do so, click the "Set initial pose" button, and then, from the tool's panel, select HunavGoals.

Once HunavGoals is active, position your mouse inside the map that is being shown and click. This will publish a MarkerArray (Cylinder) on /hunav_agent topic.

Same procedure for the agent's goals. This goals are published as a MarkerArray (Squares) on /hunav_goals topic.

## TODO

* Update initial pose and goals windows
* Allow to modify Yaml file while is being shown