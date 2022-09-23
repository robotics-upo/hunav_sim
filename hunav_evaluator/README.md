# Hunav Evaluator

A ROS2 python package to compute different human-aware navigation metrics about the simulations performed with the HuNav Simulator.

**Tested in ROS2 Foxy** 


## Dependencies

* numpy
* hunav_msgs
* hunav_agent_manager


## Description

This package subscribes to a set of topic published by the Hunav Agent Manager:

* ```/human_states``` this topic publishes messages of type *hunav_msgs/msg/Agents* with the status of the human agents for each time step.
* ```/robot_states``` this topic publishes messages of type *hunav_msgs/msg/Agent* with the status of the robot for each time step.
    
This information is stored and then is employed to compute a set of metrics related to the social robot navigation. 

The recording process is automatic. 

It starts when the first message in the subscribed topics is received.
The recording stops when a certain time passes without receiving data through the topics.

Once the recording has stopped, the computation of the metrics is performed and an output txt file is generated.


## Configuration and Parameters

The user must specify the set of metrics to be computed and some parameters about the metrics generation.
This values must be indicated in the yaml file metrics.yaml, placed in the config directory.

### Global parameters

* ```frequency```. Indicate the frequency of capturing the data (it must be slower than data publishing). If the value is set to zero, the data is captured at the same frequency than it is published.
* ```experiment_tag```. A string that can be used to identify different scenarios or experiments. This tag will be stored in the output results file. 
* ```result_file```. Full path and name of the output text file. If a new experiment is performed and the file already exists, the evaluator wil open the file and will add the computed metrics in a new row at the end of the file. This allows to repeat experiments and to store the results as rows in the same file for easier post-processing.
* ```metrics```. List with the names of the metrics to be computed. The user can just comment/uncomment the desired metrics.   

### Metrics

Currently, the metrics implemented are those employed in our previous work [1] and the SEAN simulator [2]. The metrics from the SocNavBench [3], Crowdbot [4] and the compilation indicated in the work of Gao et al. [5], are being studied and included.

Example snippet of the metrics section in the yaml file:

```yaml
metrics: 
    # Follwing metrics from:
    # Pérez-Higueras, N., Caballero, F. & Merino, L. 
    # Teaching Robot Navigation Behaviors to Optimal RRT Planners. 
    # Int.J. of Soc. Robotics 10, 235–249 (2018). https://doi.org/10.1007/s12369-017-0448-1
    - time_to_reach_goal
    - path_length
    - cumulative_heading_changes
    - avg_distance_to_closest_person
    - minimum_distance_to_people
    - intimate_space_intrusions
    - personal_space_intrusions
    - social_space_intrusions
    - group_intimate_space_intrusions
    - group_personal_space_intrusions
    - group_social_space_intrusions
    # Following metrics from:
    # N. Tsoi et al., "SEAN 2.0: Formalizing and Generating Social Situations
    # for Robot Navigation," in IEEE Robotics and Automation Letters, vol. 7,
    # no. 4, pp. 11047-11054, Oct. 2022, doi: 10.1109/LRA.2022.3196783.
    - completed
    - minimum_distance_to_target
    - final_distance_to_target
    - robot_on_person_collision
    - person_on_robot_collision
    - time_not_moving
    - static_obstacle_collision
```


## Add new metrics

The user can easily extend the current set of available metrics.

To do so, a new metric function must be implemented in the ```hunav_metrics.py``` Python file. 
Every metric function must take the following input parameters:

* ```robot``` a list of messages *hunav_msgs/msg/Agent* where all information recorded at each time step about the social robot is stored.
* ```agents``` a list of messages *hunav_msgs/msg/Agents* where all information recorded about the human agents are stored.

See the [hunav_msgs](https://github.com/robotics-upo/hunav_sim/tree/master/hunav_msgs) package to obtain more information about the data stored for each agent. 

After programming the new metric function, you must associate a name to it in the *metrics* list at the end of the ```hunav_metrics.py``` file. 

After compiling, the user can add the new function name to the ```metrics.yaml``` file.



## TODO:

* Program a visual interface (probably a RViz panel) to configure the evaluation and the metrics to be computed.
* Program a set of ROS2 services to start/stop the data recording.

## References:

[1] N. Perez-Higueras, F. Caballero, and L. Merino, "Teaching Robot Navigation Behaviors to Optimal RRT Planners," International Journal of Social Robotics, vol. 10, no. 2, pp. 235–249, 2018.

[2] N. Tsoi, A. Xiang, P. Yu, S. S. Sohn, G. Schwartz, S. Ramesh, M. Hussein, A. W. Gupta, M. Kapadia, and M. Vázquez, "Sean 2.0: Formalizing and generating social situations for robot navigation," IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 11 047–11 054, 2022.

[3] A. Biswas, A. Wang, G. Silvera, A. Steinfeld, and H. Admoni,"Socnavbench: A grounded simulation testing framework for evaluating social navigation," ACM Transactions on Human-Robot Interaction, jul 2022. [Online](https://doi.org/10.1145/3476413).

[4] Y. Gao and C.-M. Huang, "Evaluation of socially-aware robot navigation," Frontiers in Robotics and AI, vol. 8, 2022.[Online](https://www.frontiersin.org/articles/10.3389/frobt.2021.721317)


