# HuNav RViz2 Panel

A comprehensive ROS2 C++ package providing interactive RViz2 panels for configuring and managing human agents in the HuNavSim environment.

This package provides two specialized RViz2 panels:

* **HuNavPanel** (`ActorPanel`) - Advanced agent configuration interface for creating/editing human agents, visual goal placement, YAML configuration management, and behavior tree XML generation
* **HuNavMetricsPanel** (`MetricsPanel`) - Interactive metrics selection interface for simulation evaluation and analysis

## Features

* **Interactive Agent Creation**: Visual pose setting and goal placement using RViz tools
* **Behavior Tree Integration**: Automatic XML generation with Groot2 editor support
* **Advanced Force Model Configuration**: Social Force Model parameters with guided defaults
* **Real-time Visualization**: 3D agent markers, goal indicators, and navigation paths

**Tested in ROS2 Humble**

## Dependencies

### System Dependencies

* `ros-humble-nav2-map-server` - Map loading and visualization
* `ros-humble-nav2-lifecycle-manager` - Node lifecycle management  
* `ament_index_cpp` - Package resource location
* `yaml-cpp` - YAML file parsing and generation
* `qtbase5-dev` - Qt5 development libraries
* `libqt5xml5-dev` - Qt5 XML support

### ROS2 Package Dependencies

* Core: `rclcpp`, `rclcpp_lifecycle`, `std_msgs`, `geometry_msgs`
* Navigation: `nav2_msgs`, `nav_msgs`, `tf2_geometry_msgs`
* Visualization: `rviz_common`, `rviz_default_plugins`, `visualization_msgs`

## Quick Start

### Launch the Panel

A launch file is provided for easy startup:

```bash
ros2 launch hunav_rviz2_panel hunav_rviz2_launch.py
```

This will open RViz2 with the HuNav panels loaded, allowing you to start configuring agents and metrics.

### Interface Overview

After launching, you'll see the RViz2 interface with the HuNav panels available:

![RViz Interface](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/rviz.png)

## HuNavPanel - Agent Configuration

The main panel for creating/editing and managing human agents with comprehensive configuration options.

![Main Panel Interface](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/main_panel.png)

### Panel Sections

The panel is organized into functional groups:

**1. File Operations**

* **Create agents YAML**: Switch to creation mode for new configurations
* **Load agents YAML**: Load existing agent configurations from file

**2. Simulator and Map Selection**

* **Simulator Selection**: Choose from supported simulators (Gazebo [Classic](https://github.com/robotics-upo/hunav_gazebo_wrapper) | [Fortress](https://github.com/robotics-upo/hunav_gazebo_fortress_wrapper), [Isaac Sim](https://github.com/robotics-upo/Hunav_isaac_wrapper), and Webots)
* **Map Selection**: Browse and load 2D occupancy maps

**3. Agent Creation**

* **Agent Count**: Set number of agents to generate
* **Generate agents**: Launch the agent configuration process

**4. Goal Definition**

* **Goal-Picking Mode**: Interactive goal placement system
* **Goal Assignment**: Assign goals to specific agents with visual feedback
* **Goal List**: Real-time display of placed goals

**5. Save and Panel Resetting**

* **Default Directory**: Option to use standard save location
* **Save agents YAML/Generate BTs**: Generate XML files and save configurations
* **Reset**: Clear all panel data and return to initial state

**6. Behavior Tree Configuration**

* **Edit in Groot2**: Launch behavior tree editor for visualizing and editing generated trees

##

### Usage Workflow

#### Creating New Agent Configurations

**Step 1: Environment Setup**

The *Simulator & Map* section allows selection of the simulation backend—Gazebo (Classic/Fortress), Isaac Sim, or Webots—and loads any standard 2D occupancy map supported by nav2_map_server. Once loaded, the map is visualized directly in RViz2 as the interactive workspace.

![Simulator Selection](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/environment_configuration.gif)

1. **Select Simulator**: Choose target simulation environment
2. **Load Map**: Click "Select map" to choose navigation environment
   * Supports standard ROS map formats (.yaml, .pgm)
   * Map automatically loads in RViz for visualization
3. **Set Agent Count**: Enter desired number of agents
4. **Generate agents**: Click to start the sequential agent configuration process

**Step 2: Sequential Agent Configuration and Position Setting**

Agents are configured sequentially via a dedicated dialog window. Each dialog integrates parameter configuration, appearance selection, and interactive placement. After confirming each agent, the system automatically advances to the next until the full population is complete.

![Agent Configuration and Position Setting](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/agent_creation.gif)

**Basic Parameters:**

* **Desired Velocity**: Maximum agent speed (m/s)
* **Behavior Type**: Choose from 6 navigation behaviors:
  * *Regular* - Standard navigation behavior
  * *Impassive* - Robot-aware but non-reactive navigation
  * *Surprised* - Stop and observe behavior when detecting robot
  * *Scared* - Avoid robot with increased velocity
  * *Curious* - Approach robot with controlled distance
  * *Threatening* - Block robot path behavior
* **Behavior Configuration**: Choose parameter source:
  * *Default* - Optimized preset values  
  * *Custom* - Manual fine-tuning with parameter guidance
  * *Random-normal* - Gaussian distribution around optimal values
  * *Random-uniform* - Equal probability across parameter ranges

**Behavior-Specific Parameters** (automatically shown based on behavior type):

* **Duration**: Behavior reaction duration (for Surprised, Scared, Curious, Threatening)
* **Only Once**: Whether reaction occurs only once per robot encounter
* **Visibility Distance**: Detection range for robot presence
* **Agent Velocity**: Behavior-specific movement speed (for Scared, Curious)
* **Front Distance**: Goal placement distance in front of robot (for Threatening)
* **Stop Distance**: Distance at which agent stops when robot is detected (for Curious)

**Customizable Force Model Parameters** (when Custom is selected):

* **Goal Force Factor**: Attraction strength toward navigation goals
* **Obstacle Force Factor**: Repulsion strength from obstacles  
* **Social Force Factor**: Human-human interaction forces
* **Other force factor**: Extra repulsive force (specific to the Scared behavior)

**Visual Appearance** (simulator-specific):

* A selection of agent avatars is available based on the chosen simulator. Options vary by platform and include diverse character types.

**Interactive Position Setting:**

1. Click **"Set initial pose"** in the agent configuration window
2. **Automatic Tool Switch**: RViz automatically activates "HunavGoals" tool
3. **Interactive Placement**: Click and drag on map to set agent position and orientation
4. **Immediate Feedback**: 3D human marker appears instantly at selected location

**Step 3: Goal-Picking Mode (after all agents configured)**

![Goal Picking Interface](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/goal_picking_mode.gif)

Once all agents are placed, Goal-Picking Mode is activated. Clicking on the map places a goal (visualized as a marker). Each goal is added to a list and can be reassigned or repositioned at any time. Goals are not locked until they are assigned to an agent.

1. Click **"Enter Goal-Picking Mode"** to activate interactive goal placement
2. **RViz Tool Integration**: Panel automatically switches to "PublishPoint" tool
3. **Interactive Placement**: Click locations on map to place goal markers
4. **Goal Editing**: Click existing goal markers to change their location
5. **Real-time List**: Goal coordinates appear in panel's goal list widget

**Step 4: Goal Assignment Dialog**

The dialog presents a dual-list interface. Goals can be freely moved between "available" and "assigned" lists. When a goal is locked for an agent, its corresponding path is visualized immediately in RViz2 for feedback.

![Goal Assignment Dialog](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/goal_assignment.gif)

1. Once finished picking goals, click **"Assign goals to agents"** to open assignment dialog
2. **Agent Selection**: Use dropdown to select which agent to configure
3. **Available/Assigned Lists**: Two-panel interface showing available goals and assigned goals
4. **Arrow Buttons**: Use ▶ and ◀ buttons to move goals between lists  
5. **Lock Selection**: Click "Lock Selection" to confirm goals for current agent
6. **Assignment summary**: Assigned goals are displayed at the bottom of the dialog

**Step 5: Final File Generation**

![Behavior Tree Integration](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/behavior_tree_section.png)

1. **Save agents YAML/Generate BTs**: Generates both YAML configuration and XML behavior trees
2. **File Naming Dialog**: Prompts for configuration name
3. **Reset Panel**: "Reset" button available to clear all data and return to initial state
4. **Groot2 Integration**: Direct launch of behavior tree editor for visualization

Saving the scenario performs two operations:

* A **scenario YAML** file is saved as:
`[map]_agents_*.yaml` under the `/scenarios/` folder of the selected simulator wrapper.
* An individual **BT XML** is generated **per agent** as:
`[yaml basename]__agent_[id]_bt.xml` under `/behavior_trees/`.

A shortcut button is available to launch **Groot2**, allowing immediate **graphical inspection or manual refinement of each agent's behavior tree**.

##

#### Editing Existing Configurations

Opening an existing YAML will restore all agents and goals and reinstate any previous assignments and goal paths. All elements can be freely modified with immediate visual feedback, making it easy to perform ablation studies, scenario tweaks, or behavior testing without starting from scratch.

![Loaded Configuration](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/rviz_load_agents.png)

**Loading and Visualization:**

1. Click **"Load agents YAML"** and select simulator to browse existing configurations
2. Select `.yaml` file from the selected simulator wrapper's `/scenarios/` directory
3. **Automatic Visualization**: All agents, positions, goals, and paths appear immediately on the map

**Edit Mode Features:**

* **"Edit agents" Button**: Enters sequential agent editing mode with navigation controls
* **Agent Navigation**: Arrow buttons (◀ ▶) to move between existing agents

**Goal Management in Edit Mode:**

* **"Edit Goals" button**: Enter goal-picking mode to add new goals or move existing ones
* **"Reset Goals" button**: Clear all loaded goals and their visualizations

## Behavior Tree Generation

The HuNavPanel automatically generates behavior tree XML files for each configured agent based on their behavior type.

### Key Features

* **Automatic XML Generation**: Individual behavior tree files created per agent
* **Groot2 Integration**: Direct editor launch for tree visualization and editing
* **Behavior-Specific Templates**: Different XML templates for each navigation behavior
* **Goal Sequence Integration**: Agent-specific goal navigation patterns embedded in trees

### Generated Files

Files are stored in the selected simulator wrapper's `/behavior_trees/` directory with naming pattern: `[yaml basename]__agent_[agent ID]_bt.xml`

## HuNavMetricsPanel - Metrics Configuration

This panel provides an interface for selecting which metrics to compute during simulation.

![Metrics Panel](https://github.com/robotics-upo/hunav_sim/blob/BT_manual_generation/hunav_rviz2_panel/images/metrics_panel.png)

### Metrics Panel Features

* **Interactive Selection**: Checkbox interface for metric selection
* **Configuration Persistence**: Saves selections to `metrics.yaml` file

### Usage

1. Each metric is shown as a checkbox that can be selected or unselected
2. The panel loads the current configuration from `metrics.yaml` in the install directory
3. After making selections, press the **"Save metrics"** button to store changes

## Configuration Files

The panel generates standard ROS 2 configuration files for agent simulation and metrics evaluation.

### Agent Configuration (agents_*.yaml)

**Format**: Standard ROS 2 parameter structure

**Structure:**

* **Global Settings**: `simulator`, `map`, `yaml_base_name`, `publish_people`
* **Global Goals**: Shared coordinates library with unique IDs
* **Agents List**: Array of agent identifiers  
* **Agent Config**: Individual settings (ID, velocities, pose, behavior, assigned goals)

### Metrics Configuration (metrics.yaml)

**Metric Categories:**

* **Navigation**: Goal completion, path efficiency, movement time
* **Social Interaction**: Personal space respect, human-robot distances
* **Safety**: Collision detection, movement patterns
* **Performance**: Speed, acceleration, trajectory smoothness

## Available Maps

Maps are loaded from simulator-specific wrapper directories based on the selected simulator:

### Gazebo Simulator Maps

Located in `hunav_gazebo_wrapper` ROS2 package share directory under `/maps/`:

* `bookstore` - Bookstore scenario
* `cafe` - Coffee shop environment  
* `house` - Residential environment
* `warehouse` - Industrial warehouse setting

### Isaac Sim Maps

Located in Isaac Sim wrapper `/maps/` directory:

* `warehouse` - Large industrial warehouse environment
* `hospital` - Medical facility with multiple rooms and corridors  
* `office` - Modern office building with cubicles and meeting rooms

### Webots Maps

Located in Webots wrapper `/maps/` directory:

* TODO: Maps specific to the Webots simulation environment

## Troubleshooting

If you encounter issues with the HuNav RViz2 Panel, consider the following:

* **Dependencies**: Ensure all system and ROS2 package dependencies are installed correctly
* **Workspace Setup**: Verify your ROS2 workspace is properly sourced (`source install/setup.bash`)
* **Panel Not Loading**: Check for errors in the terminal output when launching RViz2
* **Configuration Issues**: If YAML files fail to load, ensure they are correctly formatted and located in the expected directories
* **Behavior Tree Errors**: If Groot2 fails to launch, ensure it is installed and it is located in `~/Groot2/bin/groot2`.

## Maintainer

Miguel Escudero Jiménez (<mescjim@upo.es>)
