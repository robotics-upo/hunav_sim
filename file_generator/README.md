# 📘 Documentation: Automatic Behavior and Environment Generation for HunavSim

## 1. Introduction
### **Context — Why was this code developed ?**
This code was developed as part of HunavSim v2.0, an open-source human navigation simulator within the euROBIN Network, a European project aimed at advancing AI tools, software, architectures, and hardware components through a reproducible and collaborative approach.

HunavSim serves as a benchmarking platform for human-aware navigation, providing controlled environments to test, compare, and evaluate algorithms for robotic navigation in human-populated spaces.

The contribution of this code lies in extending HunavSim with automatic generation of environments and behavior trees using Large Language Models (LLMs).

---
### **Objective — What is the main goal?**

The primary objective of this code is to implement **automatic file generation** using **Large Language Models (LLMs)**. The system generates:
- **Environment world files (.sdf)**
- **Behavior tree files (.xml)**
These files can then be integrated directly into HunavSim. From a simple **user prompt**, the tool generates simulation-ready files, enabling:
- A wider variety of **simulation scenarios**
- Greater **flexibility** in both environments and agent behaviors
- The ability to **scale complexity**, moving beyond predefined scenarios toward richer, user-defined experiments

---
## 2. Model and Methodology
This project uses Qwen2.5-VL-32B-Instruct-AWQ, a state-of-the-art multimodal large language model developed by Alibaba Cloud. The model was chosen for its scale, multimodal capabilities, instruction alignment, and efficient deployment through quantization.

### Core Characteristics
- **Model size**: 32 billion parameters, enabling advanced reasoning and robust multimodal understanding
- **Multimodal capacity**: processes both text and visual data (document interpretation, visual QA, math with diagrams, etc.)
- **Instruction tuning**: optimized to follow structured instructions and generate reliable responses
- **AWQ Quantization (Activation-Aware Weight Quantization)**: compressed into efficient int4 format, preserving high quality while reducing memory usage and inference latency

### Advantages
- **High performance**: competitive results on multimodal benchmarks (MMBench, DocVQA, MathVista, etc.) while remaining strong on pure text tasks
- **Efficiency**: AWQ quantization reduces the computational footprint, enabling deployment of large-scale models on moderately resourced servers
- **Versatility**: unifies language and vision tasks in a single model, avoiding the complexity of managing separate specialized models
- **Alignment**: designed to handle interactive, user-facing tasks with reliability and structure

### Justification of Choice
- **Balanced efficiency and performance**: practical deployment of a 32B parameter model without excessive hardware requirements
- **Multimodal support**: one model for both text and image processing, simplifying system design
- **Instruction alignment**: ensures consistent and structured responses suitable for code and file generation tasks
- **Robustness**: optimized AWQ version simplifies integration and server hosting

In summary, Qwen2.5-VL-32B-Instruct-AWQ represents an optimal compromise between scalability, performance, and efficiency, making it ideal for HunavSim’s behavior and environment generation system.

## 3. Code Overview and Usage
### Execution Flow
The program is structured around a **menu-based interface**, where users can choose actions through a command-line menu:

```
Main Menu
├── World Menu
│   ├── Generate
│   ├── Adjust
│   ├── Save
│   ├── Back
│   └── Quit
├── Behavior Menu
│   ├── Generate
│   ├── Save
│   ├── Back
│   └── Quit
└── Quit
```
---
### User guide
#### Requirements 
The project should be run inside a Python virtual environment with the following dependencies:

```bash
openai==1.97.0
python-dotenv==1.1.1
questionary==2.1.1
rich==14.1.0
```
All libraries are listed in requirements.txt.

#### Installation instructions
1. **Create a virtual environment (recommended)**:
```python
python3 -m venv filegen_venv
source filegen_venv/bin/activate
```

2. **Clone the repository:**
Once the virtual environment is installed (or not), you need to clone the hunavsim repo.

Go to the desired location for the project and run the following commands:
```bash
git clone https://github.com/robotics-upo/hunav_sim.git
git checkout "LLM_map_&_BT_generation"
```

3. **Install dependencies:**
Once the repo is cloned and placed on the corresponding branch, install the required libraries using the following commands:
```bash
cd hunav_sim/file_generator
pip install -r requirements.txt
```

4. **Configure connection to the LLM server:**
To enable the connection to the server, it's necessary to modify the variable `LLM_API_BASE_URL` in the `.env` file to put the connection address corresponding to the computer on which the program is executed.

--- 
#### Running the programm
To launch the program, you must be in the `/file_generator` folder then run the main program using the following command:

`python3 main.py`

The main menu will be displayed, allowing the user to select between world file generation and behavior tree generation.

---
### Menu Functions
#### World Menu
- **Generate Environment**: input a text description (prompt) to generate a .sdf file containing the requested environment. Saved by default as data/generated_world.sdf.
- **Adjust Environment**: modify the generated .sdf file by providing adjustments (e.g., reposition models, change orientation).
- **Save As**: save the environment under a new filename to preserve the current version.
- **Back / Quit**: return or exit the program.
#### Behavior Menu
- **Generate Behavior Tree**: input a prompt describing an agent’s behavior, generating an .xml file (default: data/generated_BT.xml).
- **Save As**: preserve the behavior tree under a new filename.
- **Back / Quit**: return or exit the program.

---

#### Input and Output Examples
**Environment Generation**
- **User prompt**:
    "An environment with 2 cars and a person in front of the cars"
- **Generated output**:
    A .sdf file including the models with their placement (cars and a standing person) :

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="generated_world">

    ...
    base_world.sdf content
    ...

   <!-- Models will be inserted here -->
  
    <include>
      <name>Car</name>
      <pose>2.0 3.0 0.0 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV</uri>
      <scale>1.0</scale>
    </include>
    
    <include>
      <name>Car</name>
      <pose>-2.0 -3.0 0.0 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV</uri>
      <scale>1.0</scale>
    </include>
    
    <include>
      <name>Standing_person</name>
      <pose>0.0 0.0 0.0 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person</uri>
      <scale>1.0</scale>
    </include>
    </world>
</sdf>
```

**Behavior Tree Generation**
- **User prompt**:
    "An agent that navigates to a goal and updates its goal when reached."
- **Generated output**:
    An .xml behavior tree defining navigation and goal-updating logic :

```xml
<?xml version='1.0' encoding='UTF-8'?>
<root main_tree_to_execute="RegularNavTree" BTCPP_format="4">

    <TreeNodesModel>
        <Action ID="RegularNav">
            <input_port name="agent_id" type="int">identifier of the agent</input_port>
            <input_port name="time_step" type="double">time step in seconds to compute movement</input_port>
        </Action>
        <Action ID="UpdateGoal">
            <input_port name="agent_id" type="int">identifier of the agent</input_port>
        </Action>
        <Condition ID="IsGoalReached">
            <input_port name="agent_id" type="int">identifier of the agent</input_port>
        </Condition>
    </TreeNodesModel>

    <BehaviorTree ID="RegularNavTree">
        <Fallback name="RegularNavFallback">
            <Sequence name="RegularNavigation">
                <Inverter>
                    <IsGoalReached agent_id="{id}" />
                </Inverter>
                <RegularNav agent_id="{id}" time_step="{dt}" />
            </Sequence>
            <UpdateGoal agent_id="{id}" />
        </Fallback>
    </BehaviorTree>
</root>
```

## 4. Technical Decisions
### Architecture
A **layered architecture** was adopted to separate concerns:
- **Interface layer**: user interaction via command-line menus
- **Core logic layer**: file generation, adjustment, and behavior tree management
- **Utility layer**: supporting functions (logging, LLM interaction, file handling)

```bash
file_generator/
│── main.py                         # Program entry point (user request, generation launch)
│── README.md                       # Project documentation
│── requirements.txt                # Python Dependencies
│── .env                            # Environment variables (LLM config, default paths, etc.)
│
├── config/                         # Configuration management
│   ├── logging_config.py           # Logging settings (format, levels, log rotation)
│   └── settings.py                 # Environment variables + default values
│
├── core/                           # Business logic (core of the project)
│   ├── adjustor.py                 # Adjustment of generated worlds (size, placement, etc.)
│   ├── behavior.py                 # Management/implementation of behaviors (e.g. behavior trees)
│   ├── generator.py                # Main world generator .sdf
│   └── prompts_manager.py          # Loading/managing prompts
│
├── utils/                          # Utility functions
│   ├── file_utils.py               # File management (reading, writing, paths, etc.)
│   ├── llm_utils.py                # Functions for interacting with the LLM
│   ├── menu_utils.py               # Functions for CLI menus / user interaction
│   └── sdf_utils.py                # SDF format specific functions
│
├── data/                           # Persistent data
│   ├── base_world.sdf              # Basic generic world
│   ├── generated_BT.xml            # Last behavior tree generated
│   ├── generated_world.sdf         # Last generated world
│   ├── saved_BT/                   # Directory of saved behavior trees
│   └── saved_worlds/               # Directory of saved .sdf worlds
│
├── prompts/                        # Prompts used to guide the LLM
│   ├── adjust_world.txt
│   ├── bt_generation.txt
│   ├── json_generate.txt
│   ├── model_mapper.txt
│   ├── placement.txt
│   ├── rescale.txt
│   └── world_file_generation.txt
│
└── logs/                           # Application logs
    └── app.log
```
---
**Libraries choices** :
- **User Interface**:
    - `questionary` → menu navigation with keyboard arrows
    - `rich` → structured and colored output for a clearer CLI experience
- **Logging**:
    - Python’s built-in `logging` → provides reliable and flexible logging without external dependencies
- **LLM Interaction**:
    - `openai` → compatible with Qwen’s API, simple high-level interface

---
## 5. Challenges and Solutions
- Limited model database  
    → Solution: prioritize validated models
- Models sometimes “fall through” the environment floor  
    → Solution: restrict to pre-tested usable models
- Model names too complex for LLM to handle consistently  
    → Solution: provide explicit examples in prompts
- Author names too complex for LLM-based file generation  
    → Solution: currently restricted to OpenRobotics models

---
## 6. Possible Improvements
- Add a local model database within the project or server to improve reliability and performance (storage and optimization challenges to be considered).
- Expand compatibility to support multiple model providers beyond OpenRobotics.
- Enhance the adjustment system with more fine-grained control for users.

## 7. Conclusion
This project introduces **automatic environment and behavior generation** for HunavSim using a state-of-the-art LLM.
**Key contributions:**
- Automated `.sdf` and `.xml` file generation from simple text prompts
- Modular, layered architecture for maintainability and extensibility
- Practical integration of a large multimodal LLM through efficient quantization
**Lessons learned:**
- Careful model selection and prompt design are critical for reliability
- Standardization of models and naming conventions reduces errors
- User-facing flexibility must be balanced with system stability
This system broadens the scope of HunavSim, enabling more **flexible, varied, and complex simulation scenarios**.