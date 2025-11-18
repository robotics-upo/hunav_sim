"""
Simple terminal interface manager without Rich dependencies.

Author: Quentin Dury
"""
import time
from typing import List, Dict

class InterfaceManager:
    """ Simple terminal interface manager using basic print/input functions. """
    
    def __init__(self, width: int = 60):
        """ Initialize the interface manager. """
        self.width = width
        self.generation_start_time = None
        
    # ===================================================
    #                   Private Methods
    # ===================================================

    def _print_separator(self):
        """ Print a horizontal separator line. (private method) """
        print("=" * self.width)


    def _print_title(self, title: str):
        """ Print a centered title with separators. (private method) 
        Args:
            title (str): The title to print.
        """
        print("\n" + "=" * self.width)
        print(title.center(self.width))
        print("=" * self.width + "\n")


    def _print_info(self, label: str, value: str, label_width: int = 10):
        """ Print a label-value pair, aligned neatly. (private method) 
        Args:
            label (str): The label to print.
            value (str): The corresponding value.
            label_width (int, optional): Width for label alignment. Defaults to 10.
        """
        print(f"{label:<{label_width}} {value}")


    def _print_list(self, header: str, items: list, prefix: str = "• "):
        """ Print a list of items with an optional header. (private method) 
        Args:
            header (str): Header title for the list.
            items (list): List of items to print.
            prefix (str, optional): Prefix for each item. Defaults to "• ".
        """
        if not items:
            print(f"{header}: None")
            return
        print(f"{header}:")
        for idx, item in enumerate(items, start=1):
            print(f"  {prefix} {idx} -> {item}")

    # ===================================================
    #                   Public Methods
    # ===================================================

    def show_start_screen(self, app_name: str, version: str, subtitle: str = None):
        """ Display the start screen. 
        Args:
            app_name (str): Name of the application.
            version (str): Version of the application.
            subtitle (str, optional): Optional subtitle to display.
        """
        title = f"{app_name} v{version}"
        self._print_title(title)

        if subtitle:
            print(subtitle.center(self.width) + "\n")

        self._print_separator()
        print()


    def show_scenario_info(self, scenario_name: str, agents_count: int, goals_positions: list[list[int]] = None):
        """ Display scenario information. 
        Args:
            scenario_name (str): Name of the scenario.
            agents_count (int): Number of agents in the scenario.
            goals_positions (list[list[int]], optional): List of goal positions for each agent.
        """
        self._print_title("SCENARIO INFORMATION")
        self._print_info("Scenario:", scenario_name)
        self._print_info("Number of agents:", str(agents_count))
        print()

        if goals_positions:
            for idx, goals in enumerate(goals_positions, start=1):
                formatted_goals = ", ".join(map(str, goals))
                print(f"  Agent {idx:<2} -> {formatted_goals}")
        else:
            print("No goals positions provided.")

        self._print_separator()
        print()
    

    def show_agents_summary(self, agents_info: dict):
        """ Display a summary of all agents in a neat format. 
        Args:
            agents_info (dict): Dictionary containing agents' information.
        """
        self._print_title("AGENTS SUMMARY")

        if not agents_info:
            print("No agents information provided.\n")
            self._print_separator()
            return

        for agent_id, info in agents_info.items():
            init_pose = info.get('init_pose', {})
            x = init_pose.get('x', info.get('x', 'N/A'))
            y = init_pose.get('y', info.get('y', 'N/A'))

            self._print_info(f"Agent {agent_id} Position:", f"({x}, {y})")

            goals = info.get('goals', [])
            if goals:
                goals_str = ", ".join(map(str, goals))
                self._print_info(f"Goals ({len(goals)}):", goals_str)
            else:
                self._print_info("Goals:", "None")
        
            print()  
        self._print_separator()


    def show_agent_context(self, agent_id: int, goal_positions: list):
        """ Display agent context information (private method) in a consistent format. 
        Args:
            agent_id (int): ID of the agent.
            goal_positions (list): List of goal positions for the agent.
        """
        self._print_title(f"Agent {agent_id} Context")
        
        if goal_positions:
            goals_list = [str(g) for g in goal_positions]
            self._print_list("Goals", goals_list, prefix="  -")
        else:
            self._print_info("Goals:", "None")
        print()


    def show_extracted_info(self, agents_info: dict, scenario_name: str):
        """ Display extracted scenario information in a compact, readable format. 
        Args:
            agents_info (dict): Dictionary containing agents' information.
            scenario_name (str): Name of the scenario.
        """
        self._print_title(f"Scenario Loaded: {scenario_name}")
        agents_count = len(agents_info)
        self._print_info("Found agents:", f"{agents_count}")
        print()

        for agent_id, agent_info in agents_info.items():
            name = agent_info.get('name', 'Unnamed')
            
            print(f"▹ Agent {agent_id} ({name})")
            
            init_pose = agent_info.get('init_pose', {})
            x, y = init_pose.get('x', 0), init_pose.get('y', 0)
            self._print_info("Position:", f"({x:.1f}, {y:.1f})")
            
            goals = agent_info.get('goals', [])
            if goals:
                self._print_list("Waypoints", goals, prefix="  -")
            else:
                self._print_info("Waypoints:", "None")
            
            print()  

        self._print_separator()


    def show_scenario_selection_menu(self, scenarios: list[str]) -> int:
        """ Display scenario selection menu in a consistent style. 
        Args:
            scenarios (list[str]): List of available scenario names.
        Returns:
            int: Index of the selected scenario.
        """
        self._print_title("Available Scenarios")
        self._print_list("Scenarios", [f"{i}. {s}" for i, s in enumerate(scenarios, 1)], prefix="")

        prompt = f"Select scenario (1-{len(scenarios)}) [1]: "

        while True:
            try:
                user_input = input(prompt).strip()
                choice = int(user_input) if user_input else 1

                if 1 <= choice <= len(scenarios):
                    return choice - 1
                self._print_info("Error:", f"Please enter a number between 1 and {len(scenarios)}")

            except KeyboardInterrupt:
                raise
            except ValueError:
                self._print_info("Error:", "Invalid input. Please enter a number.")


    def show_generation_complete(self, agent_id: int, filename: str):
        """ Show generation completion in a consistent, left-aligned style. 
        Args:
            agent_id (int): ID of the agent.
            filename (str): Name of the generated behavior file.
        """
        message = f"✔ Agent {agent_id} behavior generated: {filename}"
        print()
        self._print_info("Success:", message)
        self._print_separator()


    def show_final_summary(self, generated_files: list[str], generation_info: dict):
        """ Display final generation summary in a neat format. 
        Args:
            generated_files (list[str]): List of generated file names.
            generation_info (dict): Dictionary containing generation statistics.
        """
        self._print_title("GENERATION COMPLETE")
        self._print_info("Files created:", str(len(generated_files)))
        self._print_list("Generated files", [f"{i}. {file}" for i, file in enumerate(generated_files, 1)], prefix="")
        print()
        
        if getattr(self, "generation_start_time", None):
            elapsed = time.time() - self.generation_start_time
            self._print_info("Total generation time:", f"{elapsed:.2f} seconds")
        
        self._print_separator()

    
    def show_error(self, error_message: str):
        """ Display an error message in a consistent style. 
        Args:
            error_message (str): The error message to display.
        """
        print()
        self._print_info("Error:", f"✗ {error_message}")
        

    def show_cancellation(self):
        """ Display cancellation message in a consistent style. """
        print()
        self._print_info("Cancelled:", "⚠ Generation cancelled by user")