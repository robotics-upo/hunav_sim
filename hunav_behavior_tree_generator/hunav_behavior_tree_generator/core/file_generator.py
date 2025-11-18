"""
File: file_generator.py
Main class for generating behavior tree files.
Mode of operation: from YAML scenario file or manual input.

Author: Quentin Dury
"""
import logging
import os
from typing import Dict, List, Optional

from hunav_behavior_tree_generator.config.config import (
    BT_OUTPUT_DIR,
    LOG_DIR,
    setup_logging,
    get_scenarios_directory
)
from .behavior import BehaviorManager
from .scenario_manager import ScenarioManager
from hunav_behavior_tree_generator.ui.interface_manager import InterfaceManager

logger = setup_logging()


class FileGenerator:
    """ Main class for generating behavior tree files. """

    def __init__(self, ui_manager: Optional[InterfaceManager] = None):
        """ Initialize the FileGenerator and ensure output directories exist. """
        self.ui_manager = ui_manager or InterfaceManager()
        self.scenario_manager = ScenarioManager()
        self.behavior_manager = BehaviorManager()
        self.generated_files: List[str] = []
        self.generation_info: Dict[int, Dict] = {}
        self._ensure_output_directories()
    
    # ===================================================
    #                   Private Methods
    # ===================================================
    
    def _ensure_output_directories(self):
        """ Create output directories if they don't exist. (private method) """
        os.makedirs(BT_OUTPUT_DIR, exist_ok=True)
        os.makedirs(LOG_DIR, exist_ok=True)

        logger.info("Check output directories -> DONE")

    def _clean_content(self, content: str) -> str:
        """ Clean content by removing markdown code blocks but keep indentation. (private method)
        Args:
            content (str): Raw content with possible markdown formatting
        Returns:
            str: Cleaned content without markdown formatting
        """
        content = content.replace("```xml", "").replace("```", "")
        logger.info("Content cleaning from markdown formatting -> DONE")
        return content


    def _save_behavior_file(self, content: str, agent_id: int, scenario_name: str) -> str:
        """ Save behavior tree content to file. (private method)
        Args:
            content (str): Behavior tree content
            agent_id (int): Agent identifier
            scenario_name (str): Scenario name
        Returns:
            str: Filename of the saved behavior tree
        """
        
        cleaned_content = self._clean_content(content)
        if scenario_name.endswith('.yaml'):
            scenario_name = scenario_name[:-5]

        filename = f"{scenario_name}__agent_{agent_id}_bt.xml"
        filepath = os.path.join(BT_OUTPUT_DIR, filename)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(cleaned_content)
        
        logger.info(f"Behavior Tree saving to file '{filename}' -> DONE")
        return filename

    def _ask_number_of_agents(self) -> int:
        """ Ask user for number of agents with console feedback. (private method)
        Returns:
            int: Number of agents provided by the user
        """
        while True:
            try:
                user_input = input("Enter the number of agents: ").strip()
                num = int(user_input)
                
                if num > 0:
                    print(f"✔ Number of agents set to {num}\n")
                    logger.info("Asking number of agents -> DONE")
                    return num
                else:
                    print("✗ Number of agents must be positive\n")
                    logger.error("Asking number of agents -> Negative number provided, retrying")
                    
            except ValueError:
                print("✗ Please enter a valid number\n")
                logger.error("Asking number of agents -> Invalid input, retrying")


    def _ask_scenario_name(self) -> str:
        """ Ask user for scenario name with console feedback. (private method)
        Returns:
            str: Scenario name provided by the user
        """
        while True:
            name = input("Enter the scenario name: ").strip()
            
            if name:
                print(f"✔ Scenario name set to: {name}\n")
                logger.info("Asking scenario name -> DONE")
                return name
            else:
                print("✗ Scenario name cannot be empty\n")
                logger.warning("Asking scenario name -> Empty input, retrying")


    def _ask_goals_for_agent(self) -> list[int]:
        """ Ask user for goals positions for an agent with console feedback. (private method)
        Returns:
            list[int]: List of goal numbers provided by the user
        """
        while True:
            goals = input("Enter the goals number for the agent (comma-separated): ").strip()

            try:
                goals_list = [int(x.strip()) for x in goals.split(",") if x.strip()]

                if len(goals_list) == 0:
                    print("✗ You must enter at least one number\n")
                    continue

                print(f"✔ Goals set for agent: {goals_list}\n")
                logger.info("Asking goals for agent -> DONE")
                return goals_list

            except ValueError:
                print("✗ Invalid format. Please enter numbers separated by commas\n")
                logger.error("Asking goals for agent -> Invalid input, retrying")


    def _get_list_of_unique_goals(self, goals_positions: list[list[int]]) -> list[int]:
        """ Return the list of unique goals across all agents. (private method) 
        Args:
            goals_positions (list[list[int]]): List of goals for each agent
        Returns:
            list[int]: List of unique goal numbers
        """
        all_goals = set()

        for agent_goals_list in goals_positions:
            for goal in agent_goals_list:
                all_goals.add(goal)
        logger.info("Getting list of unique goals -> DONE")
        return list(all_goals)

    def _ask_behavior_description(self, agent_id: int, agent_goals: list[int]) -> str:
        """ Ask user for behavior description for a specific agent with console feedback. (private method)
        Args:
            agent_id (int): Agent identifier
            agent_goals (list[int]): List of goal numbers for the agent
        Returns:
            str: Behavior description provided by the user
        """
        predefined_behaviors = ["Curious", "Regular", "Scared", "Surprised", "Threatening"]
        
        print(f"\n--- Agent {agent_id} ---")
        print(f"Goals assigned to this agent: {agent_goals}\n")
        
        print("Choose a predefined behavior or enter a custom one:")

        for idx, behavior in enumerate(predefined_behaviors, start=1):
            print(f"  {idx}. {behavior}")
        print(f"  {len(predefined_behaviors)+1}. Custom behavior description\n")
        
        while True:
            choice = input(f"Select behavior (1-{len(predefined_behaviors)+1}): ").strip()
            
            if choice.isdigit():
                choice = int(choice)
                if 1 <= choice <= len(predefined_behaviors):
                    print(f"✔ Predefined behavior selected: {predefined_behaviors[choice-1]}\n")
                    logger.info(f"Asking behavior description -> Predefined behavior '{predefined_behaviors[choice-1]}' selected")
                    return predefined_behaviors[choice-1]
                elif choice == len(predefined_behaviors) + 1:
                    custom_desc = input("Enter a custom behavior description (warning: may not work correctly in simulation): ").strip()
                    if custom_desc:
                        print(f"✔ Custom behavior provided: {custom_desc}\n")
                        logger.info("Asking behavior description -> Custom description provided")
                        return custom_desc
                    else:
                        print("✗ Custom behavior description cannot be empty\n")
                        logger.warning("Asking behavior description -> Empty custom description, retrying")
                else:
                    print(f"✗ Please enter a number between 1 and {len(predefined_behaviors)+1}\n")
                    logger.warning("Asking behavior description -> Invalid choice, retrying")
            else:
                print("✗ Invalid input. Please enter a number\n")
                logger.warning("Asking behavior description -> Non-numeric input, retrying")



    # ===================================================
    #                   Public Methods
    # ===================================================

    # Case 1 : Manual generation from user input
    def generate_BT_from_manual_input(self):
        """ Generate behavior trees using manual agent configuration. """
        try:
            # ---------- INITIALIZATION ----------
            self.ui_manager.show_start_screen(app_name="HuNav Behavior Tree Generator", version="1.0", subtitle="Initializing behavior tree generation system...")
            logger.info("Starting manual behavior tree generation")

            # ---------- AGENT CONFIGURATION ----------
            num_agents = self._ask_number_of_agents()
            goals_positions = []

            for i in range(num_agents):
                print(f"\n--- Agent {i+1} ---")
                goals_positions.append(self._ask_goals_for_agent())

            unique_goals = self._get_list_of_unique_goals(goals_positions)
            scenario_name = self._ask_scenario_name()

            self.ui_manager.show_scenario_info(scenario_name, num_agents, goals_positions)

            # ---------- GENERATION ----------
            for agent_id, agent_goals in enumerate(goals_positions, start=1):
                print(f"Generating behavior for Agent {agent_id} ({agent_id}/{num_agents})\n")
                user_behavior_desc = self._ask_behavior_description(agent_id, agent_goals)

                generated_bt = self.behavior_manager.generate_agent_behavior(
                    agent_id,
                    user_behavior_desc,
                    agent_goals
                )

                filename = self._save_behavior_file(generated_bt, agent_id, scenario_name)
                self.generated_files.append(filename)

                self.ui_manager.show_generation_complete(agent_id, filename)

            # ---------- FINAL SUMMARY ----------
            self.ui_manager.show_final_summary(self.generated_files, self.generation_info)
            logger.info("Behavior Tree generation -> COMPLETE")

        except KeyboardInterrupt:
            self.ui_manager.show_cancellation()
            logger.info("Generation cancelled by user.")
        except Exception as e:
            self.ui_manager.show_error(str(e))
            logger.error(f"Error during scenario generation: {e}")


    # Case 2 : Generation from YAML scenario file
    def generate_BT_from_yaml_file(self, scenario_name: Optional[str] = None):
        """Generate behavior trees from a YAML scenario file with console feedback."""
        
        try:
            # ---------- START SCREEN ----------
            self.ui_manager.show_start_screen(app_name="HuNav Behavior Tree Generator", version="1.0", subtitle="Initializing behavior tree generation system...")
            logger.info("Starting behavior tree generation from YAML scenario")

            # ---------- CHOOSE SCENARIO ----------
            if not scenario_name:
                scenario_path = self.scenario_manager.choose_scenario()
                if not scenario_path:
                    print("✗ No scenario selected. Exiting.\n")
                    logger.error("No scenario selected by user")
                    return
                scenario_name = os.path.splitext(os.path.basename(scenario_path))[0]
            else:
                scenarios_dir = get_scenarios_directory()
                scenario_path = os.path.join(scenarios_dir, scenario_name)
                if not os.path.exists(scenario_path):
                    print(f"✗ Scenario '{scenario_name}' not found. Exiting.\n")
                    logger.error(f"Scenario file not found: {scenario_path}")
                    return

            # ---------- LOAD SCENARIO ----------
            scenario_data = self.scenario_manager.load_scenario(scenario_path)
            if not scenario_data:
                print("✗ Failed to load scenario. Exiting.\n")
                logger.error("Failed to load scenario data")
                return

            # ---------- EXTRACT AGENTS AND GOALS ----------
            agents_info, agents_goals_list = self.scenario_manager.extract_agents_and_goals_for_generation(scenario_data, scenario_name)
            if not agents_info:
                print("✗ No agents found in scenario. Exiting.\n")
                logger.error("No agents found in scenario data")
                return

            # ---------- DISPLAY AGENTS INFO ----------
            self.ui_manager.show_agents_summary(agents_info)

            # ---------- ASK BEHAVIOR AND GENERATE ----------
            for agent_id in agents_info.keys():
                print(f"\nGenerating behavior for Agent {agent_id} ({agent_id}/{len(agents_info)})\n")
                
                user_behavior_desc = self._ask_behavior_description(agent_id, agents_goals_list[agent_id])

                generated_bt = self.behavior_manager.generate_agent_behavior(
                    agent_id,
                    user_behavior_desc,
                    agents_goals_list[agent_id]
                )

                filename = self._save_behavior_file(generated_bt, agent_id, scenario_name)
                self.generated_files.append(filename)

                self.ui_manager.show_generation_complete(agent_id, filename)

            # ---------- FINAL SUMMARY ----------
            self.ui_manager.show_final_summary(self.generated_files, self.generation_info)
            logger.info("Behavior Tree generation -> COMPLETE")

        except KeyboardInterrupt:
            self.ui_manager.show_cancellation()
            logger.info("Generation cancelled by user.")
        except Exception as e:
            self.ui_manager.show_error(str(e))
            logger.error(f"Error during scenario generation: {e}")
