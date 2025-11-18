"""
Scenario management functionality - Revised architecture.

Author: Quentin Dury
"""
import os
import yaml
import logging
from typing import List, Dict, Any, Tuple, Optional
from hunav_behavior_tree_generator.config.config import get_scenarios_directory
from hunav_behavior_tree_generator.ui.interface_manager import InterfaceManager

logger = logging.getLogger(__name__)

class ScenarioManager:
    """Manage scenario files and extract agent information."""
    
    def __init__(self):
        """Initialize the ScenarioManager."""
        self.scenarios_dir = get_scenarios_directory()
        self.interface_manager = InterfaceManager()
        
    # ===================================================
    #                   Private Methods 
    # ===================================================
    
    #/!\ Functions for extracting goals positions from YAML. Currently not used. /!\
    #def _extract_goals_info_from_yaml(self, scenario_data: Dict) -> Dict[str, Dict[str, float]]:
    #    """ Extract goal positions from scenario data. (private method)
    #    Args:
    #        scenario_data (Dict): Loaded scenario data
    #    Returns:
    #        Dict[str, Dict]: Extracted goals information
    #    """
    #    goals_info = {}
        
    #    try:
    #        params = scenario_data.get('hunav_loader', {}).get('ros__parameters', {})
    #        global_goals = params.get('global_goals', {})
            
    #        for goal_id, goal_data in global_goals.items():
    #            if isinstance(goal_data, dict):
    #                goals_info[str(goal_id)] = {
    #                    'x': goal_data.get('x', 0.0),
    #                    'y': goal_data.get('y', 0.0),
    #                    'h': goal_data.get('h', 0.0)
    #                }
                    
    #        logger.info(f"Extracted {len(goals_info)} goal positions")
    #        return goals_info
            
    #    except Exception as e:
    #        logger.error(f"Error extracting goals info: {e}")
    #        return {}


    def _extract_agents_info_from_yaml(self, scenario_data: Dict) -> Dict[int, Dict[str, Any]]:
        """ Extract agents information from scenario data. (private method)
        Args:
            scenario_data (Dict): Loaded scenario data
        Returns:
            Dict[int, Dict]: Extracted agents information
        """
        agents_info = {}
        try:
            params = scenario_data.get('hunav_loader', {}).get('ros__parameters', {})
            agent_names = params.get('agents', [])
            logger.info(f"Found agent names: {agent_names}")
            
            for agent in agent_names:
                agent_data = params.get(agent, {})
                
                if not isinstance(agent_data, dict):
                    logger.warning(f"Agent '{agent}' has invalid data format: {agent_data}")
                    continue

                agent_id = agent_data.get('id', None)
                if agent_id is None:
                    logger.warning(f"Could not determine agent ID for: {agent}")
                    continue
                
                name = agent_data.get('name', agent)

                agents_info[agent_id] = {
                    'name': name,
                    'init_pose': agent_data.get('init_pose', {'x': 0.0, 'y': 0.0, 'h': 0.0}),
                    'max_vel': agent_data.get('max_vel', 1.0),
                    'goal_radius': agent_data.get('goal_radius', 0.5),
                    'cyclic_goals': agent_data.get('cyclic_goals', False),
                    'goals': agent_data.get('goals', [])
                }
                
                goal_count = len(agent_data.get('goals', []))
                logger.info(f"Extracted agent {agent_id} ('{name}') with {goal_count} goals")
            
            logger.info(f"Successfully extracted information for {len(agents_info)} agents")
            return agents_info

        except Exception as e:
            logger.error(f"Error extracting agents info: {e}")
            raise


    def _get_available_scenarios(self) -> List[str]:
        """ Get list of available scenario files. (private method) 
        Returns:
            List[str]: List of scenario filenames
        """
        try:
            if not os.path.exists(self.scenarios_dir):
                logger.warning(f"Scenarios directory not found: {self.scenarios_dir}")
                return []
            
            scenarios = [
                f for f in os.listdir(self.scenarios_dir) 
                if f.endswith('.yaml') or f.endswith('.yml')
            ]
            
            logger.info(f"Found {len(scenarios)} scenario files")
            return sorted(scenarios)
            
        except Exception as e:
            logger.error(f"Error listing scenarios: {e}")
            return []


    def _extract_agents_and_goals(self, scenario_data: dict, scenario_name: str) -> dict[int, dict]:
        """ Extract agents and goals information from scenario with console feedback. (private method)
        Args:
            scenario_data (dict): Loaded scenario data
            scenario_name (str): Name of the scenario
        Returns:
            dict[int, dict]: Extracted agents information
        """
        print("\nExtracting Information")
        
        agents_info = self._extract_agents_info_from_yaml(scenario_data)   
        if not agents_info:
            print("✗ No agents found in scenario\n")
            return {}
        
        self.interface_manager.show_extracted_info(agents_info, scenario_name)
        
        print(f"✔ Extracted {len(agents_info)} agent{'s' if len(agents_info) > 1 else ''}\n")
        return agents_info

    # ===================================================
    #                   Main Methods 
    # ===================================================
    
    def choose_scenario(self) -> Optional[str]:
        """ Let user choose a scenario file in case of scenario not already selected. 
        Returns:
            Optional[str]: Path to the selected scenario file or None if cancelled
        """
        print("\nScenario Selection")
        
        scenarios = self._get_available_scenarios()
        if not scenarios:
            print("✗ No scenario files found!")
            print(f"Looking in: {self.scenarios_dir}")
            return None
        
        selected_id = self.interface_manager.show_scenario_selection_menu(scenarios)
        selected_scenario = scenarios[selected_id]
        scenario_path = os.path.join(self.scenarios_dir, selected_scenario)
        
        print(f"✔ You selected: {selected_scenario}\n")
        logger.info(f"Selected scenario: {selected_scenario}")
        
        return scenario_path

    
    def load_scenario(self, scenario_path: str) -> Optional[dict]:
        """ Load scenario data from YAML file with console feedback.
        Args:
            scenario_path (str): Path to the scenario YAML file
        Returns:
            Optional[dict]: Loaded scenario data or None if error
        """
        print("\nLoading Scenario Data")
        try:
            with open(scenario_path, 'r', encoding='utf-8') as file:
                scenario_data = yaml.safe_load(file)
            
            print(f"✔ Scenario loaded successfully: \n{scenario_path}\n")
            logger.info(f"Successfully loaded scenario: {scenario_path}")
            return scenario_data
            
        except FileNotFoundError:
            msg = f"✗ Scenario file not found: {scenario_path}"
        except yaml.YAMLError as e:
            msg = f"✗ Error parsing YAML file: {e}"
        except Exception as e:
            msg = f"✗ Error loading scenario: {e}"

        print(msg + "\n")
        logger.error(msg)
        return None

    
    def extract_agents_and_goals_for_generation(self, scenario_data: Dict, scenario_name: str) -> Tuple[Dict[int, Dict], Dict[int, List[Dict]]]:
        """ Extract agents and prepare goals list for each agent for behavior generation.      
        Args:
            scenario_data (Dict): Loaded scenario data
            scenario_name (str): Name of the scenario
        Returns:
            Tuple[Dict[int, Dict], Dict[int, List[Dict]]]: (agents_info, goals_list_per_agent)
        """
        agents_info_from_scenario = self._extract_agents_and_goals(scenario_data, scenario_name)
        
        agents_goals_list = {}
        for agent_id, agent_info in agents_info_from_scenario.items():
            agent_goals = agent_info.get('goals', [])
            agents_goals_list[agent_id] = agent_goals
        
        return agents_info_from_scenario, agents_goals_list
        