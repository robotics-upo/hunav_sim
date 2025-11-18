"""
Behavior generation functionality - Core functions.

Author: Quentin Dury
"""
import logging
from typing import Tuple

from hunav_behavior_tree_generator.utils.llm_utils import LLMManager
from hunav_behavior_tree_generator.ui.interface_manager import InterfaceManager

logger = logging.getLogger(__name__)

class BehaviorManager:
    """Manages behavior generation for agents."""
    
    def __init__(self):
        """Initialize the BehaviorManager."""
        self.llm_manager = LLMManager()
        self.interface_manager = InterfaceManager()
    
    # ===================================================
    #                   Private Methods
    # ===================================================
    
    def _prepare_llm_context(self, agent_id: int, behavior_desc: str, goal_positions: dict) -> str:
        """ Prepare context for LLM generation. (private method)
        Args:
            agent_id (int): Agent identifier
            behavior_desc (str): Behavior description
            goal_positions (dict): Goal positions information
        Returns:
            Tuple[str, list]: (prepared_prompt, context_list)
        """
        context_parts = []
        
        if goal_positions :
            goal_info = f"Agent {agent_id} goals: {goal_positions}"
            context_parts.append(goal_info)

        context_str = " | ".join(context_parts) if context_parts else ""
        full_prompt = f"{behavior_desc}"
        if context_str:
            full_prompt += f" | Context: {context_str}"
        
        return full_prompt
        
    def _split_llm_response(self, llm_response: str) -> str:
        """ Split LLM response to extract valid behavior content. (private method)
        Args:
            llm_response (str): Raw response from the LLM
        Returns:
            str: Extracted valid behavior content
        """
        llm_response = llm_response.strip()
        if not llm_response:
            return ""

        end_tag = "</root>"
        idx = llm_response.rfind(end_tag)

        if idx == -1:
            return llm_response

        return llm_response[:idx + len(end_tag)].strip()
    
    # ===================================================
    #                   Main Methods
    # ===================================================

    def generate_agent_behavior(self, agent_id: int, behavior_desc: str, goal_positions: dict) -> str:
        """ Generate behavior tree for a specific agent.
        Args:
            agent_id (int): Agent identifier
            behavior_desc (str): Natural language description of the behavior
            goal_positions (dict): Goal positions for the agent
        Returns:
            str: behavior_content (XML)
        """
        logger.info(f"Generating behavior for agent {agent_id}")

        self.interface_manager.show_agent_context(agent_id, goal_positions)
        prepared_prompt = self._prepare_llm_context(agent_id, behavior_desc, goal_positions)
        llm_response = self.llm_manager.generate_behavior_tree(prepared_prompt, )
        
        if not llm_response:
            logger.error(f"No response from LLM for agent {agent_id}")
            raise ValueError(f"Failed to generate behavior for agent {agent_id}")

        generated_behavior_tree_content = self._split_llm_response(llm_response)
        
        if not generated_behavior_tree_content:
            logger.error(f"Empty behavior content for agent {agent_id}")
            raise ValueError(f"Generated behavior is empty for agent {agent_id}")
        
        logger.info(f"Successfully generated behavior for agent {agent_id}")
        return generated_behavior_tree_content


