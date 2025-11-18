"""
LLM communication utility functions.

Author: Quentin Dury
"""
import logging
import os
from openai import OpenAI
from typing import List, Dict
from hunav_behavior_tree_generator.config.config import (
    PROMPTS_DIR,
    LLM_API_KEY,
    LLM_API_BASE_URL,
    LLM_MODEL,
    LLM_MAX_TOKENS,
    LLM_TEMPERATURE,
    TEMPLATES_DIR
)

logger = logging.getLogger(__name__)


class LLMManager:
    """ Manages LLM communication for behavior tree generation. """
    
    def __init__(self):
        """ Initialize LLM client. """
        self.client = OpenAI(
            api_key=LLM_API_KEY,
            base_url=LLM_API_BASE_URL
        )
    
    # ===================================================
    #                   Private Methods
    # ===================================================
    
    def _load_prompt_template(self, prompt_name: str) -> str:
        """ Load a prompt template by name. (private method)
        Args:
            prompt_name (str): Name of the prompt template file (without extension).
        Returns:
            str: The content of the prompt template.
        """
        prompt_path = os.path.join(PROMPTS_DIR, f"{prompt_name}.txt")

        if not os.path.isfile(prompt_path):
            raise FileNotFoundError(f"Prompt template '{prompt_name}.txt' does not exist in directory: {PROMPTS_DIR}")

        try:
            with open(prompt_path, "r", encoding="utf-8") as f:
                return f.read()
        except Exception as e:
            raise RuntimeError(f"Failed to load prompt '{prompt_name}': {e}")


    
    def _get_response(self, messages, top_p=0.9):
        """ Get response from LLM given messages. (private method)
        Args:
            messages (List[Dict[str, str]]): List of messages for the chat completion.
            top_p (float): Top-p sampling parameter.
        Returns:
            str: The content of the LLM response.
        """        
        try:
            response = self.client.chat.completions.create(
                model=LLM_MODEL,
                messages=messages,
                max_tokens=LLM_MAX_TOKENS,
                temperature=LLM_TEMPERATURE,
                top_p=top_p
            )
            return response.choices[0].message.content
        
        except Exception as e:
            logger.error(f"Error getting LLM response: {e}")
            raise

    
    def _assemble_behavior_tree_xml(self, behavior_section: str) -> str:
        """ Assemble the complete behavior tree XML from template parts and the generated behavior. (private method)
        Args:
            behavior_section (str): The generated BehaviorTree section from the LLM.
        Returns:
            str: The complete behavior tree XML.
        """
        header_path = os.path.join(TEMPLATES_DIR, "bt_template_header.xml")
        footer_path = os.path.join(TEMPLATES_DIR, "bt_template_footer.xml")

        with open(header_path, "r", encoding="utf-8") as f:
            header = f.read()
        with open(footer_path, "r", encoding="utf-8") as f:
            footer = f.read()

        behavior = behavior_section.replace("```xml", "").replace("```", "")
        lines = behavior.splitlines()

        cleaned = []
        for line in lines:
            stripped = line.lstrip()
            if stripped.startswith("<?xml"):
                continue

            if stripped.startswith("<root>") or stripped.startswith("</root>"):
                continue

            cleaned.append(line)

        behavior_clean = "\n".join(cleaned).strip("\n")
        return f"{header}\n{behavior_clean}\n{footer}"
    
    # ===================================================
    #                   Main Methods
    # ===================================================
    
    def generate_behavior_tree(self, user_description: str, prompt_name: str = "BT_gen", ui_manager=None, agent_id: int = 1) -> str:
        """ Generate behavior tree XML using template assembly approach. 
        Args:
            user_description (str): User's behavior description.
            prompt_name (str): Name of the prompt template to use.
            ui_manager: UI manager for display updates.
            agent_id (int): Agent ID for UI updates.
        Returns:
            str: The complete behavior tree XML assembled from template + generated behavior.
        """
        try:
            system_prompt = self._load_prompt_template(prompt_name)

            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_description}
            ]

            behavior_section = self._get_response(messages)

            if not behavior_section or not behavior_section.strip():
                raise ValueError("LLM returned an empty behavior section.")

            complete_xml = self._assemble_behavior_tree_xml(behavior_section)
            return complete_xml

        except Exception as e:
            logger.error(f"Error generating behavior tree: {e}")
            raise
