"""
Prompt Manager

Utility to load prompt templates from the prompts directory.
"""

import os

def load_prompt(prompt_name: str) -> str:
    """
    Load a prompt template by name.

    Args:
        prompt_name (str): The name of the prompt file (without .txt extension).

    Returns:
        str: The content of the prompt file.
    """
    
    prompts_dir = os.environ.get("PROMPTS_PATH", "prompts/")
    prompt_path = os.path.join(prompts_dir, f"{prompt_name}.txt")
    if not os.path.exists(prompt_path):
        raise FileNotFoundError(f"Prompt '{prompt_name}' not found in {prompts_dir}")
    with open(prompt_path, "r", encoding="utf-8") as f:
        return f.read()