"""
Settings Loader

Loads environment variables from .env and sets defaults for the application.
"""

import os
from dotenv import load_dotenv

def load_env_variables():
    """
    Load environment variables from .env and set defaults.
    """
    
    load_dotenv()
    os.environ.setdefault("LLM_API_KEY", "sk-dummy-key")
    os.environ.setdefault("LLM_API_BASE_URL", "http://100.115.56.116:8000/v1")
    os.environ.setdefault("LLM_MODEL", "Qwen/Qwen2.5-VL-32B-Instruct-AWQ")
    os.environ.setdefault("BASE_WORLD_PATH", "data/base_world.sdf")
    os.environ.setdefault("LOG_PATH", "logs/app.log")
    os.environ.setdefault("PROMPTS_PATH", "prompts/")