from pathlib import Path
import os
import logging

# ===================================================
# Paths Configuration
# ===================================================

# Configuration based on simulator version
SIMULATOR_VERSION = os.environ.get("SIMULATOR_VERSION", "Gazebo Classic")

match SIMULATOR_VERSION:
    case "Gazebo Classic":
        ws_name = "hunav_gz_classic_ws"
        package_name = "hunav_gazebo_wrapper"
    case "Gazebo Fortress":
        ws_name = "hunav_gz_fortress_ws"
        package_name = "hunav_gazebo_fortress_wrapper"
    # Possible implementation for Isaac Sim or Webots
    case _:  # Default case
        ws_name = "hunav_gz_classic_ws"
        package_name = "hunav_gazebo_wrapper"

WORKSPACE_NAME = ws_name
PACKAGE_DIR = Path(__file__).parent.parent.parent

current_path = PACKAGE_DIR

while current_path.name != WORKSPACE_NAME and current_path.parent != current_path:
    current_path = current_path.parent

WORKSPACE_ROOT = current_path
WRAPPER_PATH = WORKSPACE_ROOT / "src" / package_name
SRC_PACKAGE_DIR = WORKSPACE_ROOT / "src" / "hunav_sim" / "hunav_behavior_tree_generator"
PROMPTS_DIR = SRC_PACKAGE_DIR / "hunav_behavior_tree_generator" / "prompts"
TEMPLATES_DIR = SRC_PACKAGE_DIR / "hunav_behavior_tree_generator" / "templates"

WORLD_DIR = WRAPPER_PATH / "worlds"
BT_OUTPUT_DIR = WRAPPER_PATH / "behavior_trees"
SCENARIOS_DIR = WRAPPER_PATH / "scenarios"

# ===================================================
# LLM Configuration
# ===================================================
LLM_API_KEY = "sk-dummy-key"
LLM_API_BASE_URL = "http://100.115.56.116:8000/v1"
LLM_MODEL = "Qwen/Qwen3-VL-30B-A3B-Instruct-FP8"

# Previous model used :
# Qwen/Qwen2.5-VL-32B-Instruct-AWQ

LLM_TIMEOUT = 30  # seconds
LLM_RETRIES = 3
LLM_BACKOFF_FACTOR = 0.3

LLM_MAX_TOKENS = 4096
LLM_TEMPERATURE = 0.7

# ===================================================
# Logging Configuration
# ===================================================
LOG_DIR = SRC_PACKAGE_DIR / "logs"
LOG_FILE = LOG_DIR / "app.log"
LOG_FORMAT = '%(asctime)s - %(levelname)s - %(message)s'
LOG_DATE_FORMAT = '%m-%d %H:%M:%S'

# ===================================================
# Scenarios Configuration
# ===================================================
SUPPORTED_SCENARIO_EXTENSIONS = [".yaml", ".yml"]

def get_scenarios_directory():
    """ Get the scenarios directory path. """
    scenarios_path = Path(SCENARIOS_DIR).resolve()
    if not scenarios_path.exists():
        logging.warning(f"Scenarios directory not found: {scenarios_path}")
        scenarios_path = Path("scenarios").resolve()
    return str(scenarios_path)

# ===================================================
# Default Settings
# ===================================================
DEFAULT_BT_FILENAME = "generated_behavior_tree.xml"
DEFAULT_WORLD_FILE = WORLD_DIR / "house.world"

def setup_logging():
    """ Setup logging configuration. """
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        
        handlers = [logging.FileHandler(LOG_FILE)]
    except (PermissionError, OSError):
        handlers = [logging.StreamHandler()]
    
    logging.basicConfig(
        level=logging.INFO,
        format=LOG_FORMAT,
        datefmt=LOG_DATE_FORMAT,
        handlers=handlers,
        force=True
    )
    
    return logging.getLogger(__name__)