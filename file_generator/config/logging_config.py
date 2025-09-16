"""
Logging Configuration

Sets up logging for the application.
"""

import logging
import os

def setup_logging():
    """
    Configure logging to file with INFO level and a standard format.
    """
    
    log_path = os.environ.get("LOG_PATH", "logs/app.log")
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    logging.basicConfig(
        filename=log_path,
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        filemode="a"
    )