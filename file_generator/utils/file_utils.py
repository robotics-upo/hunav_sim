"""
File Utilities

Functions for saving and loading files and JSON data.
"""

import os
import json

def save_file(file_path: str, content: str):
    """
    Save string content to a file, creating directories if needed.
    """

    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)

def load_json(file_path: str):
    """
    Load JSON data from a file.
    """

    with open(file_path, "r", encoding="utf-8") as f:
        return json.load(f)

def save_json(file_path: str, data):
    """
    Save JSON data to a file, creating directories if needed.
    """

    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

def file_exist(file_path: str) -> bool:
    return os.path.isfile(file_path)