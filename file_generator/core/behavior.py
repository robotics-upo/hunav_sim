"""
Behavior Tree Generation Module

This module provides a function to generate a Behavior Tree (BT) XML file
from a user description using an LLM. The generated XML is saved to disk in the data folder .
"""

import os
import json
import logging

from utils.llm_utils import generate_bt_from_description,extract_xml_from_markdown


def bt_generation(user_description: str, filename: str = "generated_BT.xml") -> str:
    """
    Generate a Behavior Tree XML file from a user description.

    Args:
        user_description (str): The natural language description of the desired behavior.
        filename (str): The output XML filename (default: "generated_BT.xml").

    Returns:
        str: The path to the generated BT XML file.
    """
    logging.info("Generating Behavior Tree XML from user description...")

    # Query the LLM to generate the BT XML content (as markdown code block)
    bt_xml_content = generate_bt_from_description(user_description)
    
    # Extract the XML content from the markdown code block
    bt_xml_content = extract_xml_from_markdown(bt_xml_content)
    
    # Get the directory where to save the BT XML file (from environment variable)
    behavior_path = os.environ.get("DATA_PATH")
    bt_xml_file_path = os.path.join(behavior_path, filename)

    # Save the XML content to file
    with open(bt_xml_file_path, "w", encoding="utf-8") as f:
        f.write(bt_xml_content)
        
    logging.info(f"BT XML file saved in: {bt_xml_file_path}")
    logging.info("Behavior Tree generation completed.")
    return bt_xml_file_path