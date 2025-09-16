"""
World Generation Core Module

This module provides the main function to generate a Gazebo SDF world file
from a user prompt using LLMs and prompt engineering.
"""

import os
import json
import logging
from core.prompts_manager import load_prompt
from utils.llm_utils import get_llm_response, extract_json_from_markdown
from utils.file_utils import save_file, load_json, save_json
from utils.sdf_utils import update_sdf_with_models

def generate_world_from_prompt(user_prompt: str) -> str:
    """
    Generate a Gazebo SDF world file from a user prompt.

    Args:
        user_prompt (str): The natural language description of the environment.

    Returns:
        str: The path to the generated SDF world file.
    """
    logging.info("World generation started.")

    # Step 1: Generate initial JSON from user prompt
    prompt_content = load_prompt("json_generate")
    messages = [
        {"role": "system", "content": prompt_content},
        {"role": "user", "content": user_prompt}
    ]
    response = get_llm_response(messages)
    logging.info(f"LLM raw response (step 1):\n{response}")
    models_json = extract_json_from_markdown(response)  
    logging.info("Step 1: Models extracted from prompt.")

    # Step 2: Map models to URIs
    labels = [obj["Model"] for obj in models_json]
    prompt_content = load_prompt("model_mapper")
    label_block = "\n".join([f'- "{label}"' for label in labels])
    prompt_text = prompt_content.replace("{labels}", label_block)
    messages = [{"role": "system", "content": prompt_text}]
    response = get_llm_response(messages)
    logging.info(f"LLM raw response (step 2):\n{response}")
    mapped_models = extract_json_from_markdown(response)  
    logging.info("Step 2: Models mapped to URIs.")

    # Step 3: Attach URIs and sizes
    final_models = []
    for label in labels:
        match = next((r for r in mapped_models if r["label"].lower() == label.lower()), None)
        uri = match["uri"] if match else None
        if uri and uri != "null":
            final_models.append({
                "label": label,
                "uri": uri,
                "size": None
            })
    logging.info("Step 3: Final models prepared.")

    # Step 4: Rescale models
    prompt_content = load_prompt("rescale")
    messages = [
        {"role": "system", "content": prompt_content},
        {"role": "user", "content": f"Models:\n{final_models}"}
    ]
    response = get_llm_response(messages)
    logging.info(f"LLM raw response (step 4):\n{response}")
    rescaled_models = extract_json_from_markdown(response)  
    scale_map = {(obj["label"], obj["uri"]): obj["Scale"] for obj in rescaled_models}
    for obj in final_models:
        obj["scale"] = scale_map.get((obj["label"], obj["uri"]), 1.0)
    logging.info("Step 4: Models rescaled.")

    # Step 5: Place models
    prompt_content = load_prompt("placement")
    messages = [
        {"role": "system", "content": prompt_content},
        {"role": "user", "content": f"Models:\n{final_models}"}
    ]
    response = get_llm_response(messages)
    logging.info(f"LLM raw response (step 5):\n{response}")
    placed_models = extract_json_from_markdown(response)  
    for obj, placed in zip(final_models, placed_models):
        obj["pose"] = placed.get("pose", [0.0, 0.0, 0.0])
    logging.info("Step 5: Models placed.")

    # Step 6: Generate world file
    base_world_path = os.environ.get("BASE_WORLD_PATH", "data/base_world.sdf")
    sdf_content = update_sdf_with_models(base_world_path, final_models)
    output_path = os.path.join("data", "generated_world.sdf")
    save_file(output_path, sdf_content)
    logging.info("Step 6: World file generated.")

    return output_path