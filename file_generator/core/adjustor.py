import logging
from core.prompts_manager import load_prompt
from utils.llm_utils import get_llm_response, extract_xml_from_markdown
from utils.file_utils import save_file

def adjust_world_file(user_instruction: str, world_file_path: str) -> str:
    """
    Adjust an existing Gazebo world file according to a user instruction using an LLM.

    Args:
        user_instruction (str): The user's natural language instruction for modifying the world.
        world_file_path (str): Path to the existing SDF world file to be adjusted.

    Returns:
        str: The path to the adjusted world file.
    """
    logging.info("Adjusting world file according to user instruction.")

    # Read the current world file content
    with open(world_file_path, "r", encoding="utf-8") as file:
        world_context = file.read()

    # Load the prompt template for world adjustment
    prompt_content = load_prompt("adjust_world")

    # Prepare the messages for the LLM: system prompt and user instruction with world context
    messages = [
        {"role": "system", "content": prompt_content},
        {"role": "user", "content": f'Instruction: {user_instruction}\nWorld Context:\n{world_context}'}
    ]

    # Query the LLM for the adjusted world file (increase max_tokens if needed for large worlds)
    response = get_llm_response(messages, max_tokens=2048)

    # Extract the XML content from the LLM's markdown code block
    new_world_content = extract_xml_from_markdown(response)

    # Overwrite the original world file with the new content
    save_file(world_file_path, new_world_content)
    logging.info("World file successfully adjusted.")
    return world_file_path