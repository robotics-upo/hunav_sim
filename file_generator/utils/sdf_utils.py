"""
SDF Utilities

Functions for updating SDF world files with model <include> blocks.
"""

def update_sdf_with_models(base_world_path: str, models: list) -> str:
    """
    Insert <include> blocks for models into a base SDF world file.

    Args:
        base_world_path (str): Path to the base SDF world file.
        models (list): List of model dicts with 'label', 'uri', 'pose', 'scale'.

    Returns:
        str: The updated SDF world file content as a string.
    """

    # Load base SDF world
    with open(base_world_path, "r", encoding="utf-8") as f:
        sdf_content = f.read()

    # Build <include> blocks for each model
    include_blocks = ""
    for model in models:
        include_blocks += f"""
    <include>
      <name>{model['label'].replace(' ', '_')}</name>
      <pose>{' '.join(str(x) for x in model.get('pose', [0.0, 0.0, 0.0]))} 0 0 0</pose>
      <uri>{model['uri']}</uri>
      <scale>{model.get('scale', 1.0)}</scale>
    </include>
    """
    # Insert include_blocks before </world>
    insert_index = sdf_content.rfind("</world>")
    if insert_index == -1:
        raise ValueError("Invalid base SDF file: missing </world> tag.")
    sdf_content = sdf_content[:insert_index] + include_blocks + sdf_content[insert_index:]
    return sdf_content