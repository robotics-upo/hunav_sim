"""
LLM Utilities

Functions for interacting with the LLM, extracting JSON/XML from markdown, and generating behavior trees.
"""

from openai import OpenAI
import os
import json
import re
from core.prompts_manager import load_prompt

def get_llm_response(messages, max_tokens=2048, temperature=0.3, top_p=0.9):
    """
    Send a chat completion request to the LLM and return the response content.
    """

    client = OpenAI(
        api_key=os.environ.get("LLM_API_KEY"),
        base_url=os.environ.get("LLM_API_BASE_URL")
    )
    model = os.environ.get("LLM_MODEL")
    response = client.chat.completions.create(
        model=model,
        messages=messages,
        max_tokens=max_tokens,
        temperature=temperature,
        top_p=top_p
    )
    return response.choices[0].message.content

def extract_json_from_markdown(md: str):
    """
    Extract JSON from a markdown code block.
    """

    match = re.search(r"```json\s*(.*?)\s*```", md, re.DOTALL)
    if match:
        return json.loads(match.group(1))
    return json.loads(md)

def extract_xml_from_markdown(md: str) -> str:
    """
    Extract XML from a markdown code block.
    """

    import re
    match = re.search(r"```xml\s*(.*?)\s*```", md, re.DOTALL)
    if match:
        return match.group(1)
    raise ValueError("No XML code block found in LLM response.")


def generate_bt_from_description(user_description, prompt_name="bt_generation"):
    """
    Generate a Behavior Tree XML from a user description using the LLM.
    """

    bt_prompt = load_prompt(prompt_name)
    messages = [
        {"role": "system", "content": bt_prompt},
        {"role": "user", "content": user_description}
    ]
    response = get_llm_response(messages, max_tokens=2048)
    return response


def extract_xml_from_markdown(markdown_text: str) -> str:
    """
    Extract only the XML part of a Behavior Tree
    """

    start_tag = "```xml"
    end_tag = "```"
    start_idx = markdown_text.find(start_tag)
    if start_idx != -1:
        start_idx += len(start_tag)
        end_idx = markdown_text.find(end_tag, start_idx)
        if end_idx != -1:
            return markdown_text[start_idx:end_idx].strip()
    return markdown_text.strip()