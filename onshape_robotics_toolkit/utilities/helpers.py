"""
This module contains utility functions used across the Onshape API package.

Functions:
    - **xml_escape**: Escape XML characters in a string.
    - **format_number**: Format a number to 8 significant figures.
    - **generate_uid**: Generate a 16-character unique identifier from a list of strings.
    - **print_dict**: Print a dictionary with indentation for nested dictionaries.
    - **get_random_files**: Get random files from a directory with a specific file extension and count.
    - **get_random_names**: Generate random names from a list of words in a file.
"""

import hashlib
import json
import os
import random
import re
from typing import Any
from xml.sax.saxutils import escape

import dotenv
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from pydantic import BaseModel

from onshape_robotics_toolkit.log import LOGGER

# New unified key system for assembly parsing
Key = tuple[str, ...]


class CustomJSONEncoder(json.JSONEncoder):
    def default(self, obj: Any) -> Any:
        if isinstance(obj, np.ndarray):
            return obj.tolist()  # Convert numpy array to list
        if isinstance(obj, np.matrix):
            return obj.tolist()  # Convert numpy matrix to list
        if isinstance(obj, set):
            return list(obj)  # Convert set to list
        return super().default(obj)


def load_key_from_environment(key_to_load: str) -> str:
    key = os.getenv(key_to_load)

    if not key:
        raise ValueError(f"Missing environment variable: {key_to_load}")
    return key


def load_key_from_dotenv(env: str, key_to_load: str) -> str:
    if not os.path.isfile(env):
        raise FileNotFoundError(f"'{env}' file not found")
    key = dotenv.get_key(env, key_to_load)
    if not key:
        raise ValueError(f"Missing dotenv variable: {key_to_load}")
    return key


def save_model_as_json(model: BaseModel, file_path: str, indent: int = 4) -> None:
    """
    Save a Pydantic model as a JSON file

    Args:
        model (BaseModel): Pydantic model to save
        file_path (str): File path to save JSON file

    Returns:
        None

    Examples:
        >>> class TestModel(BaseModel):
        ...     a: int
        ...     b: str
        ...
        >>> save_model_as_json(TestModel(a=1, b="hello"), "test.json")
    """

    with open(file_path, "w") as file:
        json.dump(model.model_dump(), file, indent=indent, cls=CustomJSONEncoder)


def xml_escape(unescaped: str) -> str:
    """
    Escape XML characters in a string

    Args:
        unescaped (str): Unescaped string

    Returns:
        str: Escaped string

    Examples:
        >>> xml_escape("hello 'world' \"world\"")
        "hello &apos;world&apos; &quot;world&quot;"

        >>> xml_escape("hello <world>")
        "hello &lt;world&gt;"
    """

    return escape(unescaped, entities={"'": "&apos;", '"': "&quot;"})


def format_number(value: float) -> str:
    """
    Format a number to 8 significant figures

    Args:
        value (float): Number to format

    Returns:
        str: Formatted number

    Examples:
        >>> format_number(0.123456789)
        "0.12345679"

        >>> format_number(123456789)
        "123456789"
    """

    return f"{value:.8g}"


def generate_uid(values: list[str]) -> str:
    """
    Generate a 16-character unique identifier from a list of strings

    Args:
        values (list[str]): List of strings to concatenate

    Returns:
        str: Unique identifier

    Examples:
        >>> generate_uid(["hello", "world"])
        "c4ca4238a0b92382"
    """

    _value = "".join(values)
    return hashlib.sha256(_value.encode()).hexdigest()[:16]


def print_dict(d: dict, indent: int = 0) -> None:
    """
    Print a dictionary with indentation for nested dictionaries

    Args:
        d (dict): Dictionary to print
        indent (int): Number of tabs to indent

    Returns:
        None

    Examples:
        >>> print_dict({"a": 1, "b": {"c": 2}})
        a
            1
        b
            c
                2
    """

    for key, value in d.items():
        print()
        print("\t" * indent + str(key))
        if isinstance(value, dict):
            print_dict(value, indent + 1)
        else:
            print("\t" * (indent + 1) + str(value))


def get_random_files(directory: str, file_extension: str, count: int) -> tuple[list[str], list[str]]:
    """
    Get random files from a directory with a specific file extension and count

    Args:
        directory (str): Directory path
        file_extension (str): File extension
        count (int): Number of files to select

    Returns:
        list[str]: List of file paths

    Raises:
        ValueError: Not enough files in directory if count exceeds number of files

    Examples:
        >>> get_random_files("json", ".json", 1)
        ["json/file.json"]

        >>> get_random_files("json", ".json", 2)
        ["json/file1.json", "json/file2.json"]
    """

    _files = [file for file in os.listdir(directory) if file.endswith(file_extension)]

    if len(_files) < count:
        raise ValueError("Not enough files in directory")

    selected_files = random.sample(_files, count)
    file_paths = [os.path.join(directory, file) for file in selected_files]

    LOGGER.info(f"Selected files: {file_paths}")

    return file_paths, [x.split(".")[0] for x in selected_files]


def get_random_names(directory: str, count: int, filename: str = "words.txt") -> list[str]:
    """
    Generate random names from a list of words in a file

    Args:
        directory: Path to directory containing words file
        count: Number of random names to generate
        filename: File containing list of words. Default is "words.txt"

    Returns:
        List of random names

    Raises:
        ValueError: If count exceeds the number of available words

    Examples:
        >>> get_random_names(directory="../", count=1)
        ["charizard"]

        >>> get_random_names(directory="../", count=2)
        ["charizard", "pikachu"]
    """

    words_file_path = os.path.join(directory, filename)

    with open(words_file_path) as file:
        words = file.read().splitlines()

    if count > len(words):
        raise ValueError("count exceeds the number of available words")

    return random.sample(words, count)


def make_unique_keys(keys: list[str]) -> dict[str, int]:
    """
    Make a list of keys unique by appending a number to duplicate keys and
    return a mapping of unique keys to their original indices.

    Args:
        keys: List of keys.

    Returns:
        A dictionary mapping unique keys to their original indices.

    Examples:
        >>> make_unique_keys(["a", "b", "a", "a"])
        {"a": 0, "b": 1, "a-1": 2, "a-2": 3}
    """
    unique_key_map = {}
    key_count: dict[str, int] = {}

    for index, key in enumerate(keys):
        if key in key_count:
            key_count[key] += 1
            unique_key = f"{key}-{key_count[key]}"
        else:
            key_count[key] = 0
            unique_key = key

        unique_key_map[unique_key] = index

    return unique_key_map


def make_unique_name(name: str, existing_names: set[str]) -> str:
    """
    Make a name unique by appending a number to the name if it already exists in a set.

    Args:
        name: Name to make unique.
        existing_names: Set of existing names.

    Returns:
        A unique name.

    Examples:
        >>> make_unique_name("name", {"name"})
        "name-1"
        >>> make_unique_name("name", {"name", "name-1"})
        "name-2"
    """
    if name not in existing_names:
        return name

    count = 1
    while f"{name}-{count}" in existing_names:
        count += 1

    return f"{name}-{count}"


def get_sanitized_name(name: str, replace_with: str = "_", remove_onshape_tags: bool = False) -> str:
    """
    Sanitize a name by removing special characters, preserving only the specified
    replacement character, and replacing spaces with it. Ensures no consecutive
    replacement characters in the result.
    Optionally preserves a trailing " <n>" tag where n is a number.

    Args:
        name (str): Name to sanitize.
        replace_with (str): Character to replace spaces and other special characters with (default is '_').
        remove_onshape_tags (bool): If True, removes a trailing " <n>" tag where n is a number. Default is False.

    Returns:
        str: Sanitized name.

    Examples:
        >>> get_sanitized_name("wheel1 <3>")
        "wheel1_3"

        >>> get_sanitized_name("wheel1 <3>", remove_onshape_tags=True)
        "wheel1"

        >>> get_sanitized_name("wheel1 <3>", replace_with='-', remove_onshape_tags=False)
        "wheel1-3"
    """

    if replace_with not in "-_":
        raise ValueError("replace_with must be either '-' or '_'")

    tag = ""
    if remove_onshape_tags:
        # Regular expression to detect a trailing " <n>" where n is one or more digits
        tag_pattern = re.compile(r"\s<\d+>$")
        match = tag_pattern.search(name)
        if match:
            tag = match.group()  # e.g., " <3>"
            if tag:
                name = name[: match.start()]

    sanitized_name = "".join(char if char.isalnum() or char in "-_ " else "" for char in name)
    sanitized_name = sanitized_name.replace(" ", replace_with)
    sanitized_name = re.sub(f"{re.escape(replace_with)}{{2,}}", replace_with, sanitized_name)

    return sanitized_name


def clean_name_for_urdf(name: str) -> str:
    """
    Clean a name to be URDF-safe by replacing problematic characters.

    This is similar to get_sanitized_name but specifically for URDF compatibility,
    following the reference implementation's approach.

    Args:
        name: Name to clean for URDF compatibility.

    Returns:
        URDF-safe name.

    Examples:
        >>> clean_name_for_urdf("wheel <1>")
        "wheel_(1)"
        >>> clean_name_for_urdf("joint/arm\\link")
        "joint_arm_link"
    """
    name = name.replace("<", "(").replace(">", ")")
    name = re.sub(r"\s+", "_", name)
    name = re.sub(r"[/\\]+", "_", name)
    return name


def show_video(frames: list[Any], framerate: int = 60) -> None:
    fig, ax = plt.subplots()
    ax.axis("off")

    im = ax.imshow(frames[0], animated=True)

    def update(frame: Any) -> list[Any]:
        im.set_array(frame)
        return [im]

    animation.FuncAnimation(fig, update, frames=frames, interval=1000 / framerate, blit=True)

    plt.show()


def save_gif(frames: list[Any], filename: str = "sim.gif", framerate: int = 60) -> None:
    images = [Image.fromarray(frame) for frame in frames]
    images[0].save(filename, save_all=True, append_images=images[1:], duration=1000 / framerate, loop=0)


if __name__ == "__main__":
    LOGGER.info(get_sanitized_name("Part 3 <1>", remove_onshape_tags=True))
