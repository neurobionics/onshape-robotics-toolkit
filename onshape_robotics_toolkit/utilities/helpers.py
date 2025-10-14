"""
This module contains utility functions used across the Onshape API package.

Functions:
    - **xml_escape**: Escape XML characters in a string.
    - **format_number**: Format a number to 8 significant figures.
    - **generate_uid**: Generate a 16-character unique identifier from a list of strings.
    - **print_dict**: Print a dictionary with indentation for nested dictionaries.
    - **get_random_files**: Get random files from a directory with a specific file extension and count.
    - **get_random_names**: Generate random names from a list of words in a file.

Logging Configuration Functions:
    - **setup_default_logging**: Configure logging with console (INFO) and file (DEBUG) output.
    - **setup_minimal_logging**: Configure console-only logging.
    - **setup_quiet_logging**: Configure file-only logging.
    - **setup_console_logging**: Add a console logging handler.
    - **setup_file_logging**: Add a file logging handler with rotation.
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
from loguru import logger
from PIL import Image
from pydantic import BaseModel

# New unified key system for assembly parsing
Key = tuple[str, ...]


def clean_numeric_value(value: float, threshold: float = 1e-10, decimals: int = 8) -> float:
    """
    Clean a numeric value by:
    1. Setting values below threshold to exactly 0
    2. Rounding to specified decimal places

    Args:
        value: The numeric value to clean
        threshold: Values with absolute value below this are set to 0 (default 1e-10)
        decimals: Number of decimal places to round to (default 8)

    Returns:
        Cleaned numeric value

    Examples:
        >>> clean_numeric_value(5.62050406e-16)
        0.0
        >>> clean_numeric_value(0.123456789012345, decimals=5)
        0.12346
        >>> clean_numeric_value(-1e-11)
        0.0
    """
    # First check if value is below threshold
    if abs(value) < threshold:
        return 0.0
    # Round to specified decimal places
    return round(value, decimals)


def clean_numeric_list(data: Any, threshold: float = 1e-10, decimals: int = 8) -> Any:
    """
    Recursively clean numeric values in nested lists/arrays.

    Args:
        data: Data structure (can be list, nested list, or scalar)
        threshold: Values with absolute value below this are set to 0
        decimals: Number of decimal places to round to

    Returns:
        Cleaned data structure
    """
    if isinstance(data, (list, tuple)):
        return [clean_numeric_list(item, threshold, decimals) for item in data]
    elif isinstance(data, (float, np.floating)):
        return clean_numeric_value(float(data), threshold, decimals)
    elif isinstance(data, (int, np.integer)):
        return int(data)
    else:
        return data


class CustomJSONEncoder(json.JSONEncoder):
    """
    Custom JSON encoder that:
    1. Converts numpy arrays/matrices to lists
    2. Cleans numerical values (removes near-zero noise, rounds to precision)
    3. Converts sets to lists
    """

    def __init__(
        self, *args: Any, clean_numerics: bool = True, threshold: float = 1e-10, decimals: int = 8, **kwargs: Any
    ):
        """
        Args:
            clean_numerics: If True, clean numeric values (default True)
            threshold: Values below this are set to 0 (default 1e-10)
            decimals: Number of decimal places to round to (default 8)
        """
        super().__init__(*args, **kwargs)
        self.clean_numerics = clean_numerics
        self.threshold = threshold
        self.decimals = decimals

    def encode(self, obj: Any) -> str:
        """Override encode to clean numerics in the entire structure."""
        if self.clean_numerics:
            obj = self._clean_object(obj)
        return super().encode(obj)

    def _clean_object(self, obj: Any) -> Any:
        """Recursively clean numeric values in any object."""
        if isinstance(obj, dict):
            return {k: self._clean_object(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [self._clean_object(item) for item in obj]
        elif isinstance(obj, (float, np.floating)):
            return clean_numeric_value(float(obj), self.threshold, self.decimals)
        elif isinstance(obj, (int, np.integer)):
            return int(obj)
        elif isinstance(obj, (np.ndarray, np.matrix)):
            # Convert to list and clean
            return clean_numeric_list(obj.tolist(), self.threshold, self.decimals)
        elif isinstance(obj, set):
            return [self._clean_object(item) for item in obj]
        else:
            return obj

    def default(self, obj: Any) -> Any:
        """Handle non-serializable objects."""
        if isinstance(obj, (np.ndarray, np.matrix)):
            cleaned = clean_numeric_list(obj.tolist(), self.threshold, self.decimals)
            return cleaned
        if isinstance(obj, set):
            return list(obj)
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


def save_model_as_json(
    model: BaseModel,
    file_path: str,
    indent: int = 4,
    clean_numerics: bool = True,
    threshold: float = 1e-10,
    decimals: int = 8,
) -> None:
    """
    Save a Pydantic model as a JSON file with optional numeric cleaning.

    Args:
        model (BaseModel): Pydantic model to save
        file_path (str): File path to save JSON file
        indent (int): JSON indentation level
        clean_numerics (bool): If True, clean numeric values (default True)
        threshold (float): Values below this are set to 0 (default 1e-10)
        decimals (int): Number of decimal places to round to (default 8)

    Returns:
        None

    Examples:
        >>> class TestModel(BaseModel):
        ...     a: int
        ...     b: str
        ...
        >>> save_model_as_json(TestModel(a=1, b="hello"), "test.json")
        >>> save_model_as_json(model, "test.json", decimals=5, threshold=1e-8)
    """
    with open(file_path, "w") as file:
        encoder = CustomJSONEncoder(
            indent=indent, clean_numerics=clean_numerics, threshold=threshold, decimals=decimals
        )
        file.write(encoder.encode(model.model_dump()))


def clean_json_numerics(data: Any, threshold: float = 1e-10, decimals: int = 8) -> Any:
    """
    Recursively clean numeric values in a JSON-like data structure.

    Args:
        data: JSON data (dict, list, or scalar)
        threshold: Values below this are set to 0
        decimals: Number of decimal places to round to

    Returns:
        Cleaned data structure
    """
    if isinstance(data, dict):
        return {k: clean_json_numerics(v, threshold, decimals) for k, v in data.items()}
    elif isinstance(data, list):
        return [clean_json_numerics(item, threshold, decimals) for item in data]
    elif isinstance(data, float):
        return clean_numeric_value(data, threshold, decimals)
    else:
        return data


def load_model_from_json(
    model_class: type[BaseModel],
    file_path: str,
    clean_numerics: bool = True,
    threshold: float = 1e-10,
    decimals: int = 8,
) -> BaseModel:
    """
    Load a Pydantic model from a JSON file with optional numeric cleaning.

    Args:
        model_class (type[BaseModel]): The Pydantic model class to instantiate
        file_path (str): Path to JSON file
        clean_numerics (bool): If True, clean numeric values before validation (default True)
        threshold (float): Values below this are set to 0 (default 1e-10)
        decimals (int): Number of decimal places to round to (default 8)

    Returns:
        BaseModel: Instance of the model class populated from JSON

    Examples:
        >>> class TestModel(BaseModel):
        ...     a: int
        ...     b: str
        ...
        >>> model = load_model_from_json(TestModel, "test.json")
        >>> print(model.a, model.b)
        1 hello

        >>> from onshape_robotics_toolkit.models import Assembly
        >>> assembly = load_model_from_json(Assembly, "assembly.json")
        >>> assembly = load_model_from_json(Assembly, "assembly.json", decimals=5)
    """
    with open(file_path) as file:
        data = json.load(file)
        if clean_numerics:
            data = clean_json_numerics(data, threshold, decimals)
        return model_class.model_validate(data)


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


def print_tf(tf: np.ndarray) -> None:
    """
    Print a 4x4 transformation matrix in a readable format

    Args:
        tf (np.ndarray): 4x4 transformation matrix

    Returns:
        None

    Examples:
        >>> import numpy as np
        >>> tf = np.array([[1, 0, 0, 1],
        ...                [0, 1, 0, 2],
        ...                [0, 0, 1, 3],
        ...                [0, 0, 0, 1]])
        >>> print_tf(tf)
        [[1.         0.         0.         1.        ]
         [0.         1.         0.         2.        ]
         [0.         0.         1.         3.        ]
         [0.         0.         0.         1.        ]]
    """
    if tf.shape != (4, 4):
        raise ValueError("Input must be a 4x4 matrix")
    with np.printoptions(precision=8, suppress=True):
        print(tf)


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

    logger.info(f"Selected files: {file_paths}")

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


_LOG_JOINER = " | "

_DEFAULT_COMPONENTS = [
    "{time:HH:mm:SSS}",
    "{level}",
    "{module}:{function}:{line}",
    "{message}",
]

_CONSOLE_STYLED_COMPONENTS = [
    "<green>{time:HH:mm:SSS}</green>",
    "<level>{level}</level>",
    "<cyan>{module}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan>",
    "<level>{message}</level>",
]

_MINIMAL_CONSOLE_STYLED_COMPONENTS = [
    "<level>{level}</level>",
    "<level>{message}</level>",
]

# Default format strings for logging
DEFAULT_CONSOLE_FORMAT = _LOG_JOINER.join(_CONSOLE_STYLED_COMPONENTS)
DEFAULT_FILE_FORMAT = _LOG_JOINER.join(_DEFAULT_COMPONENTS)
MINIMAL_CONSOLE_FORMAT = _LOG_JOINER.join(_MINIMAL_CONSOLE_STYLED_COMPONENTS)


def setup_console_logging(
    level: str = "INFO",
    format_string: str | None = None,
    colorize: bool = True,
) -> int:
    """Add a console (stderr) logging handler.

    Args:
        level: Minimum log level to display (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        format_string: Custom format string. If None, uses DEFAULT_CONSOLE_FORMAT
        colorize: Whether to colorize the output

    Returns:
        Handler ID that can be used with logger.remove() if needed

    Example:
        >>> from onshape_robotics_toolkit.utilities import setup_console_logging
        >>> setup_console_logging(level="DEBUG")
    """
    import sys

    fmt = format_string if format_string is not None else DEFAULT_CONSOLE_FORMAT

    handler_id = logger.add(
        sys.stderr,
        format=fmt,
        level=level,
        colorize=colorize,
    )
    return handler_id


def setup_file_logging(
    file_path: str = "onshape_toolkit.log",
    level: str = "DEBUG",
    format_string: str | None = None,
    rotation: str = "10 MB",
    retention: str = "7 days",
    compression: str = "zip",
    enqueue: bool = True,
    delay: bool = False,
) -> int:
    """Add a file logging handler with rotation and compression.

    Args:
        file_path: Path to the log file
        level: Minimum log level to write (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        format_string: Custom format string. If None, uses DEFAULT_FILE_FORMAT
        rotation: When to rotate the log file (e.g., "10 MB", "1 day", "12:00")
        retention: How long to keep old log files (e.g., "7 days", "10 files")
        compression: Compression format for rotated files ("zip", "gz", "bz2", or None)
        enqueue: Whether to use thread-safe logging (recommended)

    Returns:
        Handler ID that can be used with logger.remove() if needed

    Example:
        >>> from onshape_robotics_toolkit.utilities import setup_file_logging
        >>> setup_file_logging("my_robot.log", level="DEBUG", rotation="50 MB")
    """
    fmt = format_string if format_string is not None else DEFAULT_FILE_FORMAT

    handler_id = logger.add(
        file_path,
        format=fmt,
        level=level,
        rotation=rotation,
        retention=retention,
        compression=compression,
        enqueue=enqueue,
        delay=delay,
    )
    return handler_id


def setup_default_logging(
    console_level: str = "INFO",
    file_level: str = "DEBUG",
    file_path: str = "onshape_toolkit.log",
    clear_existing_handlers: bool = True,
    delay_file_creation: bool = False,
) -> tuple[int, int]:
    """Configure logging with sensible defaults: console at INFO + file at DEBUG.

    This is the recommended way to set up logging for most users. It provides:
    - Colored console output at INFO level or higher
    - Detailed file logging at DEBUG level or higher
    - Automatic log rotation (10 MB) and retention (7 days)
    - Compressed archives of rotated logs

    Args:
        console_level: Minimum level for console output (default: "INFO")
        file_level: Minimum level for file output (default: "DEBUG")
        file_path: Path to the log file (default: "onshape_toolkit.log")
        clear_existing_handlers: Whether to remove existing handlers first (default: True)

    Returns:
        Tuple of (console_handler_id, file_handler_id)

    Example:
        >>> from onshape_robotics_toolkit.utilities import setup_default_logging
        >>> setup_default_logging()  # Use all defaults
        >>> # Or customize:
        >>> setup_default_logging(console_level="DEBUG", file_path="my_robot.log")
    """
    if clear_existing_handlers:
        logger.remove()

    console_id = setup_console_logging(level=console_level, colorize=True)
    file_id = setup_file_logging(file_path=file_path, level=file_level, delay=delay_file_creation)

    return console_id, file_id


def setup_minimal_logging(level: str = "INFO") -> int:
    """Configure minimal console-only logging without file output.

    Useful for quick scripts or when you don't want log files.

    Args:
        level: Minimum log level to display (default: "INFO")

    Returns:
        Handler ID that can be used with logger.remove() if needed

    Example:
        >>> from onshape_robotics_toolkit.utilities import setup_minimal_logging
        >>> setup_minimal_logging(level="WARNING")
    """
    logger.remove()
    return setup_console_logging(level=level, format_string=MINIMAL_CONSOLE_FORMAT)


def setup_quiet_logging(file_path: str = "onshape_toolkit.log", level: str = "DEBUG") -> int:
    """Configure file-only logging with no console output.

    Useful for background tasks or automated scripts where console output
    would be distracting.

    Args:
        file_path: Path to the log file (default: "onshape_toolkit.log")
        level: Minimum log level to write (default: "DEBUG")

    Returns:
        Handler ID that can be used with logger.remove() if needed

    Example:
        >>> from onshape_robotics_toolkit.utilities import setup_quiet_logging
        >>> setup_quiet_logging("background_task.log")
    """
    logger.remove()
    return setup_file_logging(file_path=file_path, level=level)


# ============================================================================
# Initialize default logging configuration
# ============================================================================
# Configure default logging behavior when the module is imported.
# By default, logs to console at INFO level and to file at DEBUG level.
# Users can override this by calling any of the setup_*_logging functions.

# Remove loguru's default handler (which logs everything to stderr at DEBUG level)
setup_default_logging(
    console_level="INFO",
    file_level="DEBUG",
    file_path="ORTK.log",
    clear_existing_handlers=True,
    delay_file_creation=True,
)


if __name__ == "__main__":
    logger.info(get_sanitized_name("Part 3 <1>", remove_onshape_tags=True))
