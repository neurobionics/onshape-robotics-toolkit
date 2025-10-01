"""Streamlined transform utilities for Onshape assemblies.

Demonstrates:
1. plot_transform() - Visualize a 4x4 transform matrix
2. get_occurrence_transform_by_name() - Get occurrence transform by part name (supports raw or sanitized names)
3. Using the new CAD and PathKey system for assembly navigation
"""

import os

import matplotlib.pyplot as plt
import numpy as np

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models import Assembly
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD, PathKey
from onshape_robotics_toolkit.utilities import load_model_from_json, save_model_as_json
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name

DEPTH = 2
NAME = f"transforms_{DEPTH}"
USE_CACHED = True  # Set to False to force fresh API call
LOG_ASSEMBLY = False

TRANSFORMS = (
    "https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/46679c6ab890a1b7a6d11a88"
)


def plot_transform(transform: np.ndarray | list[float], label: str = "", scale: float = 0.1):
    """
    Plot a 4x4 transform matrix as a 3D coordinate frame.

    Args:
        transform: 4x4 transform matrix (numpy array) or 16-element list
        label: Label for the coordinate frame
        scale: Scale factor for the axes vectors
    """
    # Convert to numpy array if needed
    if isinstance(transform, list):
        transform = np.array(transform).reshape(4, 4)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Extract origin and rotation
    origin = transform[:3, 3]
    rotation = transform[:3, :3]

    # Scale axes
    x_axis = rotation[:, 0] * scale
    y_axis = rotation[:, 1] * scale
    z_axis = rotation[:, 2] * scale

    # Plot axes as arrows (X=red, Y=green, Z=blue)
    ax.quiver(
        origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color="red", arrow_length_ratio=0.1, label="X"
    )
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        y_axis[0],
        y_axis[1],
        y_axis[2],
        color="green",
        arrow_length_ratio=0.1,
        label="Y",
    )
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        z_axis[0],
        z_axis[1],
        z_axis[2],
        color="blue",
        arrow_length_ratio=0.1,
        label="Z",
    )

    # Add label at origin
    if label:
        ax.text(origin[0], origin[1], origin[2], label, fontsize=10)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Transform: {label}" if label else "Transform")
    ax.set_box_aspect([1, 1, 1])
    ax.legend()

    plt.tight_layout()
    plt.show()


def get_occurrence_transform_by_name(
    cad: CAD, part_name: str, depth: int | None = None
) -> tuple[PathKey, np.ndarray] | None:
    """
    Get the occurrence transform for a part by name.

    Args:
        cad: CAD containing assembly data
        part_name: Name of the part (can be raw like "Part 14 <1>" or sanitized like "Part_14_1")
        depth: Optional depth filter (0 = root level, None = any depth)

    Returns:
        Tuple of (PathKey, 4x4 transform matrix) if found, None otherwise

    Examples:
        # Get transform using raw name (auto-sanitized)
        result = get_occurrence_transform_by_name(cad, "Part 14 <1>")
        if result:
            key, transform = result
            print(f"Found at {key}: {transform}")

        # Get transform using sanitized name at root level only
        result = get_occurrence_transform_by_name(cad, "Part_14_1", depth=0)
    """
    # Sanitize the name
    sanitized_name = get_sanitized_name(part_name)

    # Look up by name in the instance registry
    keys = cad.root_assembly.instances.lookup_by_name(sanitized_name, depth=depth)

    if not keys:
        return None

    # Try to get transform for the first matching key
    # (in case of duplicates, you might want to handle this differently)
    for key in keys:
        transform = cad.get_transform(key)
        if transform is not None:
            return (key, transform)

    return None


if __name__ == "__main__":
    LOGGER.set_file_name(f"{NAME}.log")
    LOGGER.set_stream_level(LogLevel.INFO)

    client = Client(env=".env")

    document = Document.from_url(url=TRANSFORMS)
    client.set_base_url(document.base_url)

    if USE_CACHED and os.path.exists(f"{NAME}.json"):
        LOGGER.info(f"Loading assembly from cached JSON: {NAME}.json")
        assembly = load_model_from_json(Assembly, f"{NAME}.json")
    else:
        LOGGER.info("Fetching assembly from Onshape API...")
        assembly = client.get_assembly(
            did=document.did,
            wtype=document.wtype,
            wid=document.wid,
            eid=document.eid,
            log_response=LOG_ASSEMBLY,
            with_meta_data=True,
        )
        # Save to cache
        save_model_as_json(assembly, f"{NAME}.json")
        LOGGER.info(f"Assembly cached to {NAME}.json")

    LOGGER.info("Creating CAD from assembly...")
    cad = CAD.from_assembly(assembly, max_depth=DEPTH)
    cad.show()

    result = get_occurrence_transform_by_name(cad, "Part 10 <1>")
    print(result[-1])
