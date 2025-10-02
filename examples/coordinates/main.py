"""Streamlined transform utilities for Onshape assemblies.

Demonstrates:
1. plot_all_transforms() - Visualize all occurrence transforms in 3D space
2. get_occurrence_transform_by_name() - Get occurrence transform by part name (supports raw or sanitized names)
3. Using the new CAD and PathKey system for assembly navigation

The 3D visualization shows:
- All parts/assemblies with their coordinate frames (X=red, Y=green, Z=blue)
- Global origin and axes for reference
- Labels for each occurrence to understand spatial relationships
"""

import os

import matplotlib.pyplot as plt
import numpy as np

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicTree
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models import Assembly
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD, PathKey
from onshape_robotics_toolkit.utilities import load_model_from_json, save_model_as_json
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name

DEPTH = 1
NAME = f"transforms_{DEPTH}"
USE_CACHED = True  # Set to False to force fresh API call
LOG_ASSEMBLY = False

TRANSFORMS = (
    "https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/46679c6ab890a1b7a6d11a88"
)


def plot_all_transforms(cad: CAD, scale: float = 50.0, save_name: str = "assembly_3d"):
    """
    Plot all occurrence transforms in 3D space to visualize assembly structure.

    Args:
        cad: CAD object containing assembly data
        scale: Scale factor for the coordinate frame axes (in mm)
        save_name: Base filename for saving the plot
    """

    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection="3d")

    # Axis colors for X, Y, Z
    axis_colors = ["red", "green", "blue"]
    axis_labels = ["X", "Y", "Z"]

    # Plot global origin
    ax.scatter([0], [0], [0], color="black", s=100, marker="o", label="Global Origin", zorder=1000)

    # Plot global axes
    for i, (color, _label) in enumerate(zip(axis_colors, axis_labels)):
        direction = np.zeros(3)
        direction[i] = scale * 1.5
        ax.quiver(
            0,
            0,
            0,
            direction[0],
            direction[1],
            direction[2],
            color=color,
            arrow_length_ratio=0.15,
            linewidth=2.5,
            alpha=0.3,
            linestyle="--",
        )

    # Collect all transforms
    transform_data = []
    for key, occurrence in cad.root_assembly.occurrences.occurrences.items():
        transform = occurrence.transform
        if transform is not None:
            # Convert to numpy array if needed
            if isinstance(transform, list):
                transform = np.array(transform).reshape(4, 4)

            # Get the instance name - check both parts and assemblies
            part = cad.root_assembly.instances.parts.get(key)
            assembly = cad.root_assembly.instances.assemblies.get(key)

            if part:
                name = part.name
            elif assembly:
                name = assembly.name
            else:
                name = str(key.leaf_id)

            transform_data.append((name, key, transform))

    LOGGER.info(f"Plotting {len(transform_data)} occurrence transforms")

    # Plot each transform
    for name, _, transform in transform_data:
        # Extract origin and rotation
        origin = transform[:3, 3] * 1000  # Convert to mm
        rotation = transform[:3, :3]

        # Plot origin point
        ax.scatter(*origin, s=30, alpha=0.6, zorder=100)

        # Plot coordinate axes for this occurrence
        for i, (color, _axis_label) in enumerate(zip(axis_colors, axis_labels)):
            # Get the i-th column of the rotation matrix (the axis direction)
            axis_direction = rotation[:, i] * scale

            ax.quiver(
                origin[0],
                origin[1],
                origin[2],
                axis_direction[0],
                axis_direction[1],
                axis_direction[2],
                color=color,
                arrow_length_ratio=0.2,
                linewidth=1.5,
                alpha=0.7,
            )

        # Add label near the origin
        ax.text(origin[0], origin[1], origin[2], f"  {name}", fontsize=7, alpha=0.8)

    # Set labels and title
    ax.set_xlabel("X (mm)", fontsize=12, fontweight="bold")
    ax.set_ylabel("Y (mm)", fontsize=12, fontweight="bold")
    ax.set_zlabel("Z (mm)", fontsize=12, fontweight="bold")
    ax.set_title(
        f"Assembly Occurrence Transforms ({len(transform_data)} parts/assemblies)",
        fontsize=14,
        fontweight="bold",
        pad=20,
    )

    # Equal aspect ratio
    # Get the data limits
    all_origins = np.array([t[2][:3, 3] * 1000 for t in transform_data])
    if len(all_origins) > 0:
        max_range = (
            np.array([
                all_origins[:, 0].max() - all_origins[:, 0].min(),
                all_origins[:, 1].max() - all_origins[:, 1].min(),
                all_origins[:, 2].max() - all_origins[:, 2].min(),
            ]).max()
            / 2.0
        )

        mid_x = (all_origins[:, 0].max() + all_origins[:, 0].min()) * 0.5
        mid_y = (all_origins[:, 1].max() + all_origins[:, 1].min()) * 0.5
        mid_z = (all_origins[:, 2].max() + all_origins[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Add grid
    ax.grid(True, alpha=0.3)

    # Add legend
    ax.legend(loc="upper left", fontsize=10)

    plt.tight_layout()
    plt.savefig(f"{save_name}.png", dpi=150, bbox_inches="tight")
    LOGGER.info(f"Saved 3D visualization to {save_name}.png")
    plt.close()


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

    cad.process_mates_and_relations()
    tree = KinematicTree(cad, use_user_defined_root=True)
    tree.show()
