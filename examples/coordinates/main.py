"""
Transform Exploration Building Blocks

This module provides comprehensive building blocks for exploring and visualizing
transforms, mate coordinates, and assembly properties in Onshape assemblies.

BUILDING BLOCKS INCLUDED:

1. TRANSFORM VISUALIZATION:
   - 3D plotting of coordinate frames with matplotlib
   - Transform matrix analysis (rotation/translation extraction)
   - Comparison between transforms
   - Custom coordinate frame creation

2. OCCURRENCE EXPLORATION:
   - Extract and analyze occurrence transforms
   - Find transforms near specific points
   - Visualize all occurrence transforms in assembly

3. MATE COORDINATE SYSTEMS:
   - Explore mate coordinate systems and their properties
   - Visualize mate coordinate frames
   - Analyze mate relationships between parts

4. ASSEMBLY EXPLORATION:
   - Comprehensive assembly summaries
   - Instance-specific exploration
   - Export capabilities for external analysis

5. UTILITY FUNCTIONS:
   - Transform comparisons
   - Point transformations
   - Data export to JSON

The goal is to understand how global occurrence transforms work within Onshape
root assembly data. This is crucial for supporting root level or flexible
sub-assembly level mates that reference parts within rigid sub-assemblies.
"""

import os
import pickle
from typing import Any, Optional, Union

import matplotlib.pyplot as plt
import numpy as np

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models.assembly import (
    AssemblyInstance,
    MatedCS,
    MateFeatureData,
    Occurrence,
    PartInstance,
)
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import (
    CAD,
)

TRANSFORMS = (
    "https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/46679c6ab890a1b7a6d11a88"
)

DEPTH = 1
NAME = f"transforms_{DEPTH}"
USE_CACHED = False  # Set to False to force fresh API call
PICKLE_PATH = f"{NAME}.pkl"
LOG_ASSEMBLY = False

# ============================================================================
# TRANSFORM VISUALIZATION BUILDING BLOCKS
# ============================================================================


def matrix_from_list(transform_list: list[float]) -> np.ndarray:
    """Convert a 16-element transform list to a 4x4 numpy matrix."""
    return np.array(transform_list).reshape(4, 4)


def plot_coordinate_frame(ax, transform_matrix: np.ndarray, scale: float = 0.1, label: str = "", alpha: float = 1.0):
    """
    Plot a coordinate frame (X=red, Y=green, Z=blue axes) given a transform matrix.

    Args:
        ax: 3D matplotlib axes object
        transform_matrix: 4x4 transformation matrix as numpy array
        scale: Scale factor for the axes vectors
        label: Optional label for the frame
        alpha: Transparency (0-1)
    """
    origin = transform_matrix[:3, 3]

    # Extract rotation matrix and scale by desired length
    rotation = transform_matrix[:3, :3]
    x_axis = rotation[:, 0] * scale
    y_axis = rotation[:, 1] * scale
    z_axis = rotation[:, 2] * scale

    # Plot axes as arrows
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        x_axis[0],
        x_axis[1],
        x_axis[2],
        color="red",
        alpha=alpha,
        arrow_length_ratio=0.1,
    )
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        y_axis[0],
        y_axis[1],
        y_axis[2],
        color="green",
        alpha=alpha,
        arrow_length_ratio=0.1,
    )
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        z_axis[0],
        z_axis[1],
        z_axis[2],
        color="blue",
        alpha=alpha,
        arrow_length_ratio=0.1,
    )

    # Add label at origin if provided
    if label:
        ax.text(origin[0], origin[1], origin[2], label, fontsize=8)


def plot_transforms_3d(
    transforms: dict[str, list[float]],
    title: str = "Transform Visualization",
    figsize: tuple = (12, 8),
    show_world_frame: bool = True,
):
    """
    Plot multiple transforms in a 3D visualization.

    Args:
        transforms: Dictionary mapping names to 16-element transform lists
        title: Plot title
        figsize: Figure size
        show_world_frame: Whether to show world coordinate frame at origin
    """
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")

    # Plot world frame at origin if requested
    if show_world_frame:
        world_transform = np.eye(4)
        plot_coordinate_frame(ax, world_transform, scale=0.15, label="World", alpha=0.7)

    # Plot each transform
    for name, transform_list in transforms.items():
        transform_matrix = matrix_from_list(transform_list)
        plot_coordinate_frame(ax, transform_matrix, scale=0.1, label=name)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)

    # Make axes equal
    ax.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show()


def analyze_transform(transform_list: list[float], name: str = "Transform") -> dict[str, Any]:
    """
    Analyze a transform and extract key information.

    Args:
        transform_list: 16-element transform list
        name: Name for this transform

    Returns:
        Dictionary with transform analysis
    """
    matrix = matrix_from_list(transform_list)

    # Extract components
    translation = matrix[:3, 3]
    rotation = matrix[:3, :3]

    # Calculate rotation angle and axis (Rodrigues' rotation formula)
    trace = np.trace(rotation)
    angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))

    if np.abs(angle) < 1e-6:
        axis = np.array([0, 0, 1])  # No rotation, arbitrary axis
    else:
        axis = np.array([
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]) / (2 * np.sin(angle))
        axis = axis / np.linalg.norm(axis)

    return {
        "name": name,
        "translation": translation.tolist(),
        "translation_magnitude": float(np.linalg.norm(translation)),
        "rotation_matrix": rotation.tolist(),
        "rotation_angle_rad": float(angle),
        "rotation_angle_deg": float(np.degrees(angle)),
        "rotation_axis": axis.tolist(),
        "transform_matrix": matrix.tolist(),
        "is_identity": bool(np.allclose(matrix, np.eye(4))),
        "determinant": float(np.linalg.det(rotation)),  # Should be 1 for valid rotation
    }


# ============================================================================
# OCCURRENCE EXPLORATION BUILDING BLOCKS
# ============================================================================


def explore_occurrence(occurrence: Occurrence, name: str = "Occurrence") -> dict[str, Any]:
    """
    Explore an occurrence and extract all its properties.

    Args:
        occurrence: Occurrence object to explore
        name: Name for this occurrence

    Returns:
        Dictionary with occurrence properties
    """
    transform_analysis = analyze_transform(occurrence.transform, f"{name}_transform")

    return {
        "name": name,
        "fixed": occurrence.fixed,
        "hidden": occurrence.hidden,
        "path": occurrence.path,
        "path_length": len(occurrence.path),
        "transform_raw": occurrence.transform,
        "transform_analysis": transform_analysis,
    }


def get_all_occurrences_info(cad: CAD) -> dict[str, dict[str, Any]]:
    """Get information about all occurrences in the CAD assembly."""
    occurrences_info = {}

    for key, occurrence in cad.occurrences.items():
        name = cad.key_namer.get_name(key)
        occurrences_info[name] = explore_occurrence(occurrence, name)

    return occurrences_info


def plot_all_occurrence_transforms(cad: CAD, title: str = "All Occurrence Transforms"):
    """Plot transforms for all occurrences in the assembly."""
    transforms = {}

    for key, occurrence in cad.occurrences.items():
        name = cad.key_namer.get_name(key)
        transforms[name] = occurrence.transform

    plot_transforms_3d(transforms, title)


# ============================================================================
# MATE COORDINATE EXPLORATION BUILDING BLOCKS
# ============================================================================


def explore_mated_cs(mated_cs: MatedCS, name: str = "MatedCS") -> dict[str, Any]:
    """
    Explore a mated coordinate system.

    Args:
        mated_cs: MatedCS object to explore
        name: Name for this coordinate system

    Returns:
        Dictionary with coordinate system properties
    """
    # Create transformation matrix from CS data
    transform_matrix = np.eye(4)
    transform_matrix[:3, 0] = mated_cs.xAxis  # X axis
    transform_matrix[:3, 1] = mated_cs.yAxis  # Y axis
    transform_matrix[:3, 2] = mated_cs.zAxis  # Z axis
    transform_matrix[:3, 3] = mated_cs.origin  # Origin

    # Verify orthogonality
    x_dot_y = np.dot(mated_cs.xAxis, mated_cs.yAxis)
    y_dot_z = np.dot(mated_cs.yAxis, mated_cs.zAxis)
    z_dot_x = np.dot(mated_cs.zAxis, mated_cs.xAxis)

    return {
        "name": name,
        "origin": mated_cs.origin,
        "xAxis": mated_cs.xAxis,
        "yAxis": mated_cs.yAxis,
        "zAxis": mated_cs.zAxis,
        "transform_matrix": transform_matrix.tolist(),
        "part_tf": mated_cs.part_tf.tolist() if mated_cs.part_tf is not None else None,
        "orthogonality_check": {
            "x_dot_y": float(x_dot_y),
            "y_dot_z": float(y_dot_z),
            "z_dot_x": float(z_dot_x),
            "is_orthogonal": bool(abs(x_dot_y) < 1e-6 and abs(y_dot_z) < 1e-6 and abs(z_dot_x) < 1e-6),
        },
        "axis_lengths": {
            "x_length": float(np.linalg.norm(mated_cs.xAxis)),
            "y_length": float(np.linalg.norm(mated_cs.yAxis)),
            "z_length": float(np.linalg.norm(mated_cs.zAxis)),
        },
    }


def explore_mate(mate: MateFeatureData, name: str = "Mate") -> dict[str, Any]:
    """
    Explore a mate feature and all its coordinate systems.

    Args:
        mate: MateFeatureData object to explore
        name: Name for this mate

    Returns:
        Dictionary with mate properties
    """
    mate_info = {
        "name": name,
        "mate_type": mate.mateType,
        "mate_name": mate.name,
        "num_entities": len(mate.matedEntities),
        "entities": [],
    }

    for i, entity in enumerate(mate.matedEntities):
        entity_info = {
            "entity_index": i,
            "mated_occurrence": entity.matedOccurrence,
            "coordinate_system": explore_mated_cs(entity.matedCS, f"{name}_entity_{i}_cs"),
        }
        mate_info["entities"].append(entity_info)

    return mate_info


def get_all_mates_info(cad: CAD) -> dict[str, dict[str, Any]]:
    """Get detailed information about all mates in the assembly."""
    mates_info = {}

    for key, mate in cad.mates.items():
        name = cad.key_namer.get_name(key)
        mates_info[name] = explore_mate(mate, name)

    return mates_info


def plot_mate_coordinate_systems(mate: MateFeatureData, title: Optional[str] = None):
    """Plot coordinate systems for all entities in a mate."""
    if title is None:
        title = f"Mate Coordinate Systems: {mate.name}"

    transforms = {}

    for i, entity in enumerate(mate.matedEntities):
        cs = entity.matedCS

        # Create transform matrix from coordinate system
        transform_matrix = np.eye(4)
        transform_matrix[:3, 0] = cs.xAxis
        transform_matrix[:3, 1] = cs.yAxis
        transform_matrix[:3, 2] = cs.zAxis
        transform_matrix[:3, 3] = cs.origin

        name = f"Entity_{i}_CS"
        transforms[name] = transform_matrix.flatten().tolist()

        # Also plot part_tf if available
        if cs.part_tf is not None:
            part_name = f"Entity_{i}_PartTF"
            transforms[part_name] = cs.part_tf.flatten().tolist()

    plot_transforms_3d(transforms, title)


# ============================================================================
# ASSEMBLY EXPLORATION BUILDING BLOCKS
# ============================================================================


def explore_assembly_instance(
    instance: Union[AssemblyInstance, PartInstance], name: str = "Instance"
) -> dict[str, Any]:
    """
    Explore an assembly or part instance.

    Args:
        instance: AssemblyInstance or PartInstance to explore
        name: Name for this instance

    Returns:
        Dictionary with instance properties
    """
    base_info = {
        "name": name,
        "id": instance.id,
        "type": type(instance).__name__,
        "suppressed": instance.suppressed,
        "instance_type": instance.type.value if hasattr(instance, "type") else None,
    }

    if isinstance(instance, AssemblyInstance):
        base_info.update({
            "assembly_specific": {
                "is_assembly": True,
                "document_id": getattr(instance, "documentId", None),
                "document_microversion": getattr(instance, "documentMicroversion", None),
                "element_id": getattr(instance, "elementId", None),
            }
        })
    else:  # PartInstance
        base_info.update({
            "part_specific": {
                "is_part": True,
                "part_id": getattr(instance, "partId", None),
                "body_type": getattr(instance, "bodyType", None),
            }
        })

    return base_info


def summary_report(cad: CAD) -> dict[str, Any]:
    """Generate a comprehensive summary report of the CAD assembly."""
    return {
        "overview": {
            "total_instances": len(cad.instances),
            "total_parts": len(cad.parts),
            "total_assemblies": len(cad.rigid_assembly_instances) + len(cad.flexible_assembly_instances),
            "rigid_assemblies": len(cad.rigid_assembly_instances),
            "flexible_assemblies": len(cad.flexible_assembly_instances),
            "total_occurrences": len(cad.occurrences),
            "total_mates": len(cad.mates),
            "max_depth": cad.max_depth,
        },
        "assembly_breakdown": {
            "rigid_assembly_names": [cad.key_namer.get_name(key) for key in cad.rigid_assembly_keys],
            "flexible_assembly_names": [cad.key_namer.get_name(key) for key in cad.flexible_assembly_keys],
            "part_names": [cad.key_namer.get_name(key) for key in cad.part_keys][:10],  # First 10 parts
            "mate_names": [cad.key_namer.get_name(key) for key in cad.mate_keys],
        },
    }


# ============================================================================
# CONVENIENCE FUNCTIONS
# ============================================================================


def plot_assembly_overview(cad: CAD):
    """Plot an overview of the entire assembly with all occurrence transforms."""
    plot_all_occurrence_transforms(cad, "Assembly Overview - All Occurrence Transforms")


def explore_specific_assembly(cad: CAD, assembly_name: str, plot_transforms: bool = True) -> dict[str, Any]:
    """
    Explore a specific assembly by name and optionally plot its transforms.

    Args:
        cad: CAD object
        assembly_name: Name of assembly to explore
        plot_transforms: Whether to plot coordinate systems

    Returns:
        Dictionary with assembly exploration results
    """
    # Try to find the assembly
    assembly = cad.lookup_flexible_assembly(assembly_name)
    if assembly is None:
        assembly = cad.lookup_rigid_assembly(assembly_name)

    if assembly is None:
        return {"error": f"Assembly '{assembly_name}' not found"}

    # Get assembly instance info
    instance_info = explore_assembly_instance(assembly, assembly_name)

    # Find related occurrences and mates
    related_info = {"assembly_info": instance_info, "related_occurrences": [], "related_mates": []}

    # Look for occurrences related to this assembly
    for key, occurrence in cad.occurrences.items():
        occ_name = cad.key_namer.get_name(key)
        if assembly_name.lower() in occ_name.lower():
            related_info["related_occurrences"].append({
                "name": occ_name,
                "info": explore_occurrence(occurrence, occ_name),
            })

    # Look for mates that might involve this assembly
    for key, mate in cad.mates.items():
        mate_name = cad.key_namer.get_name(key)
        related_info["related_mates"].append({"name": mate_name, "info": explore_mate(mate, mate_name)})

    if plot_transforms and related_info["related_occurrences"]:
        transforms = {}
        for occ_data in related_info["related_occurrences"]:
            occ_name = occ_data["name"]
            transforms[occ_name] = occ_data["info"]["transform_raw"]

        if transforms:
            plot_transforms_3d(transforms, f"Transforms for {assembly_name}")

    return related_info


# ============================================================================
# ADDITIONAL UTILITY FUNCTIONS
# ============================================================================


def compare_transforms(
    transform1: list[float], transform2: list[float], name1: str = "Transform1", name2: str = "Transform2"
) -> dict[str, Any]:
    """
    Compare two transforms and analyze their differences.

    Args:
        transform1: First 16-element transform list
        transform2: Second 16-element transform list
        name1: Name for first transform
        name2: Name for second transform

    Returns:
        Dictionary with comparison results
    """
    matrix1 = matrix_from_list(transform1)
    matrix2 = matrix_from_list(transform2)

    # Calculate relative transform
    relative_transform = np.linalg.inv(matrix1) @ matrix2

    # Extract translation and rotation differences
    translation_diff = matrix2[:3, 3] - matrix1[:3, 3]

    return {
        "transform1_name": name1,
        "transform2_name": name2,
        "translation_difference": translation_diff.tolist(),
        "translation_difference_magnitude": float(np.linalg.norm(translation_diff)),
        "relative_transform": relative_transform.tolist(),
        "relative_transform_analysis": analyze_transform(relative_transform.flatten().tolist(), "Relative"),
        "are_identical": bool(np.allclose(matrix1, matrix2)),
        "max_element_difference": float(np.max(np.abs(matrix1 - matrix2))),
    }


def find_transforms_near_point(cad: CAD, target_point: list[float], tolerance: float = 0.1) -> list[dict[str, Any]]:
    """
    Find all occurrences with transforms near a target point.

    Args:
        cad: CAD object
        target_point: [x, y, z] point to search near
        tolerance: Distance tolerance

    Returns:
        List of occurrences near the target point
    """
    target = np.array(target_point)
    nearby_transforms = []

    for key, occurrence in cad.occurrences.items():
        transform_matrix = matrix_from_list(occurrence.transform)
        origin = transform_matrix[:3, 3]
        distance = np.linalg.norm(origin - target)

        if distance <= tolerance:
            name = cad.key_namer.get_name(key)
            nearby_transforms.append({
                "name": name,
                "distance": float(distance),
                "origin": origin.tolist(),
                "occurrence": occurrence,
            })

    # Sort by distance
    nearby_transforms.sort(key=lambda x: x["distance"])
    return nearby_transforms


def create_custom_coordinate_frame(
    origin: list[float], x_axis: list[float], y_axis: list[float], z_axis: list[float]
) -> np.ndarray:
    """
    Create a transformation matrix from coordinate frame components.

    Args:
        origin: [x, y, z] origin point
        x_axis: [x, y, z] x-axis direction
        y_axis: [x, y, z] y-axis direction
        z_axis: [x, y, z] z-axis direction

    Returns:
        4x4 transformation matrix
    """
    transform = np.eye(4)
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    transform[:3, 3] = origin
    return transform


def transform_point(point: list[float], transform_matrix: np.ndarray) -> list[float]:
    """
    Transform a point using a transformation matrix.

    Args:
        point: [x, y, z] point to transform
        transform_matrix: 4x4 transformation matrix

    Returns:
        Transformed [x, y, z] point
    """
    point_homogeneous = np.array([point[0], point[1], point[2], 1.0])
    transformed = transform_matrix @ point_homogeneous
    return transformed[:3].tolist()


def get_mate_relationship_analysis(cad: CAD) -> dict[str, Any]:
    """
    Analyze relationships between all mates in the assembly.

    Args:
        cad: CAD object

    Returns:
        Dictionary with mate relationship analysis
    """
    mate_types = {}
    mate_connections = {}

    for key, mate in cad.mates.items():
        mate_name = cad.key_namer.get_name(key)

        # Count mate types
        mate_type = mate.mateType
        if mate_type not in mate_types:
            mate_types[mate_type] = []
        mate_types[mate_type].append(mate_name)

        # Analyze connections between occurrences
        if len(mate.matedEntities) == 2:
            occ1 = mate.matedEntities[0].matedOccurrence
            occ2 = mate.matedEntities[1].matedOccurrence

            # Convert occurrence paths to strings for easier handling
            occ1_str = str(occ1)
            occ2_str = str(occ2)

            connection = f"{occ1_str} <-> {occ2_str}"
            mate_connections[mate_name] = {
                "occurrence_1": occ1_str,
                "occurrence_2": occ2_str,
                "mate_type": mate_type,
                "connection": connection,
            }

    return {
        "mate_type_summary": {mate_type: len(mates) for mate_type, mates in mate_types.items()},
        "mate_types_detail": mate_types,
        "mate_connections": mate_connections,
        "total_mates": len(cad.mates),
        "unique_mate_types": len(mate_types),
    }


def export_transforms_to_file(cad: CAD, filename: str = "transforms_export.json"):
    """
    Export all transform data to a JSON file for external analysis.

    Args:
        cad: CAD object
        filename: Output filename
    """
    import json

    export_data = {
        "metadata": {
            "export_timestamp": __import__("datetime").datetime.now().isoformat(),
            "total_occurrences": len(cad.occurrences),
            "total_mates": len(cad.mates),
        },
        "occurrences": {},
        "mates": {},
        "summary": summary_report(cad),
    }

    # Export occurrence transforms
    for key, occurrence in cad.occurrences.items():
        name = cad.key_namer.get_name(key)
        export_data["occurrences"][name] = {
            "transform": occurrence.transform,
            "fixed": occurrence.fixed,
            "hidden": occurrence.hidden,
            "path": occurrence.path,
            "analysis": analyze_transform(occurrence.transform, name),
        }

    # Export mate coordinate systems
    for key, mate in cad.mates.items():
        name = cad.key_namer.get_name(key)
        export_data["mates"][name] = explore_mate(mate, name)

    with open(filename, "w") as f:
        json.dump(export_data, f, indent=2)

    print(f"Transform data exported to {filename}")


def plot_subassembly_transforms(cad: CAD, subassembly_name: str):
    """
    Plot both global occurrence transform and subassembly's internal occurrence transforms.

    Args:
        cad: CAD object
        subassembly_name: Name of the subassembly to explore

    This function demonstrates the key concept: how a subassembly's global occurrence
    transform relates to its internal occurrence transforms.
    """
    print(f"Exploring transforms for subassembly: {subassembly_name}")

    # Get the subassembly instance (this is in the main assembly)
    subassembly_instance = cad.lookup_flexible_assembly(subassembly_name)
    if not subassembly_instance:
        subassembly_instance = cad.lookup_rigid_assembly(subassembly_name)

    if not subassembly_instance:
        print(f"Error: Subassembly '{subassembly_name}' not found")
        return

    # Find the occurrence of this subassembly in the main assembly
    subassembly_occurrence = None
    for key, occurrence in cad.occurrences.items():
        occ_name = cad.key_namer.get_name(key)
        if subassembly_name.lower() in occ_name.lower():
            subassembly_occurrence = occurrence
            break

    if not subassembly_occurrence:
        print(f"Error: Could not find occurrence for subassembly '{subassembly_name}'")
        return

    # Get the subassembly data (internal structure)
    subassembly_data = cad.lookup_subassembly(subassembly_name)
    if not subassembly_data:
        subassembly_data = cad.lookup_rigid_subassembly(subassembly_name)

    if not subassembly_data:
        print(f"Error: Could not find subassembly data for '{subassembly_name}'")
        return

    # Create visualization data
    transforms_to_plot = {}

    # Add the global occurrence transform (how the subassembly is positioned in the main assembly)
    transforms_to_plot["Global_Occurrence"] = subassembly_occurrence.transform
    print("Global occurrence transform (subassembly position in main assembly):")
    global_analysis = analyze_transform(subassembly_occurrence.transform, "Global")
    print(f"  Translation: {global_analysis['translation']}")
    print(f"  Rotation: {global_analysis['rotation_angle_deg']:.2f}°")

    # Add internal occurrence transforms (how parts are positioned within the subassembly)
    # Both flexible and rigid subassemblies now use RootAssembly objects
    internal_occurrences = subassembly_data.occurrences

    print(f"\nSubassembly internal occurrences ({len(internal_occurrences)} total):")
    for i, internal_occ in enumerate(internal_occurrences[:5]):  # First 5 internal occurrences
        name = f"Internal_Occ_{i + 1}"
        transforms_to_plot[name] = internal_occ.transform
        internal_analysis = analyze_transform(internal_occ.transform, name)
        print(
            f"  {name}: translation={internal_analysis['translation']}, "
            f"rotation={internal_analysis['rotation_angle_deg']:.2f}°"
        )

    # Plot the transforms
    title = f"Global vs Internal Transforms: {subassembly_name}"
    plot_transforms_3d(transforms_to_plot, title)

    # Show relationship between global and internal transforms
    if len(internal_occurrences) > 0:
        print("\nTransform Relationship Analysis:")
        first_internal = internal_occurrences[0].transform
        comparison = compare_transforms(
            subassembly_occurrence.transform, first_internal, "Global_Occurrence", "First_Internal_Occurrence"
        )
        print(f"  Translation difference magnitude: {comparison['translation_difference_magnitude']:.6f}")
        print(f"  Relative rotation: {comparison['relative_transform_analysis']['rotation_angle_deg']:.2f}°")

        # Key insight: Internal transforms are typically relative to the subassembly's origin,
        # while the global transform positions the entire subassembly in the main assembly
        print("\nKey Insight:")
        print("  - Global transform positions the entire subassembly in the main assembly")
        print("  - Internal transforms position parts relative to the subassembly's origin")
        print("  - To get world coordinates of internal parts, you need to compose:")
        print("    World_Part_Transform = Global_Subassembly_Transform * Internal_Part_Transform")


def get_pickled_assembly():
    if USE_CACHED and os.path.exists(PICKLE_PATH):
        LOGGER.info("Loading assembly from cached pickle file...")
        with open(PICKLE_PATH, "rb") as f:
            assembly = pickle.load(f)  # noqa: S301
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
        # Cache for next time
        with open(PICKLE_PATH, "wb") as f:
            pickle.dump(assembly, f)
        LOGGER.info(f"Assembly cached to {PICKLE_PATH}")

    return assembly


if __name__ == "__main__":
    LOGGER.set_file_name(f"{NAME}.log")
    LOGGER.set_stream_level(LogLevel.INFO)

    # Initialize client
    client = Client(env=".env")

    document = Document.from_url(url=TRANSFORMS)
    client.set_base_url(document.base_url)
    assembly = get_pickled_assembly()

    cad = CAD.from_assembly(assembly, client, max_depth=DEPTH)
    total_assemblies = len(cad.rigid_assembly_instances) + len(cad.flexible_assembly_instances)
    LOGGER.info(
        f"Complete parsing: {len(cad.instances)} instances, {len(cad.parts)} parts, "
        f"{total_assemblies} assemblies ({len(cad.rigid_assembly_instances)} rigid, "
        f"{len(cad.flexible_assembly_instances)} flexible), "
        f"{len(cad.flexible_assembly_data)} flexible assembly data, "
        f"{len(cad.rigid_assembly_data)} rigid assembly data, "
        f"{len(cad.mates)} mates"
    )
    # cad.show_tree()

    # Debug: Check what assemblies we have
    print(f"Flexible assembly instances: {len(cad.flexible_assembly_instances)}")
    print(f"Flexible assembly data: {len(cad.flexible_assembly_data)}")
    print(f"Rigid assembly instances: {len(cad.rigid_assembly_instances)}")
    print(f"Rigid assembly data: {len(cad.rigid_assembly_data)}")

    # Get a flexible subassembly and plot its transforms
    report = summary_report(cad)
    flexible_assemblies = report["assembly_breakdown"]["flexible_assembly_names"]
    print(f"Flexible assemblies from report: {flexible_assemblies}")

    # Debug: print all keys and names for flexible data
    print("Debug - Flexible assembly data keys:")
    for key in cad.flexible_assembly_data:
        name = cad.key_namer.get_name(key)
        print(f"  Key: {key} -> Name: {name}")

    print("Debug - Flexible assembly instance keys:")
    for key in cad.flexible_assembly_instances:
        name = cad.key_namer.get_name(key)
        print(f"  Key: {key} -> Name: {name}")

    if flexible_assemblies:
        target_subassembly = flexible_assemblies[0]
        plot_subassembly_transforms(cad, target_subassembly)
