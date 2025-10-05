"""Complete example: From Onshape assembly to Robot URDF using PathKey system.

This example demonstrates the full workflow:

1. **Load Assembly**: Fetch from Onshape or use cached JSON
2. **Create CAD**: Parse assembly into PathKey-based registries
3. **Process Mates**: Filter, remap, and expand patterns
4. **Build KinematicGraph**: Construct directed graph from mates
5. **Create Robot**: Generate URDF structure with links and joints
6. **Visualize**: Plot transforms and show graph structure
7. **Save**: Export URDF with STL meshes

Key Features Demonstrated:
- plot_all_transforms() - 3D visualization of occurrence transforms
- get_occurrence_transform_by_name() - Lookup transforms by part name
- create_robot_from_kinematic_graph() - Generate Robot from KinematicGraph
- PathKey-based navigation - Efficient assembly traversal
- Selective mass property fetching - Only fetch for kinematic parts

The workflow shows how the new PathKey system provides:
✓ Clean separation of concerns (CAD → Graph → Robot)
✓ Efficient API usage (batch fetching, caching)
✓ Type safety with Pydantic models
✓ Automatic rigid assembly handling
"""

import os

import numpy as np

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models import Assembly
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD, PathKey, fetch_mass_properties_for_kinematic_parts
from onshape_robotics_toolkit.robot import Robot, RobotType, get_robot_from_kinematic_graph
from onshape_robotics_toolkit.utilities import load_model_from_json, save_model_as_json
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name

DEPTH = 0
NAME = f"transforms_{DEPTH}"
USE_CACHED = True  # Set to False to force fresh API call
LOG_ASSEMBLY = False

# TRANSFORMS = (
#     "https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/996c149cb195c9268ab062b4"
# )

TRANSFORMS = (
    "https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/46679c6ab890a1b7a6d11a88"
)


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


def create_robot_from_kinematic_graph(
    cad: CAD,
    kinematic_graph: KinematicGraph,
    client: Client,
    robot_name: str = "robot",
    robot_type: RobotType = RobotType.URDF,
    include_rigid_subassembly_parts: bool = False,
) -> Robot:
    """
    Create a Robot structure from a KinematicGraph.

    This demonstrates the new PathKey-based robot creation workflow:
    1. Fetch mass properties for parts in kinematic chain
    2. Generate Robot structure with links and joints
    3. Save URDF/MJCF files

    Args:
        cad: CAD object with PathKey registries
        kinematic_graph: KinematicGraph with parts and mates
        client: Onshape client for downloading assets
        robot_name: Name for the robot
        robot_type: URDF or MJCF output format
        include_rigid_subassembly_parts: Whether to include rigid subassembly parts

    Returns:
        Robot object ready to save

    Examples:
        >>> robot = create_robot_from_kinematic_graph(cad, tree, client, "my_robot")
        >>> robot.save("my_robot.urdf", download_assets=True)
    """
    # Step 1: Fetch mass properties for kinematic parts
    fetch_mass_properties_for_kinematic_parts(
        cad=cad,
        kinematic_graph=kinematic_graph,
        client=client,
        include_rigid_subassembly_parts=include_rigid_subassembly_parts,
    )

    parts_with_mass = sum(1 for part in cad.parts.values() if part.MassProperty is not None)
    LOGGER.info(f"Mass properties loaded: {parts_with_mass}/{len(cad.parts)} parts")

    # Step 2: Generate Robot structure
    robot = get_robot_from_kinematic_graph(
        cad=cad,
        kinematic_graph=kinematic_graph,
        client=client,
        robot_name=robot_name,
    )
    robot.type = robot_type

    return robot


if __name__ == "__main__":
    LOGGER.set_file_name(f"{NAME}.log")
    LOGGER.set_stream_level(LogLevel.INFO)

    client = Client(env=".env")

    document = Document.from_url(url=TRANSFORMS)
    client.set_base_url(document.base_url)

    if USE_CACHED and os.path.exists(f"{NAME}.json"):
        assembly = load_model_from_json(Assembly, f"{NAME}.json")
    else:
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

    cad = CAD.from_assembly(assembly, max_depth=DEPTH)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
    graph.show(f"KinematicGraph_Depth_{DEPTH}")

    # Create robot from kinematic graph using new Robot.from_graph() method
    robot = Robot.from_graph(
        cad=cad,
        kinematic_graph=graph,
        client=client,
        name=f"robot_{DEPTH}",
        robot_type=RobotType.URDF,
        include_rigid_subassembly_parts=False,
    )
    robot.save(f"robot_{DEPTH}.urdf", download_assets=True)
