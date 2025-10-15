"""
This module contains classes for creating a URDF robot model

Dataclass:
    - **Robot**: Represents a robot model in URDF format, containing links and joints.

"""

import asyncio
from typing import TYPE_CHECKING, Optional, Union

import networkx as nx
import numpy as np
from lxml import etree as ET

if TYPE_CHECKING:
    from onshape_robotics_toolkit.graph import KinematicGraph
    from onshape_robotics_toolkit.parse import PathKey

import random

from loguru import logger

from onshape_robotics_toolkit.config import (
    record_robot_config,
    resolve_mate_limits,
    resolve_mate_name,
    resolve_part_name,
)
from onshape_robotics_toolkit.connect import Asset, Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.assembly import (
    MatedCS,
    MateFeatureData,
    MateType,
    Part,
)
from onshape_robotics_toolkit.models.document import WorkspaceType
from onshape_robotics_toolkit.models.geometry import MeshGeometry
from onshape_robotics_toolkit.models.joint import (
    BaseJoint,
    ContinuousJoint,
    DummyJoint,
    FixedJoint,
    FloatingJoint,
    JointLimits,
    # JointDynamics,
    JointMimic,
    JointType,
    PrismaticJoint,
    RevoluteJoint,
)
from onshape_robotics_toolkit.models.link import (
    Axis,
    CollisionLink,
    Colors,
    Inertia,
    InertialLink,
    Link,
    Material,
    Origin,
    VisualLink,
)
from onshape_robotics_toolkit.parse import (
    PathKey,
)
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name, make_unique_name


def set_joint_from_xml(element: ET._Element) -> BaseJoint | None:
    """
    Set the joint type from an XML element.

    Args:
        element (ET.Element): The XML element.

    Returns:
        BaseJoint: The joint type.

    Examples:
        >>> element = ET.Element("joint", type="fixed")
        >>> set_joint_from_xml(element)
        <FixedJoint>
    """
    joint_type = element.get("type")
    if joint_type is None:
        return None
    if joint_type == JointType.FIXED:
        return FixedJoint.from_xml(element)
    elif joint_type == JointType.REVOLUTE:
        return RevoluteJoint.from_xml(element)
    elif joint_type == JointType.CONTINUOUS:
        return ContinuousJoint.from_xml(element)
    elif joint_type == JointType.PRISMATIC:
        return PrismaticJoint.from_xml(element)
    elif joint_type == JointType.FLOATING:
        return FloatingJoint.from_xml(element)
    return None


def get_robot_link(
    name: str,
    part: Part,
    client: Client,
    mate: Optional[Union[MateFeatureData, None]] = None,
    mesh_dir: Optional[str] = None,
) -> tuple[Link, np.matrix, Asset]:
    """
    Generate a URDF link from an Onshape part.

    Args:
        name: The name of the link.
        part: The Onshape part object.
        client: The Onshape client object to use for sending API requests.
        mate: MateFeatureData object to use for generating the transformation matrix.
        mesh_dir: Optional custom directory for mesh files.

    Returns:
        tuple[Link, np.matrix]: The generated link object
            and the transformation matrix from the STL origin to the link origin.

    Examples:
        >>> get_robot_link("root", part, wid, client)
        (
            Link(name='root', visual=VisualLink(...), collision=CollisionLink(...), inertial=InertialLink(...)),
            np.matrix([[1., 0., 0., 0.],
                [0., 1., 0., 0.],
                [0., 0., 1., 0.],
                [0., 0., 0., 1.]])
        )

    """
    # place link at world origin by default
    _link_pose_wrt_world = np.eye(4)

    if mate is not None:
        # NOTE: we remapped the mates to always be parent->child, regardless of
        # how Onshape considers (parent, child) of a mate
        child_part_to_mate: MatedCS = mate.matedEntities[-1].matedCS
        # NOTE: child link's origin is always at the mate location, and since
        # the joint origin is already transformed to world coordinates,
        # we only use the child part's mate location to determine
        # the child link's origin
        _link_pose_wrt_world = child_part_to_mate.to_tf
    else:
        if part.worldToPartTF is not None:
            _link_pose_wrt_world = part.worldToPartTF.to_tf
        else:
            logger.warning(f"Part {name} has no worldToPartTF, using identity matrix")

    world_to_link_tf = np.linalg.inv(_link_pose_wrt_world)
    _origin = Origin.zero_origin()
    _principal_axes_rotation = (0.0, 0.0, 0.0)

    # Check if part has mass properties
    if part.MassProperty is None:
        # TODO: use downloaded assets + material library to find these values
        # using numpy-stl library
        logger.warning(f"Part {name} has no mass properties, using default values")
        _mass = 1.0  # Default mass
        _com = (0.0, 0.0, 0.0)  # Default center of mass at origin
        _inertia = np.eye(3)  # Default identity inertia matrix
    else:
        _mass = part.MassProperty.mass[0]
        # Convert ndarray to matrix for compatibility with MassProperty methods
        world_to_link_matrix = np.matrix(world_to_link_tf)
        _com = tuple(part.MassProperty.center_of_mass_wrt(world_to_link_matrix))
        _inertia = part.MassProperty.inertia_wrt(world_to_link_matrix[:3, :3])

    logger.info(f"Creating robot link for {name}")

    # Determine workspace type and ID for fetching the mesh
    mvwid: str
    if part.documentVersion:
        # Part from a specific version
        wtype = WorkspaceType.V.value
        mvwid = part.documentVersion
    elif part.isRigidAssembly:
        # Rigid assembly - use workspace type with its workspace ID
        # The assembly STL API requires workspace type and workspace ID
        wtype = WorkspaceType.W.value
        if part.rigidAssemblyWorkspaceId is not None:
            mvwid = part.rigidAssemblyWorkspaceId
        else:
            logger.error("Rigid part is missing workspace ID")
    else:
        # Regular part - use its documentMicroversion with microversion type
        wtype = WorkspaceType.M.value
        mvwid = part.documentMicroversion

    asset = Asset(
        did=part.documentId,
        wtype=wtype,
        wid=mvwid,
        eid=part.elementId,
        partID=part.partId,
        client=client,
        transform=world_to_link_tf,
        is_rigid_assembly=part.isRigidAssembly,
        file_name=f"{name}.stl",
        mesh_dir=mesh_dir,
    )
    _mesh_path = asset.relative_path

    link = Link(
        name=name,
        visual=VisualLink(
            name=f"{name}_visual",
            origin=_origin,
            geometry=MeshGeometry(_mesh_path),
            material=Material.from_color(name=f"{name}-material", color=random.SystemRandom().choice(list(Colors))),
        ),
        inertial=InertialLink(
            origin=Origin(
                xyz=_com,
                rpy=_principal_axes_rotation,
            ),
            mass=_mass,
            inertia=Inertia(
                ixx=_inertia[0, 0],
                ixy=_inertia[0, 1],
                ixz=_inertia[0, 2],
                iyy=_inertia[1, 1],
                iyz=_inertia[1, 2],
                izz=_inertia[2, 2],
            ),
        ),
        collision=CollisionLink(
            name=f"{name}_collision",
            origin=_origin,
            geometry=MeshGeometry(_mesh_path),
        ),
    )

    # Convert to matrix for compatibility with downstream code
    world_to_link_matrix = np.matrix(world_to_link_tf)
    return link, world_to_link_matrix, asset


def get_robot_joint(
    parent_key: PathKey,
    child_key: PathKey,
    mate: MateFeatureData,
    world_to_parent_tf: np.matrix,
    used_joint_names: set,
    mimic: Optional[JointMimic] = None,
) -> tuple[dict[tuple[PathKey, PathKey], BaseJoint], Optional[dict[PathKey, Link]]]:
    """
    Generate a URDF joint from an Onshape mate feature.

    Args:
        parent_key: The PathKey of the parent link.
        child_key: The PathKey of the child link.
        mate: The Onshape mate feature object.
        world_to_parent_tf: The transformation matrix from world to parent link origin.
        used_joint_names: Set of already used joint names for uniqueness checking.
        mimic: The mimic joint object.

    Returns:
        tuple[dict[tuple[PathKey, PathKey], BaseJoint], Optional[dict[PathKey, Link]]]:
            The generated joints dict and optional dummy links dict.

    Examples:
        >>> get_robot_joint("root", "link1", mate, np.eye(4))
        (
            [
                RevoluteJoint(
                    name='base_link_to_link1',
                    parent='root',
                    child='link1',
                    origin=Origin(...),
                    limits=JointLimits(...),
                    axis=Axis(...),
                    dynamics=JointDynamics(...)
                )
            ],
            None
        )

    """
    links: dict[PathKey, Link] = {}
    joints: dict[tuple[PathKey, PathKey], BaseJoint] = {}

    world_to_joint_tf = np.eye(4)

    # NOTE: we remapped the mates to always be parent->child, regardless of
    # how Onshape considers (parent, child) of a mate
    parent_part_to_mate = mate.matedEntities[0].matedCS
    world_to_joint_tf = world_to_parent_tf @ parent_part_to_mate.to_tf

    origin = Origin.from_matrix(world_to_joint_tf)
    base_name = get_sanitized_name(mate.name)
    resolved_name = resolve_mate_name(base_name)
    joint_name = make_unique_name(resolved_name, used_joint_names)
    used_joint_names.add(joint_name)

    logger.info(f"Creating robot joint from {parent_key} to {child_key}")

    parent_link_name = resolve_part_name(str(parent_key))
    child_link_name = resolve_part_name(str(child_key))

    if mate.mateType == MateType.REVOLUTE:
        # Extract limits with priority order:
        # 1. config limits (user overrides)
        # 2. mate.limits (fetched from API)
        # 3. None (omit limits for revolute joints)
        revolute_limits = None
        limit_source = None

        config_limits = resolve_mate_limits(base_name)
        if config_limits is not None and "min" in config_limits and "max" in config_limits:
            revolute_limits = JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=config_limits["min"],
                upper=config_limits["max"],
            )
            limit_source = "config"
        elif mate.limits is not None and "min" in mate.limits and "max" in mate.limits:
            # Fallback to API limits when no override is provided
            revolute_limits = JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=mate.limits["min"],
                upper=mate.limits["max"],
            )
            limit_source = "API"

        if revolute_limits is None:
            revolute_limits = JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=-2 * np.pi,
                upper=2 * np.pi,
            )
            limit_source = "default"

        logger.debug(
            f"Using {limit_source} limits for mate '{mate.name}': "
            f"min={revolute_limits.lower:.4f}, max={revolute_limits.upper:.4f}"
        )

        joints[(parent_key, child_key)] = RevoluteJoint(
            name=joint_name,
            parent=parent_link_name,
            child=child_link_name,
            origin=origin,
            limits=revolute_limits,
            axis=Axis((0.0, 0.0, -1.0)),
            # dynamics=JointDynamics(damping=0.1, friction=0.1),
            mimic=mimic,
        )

    elif mate.mateType == MateType.FASTENED:
        joints[(parent_key, child_key)] = FixedJoint(
            name=joint_name, parent=parent_link_name, child=child_link_name, origin=origin
        )

    elif mate.mateType == MateType.SLIDER or mate.mateType == MateType.CYLINDRICAL:
        # For prismatic joints, use fetched limits or defaults (in meters)
        # NOTE: Onshape limits are defined along +Z axis, but URDF uses -Z axis
        # So we need to negate and swap min/max to account for the flipped direction
        prismatic_lower: float | None = None
        prismatic_upper: float | None = None
        limit_source = None

        config_limits = resolve_mate_limits(base_name)
        if config_limits is not None and "min" in config_limits and "max" in config_limits:
            prismatic_lower = -config_limits["max"]
            prismatic_upper = -config_limits["min"]
            limit_source = "config"
        elif mate.limits is not None and "min" in mate.limits and "max" in mate.limits:
            # Swap and negate: Onshape's min becomes URDF's upper (negated)
            # and Onshape's max becomes URDF's lower (negated)
            prismatic_lower = -mate.limits["max"]
            prismatic_upper = -mate.limits["min"]
            limit_source = "API"

        if prismatic_lower is None or prismatic_upper is None:
            prismatic_lower = -0.1
            prismatic_upper = 0.1
            limit_source = "default"

        if limit_source == "default":
            logger.debug(
                f"No limits available for mate '{mate.name}', using default prismatic range "
                f"lower={prismatic_lower:.4f}, upper={prismatic_upper:.4f}"
            )
        else:
            logger.debug(
                f"Using {limit_source} limits for mate '{mate.name}': "
                f"lower={prismatic_lower:.4f}, upper={prismatic_upper:.4f}"
            )

        joints[(parent_key, child_key)] = PrismaticJoint(
            name=joint_name,
            parent=parent_link_name,
            child=child_link_name,
            origin=origin,
            limits=JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=prismatic_lower,
                upper=prismatic_upper,
            ),
            axis=Axis((0.0, 0.0, -1.0)),
            # dynamics=JointDynamics(damping=0.1, friction=0.1),
            mimic=mimic,
        )

    elif mate.mateType == MateType.BALL:
        dummy_x_key = PathKey(
            path=(*parent_key.path, joint_name, "x"),
            name_path=(*parent_key.name_path, joint_name, "x"),
        )
        dummy_y_key = PathKey(
            path=(*parent_key.path, joint_name, "y"),
            name_path=(*parent_key.name_path, joint_name, "y"),
        )

        dummy_x_link = Link(
            name=str(dummy_x_key),
            inertial=InertialLink(
                mass=0.0,
                inertia=Inertia.zero_inertia(),
                origin=Origin.zero_origin(),
            ),
        )
        dummy_y_link = Link(
            name=str(dummy_y_key),
            inertial=InertialLink(
                mass=0.0,
                inertia=Inertia.zero_inertia(),
                origin=Origin.zero_origin(),
            ),
        )

        links[dummy_x_key] = dummy_x_link
        links[dummy_y_key] = dummy_y_link

        joints[(parent_key, dummy_x_key)] = RevoluteJoint(
            name=joint_name + "_x",
            parent=parent_link_name,
            child=str(dummy_x_key),
            origin=origin,
            limits=JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=-2 * np.pi,
                upper=2 * np.pi,
            ),
            axis=Axis((1.0, 0.0, 0.0)),
            # dynamics=JointDynamics(damping=0.1, friction=0.1),
            mimic=mimic,
        )
        joints[(dummy_x_key, dummy_y_key)] = RevoluteJoint(
            name=joint_name + "_y",
            parent=str(dummy_x_key),
            child=str(dummy_y_key),
            origin=Origin.zero_origin(),
            limits=JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=-2 * np.pi,
                upper=2 * np.pi,
            ),
            axis=Axis((0.0, 1.0, 0.0)),
            # dynamics=JointDynamics(damping=0.1, friction=0.1),
            mimic=mimic,
        )
        joints[(dummy_y_key, child_key)] = RevoluteJoint(
            name=joint_name + "_z",
            parent=str(dummy_y_key),
            child=child_link_name,
            origin=Origin.zero_origin(),
            limits=JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=-2 * np.pi,
                upper=2 * np.pi,
            ),
            axis=Axis((0.0, 0.0, -1.0)),
            # dynamics=JointDynamics(damping=0.1, friction=0.1),
            mimic=mimic,
        )

    else:
        logger.warning(f"Unsupported joint type: {mate.mateType}")
        joints[(parent_key, child_key)] = DummyJoint(
            name=joint_name, parent=parent_link_name, child=child_link_name, origin=origin
        )

    return joints, links


class Robot(nx.DiGraph):
    """
    Represents a robot model with a graph structure for links and joints.

    The Robot class is the final output of the CAD → KinematicGraph → Robot pipeline.
    It stores the robot structure as a NetworkX directed graph where nodes are links
    and edges are joints, along with associated STL assets.

    **Recommended Creation Methods:**
    - `Robot.from_graph()`: Create from pre-built CAD + KinematicGraph (most efficient)
    - `Robot.from_url()`: Create directly from Onshape URL (most convenient)

    **Attributes:**
        name (str): The name of the robot
        kinematic_graph (KinematicGraph): The kinematic graph used to create the robot
        graph (nx.DiGraph): Graph structure holding links (nodes) and joints (edges)

    **Key Methods:**
        show_tree: Display the robot's graph as a tree structure
        show_graph: Display the robot's graph as a directed graph
        from_graph: Create robot from KinematicGraph (recommended)
        from_url: Create robot from Onshape URL

    **Serialization:**
    To export the robot to URDF or MJCF formats, use the format-specific serializers
    from the `onshape_robotics_toolkit.formats` module:

        >>> from onshape_robotics_toolkit.formats import URDFSerializer, MJCFSerializer, MJCFConfig
        >>> # URDF export
        >>> urdf_serializer = URDFSerializer()
        >>> urdf_serializer.save(robot, "robot.urdf", download_assets=True)
        >>>
        >>> # MJCF export with configuration
        >>> mjcf_config = MJCFConfig(position=(0, 0, 1), add_ground_plane=True)
        >>> mjcf_serializer = MJCFSerializer(mjcf_config)
        >>> mjcf_serializer.save(robot, "robot.xml", download_assets=True)

    **Example:**
        >>> from onshape_robotics_toolkit.connect import Client
        >>> from onshape_robotics_toolkit.graph import KinematicGraph
        >>> from onshape_robotics_toolkit.formats import URDFSerializer
        >>>
        >>> # Option 1: From URL (convenient)
        >>> robot = Robot.from_url(
        ...     name="my_robot",
        ...     url="https://cad.onshape.com/documents/...",
        ...     client=Client(),
        ...     max_depth=1
        ... )
        >>>
        >>> # Option 2: From KinematicGraph (efficient, more control)
        >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
        >>> robot = Robot.from_graph(graph, Client(), "my_robot")
        >>>
        >>> # Save to file using format serializers
        >>> serializer = URDFSerializer()
        >>> serializer.save(robot, "robot.urdf", download_assets=True)
    """

    def __init__(self, kinematic_graph: KinematicGraph, name: str):
        """
        Initialize a Robot instance.

        Args:
            kinematic_graph: The kinematic graph containing robot structure
            name: The name of the robot

        Note:
            This constructor is typically not called directly. Use Robot.from_graph()
            to create robot instances from kinematic graphs.
        """
        self.kinematic_graph: KinematicGraph = kinematic_graph
        super().__init__(name=name)

    # TODO: implement from URDF method with PathKeys and new graph system
    @classmethod
    def from_graph(
        cls,
        kinematic_graph: "KinematicGraph",
        client: Client,
        name: str,
        fetch_mass_properties: bool = True,
    ) -> "Robot":
        """
        Create a Robot from pre-built CAD and KinematicGraph objects.

        This is the recommended method for creating robots when you already have
        CAD and KinematicGraph instances. It handles mass property fetching
        and robot generation in an efficient, streamlined way.

        Args:
            kinematic_graph: Kinematic graph with parts and mates
            client: Onshape client for downloading assets and fetching mass properties
            name: The name of the robot
            fetch_mass_properties: Whether to fetch mass properties for kinematic parts

        Returns:
            Robot: The generated robot model

        Example:
            >>> from onshape_robotics_toolkit.parse import CAD
            >>> from onshape_robotics_toolkit.graph import KinematicGraph
            >>> from onshape_robotics_toolkit.formats import URDFSerializer
            >>> cad = CAD.from_assembly(assembly, max_depth=1)
            >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
            >>> robot = Robot.from_graph(graph, client, "my_robot")
            >>> # Save using format serializers
            >>> serializer = URDFSerializer()
            >>> serializer.save(robot, "robot.urdf", download_assets=True)
        """
        # Check for empty kinematic graph
        if len(kinematic_graph.nodes) == 0:
            raise ValueError(
                "Cannot create robot from empty kinematic graph. "
                "The assembly contains only mate groups with no rigid assemblies or fixed parts. "
                "Cannot determine a root link for the robot. "
                "Mark at least one part or subassembly as fixed in Onshape, or ensure rigid assemblies exist."
            )

        if fetch_mass_properties:
            asyncio.run(kinematic_graph.cad.fetch_mass_properties_for_parts(client))

        # Generate robot structure from kinematic graph
        robot = cls(
            kinematic_graph=kinematic_graph,
            name=name,
        )
        record_robot_config(
            name=name,
            fetch_mass_properties=fetch_mass_properties,
        )

        # Get root node from kinematic graph
        if kinematic_graph.root is None:
            raise ValueError("Kinematic graph has no root node")

        root_key = kinematic_graph.root
        logger.info(f"Processing root node: {root_key}")

        root_part = robot.kinematic_graph.nodes[root_key]["data"]
        # NOTE: make sure Pathkey.__str__ produces names without
        # special characters that are invalid in URDF/MJCF
        root_default_name = str(root_key)
        root_name = resolve_part_name(root_default_name)
        root_link, world_to_root_link, root_asset = get_robot_link(
            name=root_name,
            part=root_part,
            client=client,
            mate=None,
        )

        robot.add_node(root_key, data=root_link, asset=root_asset, world_to_link_tf=world_to_root_link)
        logger.info(f"Processing {len(kinematic_graph.edges)} edges in the kinematic graph.")

        used_joint_names: set[str] = set()

        # Process edges in topological order
        for parent_key, child_key in robot.kinematic_graph.edges:
            logger.info(f"Processing edge: {parent_key} → {child_key}")

            # Get parent transform
            world_to_parent_tf = robot.nodes[parent_key]["world_to_link_tf"]

            robot.kinematic_graph.nodes[parent_key]["data"]
            child_part: Part = robot.kinematic_graph.nodes[child_key]["data"]

            # Get mate data from graph edge
            mate_data: MateFeatureData = robot.kinematic_graph.get_edge_data(parent_key, child_key)["data"]
            if mate_data is None:
                logger.warning(f"No mate data found for edge {parent_key} → {child_key}. Skipping.")
                continue

            # Check for mate relations (mimic joints)
            joint_mimic = None
            # TODO: Implement mate relation support with PathKey system
            # This will require updating the relation processing to use PathKeys

            # Create/get joint(s)
            # For spherical joints, dummy links and joints are created
            joints_dict, links_dict = get_robot_joint(
                parent_key=parent_key,
                child_key=child_key,
                mate=mate_data,
                world_to_parent_tf=world_to_parent_tf,
                used_joint_names=used_joint_names,
                mimic=joint_mimic,
            )

            # Create child link
            child_default_name = str(child_key)
            child_name = resolve_part_name(child_default_name)

            link, world_to_link_tf, asset = get_robot_link(
                name=child_name,
                part=child_part,
                client=client,
                mate=mate_data,
            )

            # Add child link if not already in graph
            if child_key not in robot.nodes:
                robot.add_node(child_key, data=link, asset=asset, world_to_link_tf=world_to_link_tf)
            else:
                # NOTE: possible cause for this: the kinematic graph has a loop
                logger.warning(f"Link {child_key} already exists in the robot graph. Skipping.")

            if links_dict is not None:
                for _link_key, _link in links_dict.items():
                    if _link_key not in robot.nodes:
                        robot.add_node(
                            _link_key,
                            data=_link,
                            asset=None,
                            world_to_link_tf=None,
                        )
                    else:
                        logger.warning(f"Link {_link_key} already exists in the robot graph. Skipping.")

            # Add joints
            for _joint_key, _joint_data in joints_dict.items():
                robot.add_edge(_joint_key[0], _joint_key[1], data=_joint_data)

        return robot

    def show_tree(self) -> None:
        """Display the robot's graph as a tree structure."""

        def print_tree(node: str, depth: int = 0) -> None:
            prefix = "    " * depth
            print(f"{prefix}{node}")
            for child in self.kinematic_graph.successors(node):
                print_tree(child, depth + 1)

        root_nodes = [n for n in self.kinematic_graph.nodes if self.kinematic_graph.in_degree(n) == 0]
        for root in root_nodes:
            print_tree(root)

    async def _download_assets(self, mesh_dir: Optional[str] = None) -> None:
        """Asynchronously download the assets.

        Args:
            mesh_dir: Optional custom directory for mesh files. If provided, updates all assets
                to use this directory before downloading.
        """
        tasks = []
        for _node, data in self.nodes(data=True):
            asset = data.get("asset")
            if asset and not asset.is_from_file:
                # Update asset's mesh directory if specified
                if mesh_dir is not None:
                    asset.mesh_dir = mesh_dir
                tasks.append(asset.download())
        try:
            await asyncio.gather(*tasks)
            logger.info("All assets downloaded successfully.")
        except Exception as e:
            logger.error(f"Error downloading assets: {e}")
