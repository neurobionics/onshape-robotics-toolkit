"""
This module contains classes for creating a URDF robot model

Dataclass:
    - **Robot**: Represents a robot model in URDF format, containing links and joints.

"""

import asyncio
import os
from enum import Enum
from typing import TYPE_CHECKING, Any, Optional, Union

import networkx as nx
import numpy as np
from lxml import etree as ET
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from onshape_robotics_toolkit.graph import KinematicGraph
    from onshape_robotics_toolkit.parse import PathKey


import random

from onshape_robotics_toolkit.connect import Asset, Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.log import LOGGER
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
from onshape_robotics_toolkit.models.mjcf import Actuator, Encoder, ForceSensor, Light, Sensor
from onshape_robotics_toolkit.parse import (
    PathKey,
)
from onshape_robotics_toolkit.utilities.helpers import format_number, get_sanitized_name, make_unique_name

DEFAULT_COMPILER_ATTRIBUTES = {
    "angle": "radian",
    "eulerseq": "xyz",
    # "meshdir": "meshes",
}

DEFAULT_OPTION_ATTRIBUTES = {"timestep": "0.001", "gravity": "0 0 -9.81", "iterations": "50"}

URDF_EULER_SEQ = "xyz"  # URDF uses XYZ fixed angles
MJCF_EULER_SEQ = "XYZ"  # MuJoCo uses XYZ extrinsic rotations, capitalization matters

ACTUATOR_SUFFIX = "-actuator"

# TODO: Add custom path for meshes and robot description file


class RobotType(str, Enum):
    """
    Enum for different types of robots.
    """

    URDF = "urdf"
    MJCF = "xml"

    def __str__(self) -> str:
        return self.value


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
            LOGGER.warning(f"Part {name} has no worldToPartTF, using identity matrix")

    world_to_link_tf = np.linalg.inv(_link_pose_wrt_world)
    _origin = Origin.zero_origin()
    _principal_axes_rotation = (0.0, 0.0, 0.0)

    # Check if part has mass properties
    if part.MassProperty is None:
        # TODO: use downloaded assets + material library to find these values
        # using numpy-stl library
        LOGGER.warning(f"Part {name} has no mass properties, using default values")
        _mass = 1.0  # Default mass
        _com = (0.0, 0.0, 0.0)  # Default center of mass at origin
        _inertia = np.eye(3)  # Default identity inertia matrix
    else:
        _mass = part.MassProperty.mass[0]
        # Convert ndarray to matrix for compatibility with MassProperty methods
        world_to_link_matrix = np.matrix(world_to_link_tf)
        _com = tuple(part.MassProperty.center_of_mass_wrt(world_to_link_matrix))
        _inertia = part.MassProperty.inertia_wrt(world_to_link_matrix[:3, :3])

    LOGGER.info(f"Creating robot link for {name}")

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
            LOGGER.error("Rigid part is missing workspace ID")
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
    joint_name = make_unique_name(base_name, used_joint_names)
    used_joint_names.add(joint_name)

    LOGGER.info(f"Creating robot joint from {parent_key} to {child_key}")

    if mate.mateType == MateType.REVOLUTE:
        joints[(parent_key, child_key)] = RevoluteJoint(
            name=joint_name,
            parent=str(parent_key),
            child=str(child_key),
            origin=origin,
            limits=JointLimits(
                effort=1.0,
                velocity=1.0,
                # TODO: add support for joint limits from Onshape
                lower=-2 * np.pi,
                upper=2 * np.pi,
            ),
            axis=Axis((0.0, 0.0, -1.0)),
            # dynamics=JointDynamics(damping=0.1, friction=0.1),
            mimic=mimic,
        )

    elif mate.mateType == MateType.FASTENED:
        joints[(parent_key, child_key)] = FixedJoint(
            name=joint_name, parent=str(parent_key), child=str(child_key), origin=origin
        )

    elif mate.mateType == MateType.SLIDER or mate.mateType == MateType.CYLINDRICAL:
        joints[(parent_key, child_key)] = PrismaticJoint(
            name=joint_name,
            parent=str(parent_key),
            child=str(child_key),
            origin=origin,
            limits=JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=-0.1,
                upper=0.1,
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
            parent=str(parent_key),
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
            child=str(child_key),
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
        LOGGER.warning(f"Unsupported joint type: {mate.mateType}")
        joints[(parent_key, child_key)] = DummyJoint(
            name=joint_name, parent=str(parent_key), child=str(child_key), origin=origin
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
    - `Robot.from_urdf()`: Load from existing URDF file

    **Attributes:**
        name (str): The name of the robot
        graph (nx.DiGraph): Graph structure holding links (nodes) and joints (edges)
        assets (dict[str, Asset]): STL assets associated with the robot's links
        type (RobotType): The type of the robot (URDF or MJCF)

    **Key Methods:**
        add_link: Add a link to the graph
        add_joint: Add a joint to the graph
        to_urdf: Generate URDF XML from the graph
        to_mjcf: Generate MuJoCo MJCF XML from the graph
        save: Save the robot model to a file (URDF or MJCF)
        show_tree: Display the robot's graph as a tree
        show_graph: Display the robot's graph as a directed graph
        from_graph: Create robot from CAD + KinematicGraph (recommended)
        from_url: Create robot from Onshape URL
        from_urdf: Load robot from URDF file

    **Example:**
        >>> from onshape_robotics_toolkit.connect import Client
        >>> from onshape_robotics_toolkit.parse import CAD
        >>> from onshape_robotics_toolkit.graph import KinematicGraph
        >>>
        >>> # Option 1: From URL (convenient)
        >>> robot = Robot.from_url(
        ...     name="my_robot",
        ...     url="https://cad.onshape.com/documents/...",
        ...     client=Client(),
        ...     max_depth=1
        ... )
        >>>
        >>> # Option 2: From CAD + Graph (efficient, more control)
        >>> cad = CAD.from_assembly(assembly, max_depth=1)
        >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
        >>> robot = Robot.from_graph(cad, graph, Client(), "my_robot")
        >>>
        >>> # Save to file
        >>> robot.save("robot.urdf", download_assets=True)
    """

    def __init__(self, kinematic_graph: KinematicGraph, name: str, robot_type: RobotType = RobotType.URDF):
        self.kinematic_graph: KinematicGraph = kinematic_graph
        self.type: RobotType = robot_type

        self.model: Optional[ET._Element] = None

        # MuJoCo attributes
        self.lights: dict[str, Any] = {}
        self.cameras: dict[str, Any] = {}
        self.actuators: dict[str, Any] = {}
        self.sensors: dict[str, Any] = {}
        self.custom_elements: dict[str, Any] = {}
        self.mutated_elements: dict[str, Any] = {}

        self.position: tuple[float, float, float] = (0, 0, 0)
        self.ground_position: tuple[float, float, float] = (0, 0, 0)
        self.compiler_attributes: dict[str, str] = DEFAULT_COMPILER_ATTRIBUTES
        self.option_attributes: dict[str, str] = DEFAULT_OPTION_ATTRIBUTES

        super().__init__(name=name)

    # TODO: implement from URDF method with PathKeys and new graph system
    @classmethod
    def from_graph(
        cls,
        kinematic_graph: "KinematicGraph",
        client: Client,
        name: str,
        robot_type: RobotType = RobotType.URDF,
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
            robot_type: The type of the robot (URDF or MJCF)
            fetch_mass_properties: Whether to fetch mass properties for kinematic parts

        Returns:
            Robot: The generated robot model

        Example:
            >>> from onshape_robotics_toolkit.parse import CAD
            >>> from onshape_robotics_toolkit.graph import KinematicGraph
            >>> cad = CAD.from_assembly(assembly, max_depth=1)
            >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
            >>> robot = Robot.from_graph(cad, graph, client, "my_robot")
            >>> robot.save("robot.urdf", download_assets=True)
        """
        if fetch_mass_properties:
            asyncio.run(kinematic_graph.cad.fetch_mass_properties_for_parts(client))

        # Generate robot structure from kinematic graph
        robot = cls(
            kinematic_graph=kinematic_graph,
            name=name,
            robot_type=robot_type,
        )

        # Get root node from kinematic graph
        if kinematic_graph.root is None:
            raise ValueError("Kinematic graph has no root node")

        root_key = kinematic_graph.root
        LOGGER.info(f"Processing root node: {root_key}")

        root_part = robot.kinematic_graph.nodes[root_key]["data"]
        # NOTE: make sure Pathkey.__str__ produces names without
        # special characters that are invalid in URDF/MJCF
        root_name = str(root_key)
        root_link, world_to_root_link, root_asset = get_robot_link(
            name=root_name,
            part=root_part,
            client=client,
            mate=None,
        )

        robot.add_node(root_key, data=root_link, asset=root_asset, world_to_link_tf=world_to_root_link)
        LOGGER.info(f"Processing {len(kinematic_graph.edges)} edges in the kinematic graph.")

        used_joint_names: set[str] = set()

        # Process edges in topological order
        for parent_key, child_key in robot.kinematic_graph.edges:
            LOGGER.info(f"Processing edge: {parent_key} -> {child_key}")

            # Get parent transform
            world_to_parent_tf = robot.nodes[parent_key]["world_to_link_tf"]

            robot.kinematic_graph.nodes[parent_key]["data"]
            child_part: Part = robot.kinematic_graph.nodes[child_key]["data"]

            # Get mate data from graph edge
            mate_data: MateFeatureData = robot.kinematic_graph.get_edge_data(parent_key, child_key)["data"]
            if mate_data is None:
                LOGGER.warning(f"No mate data found for edge {parent_key} -> {child_key}. Skipping.")
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
            link, world_to_link_tf, asset = get_robot_link(
                name=str(child_key),
                part=child_part,
                client=client,
                mate=mate_data,
            )

            # Add child link if not already in graph
            if child_key not in robot.nodes:
                robot.add_node(child_key, data=link, asset=asset, world_to_link_tf=world_to_link_tf)
            else:
                # NOTE: possible cause for this: the kinematic graph has a loop
                LOGGER.warning(f"Link {child_key} already exists in the robot graph. Skipping.")

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
                        LOGGER.warning(f"Link {_link_key} already exists in the robot graph. Skipping.")

            # Add joints
            for _joint_key, _joint_data in joints_dict.items():
                robot.add_edge(_joint_key[0], _joint_key[1], data=_joint_data)

        return robot

    def set_robot_position(self, pos: tuple[float, float, float]) -> None:
        """
        Set the position for the robot model.

        Args:
            pos: The position to set.
        """
        self.position = pos

    def set_ground_position(self, pos: tuple[float, float, float]) -> None:
        """
        Set the ground position for the robot model.

        Args:
            pos: The position to set.
        """
        self.ground_position = pos

    def set_compiler_attributes(self, attributes: dict[str, str]) -> None:
        """
        Set the compiler attributes for the robot model.

        Args:
            attributes: The compiler attributes to set.
        """
        self.compiler_attributes = attributes

    def set_option_attributes(self, attributes: dict[str, str]) -> None:
        """
        Set the option attributes for the robot model.

        Args:
            attributes: The option attributes to set.
        """
        self.option_attributes = attributes

    def add_light(
        self,
        name: str,
        directional: bool,
        diffuse: tuple[float, float, float],
        specular: tuple[float, float, float],
        pos: tuple[float, float, float],
        direction: tuple[float, float, float],
        castshadow: bool,
    ) -> None:
        """
        Add a light to the robot model.

        Args:
            name: The name of the light.
            directional: Whether the light is directional.
            diffuse: The diffuse color of the light.
            specular: The specular color of the light.
            pos: The position of the light.
            direction: The direction of the light.
            castshadow: Whether the light casts shadows.
        """
        self.lights[name] = Light(
            directional=directional,
            diffuse=diffuse,
            specular=specular,
            pos=pos,
            direction=direction,
            castshadow=castshadow,
        )

    def add_actuator(
        self,
        actuator_name: str,
        joint_name: str,
        ctrl_limited: bool = False,
        add_encoder: bool = True,
        add_force_sensor: bool = True,
        ctrl_range: tuple[float, float] = (0, 0),
        gear: float = 1.0,
    ) -> None:
        """
        Add an actuator to the robot model.

        Args:
            actuator_name: The name of the actuator.
            joint_name: The name of the joint.
            ctrl_limited: Whether the actuator is limited.
            gear: The gear ratio.
            add_encoder: Whether to add an encoder.
            add_force_sensor: Whether to add a force sensor.
            ctrl_range: The control range.
        """
        self.actuators[actuator_name] = Actuator(
            name=actuator_name,
            joint=joint_name,
            ctrllimited=ctrl_limited,
            gear=gear,
            ctrlrange=ctrl_range,
        )

        if add_encoder:
            self.add_sensor(actuator_name + "-enc", Encoder(actuator_name, actuator_name))

        if add_force_sensor:
            self.add_sensor(actuator_name + "-frc", ForceSensor(actuator_name + "-frc", actuator_name))

    def add_sensor(self, name: str, sensor: Sensor) -> None:
        """
        Add a sensor to the robot model.

        Args:
            name: The name of the sensor.
            sensor: The sensor to add.
        """
        self.sensors[name] = sensor

    def add_custom_element_by_tag(
        self,
        name: str,
        parent_tag: str,  # Like 'asset', 'worldbody', etc.
        element: ET._Element,
    ) -> None:
        """
        Add a custom XML element to the first occurrence of a parent tag.

        Args:
            name: Name for referencing this custom element
            parent_tag: Tag name of parent element (e.g. "asset", "worldbody")
            element: The XML element to add

        Examples:
            >>> # Add texture to asset section
            >>> texture = ET.Element("texture", ...)
            >>> robot.add_custom_element_by_tag(
            ...     "wood_texture",
            ...     "asset",
            ...     texture
            ... )
        """
        self.custom_elements[name] = {"parent": parent_tag, "element": element, "find_by_tag": True}

    def add_custom_element_by_name(
        self,
        name: str,
        parent_name: str,  # Like 'Part-3-1', 'ballbot', etc.
        element: ET._Element,
    ) -> None:
        """
        Add a custom XML element to a parent element with specific name.

        Args:
            name: Name for referencing this custom element
            parent_name: Name attribute of the parent element (e.g. "Part-3-1")
            element: The XML element to add

        Examples:
            >>> # Add IMU site to specific body
            >>> imu_site = ET.Element("site", ...)
            >>> robot.add_custom_element_by_name(
            ...     "imu",
            ...     "Part-3-1",
            ...     imu_site
            ... )
        """
        self.custom_elements[name] = {"parent": parent_name, "element": element, "find_by_tag": False}

    def set_element_attributes(
        self,
        element_name: str,
        attributes: dict[str, str],
    ) -> None:
        """
        Set or update attributes of an existing XML element.

        Args:
            element_name: The name of the element to modify
            attributes: Dictionary of attribute key-value pairs to set/update

        Examples:
            >>> # Update existing element attributes
            >>> robot.set_element_attributes(
            ...     ground_element,
            ...     {"size": "3 3 0.001", "friction": "1 0.5 0.5"}
            ... )
        """
        self.mutated_elements[element_name] = attributes

    def add_ground_plane(
        self, root: ET._Element, size: int = 4, orientation: tuple[float, float, float] = (0, 0, 0), name: str = "floor"
    ) -> ET._Element:
        """
        Add a ground plane to the root element with associated texture and material.
        Args:
            root: The root element to append the ground plane to (e.g. "asset", "worldbody")
            size: Size of the ground plane (default: 2)
            orientation: Euler angles for orientation (default: (0, 0, 0))
            name: Name of the ground plane (default: "floor")
        Returns:
            ET.Element: The ground plane element
        """
        # Create ground plane geom element
        ground_geom = ET.Element(
            "geom",
            name=name,
            type="plane",
            pos=" ".join(map(str, self.ground_position)),
            euler=" ".join(map(str, orientation)),
            size=f"{size} {size} 0.001",
            condim="3",
            conaffinity="15",
            material="grid",
        )

        # Add to custom elements
        self.add_custom_element_by_tag(name, "worldbody", ground_geom)

        return ground_geom

    def add_ground_plane_assets(self, root: ET._Element) -> None:
        """Add texture and material assets for the ground plane

        Args:
            root: The root element to append the ground plane to (e.g. "asset", "worldbody")
        """
        # Create texture element
        checker_texture = ET.Element(
            "texture",
            name="checker",
            type="2d",
            builtin="checker",
            rgb1=".1 .2 .3",
            rgb2=".2 .3 .4",
            width="300",
            height="300",
        )
        self.add_custom_element_by_tag("checker", "asset", checker_texture)

        # Create material element
        grid_material = ET.Element("material", name="grid", texture="checker", texrepeat="8 8", reflectance=".2")
        self.add_custom_element_by_tag("grid", "asset", grid_material)

    def to_urdf(self) -> str:
        """
        Generate URDF XML from the graph.

        Returns:
            The URDF XML string.
        """
        robot = ET.Element("robot", name=self.name)

        for node, data in self.nodes(data=True):
            link_data = data.get("data")
            if link_data is not None:
                link_data.to_xml(robot)
            else:
                LOGGER.warning(f"Link {node} has no data.")

        # Add joints
        joint_data_raw: Optional[BaseJoint]
        for parent, child in self.edges:
            joint_data_raw = self.get_edge_data(parent, child).get("data")
            joint_data_typed: Optional[BaseJoint] = joint_data_raw
            if joint_data_typed is not None:
                joint_data_typed.to_xml(robot)
            else:
                LOGGER.warning(f"Joint between {parent} and {child} has no data.")

        return ET.tostring(robot, pretty_print=True, encoding="unicode")

    def get_xml_string(self, element: ET._Element) -> str:
        """
        Get the XML string from an element.

        Args:
            element: The element to get the XML string from.

        Returns:
            The XML string.
        """
        return ET.tostring(element, pretty_print=True, encoding="unicode")

    def to_mjcf(self) -> str:
        """Generate MJCF XML from the graph.

        Returns:
            The MJCF XML string.
        """
        model = ET.Element("mujoco", model=self.name)

        ET.SubElement(
            model,
            "compiler",
            attrib=self.compiler_attributes,
        )

        ET.SubElement(
            model,
            "option",
            attrib=self.option_attributes,
        )

        asset_element = ET.SubElement(model, "asset")
        for _node, data in self.nodes(data=True):
            asset = data.get("asset")
            asset.to_mjcf(asset_element)

        self.add_ground_plane_assets(asset_element)

        worldbody = ET.SubElement(model, "worldbody")
        self.add_ground_plane(worldbody)

        if self.lights:
            for light in self.lights.values():
                light.to_mjcf(worldbody)

        root_body = ET.SubElement(worldbody, "body", name=self.name, pos=" ".join(map(str, self.position)))
        ET.SubElement(root_body, "freejoint", name=f"{self.name}_freejoint")

        body_elements = {self.name: root_body}

        for link_name, link_data in self.kinematic_graph.nodes(data="data"):
            if link_data is not None:
                body_elements[link_name] = link_data.to_mjcf(root_body)
            else:
                LOGGER.warning(f"Link {link_name} has no data.")

        dissolved_transforms: dict[str, tuple[np.ndarray, Rotation]] = {}

        combined_mass = 0.0
        combined_diaginertia = np.zeros(3)
        combined_pos = np.zeros(3)
        combined_euler = np.zeros(3)

        # First, process all fixed joints
        joint_data_raw: Optional[BaseJoint]
        for parent_name, child_name, joint_data_raw in self.kinematic_graph.edges(data="data"):
            joint_data_typed: Optional[BaseJoint] = joint_data_raw
            if joint_data_typed is not None and joint_data_typed.joint_type == "fixed":
                parent_body = body_elements.get(parent_name)
                child_body = body_elements.get(child_name)

                if parent_body is not None and child_body is not None:
                    LOGGER.debug(f"\nProcessing fixed joint from {parent_name} to {child_name}")

                    # Convert joint transform from URDF convention
                    joint_pos = np.array(joint_data_typed.origin.xyz)
                    joint_rot = Rotation.from_euler(URDF_EULER_SEQ, joint_data_typed.origin.rpy)

                    # If parent was dissolved, compose transformations
                    if parent_name in dissolved_transforms:
                        parent_pos, parent_rot = dissolved_transforms[parent_name]
                        # Transform position and rotation
                        joint_pos = parent_rot.apply(joint_pos) + parent_pos
                        joint_rot = parent_rot * joint_rot

                    dissolved_transforms[child_name] = (joint_pos, joint_rot)

                    # Transform geometries
                    for element in list(child_body):
                        if element.tag == "inertial":
                            # Get current inertial properties
                            current_pos = np.array([float(x) for x in (element.get("pos") or "0 0 0").split()])
                            current_euler = np.array([float(x) for x in (element.get("euler") or "0 0 0").split()])
                            current_rot = Rotation.from_euler(MJCF_EULER_SEQ, current_euler, degrees=False)

                            # Get current mass and diaginertia
                            current_mass = float(element.get("mass", 0))
                            current_diaginertia = np.array([
                                float(x) for x in (element.get("diaginertia") or "0 0 0").split()
                            ])

                            # Transform position and orientation
                            new_pos = joint_rot.apply(current_pos) + joint_pos
                            new_rot = joint_rot * current_rot

                            # Convert back to MuJoCo convention
                            from typing import cast

                            from typing_extensions import Literal

                            new_euler = new_rot.as_euler(cast(Literal["XYZ"], MJCF_EULER_SEQ), degrees=False)

                            # Accumulate inertial properties
                            combined_mass += current_mass
                            combined_diaginertia += current_diaginertia
                            combined_pos += new_pos * current_mass
                            combined_euler += new_euler * current_mass

                            continue

                        elif element.tag == "geom":
                            current_pos = np.array([float(x) for x in (element.get("pos") or "0 0 0").split()])
                            current_euler = np.array([float(x) for x in (element.get("euler") or "0 0 0").split()])

                            # Convert current rotation from MuJoCo convention
                            current_rot = Rotation.from_euler(MJCF_EULER_SEQ, current_euler, degrees=False)

                            # Apply the dissolved transformation
                            new_pos = joint_rot.apply(current_pos) + joint_pos
                            new_rot = joint_rot * current_rot  # Order matters for rotation composition

                            # Convert back to MuJoCo convention - explicitly specify intrinsic/extrinsic
                            new_euler = new_rot.as_euler(cast(Literal["XYZ"], MJCF_EULER_SEQ), degrees=False)

                            element.set("pos", " ".join(format_number(float(v)) for v in new_pos))
                            element.set("euler", " ".join(format_number(float(v)) for v in new_euler))

                        parent_body.append(element)

                    root_body.remove(child_body)
                    body_elements[child_name] = parent_body

        # Normalize the combined position and orientation by the total mass
        if combined_mass > 0:
            combined_pos /= combined_mass
            combined_euler /= combined_mass

        # Find the inertial element of the parent body
        parent_inertial = parent_body.find("inertial") if parent_body is not None else None
        if parent_inertial is not None:
            # Update the existing inertial element
            parent_inertial.set("mass", str(combined_mass))
            parent_inertial.set("pos", " ".join(format_number(v) for v in combined_pos))
            parent_inertial.set("euler", " ".join(format_number(v) for v in combined_euler))
            parent_inertial.set("diaginertia", " ".join(format_number(v) for v in combined_diaginertia))
        else:
            # If no inertial element exists, create one
            new_inertial = ET.Element("inertial")
            new_inertial.set("mass", str(combined_mass))
            new_inertial.set("pos", " ".join(format_number(v) for v in combined_pos))
            new_inertial.set("euler", " ".join(format_number(v) for v in combined_euler))
            new_inertial.set("diaginertia", " ".join(format_number(v) for v in combined_diaginertia))
            if parent_body is not None:
                parent_body.append(new_inertial)

        # Then process revolute joints
        joint_data_raw2: Optional[BaseJoint]
        for parent_name, child_name, joint_data_raw2 in self.kinematic_graph.edges(data="data"):
            joint_data_typed2: Optional[BaseJoint] = joint_data_raw2
            if joint_data_typed2 is not None and joint_data_typed2.joint_type != "fixed":
                parent_body = body_elements.get(parent_name)
                child_body = body_elements.get(child_name)

                if parent_body is not None and child_body is not None:
                    LOGGER.debug(f"\nProcessing revolute joint from {parent_name} to {child_name}")

                    # Get dissolved parent transform
                    if parent_name in dissolved_transforms:
                        parent_pos, parent_rot = dissolved_transforms[parent_name]
                    else:
                        parent_pos = np.array([0, 0, 0])
                        parent_rot = Rotation.from_euler(URDF_EULER_SEQ, [0, 0, 0])

                    # Convert joint transform from URDF convention
                    joint_pos = np.array(joint_data_typed2.origin.xyz)
                    joint_rot = Rotation.from_euler(URDF_EULER_SEQ, joint_data_typed2.origin.rpy)

                    # Apply parent's dissolved transformation
                    final_pos = parent_rot.apply(joint_pos) + parent_pos
                    final_rot = parent_rot * joint_rot

                    # Convert to MuJoCo convention while maintaining the joint axis orientation
                    final_euler = final_rot.as_euler(cast(Literal["XYZ"], MJCF_EULER_SEQ), degrees=False)

                    LOGGER.debug(f"Joint {parent_name}->{child_name}:")
                    LOGGER.debug(f"  Original: pos={joint_data_typed2.origin.xyz}, rpy={joint_data_typed2.origin.rpy}")
                    LOGGER.debug(f"  Final: pos={final_pos}, euler={final_euler}")

                    # Update child body transformation
                    child_body.set("pos", " ".join(format_number(float(v)) for v in final_pos))
                    child_body.set("euler", " ".join(format_number(float(v)) for v in final_euler))

                    # Create joint with zero origin
                    joint_data_typed2.origin.xyz = (0.0, 0.0, 0.0)
                    joint_data_typed2.origin.rpy = (0.0, 0.0, 0.0)
                    joint_data_typed2.to_mjcf(child_body)

                    # Move child under parent
                    parent_body.append(child_body)

        if self.actuators:
            actuator_element = ET.SubElement(model, "actuator")
            for actuator in self.actuators.values():
                actuator.to_mjcf(actuator_element)

        if self.sensors:
            sensor_element = ET.SubElement(model, "sensor")
            for sensor in self.sensors.values():
                sensor.to_mjcf(sensor_element)

        if self.custom_elements:
            for element_info in self.custom_elements.values():
                parent = element_info["parent"]
                find_by_tag = element_info.get("find_by_tag", False)
                element = element_info["element"]

                if find_by_tag:
                    parent_element = model if parent == "mujoco" else model.find(parent)
                else:
                    xpath = f".//body[@name='{parent}']"
                    parent_element = model.find(xpath)

                if parent_element is not None:
                    # Create new element with proper parent relationship
                    new_element: ET._Element = ET.SubElement(parent_element, element.tag, element.attrib)
                    # Copy any children if they exist
                    for child in element:
                        child_element = ET.fromstring(ET.tostring(child))  # noqa: S320
                        if isinstance(child_element, ET._Element):
                            new_element.append(child_element)
                else:
                    search_type = "tag" if find_by_tag else "name"
                    LOGGER.warning(f"Parent element with {search_type} '{parent}' not found in model.")

        for element_name, attributes in self.mutated_elements.items():
            # Search recursively through all descendants, looking for both body and joint elements
            elements = model.findall(f".//*[@name='{element_name}']")
            if elements:
                element_to_modify: ET._Element = elements[0]  # Get the first matching element
                for key, value in attributes.items():
                    element_to_modify.set(key, str(value))
            else:
                LOGGER.warning(f"Could not find element with name '{element_name}'")

        return ET.tostring(model, pretty_print=True, encoding="unicode")

    def save(
        self, file_path: Optional[str] = None, download_assets: bool = True, mesh_dir: Optional[str] = None
    ) -> None:
        """Save the robot model to a URDF file.

        Args:
            file_path: The path to the file to save the robot model.
            download_assets: Whether to download the assets.
            mesh_dir: Optional custom directory for mesh files. If not specified and file_path is provided,
                defaults to a 'meshes' subdirectory next to the file_path. If neither is provided,
                uses the current working directory.
        """
        # Determine the mesh directory with smart defaults
        resolved_mesh_dir: Optional[str] = None
        if mesh_dir is not None:
            # User explicitly provided mesh_dir
            resolved_mesh_dir = mesh_dir
        elif file_path is not None:
            # Smart default: use file_path.parent / "meshes"
            from pathlib import Path

            file_parent = Path(file_path).parent
            resolved_mesh_dir = os.path.join(str(file_parent), "meshes")

        if download_assets:
            asyncio.run(self._download_assets(resolved_mesh_dir))

        if not file_path:
            LOGGER.warning("No file path provided. Saving to current directory.")
            LOGGER.warning("Please keep in mind that the path to the assets will not be updated")
            file_path = f"{self.name}.{self.type}"
        else:
            # Validate and fix file extension based on robot type
            from pathlib import Path

            path_obj = Path(file_path)
            current_ext = path_obj.suffix.lower()
            expected_ext = f".{self.type}"

            # If no extension, add the correct one
            if not current_ext:
                file_path = str(path_obj) + expected_ext
                LOGGER.info(f"No file extension provided. Using {expected_ext}")
            # If extension doesn't match robot type, fix it
            elif current_ext != expected_ext:
                file_path = str(path_obj.with_suffix(expected_ext))
                LOGGER.warning(
                    f"File extension {current_ext} doesn't match robot type {self.type}. Changed to {expected_ext}"
                )
            # If extension matches but has different case, normalize to lowercase
            elif path_obj.suffix != expected_ext:
                file_path = str(path_obj.with_suffix(expected_ext))
                LOGGER.info(f"Normalized extension from {path_obj.suffix} to {expected_ext}")

        # Set robot_file_dir on all assets so relative paths in XML are correct
        from pathlib import Path

        robot_file_dir = str(Path(file_path).parent.absolute())
        for _node, data in self.nodes(data=True):
            asset = data.get("asset")
            link_data = data.get("data")
            if asset:
                asset.robot_file_dir = robot_file_dir
                # Update the mesh paths in the link's geometry objects
                if (
                    link_data
                    and hasattr(link_data, "visual")
                    and link_data.visual
                    and hasattr(link_data.visual.geometry, "filename")
                ):
                    link_data.visual.geometry.filename = asset.relative_path
                if (
                    link_data
                    and hasattr(link_data, "collision")
                    and link_data.collision
                    and hasattr(link_data.collision.geometry, "filename")
                ):
                    link_data.collision.geometry.filename = asset.relative_path

        xml_declaration = '<?xml version="1.0" ?>\n'

        if self.type == RobotType.URDF:
            urdf_str = xml_declaration + self.to_urdf()
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(urdf_str)

        elif self.type == RobotType.MJCF:
            mjcf_str = xml_declaration + self.to_mjcf()
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(mjcf_str)

        LOGGER.info(f"Robot model saved to {os.path.abspath(file_path)}")

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
            LOGGER.info("All assets downloaded successfully.")
        except Exception as e:
            LOGGER.error(f"Error downloading assets: {e}")

    def add_custom_element(self, parent_name: str, element: ET._Element) -> None:
        """Add a custom XML element to the robot model.

        Args:
            parent_name: The name of the parent element.
            element: The custom XML element to add.
        """
        if self.model is None:
            raise RuntimeError("Robot model not initialized")

        if parent_name == "root":
            self.model.insert(0, element)
        else:
            parent = self.model.find(f".//*[@name='{parent_name}']")
            if parent is None:
                raise ValueError(f"Parent with name '{parent_name}' not found in the robot model.")

            # Add the custom element under the parent
            parent.append(element)

        LOGGER.info(f"Custom element added to parent '{parent_name}'.")


def load_element(file_name: str) -> ET._Element:
    """
    Load an XML element from a file.

    Args:
        file_name: The path to the file.

    Returns:
        The root element of the XML file.
    """
    tree = ET.parse(file_name)  # noqa: S320
    root = tree.getroot()
    return root
