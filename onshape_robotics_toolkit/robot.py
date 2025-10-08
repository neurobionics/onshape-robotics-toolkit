"""
This module contains classes for creating a URDF robot model

Dataclass:
    - **Robot**: Represents a robot model in URDF format, containing links and joints.

"""

import asyncio
import os
from enum import Enum
from typing import TYPE_CHECKING, Any, Optional, Union

import numpy as np
from lxml import etree as ET
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from onshape_robotics_toolkit.graph import KinematicGraph
    from onshape_robotics_toolkit.parse import CAD, PathKey


import random

from onshape_robotics_toolkit.connect import Asset, Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import (
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
    CAD,
    CHILD,
    PARENT,
)
from onshape_robotics_toolkit.utilities.helpers import format_number, get_sanitized_name

DEFAULT_COMPILER_ATTRIBUTES = {
    "angle": "radian",
    "eulerseq": "xyz",
    # "meshdir": "meshes",
}

DEFAULT_OPTION_ATTRIBUTES = {"timestep": "0.001", "gravity": "0 0 -9.81", "iterations": "50"}

URDF_EULER_SEQ = "xyz"  # URDF uses XYZ fixed angles
MJCF_EULER_SEQ = "XYZ"  # MuJoCo uses XYZ extrinsic rotations, capitalization matters

ACTUATOR_SUFFIX = "-actuator"


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
    wid: str,
    client: Client,
    mate: Optional[Union[MateFeatureData, None]] = None,
) -> tuple[Link, np.matrix, Asset]:
    """
    Generate a URDF link from an Onshape part.

    Args:
        name: The name of the link.
        part: The Onshape part object.
        wid: The unique identifier of the workspace.
        client: The Onshape client object to use for sending API requests.
        mate: MateFeatureData object to use for generating the transformation matrix.

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
    _link_to_stl_tf = np.eye(4)

    if mate is None:
        try:
            if part.MassProperty is not None:
                pass
            else:
                LOGGER.warning(f"Part {part.partId} has no mass properties, using identity transform")
        except AttributeError:
            LOGGER.warning(f"Part {part.partId} has no mass properties, using identity transform")

    else:
        child_parent_cs = mate.matedEntities[CHILD].parentCS
        child_mated_cs = mate.matedEntities[CHILD].matedCS
        if child_parent_cs and child_mated_cs:
            LOGGER.debug(f"DEBUG: Link {name} has parentCS, applying hierarchical transform")
            _link_to_stl_tf = child_parent_cs.part_tf @ child_mated_cs.part_to_mate_tf
        else:
            LOGGER.debug(f"DEBUG: Link {name} using direct part_to_mate_tf")
            if mate.matedEntities[CHILD].matedCS is not None:
                _link_to_stl_tf = mate.matedEntities[CHILD].matedCS.part_to_mate_tf
            else:
                _link_to_stl_tf = np.eye(4)  # Default to identity matrix

    _stl_to_link_tf = np.matrix(np.linalg.inv(_link_to_stl_tf))
    _origin = Origin.zero_origin()
    _principal_axes_rotation = (0.0, 0.0, 0.0)

    # Check if part has mass properties
    if part.MassProperty is None:
        LOGGER.warning(f"Part {part.partId} has no mass properties, using default values")
        _mass = 1.0  # Default mass
        _com = (0.0, 0.0, 0.0)  # Default center of mass at origin
        _inertia = np.eye(3)  # Default identity inertia matrix
    else:
        _mass = part.MassProperty.mass[0]
        _com = tuple(part.MassProperty.center_of_mass_wrt(_stl_to_link_tf))
        _inertia = part.MassProperty.inertia_wrt(np.matrix(_stl_to_link_tf[:3, :3]))

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
        mvwid = part.rigidAssemblyWorkspaceId if part.rigidAssemblyWorkspaceId else wid
    else:
        # Regular part - use its documentMicroversion with microversion type
        wtype = WorkspaceType.M.value
        mvwid = part.documentMicroversion if part.documentMicroversion else wid

    _asset = Asset(
        did=part.documentId,
        wtype=wtype,
        wid=mvwid,
        eid=part.elementId,
        partID=part.partId,
        client=client,
        transform=_stl_to_link_tf,
        is_rigid_assembly=part.isRigidAssembly,
        file_name=f"{name}.stl",
    )

    _mesh_path = _asset.relative_path

    _link = Link(
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

    return _link, _stl_to_link_tf, _asset


def get_robot_joint(
    parent: str,
    child: str,
    mate: MateFeatureData,
    stl_to_parent_tf: np.matrix,
    mimic: Optional[JointMimic] = None,
    is_rigid_assembly: bool = False,
) -> tuple[list[BaseJoint], Optional[list[Link]]]:
    """
    Generate a URDF joint from an Onshape mate feature.

    Args:
        parent: The name of the parent link.
        child: The name of the child link.
        mate: The Onshape mate feature object.
        stl_to_parent_tf: The transformation matrix from the STL origin to the parent link origin.
        mimic: The mimic joint object.
        is_rigid_assembly: Whether the assembly is a rigid assembly.

    Returns:
        tuple[list[BaseJoint], Optional[list[Link]]]: The generated joint object and the links.

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
    links: list[Link] = []
    if isinstance(mate, MateFeatureData):
        # Check if we have a parentCS (hierarchical transform) regardless of rigid assembly status
        # This handles the case where flexible assemblies contain parts from collapsed rigid subassemblies
        parent_cs = mate.matedEntities[PARENT].parentCS
        mated_cs = mate.matedEntities[PARENT].matedCS
        if parent_cs is not None and mated_cs is not None:
            LOGGER.debug(f"DEBUG: Using parentCS for joint {mate.name} from {parent} to {child}")
            LOGGER.debug(f"DEBUG: parentCS.part_tf = {parent_cs.part_tf}")
            LOGGER.debug(f"DEBUG: parent matedCS.part_to_mate_tf = {mated_cs.part_to_mate_tf}")
            parent_to_mate_tf = parent_cs.part_tf @ mated_cs.part_to_mate_tf
            LOGGER.debug(f"DEBUG: combined parent_to_mate_tf = {parent_to_mate_tf}")
        else:
            LOGGER.debug(f"DEBUG: No parentCS for joint {mate.name} from {parent} to {child}, using direct transform")
            # No hierarchical transform needed, use direct part-to-mate transformation
            if mate.matedEntities[PARENT].matedCS is not None:
                parent_to_mate_tf = mate.matedEntities[PARENT].matedCS.part_to_mate_tf
            else:
                parent_to_mate_tf = np.eye(4)  # Default to identity matrix
            LOGGER.debug(f"DEBUG: direct parent_to_mate_tf = {parent_to_mate_tf}")

    LOGGER.debug(f"DEBUG: stl_to_parent_tf = {stl_to_parent_tf}")
    stl_to_mate_tf = stl_to_parent_tf @ parent_to_mate_tf
    LOGGER.debug(f"DEBUG: final stl_to_mate_tf = {stl_to_mate_tf}")
    origin = Origin.from_matrix(stl_to_mate_tf)
    LOGGER.debug(f"DEBUG: final origin = {origin.xyz} rpy={origin.rpy}")
    sanitized_name = get_sanitized_name(mate.name)

    LOGGER.info(f"Creating robot joint from {parent} to {child}")

    if mate.mateType == MateType.REVOLUTE:
        return [
            RevoluteJoint(
                name=sanitized_name,
                parent=parent,
                child=child,
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
        ], links

    elif mate.mateType == MateType.FASTENED:
        return [FixedJoint(name=sanitized_name, parent=parent, child=child, origin=origin)], links

    elif mate.mateType == MateType.SLIDER or mate.mateType == MateType.CYLINDRICAL:
        return [
            PrismaticJoint(
                name=sanitized_name,
                parent=parent,
                child=child,
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
        ], links

    elif mate.mateType == MateType.BALL:
        dummy_x = Link(
            name=f"{parent}_{get_sanitized_name(mate.name)}_x",
            inertial=InertialLink(
                mass=0.0,
                inertia=Inertia.zero_inertia(),
                origin=Origin.zero_origin(),
            ),
        )
        dummy_y = Link(
            name=f"{parent}_{get_sanitized_name(mate.name)}_y",
            inertial=InertialLink(
                mass=0.0,
                inertia=Inertia.zero_inertia(),
                origin=Origin.zero_origin(),
            ),
        )

        links = [dummy_x, dummy_y]

        return [
            RevoluteJoint(
                name=sanitized_name + "_x",
                parent=parent,
                child=dummy_x.name,
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
            ),
            RevoluteJoint(
                name=sanitized_name + "_y",
                parent=dummy_x.name,
                child=dummy_y.name,
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
            ),
            RevoluteJoint(
                name=sanitized_name + "_z",
                parent=dummy_y.name,
                child=child,
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
            ),
        ], links

    else:
        LOGGER.warning(f"Unsupported joint type: {mate.mateType}")
        return [DummyJoint(name=sanitized_name, parent=parent, child=child, origin=origin)], links


class Robot:
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

    def __init__(self, kinematic_graph: KinematicGraph, cad: CAD, name: str, robot_type: RobotType = RobotType.URDF):
        self.graph: KinematicGraph = kinematic_graph
        self.cad: CAD = cad
        self.name: str = name
        self.type: RobotType = robot_type

        self.assets: dict[PathKey, Asset] = {}
        self._stl_to_link_tf: dict[PathKey, np.ndarray] = {}

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

    @classmethod
    def from_graph(
        cls,
        kinematic_graph: "KinematicGraph",
        cad: "CAD",
        client: Client,
        name: str,
        robot_type: RobotType = RobotType.URDF,
    ) -> "Robot":
        """
        Create a Robot from pre-built CAD and KinematicGraph objects.

        This is the recommended method for creating robots when you already have
        CAD and KinematicGraph instances. It handles mass property fetching
        and robot generation in an efficient, streamlined way.

        Args:
            cad: CAD document with PathKey-based registries
            kinematic_graph: Kinematic graph with parts and mates
            client: Onshape client for downloading assets and fetching mass properties
            name: The name of the robot
            robot_type: The type of the robot (URDF or MJCF)
            include_rigid_subassembly_parts: Whether to include parts from rigid subassemblies

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
        # Generate robot structure from kinematic graph
        robot = cls(
            kinematic_graph=kinematic_graph,
            cad=cad,
            name=name,
            robot_type=robot_type,
        )

        # Get root node from kinematic graph
        if kinematic_graph.root is None:
            raise ValueError("Kinematic graph has no root node")

        root_key = kinematic_graph.root
        LOGGER.info(f"Processing root node: {root_key}")

        root_part = cad.parts[root_key]
        # NOTE: make sure Pathkey.__str__ produces names without
        # special characters that are invalid in URDF/MJCF
        root_name = str(root_key)

        # Create root link
        root_link, stl_to_root_tf, root_asset = get_robot_link(
            name=root_name,
            part=root_part,
            wid=cad.workspace_id,
            client=client,
            mate=None,
        )
        robot.add_link(root_link)
        robot.assets[root_key] = root_asset
        robot._stl_to_link_tf[root_key] = stl_to_root_tf

        LOGGER.info(f"Processing {len(kinematic_graph.edges)} edges in the kinematic graph.")

        # Process edges in topological order
        for parent_key, child_key in kinematic_graph.edges:
            LOGGER.info(f"Processing edge: {parent_key} -> {child_key}")

            # Get parent transform
            parent_tf = robot._stl_to_link_tf[parent_key]

            # Get parts
            if parent_key not in cad.parts or child_key not in cad.parts:
                LOGGER.warning(f"Part {parent_key} or {child_key} not found in parts registry. Skipping.")
                continue

            parent_part = cad.parts[parent_key]
            child_part = cad.parts[child_key]

            # Get mate data from graph edge
            mate_data = kinematic_graph.get_edge_data(parent_key, child_key)
            if mate_data is None:
                LOGGER.warning(f"No mate data found for edge {parent_key} -> {child_key}. Skipping.")
                continue

            parent_name = str(parent_key)
            child_name = str(child_key)

            # Check for mate relations (mimic joints)
            joint_mimic = None
            # TODO: Implement mate relation support with PathKey system
            # This will require updating the relation processing to use PathKeys

            # Create joint(s) and dummy links
            joint_list, link_list = get_robot_joint(
                parent=parent_name,
                child=child_name,
                mate=mate_data,
                stl_to_parent_tf=np.matrix(parent_tf),
                mimic=joint_mimic,
                is_rigid_assembly=parent_part.isRigidAssembly,
            )

            # Create child link
            link, stl_to_link_tf, asset = get_robot_link(
                name=child_name,
                part=child_part,
                wid=cad.workspace_id,
                client=client,
                mate=mate_data,
            )
            robot._stl_to_link_tf[child_key] = stl_to_link_tf
            robot.assets[child_name] = asset

            # Add child link if not already in graph
            if child_key not in robot.graph:
                robot.add_link(link)
            else:
                LOGGER.warning(f"Link {child_name} already exists in the robot graph. Skipping.")

            # Add dummy links (for ball joints, etc.)
            for link_item in link_list or []:
                if link_item.name not in robot.graph:
                    robot.add_link(link_item)
                else:
                    LOGGER.warning(f"Link {link_item.name} already exists in the robot graph. Skipping.")

            # Add joints
            for joint in joint_list:
                robot.add_joint(joint)

        return robot

    def add_link(self, link: Link) -> None:
        """
        Add a link to the graph.

        Args:
            link: The link to add.
        """
        self.graph.add_node(link.name, data=link)

    def add_joint(self, joint: BaseJoint) -> None:
        """
        Add a joint to the graph.

        Args:
            joint: The joint to add.
        """
        self.graph.add_edge(joint.parent, joint.child, data=joint)

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

        # Add links
        for link_name, link_data in self.graph.nodes(data="data"):
            if link_data is not None:
                link_data.to_xml(robot)
            else:
                LOGGER.warning(f"Link {link_name} has no data.")

        # Add joints
        joint_data_raw: Optional[BaseJoint]
        for parent, child, joint_data_raw in self.graph.edges(data="data"):
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

        if self.assets:
            asset_element = ET.SubElement(model, "asset")
            for asset in self.assets.values():
                asset.to_mjcf(asset_element)

            self.add_ground_plane_assets(asset_element)
        else:
            # Find or create asset element after default element
            asset_element = ET.SubElement(model, "asset")
            self.add_ground_plane_assets(asset_element)

        worldbody = ET.SubElement(model, "worldbody")
        self.add_ground_plane(worldbody)

        if self.lights:
            for light in self.lights.values():
                light.to_mjcf(worldbody)

        root_body = ET.SubElement(worldbody, "body", name=self.name, pos=" ".join(map(str, self.position)))
        ET.SubElement(root_body, "freejoint", name=f"{self.name}_freejoint")

        body_elements = {self.name: root_body}

        for link_name, link_data in self.graph.nodes(data="data"):
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
        for parent_name, child_name, joint_data_raw in self.graph.edges(data="data"):
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
        for parent_name, child_name, joint_data_raw2 in self.graph.edges(data="data"):
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

    @classmethod
    def from_urdf(cls, file_name: str, robot_type: RobotType) -> "Robot":
        """Load a robot model from a URDF file.

        Args:
            file_name: The path to the URDF file.
            robot_type: The type of the robot.

        Returns:
            The robot model.
        """
        tree = ET.parse(file_name)  # noqa: S320
        root = tree.getroot()

        name = root.get("name")
        if name is None:
            raise ValueError("Root element missing name attribute")
        robot = cls(name=name, robot_type=robot_type)

        for element in root:
            if element.tag == "link":
                link = Link.from_xml(element)
                robot.add_link(link)

                # Process the visual element within the link
                visual = element.find("visual")
                if visual is not None:
                    geometry = visual.find("geometry")
                    if geometry is not None:
                        mesh = geometry.find("mesh")
                        if mesh is not None:
                            visual_file_name: str | None = mesh.get("filename")
                            if visual_file_name is not None and visual_file_name not in robot.assets:
                                robot.assets[visual_file_name] = Asset.from_file(visual_file_name)

                # Process the collision element within the link
                collision = element.find("collision")
                if collision is not None:
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        mesh = geometry.find("mesh")
                        if mesh is not None:
                            collision_file_name: str | None = mesh.get("filename")
                            if collision_file_name is not None and collision_file_name not in robot.assets:
                                robot.assets[collision_file_name] = Asset.from_file(collision_file_name)

            elif element.tag == "joint":
                joint = set_joint_from_xml(element)
                if joint:
                    robot.add_joint(joint)

        return robot

    def save(self, file_path: Optional[str] = None, download_assets: bool = True) -> None:
        """Save the robot model to a URDF file.

        Args:
            file_path: The path to the file to save the robot model.
            download_assets: Whether to download the assets.
        """
        if download_assets and self.assets:
            asyncio.run(self._download_assets())

        if not file_path:
            LOGGER.warning("No file path provided. Saving to current directory.")
            LOGGER.warning("Please keep in mind that the path to the assets will not be updated")
            file_path = f"{self.name}.{self.type}"

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
            for child in self.graph.successors(node):
                print_tree(child, depth + 1)

        root_nodes = [n for n in self.graph.nodes if self.graph.in_degree(n) == 0]
        for root in root_nodes:
            print_tree(root)

    async def _download_assets(self) -> None:
        """Asynchronously download the assets."""
        if not self.assets:
            LOGGER.warning("No assets found for the robot model.")
            return

        tasks = [asset.download() for asset in self.assets.values() if not asset.is_from_file]
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


def get_robot_from_kinematic_graph(
    cad: "CAD",
    kinematic_graph: "KinematicGraph",
    client: Client,
    robot_name: str,
) -> Robot:
    """
    Generate a Robot instance from a CAD document and kinematic graph.

    This is the PathKey-based implementation that uses the CAD class
    and KinematicGraph to efficiently build the robot structure following
    the CAD → KinematicGraph → Robot pipeline.

    The function:
    1. Creates the root link from the kinematic graph root node
    2. Processes edges in topological order to create joints and child links
    3. Downloads STL assets for each part
    4. Handles mate relations for mimic joints (TODO)

    Args:
        cad: CAD document with PathKey-based registries
        kinematic_graph: Kinematic graph with parts and mates
        client: Onshape client for downloading assets
        robot_name: Name of the robot

    Returns:
        Robot: The generated Robot instance with links, joints, and assets

    Raises:
        ValueError: If kinematic graph has no root node or parts are missing
    """
    robot = Robot(name=robot_name)
    assets_map: dict[str, Asset] = {}
    stl_to_link_tf_map: dict[PathKey, np.ndarray] = {}

    # Get root node from kinematic graph
    if kinematic_graph.root_node is None:
        raise ValueError("Kinematic graph has no root node")

    root_key = kinematic_graph.root_node
    LOGGER.info(f"Processing root node: {root_key}")

    root_part = cad.parts[root_key]
    # NOTE: make sure Pathkey.__str__ produces names without
    # special characters that are invalid in URDF/MJCF
    root_name = str(root_key)

    # Create root link
    root_link, stl_to_root_tf, root_asset = get_robot_link(
        name=root_name,
        part=root_part,
        wid=cad.workspace_id,
        client=client,
        mate=None,
    )
    robot.add_link(root_link)
    assets_map[root_name] = root_asset
    stl_to_link_tf_map[root_key] = stl_to_root_tf

    LOGGER.info(f"Processing {len(kinematic_graph.graph.edges)} edges in the kinematic graph.")

    # Process edges in topological order
    for parent_key, child_key in kinematic_graph.graph.edges:
        LOGGER.info(f"Processing edge: {parent_key} -> {child_key}")

        # Get parent transform
        parent_tf = stl_to_link_tf_map[parent_key]

        # Get parts
        if parent_key not in cad.parts or child_key not in cad.parts:
            LOGGER.warning(f"Part {parent_key} or {child_key} not found in parts registry. Skipping.")
            continue

        parent_part = cad.parts[parent_key]
        child_part = cad.parts[child_key]

        # Get mate data from graph edge
        mate_data = kinematic_graph.get_mate_data(parent_key, child_key)
        if mate_data is None:
            LOGGER.warning(f"No mate data found for edge {parent_key} -> {child_key}. Skipping.")
            continue

        # Get sanitized names
        parent_instance = cad.instances.get(parent_key)
        child_instance = cad.instances.get(child_key)
        if parent_instance is None or child_instance is None:
            LOGGER.warning(f"Instance not found for {parent_key} or {child_key}. Skipping.")
            continue

        parent_name = "_".join(str(parent_key).split(" > "))
        child_name = "_".join(str(child_key).split(" > "))

        # Check for mate relations (mimic joints)
        joint_mimic = None
        # TODO: Implement mate relation support with PathKey system
        # This will require updating the relation processing to use PathKeys

        # Create joint(s) and dummy links
        joint_list, link_list = get_robot_joint(
            parent=parent_name,
            child=child_name,
            mate=mate_data,
            stl_to_parent_tf=np.matrix(parent_tf),
            mimic=joint_mimic,
            is_rigid_assembly=parent_part.isRigidAssembly,
        )

        # Create child link
        link, stl_to_link_tf, asset = get_robot_link(
            name=child_name,
            part=child_part,
            wid=cad.workspace_id,
            client=client,
            mate=mate_data,
        )
        stl_to_link_tf_map[child_key] = stl_to_link_tf
        assets_map[child_name] = asset

        # Add child link if not already in graph
        if child_name not in robot.graph:
            robot.add_link(link)
        else:
            LOGGER.warning(f"Link {child_name} already exists in the robot graph. Skipping.")

        # Add dummy links (for ball joints, etc.)
        for link_item in link_list or []:
            if link_item.name not in robot.graph:
                robot.add_link(link_item)
            else:
                LOGGER.warning(f"Link {link_item.name} already exists in the robot graph. Skipping.")

        # Add joints
        for joint in joint_list:
            robot.add_joint(joint)

    robot.assets = assets_map
    return robot
