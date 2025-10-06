"""
This module contains classes for creating a URDF robot model

Dataclass:
    - **Robot**: Represents a robot model in URDF format, containing links and joints.

"""

import asyncio
import os
from enum import Enum
from typing import TYPE_CHECKING, Any, Optional

import networkx as nx
import numpy as np
from lxml import etree as ET
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from onshape_robotics_toolkit.graph import KinematicGraph
    from onshape_robotics_toolkit.parse import CAD

from onshape_robotics_toolkit.connect import Asset, Client
from onshape_robotics_toolkit.graph import plot_graph
from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import (
    Assembly,
    MateFeatureData,
    MateRelationFeatureData,
    Part,
    RelationType,
    RootAssembly,
    SubAssembly,
)
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.models.joint import (
    BaseJoint,
    ContinuousJoint,
    FixedJoint,
    FloatingJoint,
    JointMimic,
    JointType,
    PrismaticJoint,
    RevoluteJoint,
)
from onshape_robotics_toolkit.models.link import Link
from onshape_robotics_toolkit.models.mjcf import Actuator, Encoder, ForceSensor, Light, Sensor
from onshape_robotics_toolkit.parse import (
    MATE_JOINER,
    RELATION_PARENT,
)
from onshape_robotics_toolkit.urdf import get_joint_name, get_robot_joint, get_robot_link, get_topological_mates
from onshape_robotics_toolkit.utilities.helpers import format_number

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

    def __init__(self, name: str, assets: Optional[dict[str, Asset]] = None, robot_type: RobotType = RobotType.URDF):
        self.name: str = name
        self.graph: nx.DiGraph = nx.DiGraph()  # Graph to hold links and joints

        self.assets: dict[str, Asset] = assets or {}

        self.type: RobotType = robot_type

        # DEPRECATED: Legacy Onshape assembly attributes
        # These are kept for backward compatibility but are no longer used
        # in the new CAD → KinematicGraph → Robot pipeline.
        # Use Robot.from_graph() or Robot.from_url() instead of directly accessing these.
        self.parts: dict[str, Part] = {}  # DEPRECATED: Use CAD.parts with PathKeys
        self.mates: dict[str, MateFeatureData] = {}  # DEPRECATED: Use CAD.mates with PathKeys
        self.relations: dict[str, MateRelationFeatureData] = {}  # DEPRECATED: Use CAD relations

        self.flexible_assembly_data: dict[str, SubAssembly] = {}  # DEPRECATED
        self.rigid_assembly_data: dict[str, RootAssembly] = {}  # DEPRECATED

        self.assembly: Optional[Assembly] = None  # DEPRECATED: Stored for backward compatibility only
        self.model: Optional[ET._Element] = None  # DEPRECATED

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

    def show_graph(self, file_name: Optional[str] = None) -> None:
        """
        Display the robot's graph as a directed graph.

        Args:
            file_name: The path to the file to save the graph.
        """
        plot_graph(self.graph, file_name=file_name)

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

    @classmethod
    def from_graph(
        cls,
        cad: "CAD",
        kinematic_graph: "KinematicGraph",
        client: Client,
        name: str,
        robot_type: RobotType = RobotType.URDF,
        include_rigid_subassembly_parts: bool = False,
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
        from onshape_robotics_toolkit.parse import fetch_mass_properties_for_kinematic_parts

        # Fetch mass properties for parts in kinematic chain
        fetch_mass_properties_for_kinematic_parts(
            cad=cad,
            kinematic_graph=kinematic_graph,
            client=client,
            include_rigid_subassembly_parts=include_rigid_subassembly_parts,
        )

        # Generate robot structure from kinematic graph
        robot = get_robot_from_kinematic_graph(
            cad=cad,
            kinematic_graph=kinematic_graph,
            client=client,
            robot_name=name,
        )

        robot.type = robot_type

        return robot

    @classmethod
    def from_url(
        cls,
        name: str,
        url: str,
        client: Client,
        max_depth: int = 0,
        use_user_defined_root: bool = False,
        robot_type: RobotType = RobotType.URDF,
        log_assembly: bool = False,
        include_rigid_subassembly_parts: bool = False,
    ) -> "Robot":
        """
        Load a robot model from an Onshape CAD assembly URL.

        This method fetches the assembly from Onshape, builds the CAD structure
        and kinematic graph, then delegates to Robot.from_graph() for robot creation.

        Args:
            name: The name of the robot
            url: The URL of the Onshape CAD assembly
            client: The Onshape client object
            max_depth: The maximum depth to process the assembly (controls rigid vs flexible)
            use_user_defined_root: Whether to use the user-defined root
            robot_type: The type of the robot (URDF or MJCF)
            log_assembly: Whether to log the assembly response
            include_rigid_subassembly_parts: Whether to include parts from rigid subassemblies

        Returns:
            Robot: The generated robot model

        Example:
            >>> from onshape_robotics_toolkit.connect import Client
            >>> client = Client()
            >>> robot = Robot.from_url(
            ...     name="my_robot",
            ...     url="https://cad.onshape.com/documents/...",
            ...     client=client,
            ...     max_depth=1
            ... )
            >>> robot.save("robot.urdf", download_assets=True)
        """
        from onshape_robotics_toolkit.graph import KinematicGraph
        from onshape_robotics_toolkit.parse import CAD

        # Fetch assembly from Onshape
        document = Document.from_url(url=url)
        client.set_base_url(document.base_url)

        assembly = client.get_assembly(
            did=document.did,
            wtype=document.wtype,
            wid=document.wid,
            eid=document.eid,
            log_response=log_assembly,
            with_meta_data=True,
        )

        # Create CAD structure with PathKey-based system
        cad = CAD.from_assembly(assembly=assembly, max_depth=max_depth)

        # Build kinematic graph from CAD structure
        kinematic_graph = KinematicGraph.from_cad(cad=cad, use_user_defined_root=use_user_defined_root)

        # Delegate to from_graph for robot creation
        robot = cls.from_graph(
            cad=cad,
            kinematic_graph=kinematic_graph,
            client=client,
            name=name,
            robot_type=robot_type,
            include_rigid_subassembly_parts=include_rigid_subassembly_parts,
        )

        # Store assembly reference for backward compatibility
        robot.assembly = assembly

        return robot


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
    from onshape_robotics_toolkit.parse import PathKey

    robot = Robot(name=robot_name)
    assets_map: dict[str, Asset] = {}
    stl_to_link_tf_map: dict[PathKey, np.ndarray] = {}

    # Get root node from kinematic graph
    if kinematic_graph.root_node is None:
        raise ValueError("Kinematic graph has no root node")

    root_key = kinematic_graph.root_node
    LOGGER.info(f"Processing root node: {root_key}")

    # Get root part
    if root_key not in cad.parts:
        raise ValueError(f"Root node {root_key} not found in parts registry")

    root_part = cad.parts[root_key]

    # Get sanitized name for root link
    root_instance = cad.instances.get(root_key)
    if root_instance is None:
        raise ValueError(f"Root instance {root_key} not found")
    root_name = root_key.hierarchical_name(separator="-")

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

        parent_name = parent_key.hierarchical_name(separator="-")
        child_name = child_key.hierarchical_name(separator="-")

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


def get_robot(
    assembly: Assembly,
    graph: nx.DiGraph,
    root_node: Optional[str],
    parts: dict[str, Part],
    mates: dict[str, MateFeatureData],
    relations: dict[str, MateRelationFeatureData],
    client: Client,
    robot_name: str,
) -> Robot:
    """
    Generate a Robot instance from an Onshape assembly.

    Args:
        assembly: The Onshape assembly object.
        graph: The graph representation of the assembly.
        root_node: The root node of the graph.
        parts: The dictionary of parts in the assembly.
        mates: The dictionary of mates in the assembly.
        relations: The dictionary of mate relations in the assembly.
        client: The Onshape client object.
        robot_name: Name of the robot.

    Returns:
        Robot: The generated Robot instance.
    """
    robot = Robot(name=robot_name)

    assets_map: dict[str, Asset] = {}
    stl_to_link_tf_map = {}
    topological_mates, topological_relations = get_topological_mates(graph, mates, relations)

    LOGGER.info(f"Processing root node: {root_node}")

    if assembly.document is None:
        raise ValueError("Assembly document is None")

    if root_node is None:
        raise ValueError("Root node is None")

    root_link, stl_to_root_tf, root_asset = get_robot_link(
        name=root_node, part=parts[root_node], wid=assembly.document.wid, client=client, mate=None
    )
    robot.add_link(root_link)
    assets_map[root_node] = root_asset
    stl_to_link_tf_map[root_node] = stl_to_root_tf

    LOGGER.info(f"Processing {len(graph.edges)} edges in the graph.")

    for parent, child in graph.edges:
        mate_key = f"{parent}{MATE_JOINER}{child}"
        LOGGER.info(f"Processing edge: {parent} -> {child}")
        parent_tf = stl_to_link_tf_map[parent]

        if parent not in parts or child not in parts:
            LOGGER.warning(f"Part {parent} or {child} not found in parts dictionary. Skipping.")
            continue

        joint_mimic = None
        relation = topological_relations.get(topological_mates[mate_key].id)
        if relation:
            multiplier_value = (
                relation.relationLength
                if relation.relationType == RelationType.RACK_AND_PINION
                else relation.relationRatio
            )
            if multiplier_value is None:
                raise ValueError("Relation multiplier is None")
            multiplier = multiplier_value
            joint_name = get_joint_name(relation.mates[RELATION_PARENT].featureId, mates)
            if joint_name is None:
                raise ValueError("Joint name is None")
            joint_mimic = JointMimic(
                joint=joint_name,
                multiplier=multiplier,
                offset=0.0,
            )

        joint_list, link_list = get_robot_joint(
            parent,
            child,
            topological_mates[mate_key],
            parent_tf,
            joint_mimic,
            is_rigid_assembly=parts[parent].isRigidAssembly,
        )

        if assembly.document is None:
            raise ValueError("Assembly document is None")

        link, stl_to_link_tf, asset = get_robot_link(
            child, parts[child], assembly.document.wid, client, topological_mates[mate_key]
        )
        stl_to_link_tf_map[child] = stl_to_link_tf
        assets_map[child] = asset

        if child not in robot.graph:
            robot.add_link(link)
        else:
            LOGGER.warning(f"Link {child} already exists in the robot graph. Skipping.")

        for link_item in link_list or []:
            if link_item.name not in robot.graph:
                robot.add_link(link_item)
            else:
                LOGGER.warning(f"Link {link_item.name} already exists in the robot graph. Skipping.")

        for joint in joint_list:
            robot.add_joint(joint)

    robot.assets = assets_map
    return robot


if __name__ == "__main__":
    LOGGER.set_file_name("test.log")

    robot = Robot.from_urdf(file_name="E:/onshape-robotics-toolkit/playground/ballbot.urdf", robot_type=RobotType.MJCF)
    robot.set_robot_position((0, 0, 0.6))
    robot.show_tree()
    robot.save(file_path="E:/onshape-robotics-toolkit/playground/ballbot.xml")

    # import mujoco
    # model = mujoco.MjModel.from_xml_path(file_name="E:/onshape-robotics-toolkit/playground/ballbot.urdf")
    # mujoco.mj_saveLastXML("ballbot-raw-ref.xml", model)

    # simulate_robot("test.xml")

    # robot = Robot.from_urdf("E:/onshape-robotics-toolkit/playground/20240920_umv_mini/"
    #                        "20240920_umv_mini/20240920_umv_mini.urdf")
    # robot.save(file_path="E:/onshape-robotics-toolkit/playground/test.urdf", download_assets=False)
