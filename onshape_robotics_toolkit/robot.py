"""
This module contains classes for creating a URDF robot model

Dataclass:
    - **Robot**: Represents a robot model in URDF format, containing links and joints.

"""

import asyncio
import os
from enum import Enum
from typing import Any, Optional

import networkx as nx
import numpy as np
from lxml import etree as ET
from scipy.spatial.transform import Rotation

from onshape_robotics_toolkit.connect import Asset, Client
from onshape_robotics_toolkit.graph import create_graph, plot_graph
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
    get_instances,
    get_mates_and_relations,
    get_parts,
    get_parts_without_mass_properties,
    get_subassemblies,
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

    def __str__(self):
        return self.value


def set_joint_from_xml(element: ET.Element) -> BaseJoint | None:
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
    joint_type = element.attrib["type"]
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

    Attributes:
        name (str): The name of the robot.
        graph (nx.DiGraph): The graph structure holding links (nodes) and joints (edges).
        assets (Optional[dict[str, Asset]]): Assets associated with the robot.
        type (RobotType): The type of the robot (URDF, MJCF, etc.).

    Methods:
        add_link: Add a link to the graph.
        add_joint: Add a joint to the graph.
        to_urdf: Generate URDF XML from the graph.
        save: Save the robot model to a URDF file.
        show: Display the robot's graph as a tree.
        from_urdf: Create a robot model from a URDF file.
    """

    def __init__(self, name: str, assets: Optional[dict[str, Asset]] = None, robot_type: RobotType = RobotType.URDF):
        self.name: str = name
        self.graph: nx.DiGraph = nx.DiGraph()  # Graph to hold links and joints

        if assets is None:
            self.assets: dict[str, Asset] = {}
        else:
            self.assets: dict[str, Asset] = assets

        self.type: RobotType = robot_type

        # Onshape assembly attributes
        self.parts: dict[str, Part] = {}
        self.mates: dict[str, MateFeatureData] = {}
        self.relations: dict[str, MateRelationFeatureData] = {}

        self.subassemblies: dict[str, SubAssembly] = {}
        self.rigid_subassemblies: dict[str, RootAssembly] = {}

        self.assembly: Optional[Assembly] = None
        self.model: Optional[ET.Element] = None

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
        element: ET.Element,
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
        element: ET.Element,
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
        self, root: ET.Element, size: int = 4, orientation: tuple[float, float, float] = (0, 0, 0), name: str = "floor"
    ) -> ET.Element:
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

    def add_ground_plane_assets(self, root: ET.Element) -> None:
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
        for parent, child, joint_data in self.graph.edges(data="data"):
            if joint_data is not None:
                joint_data.to_xml(robot)
            else:
                LOGGER.warning(f"Joint between {parent} and {child} has no data.")

        return ET.tostring(robot, pretty_print=True, encoding="unicode")

    def get_xml_string(self, element: ET.Element) -> str:
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
            asset = ET.SubElement(model, "asset")
            self.add_ground_plane_assets(asset)

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

        dissolved_transforms = {}

        combined_mass = 0
        combined_diaginertia = np.zeros(3)
        combined_pos = np.zeros(3)
        combined_euler = np.zeros(3)

        # First, process all fixed joints
        for parent_name, child_name, joint_data in self.graph.edges(data="data"):
            if joint_data is not None and joint_data.joint_type == "fixed":
                parent_body = body_elements.get(parent_name)
                child_body = body_elements.get(child_name)

                if parent_body is not None and child_body is not None:
                    LOGGER.debug(f"\nProcessing fixed joint from {parent_name} to {child_name}")

                    # Convert joint transform from URDF convention
                    joint_pos = np.array(joint_data.origin.xyz)
                    joint_rot = Rotation.from_euler(URDF_EULER_SEQ, joint_data.origin.rpy)

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
                            new_euler = new_rot.as_euler(MJCF_EULER_SEQ, degrees=False)

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
                            new_euler = new_rot.as_euler(MJCF_EULER_SEQ, degrees=False)

                            element.set("pos", " ".join(format_number(v) for v in new_pos))
                            element.set("euler", " ".join(format_number(v) for v in new_euler))

                        parent_body.append(element)

                    root_body.remove(child_body)
                    body_elements[child_name] = parent_body

        # Normalize the combined position and orientation by the total mass
        if combined_mass > 0:
            combined_pos /= combined_mass
            combined_euler /= combined_mass

        # Find the inertial element of the parent body
        parent_inertial = parent_body.find("inertial")
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
            parent_body.append(new_inertial)

        # Then process revolute joints
        for parent_name, child_name, joint_data in self.graph.edges(data="data"):
            if joint_data is not None and joint_data.joint_type != "fixed":
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
                    joint_pos = np.array(joint_data.origin.xyz)
                    joint_rot = Rotation.from_euler(URDF_EULER_SEQ, joint_data.origin.rpy)

                    # Apply parent's dissolved transformation
                    final_pos = parent_rot.apply(joint_pos) + parent_pos
                    final_rot = parent_rot * joint_rot

                    # Convert to MuJoCo convention while maintaining the joint axis orientation
                    final_euler = final_rot.as_euler(MJCF_EULER_SEQ, degrees=False)

                    LOGGER.debug(f"Joint {parent_name}->{child_name}:")
                    LOGGER.debug(f"  Original: pos={joint_data.origin.xyz}, rpy={joint_data.origin.rpy}")
                    LOGGER.debug(f"  Final: pos={final_pos}, euler={final_euler}")

                    # Update child body transformation
                    child_body.set("pos", " ".join(format_number(v) for v in final_pos))
                    child_body.set("euler", " ".join(format_number(v) for v in final_euler))

                    # Create joint with zero origin
                    joint_data.origin.xyz = [0, 0, 0]
                    joint_data.origin.rpy = [0, 0, 0]
                    joint_data.to_mjcf(child_body)

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
                    new_element = ET.SubElement(parent_element, element.tag, element.attrib)
                    # Copy any children if they exist
                    for child in element:
                        new_element.append(ET.fromstring(ET.tostring(child)))  # noqa: S320
                else:
                    search_type = "tag" if find_by_tag else "name"
                    LOGGER.warning(f"Parent element with {search_type} '{parent}' not found in model.")

        for element_name, attributes in self.mutated_elements.items():
            # Search recursively through all descendants, looking for both body and joint elements
            elements = model.findall(f".//*[@name='{element_name}']")
            if elements:
                element = elements[0]  # Get the first matching element
                for key, value in attributes.items():
                    element.set(key, str(value))
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

        def print_tree(node, depth=0):
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

    def add_custom_element(self, parent_name: str, element: ET.Element) -> None:
        """Add a custom XML element to the robot model.

        Args:
            parent_name: The name of the parent element.
            element: The custom XML element to add.
        """
        if self.model is None:
            self.model = self.create_robot_model()

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
        tree: ET.ElementTree = ET.parse(file_name)  # noqa: S320
        root: ET.Element = tree.getroot()

        name = root.attrib["name"]
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
                            file_name = mesh.attrib.get("filename")
                            if file_name and file_name not in robot.assets:
                                robot.assets[file_name] = Asset.from_file(file_name)

                # Process the collision element within the link
                collision = element.find("collision")
                if collision is not None:
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        mesh = geometry.find("mesh")
                        if mesh is not None:
                            file_name = mesh.attrib.get("filename")
                            if file_name and file_name not in robot.assets:
                                robot.assets[file_name] = Asset.from_file(file_name)

            elif element.tag == "joint":
                joint = set_joint_from_xml(element)
                if joint:
                    robot.add_joint(joint)

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
        Load a robot model from an Onshape CAD assembly.

        Args:
            name: The name of the robot.
            url: The URL of the Onshape CAD assembly.
            client: The Onshape client object.
            max_depth: The maximum depth to process the assembly.
            use_user_defined_root: Whether to use the user-defined root.
            robot_type: The type of the robot.
            log_assembly: Whether to log the assembly response.
            include_rigid_subassembly_parts: Whether to include parts from rigid subassemblies.

        Returns:
            The robot model.
        """

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

        instances, occurrences, id_to_name_map = get_instances(assembly=assembly, max_depth=max_depth)
        subassemblies, rigid_subassemblies = get_subassemblies(assembly=assembly, client=client, instances=instances)

        # Create parts without mass properties first (for mates processing)
        parts_temp = get_parts_without_mass_properties(
            assembly=assembly, rigid_subassemblies=rigid_subassemblies, instances=instances
        )

        mates, relations = get_mates_and_relations(
            assembly=assembly,
            subassemblies=subassemblies,
            rigid_subassemblies=rigid_subassemblies,
            id_to_name_map=id_to_name_map,
            parts=parts_temp,
        )

        graph, root_node = create_graph(
            occurrences=occurrences,
            instances=instances,
            parts=parts_temp,
            mates=mates,
            use_user_defined_root=use_user_defined_root,
        )

        # Now use the optimized get_parts with graph for efficient mass property fetching
        parts = get_parts(
            assembly=assembly,
            rigid_subassemblies=rigid_subassemblies,
            client=client,
            instances=instances,
            graph=graph,
            include_rigid_subassembly_parts=include_rigid_subassembly_parts,
        )

        robot = get_robot(
            assembly=assembly,
            graph=graph,
            root_node=root_node,
            parts=parts,
            mates=mates,
            relations=relations,
            client=client,
            robot_name=name,
        )

        robot.parts = parts
        robot.mates = mates
        robot.relations = relations

        robot.subassemblies = subassemblies
        robot.rigid_subassemblies = rigid_subassemblies

        robot.assembly = assembly

        robot.type = robot_type

        return robot


def load_element(file_name: str) -> ET.Element:
    """
    Load an XML element from a file.

    Args:
        file_name: The path to the file.

    Returns:
        The root element of the XML file.
    """
    tree: ET.ElementTree = ET.parse(file_name)  # noqa: S320
    root: ET.Element = tree.getroot()
    return root


def get_robot(
    assembly: Assembly,
    graph: nx.DiGraph,
    root_node: str,
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

    assets_map = {}
    stl_to_link_tf_map = {}
    topological_mates, topological_relations = get_topological_mates(graph, mates, relations)

    LOGGER.info(f"Processing root node: {root_node}")

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
            multiplier = (
                relation.relationLength
                if relation.relationType == RelationType.RACK_AND_PINION
                else relation.relationRatio
            )
            joint_mimic = JointMimic(
                joint=get_joint_name(relation.mates[RELATION_PARENT].featureId, mates),
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

        link, stl_to_link_tf, asset = get_robot_link(
            child, parts[child], assembly.document.wid, client, topological_mates[mate_key]
        )
        stl_to_link_tf_map[child] = stl_to_link_tf
        assets_map[child] = asset

        if child not in robot.graph:
            robot.add_link(link)
        else:
            LOGGER.warning(f"Link {child} already exists in the robot graph. Skipping.")

        for link in link_list:
            if link.name not in robot.graph:
                robot.add_link(link)
            else:
                LOGGER.warning(f"Link {link.name} already exists in the robot graph. Skipping.")

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
