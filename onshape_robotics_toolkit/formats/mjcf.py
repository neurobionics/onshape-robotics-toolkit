"""
MJCF (MuJoCo Model Format) serializer and deserializer.

This module provides classes for converting Robot objects to/from MJCF format,
which is the XML format used by the MuJoCo physics engine.
"""

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Optional, cast

import numpy as np
from loguru import logger
from lxml import etree as ET
from scipy.spatial.transform import Rotation
from typing_extensions import Literal

from onshape_robotics_toolkit.formats.base import RobotDeserializer, RobotSerializer
from onshape_robotics_toolkit.models.joint import BaseJoint
from onshape_robotics_toolkit.models.mjcf import Actuator, Light, Sensor
from onshape_robotics_toolkit.utilities.helpers import format_number

if TYPE_CHECKING:
    from onshape_robotics_toolkit.parse import PathKey
    from onshape_robotics_toolkit.robot import Robot

# Euler sequence conventions
URDF_EULER_SEQ = "xyz"  # URDF uses XYZ fixed angles
MJCF_EULER_SEQ = "XYZ"  # MuJoCo uses XYZ extrinsic rotations, capitalization matters

# Default compiler attributes for MuJoCo
DEFAULT_COMPILER_ATTRIBUTES = {
    "angle": "radian",
    "eulerseq": "xyz",
}

# Default option attributes for MuJoCo
DEFAULT_OPTION_ATTRIBUTES = {
    "timestep": "0.001",
    "gravity": "0 0 -9.81",
    "iterations": "50",
}


@dataclass
class MJCFConfig:
    """
    Configuration for MJCF (MuJoCo) serialization.

    This class holds all MJCF-specific options that can be customized
    when exporting a robot to MuJoCo format.

    **Attributes:**
        position: Robot body position in world coordinates
        ground_position: Ground plane position in world coordinates
        compiler_attributes: MuJoCo compiler settings
        option_attributes: MuJoCo simulation options
        add_ground_plane: Whether to add a ground plane to the scene
        ground_plane_size: Size of the ground plane
        ground_plane_orientation: Euler angles for ground plane orientation
        lights: Dictionary of Light objects to add to the scene
        actuators: Dictionary of Actuator objects to add to joints
        sensors: Dictionary of Sensor objects to add to the model
        custom_elements: Dictionary of custom XML elements to inject
        mutated_elements: Dictionary of element attributes to modify

    **Example:**
        >>> from onshape_robotics_toolkit.formats import MJCFConfig, MJCFSerializer
        >>> from onshape_robotics_toolkit.models.mjcf import Light, Actuator
        >>>
        >>> # Create custom configuration
        >>> config = MJCFConfig(
        ...     position=(0, 0, 1),
        ...     add_ground_plane=True,
        ...     compiler_attributes={"angle": "degree"}
        ... )
        >>>
        >>> # Add lights
        >>> config.lights["sun"] = Light(
        ...     directional=True,
        ...     diffuse=(0.8, 0.8, 0.8),
        ...     specular=(0.2, 0.2, 0.2),
        ...     pos=(0, 0, 10),
        ...     direction=(0, 0, -1),
        ...     castshadow=True
        ... )
        >>>
        >>> # Add actuators
        >>> config.actuators["motor1"] = Actuator(
        ...     name="motor1",
        ...     joint="joint1",
        ...     ctrllimited=True,
        ...     ctrlrange=(-1, 1),
        ...     gear=50.0
        ... )
        >>>
        >>> # Use configuration with serializer
        >>> serializer = MJCFSerializer(config)
        >>> serializer.save(robot, "robot.xml")
    """

    # Robot positioning
    position: tuple[float, float, float] = (0, 0, 0)
    ground_position: tuple[float, float, float] = (0, 0, 0)

    # MuJoCo settings
    compiler_attributes: dict[str, str] = field(default_factory=lambda: DEFAULT_COMPILER_ATTRIBUTES.copy())
    option_attributes: dict[str, str] = field(default_factory=lambda: DEFAULT_OPTION_ATTRIBUTES.copy())

    # Ground plane options
    add_ground_plane: bool = True
    ground_plane_size: int = 4
    ground_plane_orientation: tuple[float, float, float] = (0, 0, 0)
    ground_plane_name: str = "floor"

    # Scene elements
    lights: dict[str, Light] = field(default_factory=dict)
    cameras: dict[str, Any] = field(default_factory=dict)

    # Actuation and sensing
    actuators: dict[str, Actuator] = field(default_factory=dict)
    sensors: dict[str, Sensor] = field(default_factory=dict)

    # Advanced customization
    custom_elements: dict[str, dict[str, Any]] = field(default_factory=dict)
    mutated_elements: dict[str, dict[str, str]] = field(default_factory=dict)

    def add_custom_element_by_tag(
        self,
        name: str,
        parent_tag: str,
        element: ET._Element,
    ) -> None:
        """
        Register a custom XML element to be added to the first occurrence of a parent tag.

        Args:
            name: Name for referencing this custom element
            parent_tag: Tag name of parent element (e.g. "asset", "worldbody")
            element: The XML element to add

        Examples:
            >>> from lxml import etree as ET
            >>> config = MJCFConfig()
            >>> texture = ET.Element("texture", name="wood", ...)
            >>> config.add_custom_element_by_tag("wood_texture", "asset", texture)
        """
        self.custom_elements[name] = {
            "parent": parent_tag,
            "element": element,
            "find_by_tag": True,
        }

    def add_custom_element_by_name(
        self,
        name: str,
        parent_name: str,
        element: ET._Element,
    ) -> None:
        """
        Register a custom XML element to be added to a parent element with specific name.

        Args:
            name: Name for referencing this custom element
            parent_name: Name attribute of the parent element (e.g. "Part-3-1")
            element: The XML element to add

        Examples:
            >>> from lxml import etree as ET
            >>> config = MJCFConfig()
            >>> imu_site = ET.Element("site", name="imu", ...)
            >>> config.add_custom_element_by_name("imu_sensor", "Part-3-1", imu_site)
        """
        self.custom_elements[name] = {
            "parent": parent_name,
            "element": element,
            "find_by_tag": False,
        }

    def set_element_attributes(
        self,
        element_name: str,
        attributes: dict[str, str],
    ) -> None:
        """
        Register attribute modifications for an existing XML element.

        Args:
            element_name: The name of the element to modify
            attributes: Dictionary of attribute key-value pairs to set/update

        Examples:
            >>> config = MJCFConfig()
            >>> config.set_element_attributes(
            ...     "floor",
            ...     {"size": "10 10 0.001", "friction": "1 0.5 0.5"}
            ... )
        """
        self.mutated_elements[element_name] = attributes


class MJCFSerializer(RobotSerializer):
    """
    Serializer for converting Robot objects to MJCF (MuJoCo) format.

    MJCF is the XML format used by MuJoCo physics simulator. This serializer
    handles complex transformations including:
    - Fixed joint dissolution
    - Transform composition for parent-child relationships
    - Asset management for meshes
    - Compiler and option attributes
    - Scene elements (lights, cameras, ground plane)
    - Actuators and sensors

    **Example 1 - Simple usage without config:**
        >>> from onshape_robotics_toolkit.formats import MJCFSerializer
        >>> from onshape_robotics_toolkit.robot import Robot
        >>>
        >>> robot = Robot.from_graph(...)
        >>> serializer = MJCFSerializer()
        >>> # Pass options directly to save
        >>> serializer.save(
        ...     robot,
        ...     "robot.xml",
        ...     download_assets=True,
        ...     position=(0, 0, 1),
        ...     add_ground_plane=True
        ... )

    **Example 2 - Using MJCFConfig for complex setups:**
        >>> from onshape_robotics_toolkit.formats import MJCFConfig
        >>> from onshape_robotics_toolkit.models.mjcf import Light, Actuator
        >>>
        >>> config = MJCFConfig(
        ...     position=(0, 0, 1),
        ...     add_ground_plane=True,
        ...     compiler_attributes={"angle": "radian"}
        ... )
        >>> config.lights["sun"] = Light(...)
        >>> config.actuators["motor1"] = Actuator(...)
        >>>
        >>> serializer = MJCFSerializer(config)
        >>> serializer.save(robot, "robot.xml", download_assets=True)
    """

    def __init__(self, config: Optional[MJCFConfig] = None):
        """
        Initialize the MJCF serializer with optional configuration.

        Args:
            config: MJCF configuration options. If None, uses default configuration.
                   You can also pass config options as kwargs to serialize() or save().
        """
        self.config = config if config is not None else MJCFConfig()

    def _merge_config(self, options: dict[str, Any]) -> MJCFConfig:
        """
        Merge instance config with provided options.

        Args:
            options: Override options for config fields

        Returns:
            MJCFConfig with merged settings
        """
        # Start with a copy of the current config
        import copy
        from dataclasses import fields

        config = copy.deepcopy(self.config)

        # Update config with any provided options
        valid_fields = {f.name for f in fields(MJCFConfig)}
        for key, value in options.items():
            if key in valid_fields:
                setattr(config, key, value)

        return config

    def serialize(self, robot: "Robot", **options: Any) -> str:
        """
        Convert a Robot object to MJCF XML string.

        This method performs complex transformations to convert the robot
        structure into MuJoCo's format, including dissolving fixed joints
        and composing transformations.

        Args:
            robot: The Robot object to serialize
            **options: Configuration options that override the default config.
                      Supports all MJCFConfig fields (position, add_ground_plane, etc.)

        Returns:
            str: The MJCF XML string

        Examples:
            >>> # Simple usage without config object
            >>> serializer = MJCFSerializer()
            >>> mjcf_xml = serializer.serialize(robot, position=(0, 0, 1), add_ground_plane=True)
            >>>
            >>> # Using pre-configured instance
            >>> config = MJCFConfig(position=(0, 0, 1))
            >>> serializer = MJCFSerializer(config)
            >>> mjcf_xml = serializer.serialize(robot)
        """
        # Merge config with options - options take precedence
        config = self._merge_config(options)
        model = ET.Element("mujoco", model=robot.name)

        # Add compiler settings
        ET.SubElement(model, "compiler", attrib=config.compiler_attributes)

        # Add simulation options
        ET.SubElement(model, "option", attrib=config.option_attributes)

        # Add assets (meshes)
        asset_element = ET.SubElement(model, "asset")
        for _node, data in robot.nodes(data=True):
            asset = data.get("asset")
            if asset:
                asset.to_mjcf(asset_element)

        # Add ground plane assets if configured
        if config.add_ground_plane:
            self._add_ground_plane_assets(asset_element, config)

        # Create worldbody
        worldbody = ET.SubElement(model, "worldbody")

        # Add ground plane if configured
        if config.add_ground_plane:
            self._add_ground_plane(worldbody, config)

        # Add lights
        if config.lights:
            for light in config.lights.values():
                light.to_mjcf(worldbody)

        # Create root body with freejoint
        root_body = ET.SubElement(
            worldbody,
            "body",
            name=robot.name,
            pos=" ".join(map(str, config.position)),
        )
        ET.SubElement(root_body, "freejoint", name=f"{robot.name}_freejoint")

        # Build body elements from robot links
        body_elements: dict[PathKey, ET._Element] = {robot.name: root_body}  # type: ignore[PGH003]
        for link_key, node_data in robot.nodes(data=True):
            link_data = node_data.get("data")
            if link_data is not None:
                body_elements[link_key] = link_data.to_mjcf(root_body)
            else:
                logger.warning(f"Link {link_key} has no data.")

        # Process joints - first fixed, then others
        dissolved_transforms = self._process_fixed_joints(robot, body_elements, root_body)
        self._process_moving_joints(robot, body_elements, dissolved_transforms)

        # Add actuators
        if config.actuators:
            actuator_element = ET.SubElement(model, "actuator")
            for actuator in config.actuators.values():
                actuator.to_mjcf(actuator_element)

        # Add sensors
        if config.sensors:
            sensor_element = ET.SubElement(model, "sensor")
            for sensor in config.sensors.values():
                sensor.to_mjcf(sensor_element)

        # Add custom elements
        self._add_custom_elements(model, config)

        # Mutate elements
        self._mutate_elements(model, config)

        return ET.tostring(model, pretty_print=True, encoding="unicode")

    def _add_ground_plane_assets(self, asset_element: ET._Element, config: MJCFConfig) -> None:
        """Add texture and material assets for the ground plane."""
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
        config.add_custom_element_by_tag("checker", "asset", checker_texture)

        # Create material element
        grid_material = ET.Element(
            "material",
            name="grid",
            texture="checker",
            texrepeat="8 8",
            reflectance=".2",
        )
        config.add_custom_element_by_tag("grid", "asset", grid_material)

    def _add_ground_plane(self, worldbody: ET._Element, config: MJCFConfig) -> None:
        """Add a ground plane to the worldbody."""
        ground_geom = ET.Element(
            "geom",
            name=config.ground_plane_name,
            type="plane",
            pos=" ".join(map(str, config.ground_position)),
            euler=" ".join(map(str, config.ground_plane_orientation)),
            size=f"{config.ground_plane_size} {config.ground_plane_size} 0.001",
            condim="3",
            conaffinity="15",
            material="grid",
        )
        config.add_custom_element_by_tag(
            config.ground_plane_name,
            "worldbody",
            ground_geom,
        )

    def _process_fixed_joints(
        self,
        robot: "Robot",
        body_elements: dict["PathKey", ET._Element],
        root_body: ET._Element,
    ) -> dict["PathKey", tuple[np.ndarray, Rotation]]:
        """
        Process fixed joints by dissolving them and accumulating transforms.

        Returns:
            Dictionary mapping PathKeys to their dissolved transforms
        """
        from onshape_robotics_toolkit.parse import PathKey

        dissolved_transforms: dict[PathKey, tuple[np.ndarray, Rotation]] = {}
        combined_mass = 0.0
        combined_diaginertia = np.zeros(3)
        combined_pos = np.zeros(3)
        combined_euler = np.zeros(3)

        for parent_key, child_key in robot.edges:
            edge_data = robot.get_edge_data(parent_key, child_key)
            joint_data: Optional[BaseJoint] = edge_data.get("data") if edge_data else None

            if joint_data is not None and joint_data.joint_type == "fixed":
                parent_body = body_elements.get(parent_key)
                child_body = body_elements.get(child_key)

                if parent_body is not None and child_body is not None:
                    logger.debug(f"\nProcessing fixed joint from {parent_key} to {child_key}")

                    # Convert joint transform from URDF convention
                    joint_pos = np.array(joint_data.origin.xyz)
                    joint_rot = Rotation.from_euler(URDF_EULER_SEQ, joint_data.origin.rpy)

                    # If parent was dissolved, compose transformations
                    if parent_key in dissolved_transforms:
                        parent_pos, parent_rot = dissolved_transforms[parent_key]
                        joint_pos = parent_rot.apply(joint_pos) + parent_pos
                        joint_rot = parent_rot * joint_rot

                    dissolved_transforms[child_key] = (joint_pos, joint_rot)

                    # Transform geometries
                    for element in list(child_body):
                        if element.tag == "inertial":
                            # Accumulate inertial properties
                            current_pos = np.array([float(x) for x in (element.get("pos") or "0 0 0").split()])
                            current_euler = np.array([float(x) for x in (element.get("euler") or "0 0 0").split()])
                            current_rot = Rotation.from_euler(MJCF_EULER_SEQ, current_euler, degrees=False)
                            current_mass = float(element.get("mass", 0))
                            current_diaginertia = np.array([
                                float(x) for x in (element.get("diaginertia") or "0 0 0").split()
                            ])

                            # Transform position and orientation
                            new_pos = joint_rot.apply(current_pos) + joint_pos
                            new_rot = joint_rot * current_rot
                            new_euler = new_rot.as_euler(cast(Literal["XYZ"], MJCF_EULER_SEQ), degrees=False)

                            # Accumulate
                            combined_mass += current_mass
                            combined_diaginertia += current_diaginertia
                            combined_pos += new_pos * current_mass
                            combined_euler += new_euler * current_mass
                            continue

                        elif element.tag == "geom":
                            # Transform geometry
                            current_pos = np.array([float(x) for x in (element.get("pos") or "0 0 0").split()])
                            current_euler = np.array([float(x) for x in (element.get("euler") or "0 0 0").split()])
                            current_rot = Rotation.from_euler(MJCF_EULER_SEQ, current_euler, degrees=False)

                            # Apply transformation
                            new_pos = joint_rot.apply(current_pos) + joint_pos
                            new_rot = joint_rot * current_rot
                            new_euler = new_rot.as_euler(cast(Literal["XYZ"], MJCF_EULER_SEQ), degrees=False)

                            element.set("pos", " ".join(format_number(float(v)) for v in new_pos))
                            element.set("euler", " ".join(format_number(float(v)) for v in new_euler))

                        parent_body.append(element)

                    root_body.remove(child_body)
                    body_elements[child_key] = parent_body

        # Normalize combined inertial properties
        if combined_mass > 0:
            combined_pos /= combined_mass
            combined_euler /= combined_mass

        # Update parent body's inertial element
        parent_body = body_elements.get(next(iter(body_elements.keys())))  # Get first body
        if parent_body is not None:
            parent_inertial = parent_body.find("inertial")
            if parent_inertial is not None:
                parent_inertial.set("mass", str(combined_mass))
                parent_inertial.set("pos", " ".join(format_number(v) for v in combined_pos))
                parent_inertial.set("euler", " ".join(format_number(v) for v in combined_euler))
                parent_inertial.set("diaginertia", " ".join(format_number(v) for v in combined_diaginertia))
            elif combined_mass > 0:
                # Create new inertial element if it doesn't exist
                new_inertial = ET.Element("inertial")
                new_inertial.set("mass", str(combined_mass))
                new_inertial.set("pos", " ".join(format_number(v) for v in combined_pos))
                new_inertial.set("euler", " ".join(format_number(v) for v in combined_euler))
                new_inertial.set("diaginertia", " ".join(format_number(v) for v in combined_diaginertia))
                parent_body.append(new_inertial)

        return dissolved_transforms

    def _process_moving_joints(
        self,
        robot: "Robot",
        body_elements: dict["PathKey", ET._Element],
        dissolved_transforms: dict["PathKey", tuple[np.ndarray, Rotation]],
    ) -> None:
        """Process revolute, prismatic, and other non-fixed joints."""
        for parent_key, child_key in robot.edges:
            edge_data = robot.get_edge_data(parent_key, child_key)
            joint_data: Optional[BaseJoint] = edge_data.get("data") if edge_data else None

            if joint_data is not None and joint_data.joint_type != "fixed":
                parent_body = body_elements.get(parent_key)
                child_body = body_elements.get(child_key)

                if parent_body is not None and child_body is not None:
                    logger.debug(f"\nProcessing revolute joint from {parent_key} to {child_key}")

                    # Get dissolved parent transform
                    if parent_key in dissolved_transforms:
                        parent_pos, parent_rot = dissolved_transforms[parent_key]
                    else:
                        parent_pos = np.array([0, 0, 0])
                        parent_rot = Rotation.from_euler(URDF_EULER_SEQ, [0, 0, 0])

                    # Convert joint transform from URDF convention
                    joint_pos = np.array(joint_data.origin.xyz)
                    joint_rot = Rotation.from_euler(URDF_EULER_SEQ, joint_data.origin.rpy)

                    # Apply parent's dissolved transformation
                    final_pos = parent_rot.apply(joint_pos) + parent_pos
                    final_rot = parent_rot * joint_rot
                    final_euler = final_rot.as_euler(cast(Literal["XYZ"], MJCF_EULER_SEQ), degrees=False)

                    logger.debug(f"Joint {parent_key} → {child_key}:")
                    logger.debug(f"  Original: pos={joint_data.origin.xyz}, rpy={joint_data.origin.rpy}")
                    logger.debug(f"  Final: pos={final_pos}, euler={final_euler}")

                    # Update child body transformation
                    child_body.set("pos", " ".join(format_number(float(v)) for v in final_pos))
                    child_body.set("euler", " ".join(format_number(float(v)) for v in final_euler))

                    # Create joint with zero origin
                    joint_data.origin.xyz = (0.0, 0.0, 0.0)
                    joint_data.origin.rpy = (0.0, 0.0, 0.0)
                    joint_data.to_mjcf(child_body)

                    # Move child under parent
                    parent_body.append(child_body)

    def _add_custom_elements(self, model: ET._Element, config: MJCFConfig) -> None:
        """Add custom XML elements to the model."""
        for element_info in config.custom_elements.values():
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
                logger.warning(f"Parent element with {search_type} '{parent}' not found in model.")

    def _mutate_elements(self, model: ET._Element, config: MJCFConfig) -> None:
        """Modify attributes of existing XML elements."""
        for element_name, attributes in config.mutated_elements.items():
            elements = model.findall(f".//*[@name='{element_name}']")
            if elements:
                element_to_modify: ET._Element = elements[0]
                for key, value in attributes.items():
                    element_to_modify.set(key, str(value))
            else:
                logger.warning(f"Could not find element with name '{element_name}'")


class MJCFDeserializer(RobotDeserializer):
    """
    Deserializer for converting MJCF format to Robot objects.

    **Note:** This is a placeholder for future implementation.
    MJCF deserialization requires:
    1. Parsing MJCF XML structure
    2. Reconstructing Link objects from <body> elements and <geom> children
    3. Reconstructing Joint objects from joint elements
    4. Handling fixed joint reconstruction (undoing dissolution)
    5. Building the Robot graph structure
    6. Handling mesh file references and assets

    **Example (future):**
        >>> from onshape_robotics_toolkit.formats import MJCFDeserializer
        >>>
        >>> deserializer = MJCFDeserializer()
        >>> robot = deserializer.load("robot.xml")
    """

    def deserialize(self, content: str, **options: Any) -> "Robot":
        """
        Convert MJCF XML string to a Robot object.

        Args:
            content: The MJCF XML string
            **options: Additional deserialization options

        Returns:
            Robot: The deserialized Robot object

        Raises:
            NotImplementedError: This method is not yet implemented
        """
        raise NotImplementedError(
            "MJCF deserialization is not yet implemented. "
            "This feature is planned for a future release. "
            "Please use Robot.from_graph() to create robots from Onshape CAD data."
        )
