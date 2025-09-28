"""
This module contains classes for defining joints in a URDF robot model.

Class:
    - **BaseJoint**: Abstract base class for joint objects.
    - **DummyJoint**: Represents a dummy joint.
    - **RevoluteJoint**: Represents a revolute joint.
    - **ContinuousJoint**: Represents a continuous joint.
    - **PrismaticJoint**: Represents a prismatic joint.
    - **FixedJoint**: Represents a fixed joint.
    - **FloatingJoint**: Represents a floating joint.
    - **PlanarJoint**: Represents a planar joint.

Dataclass:
    - **JointLimits**: Contains the limits for a joint.
    - **JointMimic**: Contains the mimic information for a joint.
    - **JointDynamics**: Contains the dynamics information for a joint.
    - **Axis**: Contains the axis information for a joint.
    - **Origin**: Contains the origin information for a joint.

Enum:
    - **JointType**: Enumerates the possible joint types in Onshape (revolute, continuous, prismatic,
      fixed, floating, planar).

"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from lxml.etree import Element, SubElement, _Element

from onshape_robotics_toolkit.models.link import Axis, Origin
from onshape_robotics_toolkit.utilities import format_number


class JointType(str, Enum):
    """
    Enumerates the possible joint types in Onshape

    Attributes:
        REVOLUTE (str): Revolute joint
        CONTINUOUS (str): Continuous joint
        PRISMATIC (str): Prismatic joint
        FIXED (str): Fixed joint
        FLOATING (str): Floating joint
        PLANAR (str): Planar joint

    Examples:
        >>> JointType.REVOLUTE
        'revolute'
        >>> JointType.CONTINUOUS
        'continuous'
    """

    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    FLOATING = "floating"
    PLANAR = "planar"


MJCF_JOINT_MAP = {
    JointType.REVOLUTE: "hinge",
    JointType.CONTINUOUS: "hinge",
    JointType.PRISMATIC: "slide",
    JointType.FIXED: "fixed",
    JointType.FLOATING: "free",
    JointType.PLANAR: "slide",
}


@dataclass
class JointLimits:
    """
    Represents the limits of a joint.

    Attributes:
        effort (float): The effort limit of the joint.
        velocity (float): The velocity limit of the joint.
        lower (float): The lower limit of the joint.
        upper (float): The upper limit of the joint.

    Methods:
        to_xml: Converts the joint limits to an XML element.

    Examples:
        >>> limits = JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0)
        >>> limits.to_xml()
        <Element 'limit' at 0x7f8b3c0b4c70>
    """

    effort: float
    velocity: float
    lower: float
    upper: float

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the joint limits to an XML element.

        Args:
            root: The root element to append the joint limits to.

        Returns:
            The XML element representing the joint limits.

        Examples:
            >>> limits = JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0)
            >>> limits.to_xml()
            <Element 'limit' at 0x7f8b3c0b4c70>
        """

        limit = Element("limit") if root is None else SubElement(root, "limit")
        limit.set("effort", format_number(self.effort))
        limit.set("velocity", format_number(self.velocity))
        limit.set("lower", format_number(self.lower))
        limit.set("upper", format_number(self.upper))
        return limit


@dataclass
class JointMimic:
    """
    Represents the mimic information for a joint.

    Attributes:
        joint (str): The joint to mimic.
        multiplier (float): The multiplier for the mimic.
        offset (float): The offset for the mimic.

    Methods:
        to_xml: Converts the mimic information to an XML element.

    Examples:
        >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
        >>> mimic.to_xml()
        <Element 'mimic' at 0x7f8b3c0b4c70>
    """

    joint: str
    multiplier: float = 1.0
    offset: float = 0.0

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the mimic information to an XML element.

        Args:
            root: The root element to append the mimic information to.

        Returns:
            The XML element representing the mimic information.

        Examples:
            >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            >>> mimic.to_xml()
            <Element 'mimic' at 0x7f8b3c0b4c70>
        """

        mimic = Element("mimic") if root is None else SubElement(root, "mimic")
        mimic.set("joint", self.joint)
        mimic.set("multiplier", format_number(self.multiplier))
        mimic.set("offset", format_number(self.offset))
        return mimic

    @classmethod
    def from_xml(cls, element: _Element) -> "JointMimic":
        """
        Create a joint mimic from an XML element.

        Args:
            element: The XML element to create the joint mimic from.

        Returns:
            The joint mimic created from the XML element.

        Examples:
            >>> element = ET.Element("mimic")
            >>> element.set("joint", "joint1")
            >>> element.set("multiplier", "1.0")
            >>> element.set("offset", "0.0")
            >>> JointMimic.from_xml(element)
            JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
        """

        joint = element.attrib["joint"]
        multiplier = float(element.attrib.get("multiplier", 1.0))
        offset = float(element.attrib.get("offset", 0.0))
        return cls(joint, multiplier, offset)


@dataclass
class JointDynamics:
    """
    Represents the dynamics information for a joint.

    Attributes:
        damping (float): The damping coefficient of the joint.
        friction (float): The friction coefficient of the joint.

    Methods:
        to_xml: Converts the dynamics information to an XML element.

    Examples:
        >>> dynamics = JointDynamics(damping=0.0, friction=0.0)
        >>> dynamics.to_xml()
        <Element 'dynamics' at 0x7f8b3c0b4c70>
    """

    damping: float
    friction: float

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the dynamics information to an XML element.

        Args:
            root: The root element to append the dynamics information to.

        Returns:
            The XML element representing the dynamics information.

        Examples:
            >>> dynamics = JointDynamics(damping=0.0, friction=0.0)
            >>> dynamics.to_xml()
            <Element 'dynamics' at 0x7f8b3c0b4c70>
        """

        joint = Element("dynamics") if root is None else SubElement(root, "dynamics")
        joint.set("damping", format_number(self.damping))
        joint.set("friction", format_number(self.friction))
        return joint

    @classmethod
    def from_xml(cls, element: _Element) -> "JointDynamics":
        """
        Create joint dynamics from an XML element.

        Args:
            element: The XML element to create the joint dynamics from.

        Returns:
            The joint dynamics created from the XML element.

        Examples:
            >>> element = ET.Element("dynamics")
            >>> element.set("damping", "0.0")
            >>> element.set("friction", "0.0")
            >>> JointDynamics.from_xml(element)
            JointDynamics(damping=0.0, friction=0.0)
        """

        damping = float(element.attrib.get("damping", 0))
        friction = float(element.attrib.get("friction", 0))
        return cls(damping, friction)


@dataclass
class BaseJoint(ABC):
    """
    Abstract base class for joint objects.

    Attributes:
        name (str): The name of the joint.
        parent (str): The parent link of the joint.
        child (str): The child link of the joint.
        origin (Origin): The origin of the joint.

    Methods:
        to_xml: Converts the joint to an XML element.

    Abstract Properties:
        joint_type: Returns the type of the joint.
    """

    name: str
    parent: str
    child: str
    origin: Origin

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the joint to an XML element.

        Args:
            root: The root element to append the joint to.

        Returns:
            The XML element representing the joint.
        """

        joint = Element("joint") if root is None else SubElement(root, "joint")
        joint.set("name", self.name)
        joint.set("type", self.joint_type)
        self.origin.to_xml(joint)
        SubElement(joint, "parent", link=self.parent)
        SubElement(joint, "child", link=self.child)
        return joint

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the joint to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the joint to.
        """

        joint: _Element = SubElement(root, "joint", name=self.name, type=MJCF_JOINT_MAP[JointType(self.joint_type)])
        joint.set("pos", " ".join(map(str, self.origin.xyz)))

    @property
    @abstractmethod
    def joint_type(self) -> str: ...

    @classmethod
    @abstractmethod
    def from_xml(cls, element: _Element) -> "BaseJoint": ...


@dataclass
class DummyJoint(BaseJoint):
    """
    Represents a dummy joint.

    Properties:
        joint_type: The type of the joint.

    Examples:
        >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
        >>> joint = DummyJoint(
        ...     name="joint1",
        ...     parent="base_link",
        ...     child="link1",
        ...     origin=origin,
        ... )
        >>> joint.joint_type
        'dummy'
    """

    @classmethod
    def from_xml(cls, element: _Element) -> "DummyJoint":
        """
        Create a dummy joint from an XML element.

        Args:
            element: The XML element to create the dummy joint from.

        Returns:
            The dummy joint created from the XML element.

        Examples:
            >>> element = ET.Element("joint")
            >>> element.set("name", "joint1")
            >>> element.set("type", "dummy")
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
            >>> ET.SubElement(element, "parent", link="base_link")
            >>> ET.SubElement(element, "child", link="link1")
            >>> DummyJoint.from_xml(element)
            DummyJoint(name="joint1", parent="base_link", child="link1", origin=Origin(xyz=(0, 0, 0), rpy=(0, 0, 0)))
        """

        name = element.attrib["name"]
        parent_elem = element.find("parent")
        child_elem = element.find("child")
        origin_elem = element.find("origin")

        if parent_elem is None or child_elem is None or origin_elem is None:
            raise ValueError("Missing required elements in joint XML")

        parent = parent_elem.attrib["link"]
        child = child_elem.attrib["link"]
        origin = Origin.from_xml(origin_elem)
        return cls(name, parent, child, origin)

    @property
    def joint_type(self) -> str:
        """
        The type of the joint.

        Returns:
            The type of the joint: "dummy".
        """

        return "dummy"


@dataclass
class RevoluteJoint(BaseJoint):
    """
    Represents a revolute joint.

    Attributes:
        limits (JointLimits): The limits of the joint.
        axis (Axis): The axis of the joint.
        dynamics (JointDynamics): The dynamics of the joint.
        mimic (JointMimic): The mimic information for the joint.

    Methods:
        to_xml: Converts the revolute joint to an XML element.

    Properties:
        joint_type: The type of the joint.

    Examples:
        >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
        >>> limits = JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0)
        >>> axis = Axis(xyz=(0, 0, 1))
        >>> dynamics = JointDynamics(damping=0.0, friction=0.0)
        >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
        >>> joint = RevoluteJoint(
        ...     name="joint1",
        ...     parent="base_link",
        ...     child="link1",
        ...     origin=origin,
        ...     limits=limits,
        ...     axis=axis,
        ...     dynamics=dynamics,
        ...     mimic=mimic,
        ... )
        >>> joint.to_xml()
        <Element 'joint' at 0x7f8b3c0b4c70>
    """

    axis: Axis
    limits: JointLimits | None = None
    dynamics: JointDynamics | None = None
    mimic: JointMimic | None = None

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the revolute joint to an XML element.

        Args:
            root: The root element to append the revolute joint to.

        Returns:
            The XML element representing the revolute joint.

        Examples:
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> limits = JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0)
            >>> axis = Axis(xyz=(0, 0, 1))
            >>> dynamics = JointDynamics(damping=0.0, friction=0.0)
            >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            >>> joint = RevoluteJoint(
            ...     name="joint1",
            ...     parent="base_link",
            ...     child="link1",
            ...     origin=origin,
            ...     limits=limits,
            ...     axis=axis,
            ...     dynamics=dynamics,
            ...     mimic=mimic,
            ... )
            >>> joint.to_xml()
            <Element 'joint' at 0x7f8b3c0b4c70>
        """

        joint = super().to_xml(root)
        self.axis.to_xml(joint)
        if self.limits is not None:
            self.limits.to_xml(joint)
        if self.dynamics is not None:
            self.dynamics.to_xml(joint)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the revolute joint to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the revolute joint to.
        """

        joint: _Element = SubElement(root, "joint", name=self.name, type=MJCF_JOINT_MAP[JointType(self.joint_type)])
        joint.set("pos", " ".join(map(str, self.origin.xyz)))

        self.axis.to_mjcf(joint)
        if self.limits:
            joint.set("range", " ".join(map(str, [self.limits.lower, self.limits.upper])))

        if self.dynamics:
            joint.set("damping", str(self.dynamics.damping))
            joint.set("frictionloss", str(self.dynamics.friction))

    @classmethod
    def from_xml(cls, element: _Element) -> "RevoluteJoint":
        """
        Create a revolute joint from an XML element.

        Args:
            element: The XML element to create the revolute joint from.

        Returns:
            The revolute joint created from the XML element.

        Examples:
            >>> element = ET.Element("joint")
            >>> element.set("name", "joint1")
            >>> element.set("type", "revolute")
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
            >>> ET.SubElement(element, "parent", link="base_link")
            >>> ET.SubElement(element, "child", link="link1")
            >>> limits = ET.SubElement(element, "limit", effort="10.0", velocity="1.0", lower="-1.0", upper="1.0")
            >>> axis = ET.SubElement(element, "axis", xyz="0 0 1")
            >>> dynamics = ET.SubElement(element, "dynamics", damping="0.0", friction="0.0")
            >>> mimic = ET.SubElement(element, "mimic", joint="joint1", multiplier="1.0", offset="0.0")
            >>> RevoluteJoint.from_xml(element)

            RevoluteJoint(
                name="joint1",
                parent="base_link",
                child="link1",
                origin=Origin(xyz=(0, 0, 0), rpy=(0, 0, 0)),
                limits=JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0),
                axis=Axis(xyz=(0, 0, 1)),
                dynamics=JointDynamics(damping=0.0, friction=0.0),
                mimic=JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            )
        """

        name = element.attrib["name"]
        parent_elem = element.find("parent")
        child_elem = element.find("child")
        origin_elem = element.find("origin")

        if parent_elem is None or child_elem is None or origin_elem is None:
            raise ValueError("Missing required elements in joint XML")

        parent = parent_elem.attrib["link"]
        child = child_elem.attrib["link"]
        origin = Origin.from_xml(origin_elem)
        # Handle limits
        limit_element = element.find("limit")
        if limit_element is not None:
            limits = JointLimits(
                effort=float(limit_element.attrib.get("effort", 0)),
                velocity=float(limit_element.attrib.get("velocity", 0)),
                lower=float(limit_element.attrib.get("lower", 0)),
                upper=float(limit_element.attrib.get("upper", 0)),
            )
        else:
            limits = None

        # Handle axis
        axis = Axis.from_xml(element.find("axis"))

        # Handle dynamics
        dynamics_element = element.find("dynamics")
        if dynamics_element is not None:
            dynamics = JointDynamics(
                damping=float(dynamics_element.attrib.get("damping", 0)),
                friction=float(dynamics_element.attrib.get("friction", 0)),
            )
        else:
            dynamics = None

        # Handle mimic
        mimic_element = element.find("mimic")
        mimic = JointMimic.from_xml(mimic_element) if mimic_element is not None else None

        return cls(
            name=name,
            parent=parent,
            child=child,
            origin=origin,
            axis=axis,
            limits=limits,
            dynamics=dynamics,
            mimic=mimic,
        )

    @property
    def joint_type(self) -> str:
        """
        The type of the joint.

        Returns:
            The type of the joint: "revolute".
        """

        return JointType.REVOLUTE


@dataclass
class ContinuousJoint(BaseJoint):
    """
    Represents a continuous joint.

    Attributes:
        mimic (JointMimic): The mimic information for the joint.

    Methods:
        to_xml: Converts the continuous joint to an XML element.

    Properties:
        joint_type: The type of the joint.

    Examples:
        >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
        >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
        >>> joint = ContinuousJoint(
        ...     name="joint1",
        ...     parent="base_link",
        ...     child="link1",
        ...     origin=origin,
        ...     mimic=mimic,
        ... )
        >>> joint.to_xml()
        <Element 'joint' at 0x7f8b3c0b4c70>
    """

    mimic: JointMimic | None = None

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the continuous joint to an XML element.

        Args:
            root: The root element to append the continuous joint to.

        Returns:
            The XML element representing the continuous joint.

        Examples:
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            >>> joint = ContinuousJoint(
            ...     name="joint1",
            ...     parent="base_link",
            ...     child="link1",
            ...     origin=origin,
            ...     mimic=mimic,
            ... )
            >>> joint.to_xml()
            <Element 'joint' at 0x7f8b3c0b4c70>
        """

        joint = super().to_xml(root)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    def to_mjcf(self, root: _Element) -> None:
        return super().to_mjcf(root)

    @classmethod
    def from_xml(cls, element: _Element) -> "ContinuousJoint":
        """
        Create a continuous joint from an XML element.

        Args:
            element: The XML element to create the continuous joint from.

        Returns:
            The continuous joint created from the XML element.

        Examples:
            >>> element = ET.Element("joint")
            >>> element.set("name", "joint1")
            >>> element.set("type", "continuous")
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
            >>> ET.SubElement(element, "parent", link="base_link")
            >>> ET.SubElement(element, "child", link="link1")
            >>> mimic = ET.SubElement(element, "mimic", joint="joint1", multiplier="1.0", offset="0.0")
            >>> ContinuousJoint.from_xml(element)

            ContinuousJoint(
                name="joint1",
                parent="base_link",
                child="link1",
                origin=Origin(xyz=(0, 0, 0), rpy=(0, 0, 0)),
                mimic=JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            )
        """

        name = element.attrib["name"]
        parent_elem = element.find("parent")
        child_elem = element.find("child")
        origin_elem = element.find("origin")

        if parent_elem is None or child_elem is None or origin_elem is None:
            raise ValueError("Missing required elements in joint XML")

        parent = parent_elem.attrib["link"]
        child = child_elem.attrib["link"]
        origin = Origin.from_xml(origin_elem)

        # Handle mimic
        mimic_element = element.find("mimic")
        mimic = JointMimic.from_xml(mimic_element) if mimic_element is not None else None

        return cls(name, parent, child, origin, mimic)

    @property
    def joint_type(self) -> str:
        """
        The type of the joint.

        Returns:
            The type of the joint: "continuous".
        """

        return JointType.CONTINUOUS


@dataclass
class PrismaticJoint(BaseJoint):
    """
    Represents a prismatic joint.

    Attributes:
        limits (JointLimits): The limits of the joint.
        axis (Axis): The axis of the joint.
        dynamics (JointDynamics): The dynamics of the joint.
        mimic (JointMimic): The mimic information for the joint.

    Methods:
        to_xml: Converts the prismatic joint to an XML element.

    Properties:
        joint_type: The type of the joint.

    Examples:
        >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
        >>> limits = JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0)
        >>> axis = Axis(xyz=(0, 0, 1))
        >>> dynamics = JointDynamics(damping=0.0, friction=0.0)
        >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
        >>> joint = PrismaticJoint(
        ...     name="joint1",
        ...     parent="base_link",
        ...     child="link1",
        ...     origin=origin,
        ...     limits=limits,
        ...     axis=axis,
        ...     dynamics=dynamics,
        ...     mimic=mimic,
        ... )
        >>> joint.to_xml()
        <Element 'joint' at 0x7f8b3c0b4c70>
    """

    limits: JointLimits
    axis: Axis
    dynamics: JointDynamics | None = None
    mimic: JointMimic | None = None

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the prismatic joint to an XML element.

        Args:
            root: The root element to append the prismatic joint to.

        Returns:
            The XML element representing the prismatic joint

        Examples:
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> limits = JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0)
            >>> axis = Axis(xyz=(0, 0, 1))
            >>> dynamics = JointDynamics(damping=0.0, friction=0.0)
            >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            >>> joint = PrismaticJoint(
            ...     name="joint1",
            ...     parent="base_link",
            ...     child="link1",
            ...     origin=origin,
            ...     limits=limits,
            ...     axis=axis,
            ...     dynamics=dynamics,
            ...     mimic=mimic,
            ... )
            >>> joint.to_xml()
            <Element 'joint' at 0x7f8b3c0b4c70>
        """

        joint = super().to_xml(root)
        self.axis.to_xml(joint)
        if self.limits is not None:
            self.limits.to_xml(joint)
        if self.dynamics is not None:
            self.dynamics.to_xml(joint)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    @classmethod
    def from_xml(cls, element: _Element) -> "PrismaticJoint":
        """
        Create a prismatic joint from an XML element.

        Args:
            element: The XML element to create the prismatic joint from.

        Returns:
            The prismatic joint created from the XML element.

        Examples:
            >>> element = ET.Element("joint")
            >>> element.set("name", "joint1")
            >>> element.set("type", "prismatic")
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
            >>> ET.SubElement(element, "parent", link="base_link")
            >>> ET.SubElement(element, "child", link="link1")
            >>> limits = ET.SubElement(element, "limit", effort="10.0", velocity="1.0", lower="-1.0", upper="1.0")
            >>> axis = ET.SubElement(element, "axis", xyz="0 0 1")
            >>> dynamics = ET.SubElement(element, "dynamics", damping="0.0", friction="0.0")
            >>> mimic = ET.SubElement(element, "mimic", joint="joint1", multiplier="1.0", offset="0.0")
            >>> PrismaticJoint.from_xml(element)

            PrismaticJoint(
                name="joint1",
                parent="base_link",
                child="link1",
                origin=Origin(xyz=(0, 0, 0), rpy=(0, 0, 0)),
                limits=JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0),
                axis=Axis(xyz=(0, 0, 1)),
                dynamics=JointDynamics(damping=0.0, friction=0.0),
                mimic=JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            )
        """

        name = element.attrib["name"]
        parent_elem = element.find("parent")
        child_elem = element.find("child")
        origin_elem = element.find("origin")

        if parent_elem is None or child_elem is None or origin_elem is None:
            raise ValueError("Missing required elements in joint XML")

        parent = parent_elem.attrib["link"]
        child = child_elem.attrib["link"]
        origin = Origin.from_xml(origin_elem)

        limit_element = element.find("limit")
        if limit_element is not None:
            limits = JointLimits(
                effort=float(limit_element.attrib.get("effort", 0)),
                velocity=float(limit_element.attrib.get("velocity", 0)),
                lower=float(limit_element.attrib.get("lower", 0)),
                upper=float(limit_element.attrib.get("upper", 0)),
            )
        else:
            raise ValueError("PrismaticJoint requires limit element")

        axis = Axis.from_xml(element.find("axis"))

        dynamics_element = element.find("dynamics")
        if dynamics_element is not None:
            dynamics = JointDynamics(
                damping=float(dynamics_element.attrib.get("damping", 0)),
                friction=float(dynamics_element.attrib.get("friction", 0)),
            )
        else:
            dynamics = None

        mimic_element = element.find("mimic")
        mimic = JointMimic.from_xml(mimic_element) if mimic_element is not None else None

        return cls(name, parent, child, origin, limits, axis, dynamics, mimic)

    @property
    def joint_type(self) -> str:
        """
        The type of the joint.

        Returns:
            The type of the joint: "prismatic".
        """

        return JointType.PRISMATIC


@dataclass
class FixedJoint(BaseJoint):
    """
    Represents a fixed joint.

    Methods:
        to_xml: Converts the fixed joint to an XML element.

    Properties:
        joint_type: The type of the joint.

    Examples:
        >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
        >>> joint = FixedJoint(
        ...     name="joint1",
        ...     parent="base_link",
        ...     child="link1",
        ...     origin=origin,
        ... )
        >>> joint.to_xml()
        <Element 'joint' at 0x7f8b3c0b4c70>
    """

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the fixed joint to an XML element.

        Args:
            root: The root element to append the fixed joint to.

        Returns:
            The XML element representing the fixed joint.

        Examples:
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> joint = FixedJoint(
            ...     name="joint1",
            ...     parent="base_link",
            ...     child="link1",
            ...     origin=origin,
            ... )
            >>> joint.to_xml()
            <Element 'joint' at 0x7f8b3c0b4c70>
        """

        joint = super().to_xml(root)
        return joint

    @classmethod
    def from_xml(cls, element: _Element) -> "FixedJoint":
        """
        Create a fixed joint from an XML element.

        Args:
            element: The XML element to create the fixed joint from.

        Returns:
            The fixed joint created from the XML element.

        Examples:
            >>> element = ET.Element("joint")
            >>> element.set("name", "joint1")
            >>> element.set("type", "fixed")
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
            >>> ET.SubElement(element, "parent", link="base_link")
            >>> ET.SubElement(element, "child", link="link1")
            >>> FixedJoint.from_xml(element)

            FixedJoint(name="joint1", parent="base_link", child="link1", origin=Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
        """

        name = element.attrib["name"]
        parent_elem = element.find("parent")
        child_elem = element.find("child")
        origin_elem = element.find("origin")

        if parent_elem is None or child_elem is None or origin_elem is None:
            raise ValueError("Missing required elements in joint XML")

        parent = parent_elem.attrib["link"]
        child = child_elem.attrib["link"]
        origin = Origin.from_xml(origin_elem)
        return cls(name, parent, child, origin)

    @property
    def joint_type(self) -> str:
        """
        The type of the joint.

        Returns:
            The type of the joint: "fixed".
        """

        return JointType.FIXED


@dataclass
class FloatingJoint(BaseJoint):
    """
    Represents a floating joint.

    Attributes:
        mimic (JointMimic): The mimic information for the joint.

    Methods:
        to_xml: Converts the floating joint to an XML element.

    Properties:
        joint_type: The type of the joint.

    Examples:
        >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
        >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
        >>> joint = FloatingJoint(
        ...     name="joint1",
        ...     parent="base_link",
        ...     child="link1",
        ...     origin=origin,
        ...     mimic=mimic,
        ... )
        >>> joint.to_xml()
        <Element 'joint' at 0x7f8b3c0b4c70>
    """

    mimic: JointMimic | None = None

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the floating joint to an XML element.

        Args:
            root: The root element to append the floating joint to.

        Returns:
            The XML element representing the floating joint.

        Examples:
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            >>> joint = FloatingJoint(
            ...     name="joint1",
            ...     parent="base_link",
            ...     child="link1",
            ...     origin=origin,
            ...     mimic=mimic,
            ... )
            >>> joint.to_xml()
            <Element 'joint' at 0x7f8b3c0b4c70>
        """

        joint = super().to_xml(root)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    @classmethod
    def from_xml(cls, element: _Element) -> "FloatingJoint":
        """
        Create a floating joint from an XML element.

        Args:
            element: The XML element to create the floating joint from.

        Returns:
            The floating joint created from the XML element.

        Examples:
            >>> element = ET.Element("joint")
            >>> element.set("name", "joint1")
            >>> element.set("type", "floating")
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
            >>> ET.SubElement(element, "parent", link="base_link")
            >>> ET.SubElement(element, "child", link="link1")
            >>> mimic = ET.SubElement(element, "mimic", joint="joint1", multiplier="1.0", offset="0.0")
            >>> FloatingJoint.from_xml(element)

            FloatingJoint(
                name="joint1",
                parent="base_link",
                child="link1",
                origin=Origin(xyz=(0, 0, 0), rpy=(0, 0, 0)),
                mimic=JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            )
        """

        name = element.attrib["name"]
        parent_elem = element.find("parent")
        child_elem = element.find("child")
        origin_elem = element.find("origin")

        if parent_elem is None or child_elem is None or origin_elem is None:
            raise ValueError("Missing required elements in joint XML")

        parent = parent_elem.attrib["link"]
        child = child_elem.attrib["link"]
        origin = Origin.from_xml(origin_elem)

        mimic_element = element.find("mimic")
        mimic = JointMimic.from_xml(mimic_element) if mimic_element is not None else None

        return cls(name, parent, child, origin, mimic)

    @property
    def joint_type(self) -> str:
        """
        The type of the joint.

        Returns:
            The type of the joint: "floating".
        """

        return JointType.FLOATING


@dataclass
class PlanarJoint(BaseJoint):
    """
    Represents a planar joint.

    Attributes:
        limits (JointLimits): The limits of the joint.
        axis (Axis): The axis of the joint.
        mimic (JointMimic): The mimic information for the joint.

    Methods:
        to_xml: Converts the planar joint to an XML element.

    Properties:
        joint_type: The type of the joint.

    Examples:
        >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
        >>> limits = JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0)
        >>> axis = Axis(xyz=(0, 0, 1))
        >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
        >>> joint = PlanarJoint(
        ...     name="joint1",
        ...     parent="base_link",
        ...     child="link1",
        ...     origin=origin,
        ...     limits=limits,
        ...     axis=axis,
        ...     mimic=mimic,
        ... )
        >>> joint.to_xml()
        <Element 'joint' at 0x7f8b3c0b4c70>
    """

    limits: JointLimits
    axis: Axis
    mimic: JointMimic | None = None

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the planar joint to an XML element.

        Args:
            root: The root element to append the planar joint to.

        Returns:
            The XML element representing the planar joint.

        Examples:
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> limits = JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0)
            >>> axis = Axis(xyz=(0, 0, 1))
            >>> mimic = JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            >>> joint = PlanarJoint(
            ...     name="joint1",
            ...     parent="base_link",
            ...     child="link1",
            ...     origin=origin,
            ...     limits=limits,
            ...     axis=axis,
            ...     mimic=mimic,
            ... )
            >>> joint.to_xml()
            <Element 'joint' at 0x7f8b3c0b4c70>
        """

        joint = super().to_xml(root)
        self.axis.to_xml(joint)
        if self.limits is not None:
            self.limits.to_xml(joint)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    @classmethod
    def from_xml(cls, element: _Element) -> "PlanarJoint":
        """
        Create a planar joint from an XML element.

        Args:
            element: The XML element to create the planar joint from.

        Returns:
            The planar joint created from the XML element.

        Examples:
            >>> element = ET.Element("joint")
            >>> element.set("name", "joint1")
            >>> element.set("type", "planar")
            >>> origin = Origin(xyz=(0, 0, 0), rpy=(0, 0, 0))
            >>> ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
            >>> ET.SubElement(element, "parent", link="base_link")
            >>> ET.SubElement(element, "child", link="link1")
            >>> limits = ET.SubElement(element, "limit", effort="10.0", velocity="1.0", lower="-1.0", upper="1.0")
            >>> axis = ET.SubElement(element, "axis", xyz="0 0 1")
            >>> mimic = ET.SubElement(element, "mimic", joint="joint1", multiplier="1.0", offset="0.0")
            >>> PlanarJoint.from_xml(element)

            PlanarJoint(
                name="joint1",
                parent="base_link",
                child="link1",
                origin=Origin(xyz=(0, 0, 0), rpy=(0, 0, 0)),
                limits=JointLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0),
                axis=Axis(xyz=(0, 0, 1)),
                mimic=JointMimic(joint="joint1", multiplier=1.0, offset=0.0)
            )
        """

        name = element.attrib["name"]
        parent_elem = element.find("parent")
        child_elem = element.find("child")
        origin_elem = element.find("origin")

        if parent_elem is None or child_elem is None or origin_elem is None:
            raise ValueError("Missing required elements in joint XML")

        parent = parent_elem.attrib["link"]
        child = child_elem.attrib["link"]
        origin = Origin.from_xml(origin_elem)

        limit_element = element.find("limit")
        if limit_element is not None:
            limits = JointLimits(
                effort=float(limit_element.attrib.get("effort", 0)),
                velocity=float(limit_element.attrib.get("velocity", 0)),
                lower=float(limit_element.attrib.get("lower", 0)),
                upper=float(limit_element.attrib.get("upper", 0)),
            )
        else:
            raise ValueError("PlanarJoint requires limit element")

        axis = Axis.from_xml(element.find("axis"))

        mimic_element = element.find("mimic")
        mimic = JointMimic.from_xml(mimic_element) if mimic_element is not None else None

        return cls(name, parent, child, origin, limits, axis, mimic)

    @property
    def joint_type(self) -> str:
        """
        The type of the joint.

        Returns:
            The type of the joint: "planar".
        """

        return JointType.PLANAR
