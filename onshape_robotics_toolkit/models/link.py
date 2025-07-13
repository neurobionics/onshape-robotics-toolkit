"""
This module contains dataclasses for creating a link in a URDF robot model.

Dataclass:
    - **Origin**: Represents the origin of a link in the robot model.
    - **Axis**: Represents the axis of a link in the robot model.
    - **Inertia**: Represents the inertia properties of a link in the robot model.
    - **Material**: Represents the material properties of a link in the robot model.
    - **InertialLink**: Represents the inertial properties of a link in the robot model.
    - **VisualLink**: Represents the visual properties of a link in the robot model.
    - **CollisionLink**: Represents the collision properties of a link in the robot model.
    - **Link**: Represents a link in the robot model.

Enum:
    - **Colors**: Enumerates the possible colors for a link in the robot model.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Union

import numpy as np
from lxml import etree as ET
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R

from onshape_robotics_toolkit.models.geometry import (
    BaseGeometry,
    BoxGeometry,
    CylinderGeometry,
    MeshGeometry,
    SphereGeometry,
)
from onshape_robotics_toolkit.utilities import format_number


class Colors(tuple[float, float, float], Enum):
    """
    Enumerates the possible colors in RGBA format for a link in the robot model.

    Each color is represented as a tuple of four float values (r, g, b, a),
    where each component ranges from 0.0 to 1.0.

    Attributes:
        RED (tuple[float, float, float, float]): Color red (1, 0, 0, 1).
        GREEN (tuple[float, float, float, float]): Color green (0, 1, 0, 1).
        BLUE (tuple[float, float, float, float]): Color blue (0, 0, 1, 1).
        YELLOW (tuple[float, float, float, float]): Color yellow (1, 1, 0, 1).
        CYAN (tuple[float, float, float, float]): Color cyan (0, 1, 1, 1).
        MAGENTA (tuple[float, float, float, float]): Color magenta (1, 0, 1, 1).
        WHITE (tuple[float, float, float, float]): Color white (1, 1, 1, 1).
        BLACK (tuple[float, float, float, float]): Color black (0, 0, 0, 1).
        ORANGE (tuple[float, float, float, float]): Color orange (1, 0.5, 0, 1).
        PINK (tuple[float, float, float, float]): Color pink (1, 0, 0.5, 1).

    Examples:
        >>> Colors.RED
        <Colors.RED: (1.0, 0.0, 0.0, 1.0)>
        >>> Colors.BLUE.value
        (0.0, 0.0, 1.0, 1.0)
    """

    RED = (1.0, 0.0, 0.0, 1.0)
    GREEN = (0.0, 1.0, 0.0, 1.0)
    BLUE = (0.0, 0.0, 1.0, 1.0)
    YELLOW = (1.0, 1.0, 0.0, 1.0)
    CYAN = (0.0, 1.0, 1.0, 1.0)
    MAGENTA = (1.0, 0.0, 1.0, 1.0)
    WHITE = (1.0, 1.0, 1.0, 1.0)
    BLACK = (0.0, 0.0, 0.0, 1.0)
    ORANGE = (1.0, 0.5, 0.0, 1.0)
    PINK = (1.0, 0.0, 0.5, 1.0)


@dataclass
class Origin:
    """
    Represents the origin of a link in the robot model.

    Attributes:
        xyz (tuple[float, float, float]): The x, y, z coordinates of the origin.
        rpy (tuple[float, float, float]): The roll, pitch, yaw angles of the origin.

    Methods:
        transform: Applies a transformation matrix to the origin.
        to_xml: Converts the origin to an XML element.
        to_mjcf: Converts the origin to a MuJoCo compatible XML element.
        quat: Converts the origin's rotation to a quaternion.

    Class Methods:
        from_xml: Creates an origin from an XML element.
        from_matrix: Creates an origin from a transformation matrix.
        zero_origin: Creates an origin at (0, 0, 0) with no rotation.

    Examples:
        >>> origin = Origin(xyz=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.0))
        >>> origin.to_xml()
        <Element 'origin' at 0x7f8b3c0b4c70>

        >>> matrix = np.matrix([
        ...     [1, 0, 0, 0],
        ...     [0, 1, 0, 0],
        ...     [0, 0, 1, 0],
        ...     [0, 0, 0, 1],
        ... ])
        >>> Origin.from_matrix(matrix)
        Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0))
    """

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]

    def transform(self, matrix: np.matrix, inplace: bool = False) -> Union["Origin", None]:
        """
        Apply a transformation matrix to the origin.

        Args:
            matrix (np.matrix): The 4x4 transformation matrix to apply.
            inplace (bool): If True, modifies the current origin. If False, returns a new Origin.

        Returns:
            Union[Origin, None]: If inplace is False, returns a new transformed Origin.
                               If inplace is True, returns None and modifies current Origin.

        Examples:
            >>> origin = Origin(xyz=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.0))
            >>> matrix = np.eye(4)
            >>> new_origin = origin.transform(matrix)  # Returns new Origin
            >>> origin.transform(matrix, inplace=True)  # Modifies origin in place
        """
        new_xyz = np.dot(matrix[:3, :3], np.array(self.xyz)) + matrix[:3, 3]
        current_rotation_matrix = Rotation.from_euler("xyz", self.rpy).as_matrix()

        new_rotation_matrix = np.dot(matrix[:3, :3], current_rotation_matrix)
        new_rpy = Rotation.from_matrix(new_rotation_matrix).as_euler("xyz")
        if inplace:
            self.xyz = tuple(new_xyz)
            self.rpy = tuple(new_rpy)
            return None

        return Origin(new_xyz, new_rpy)

    def to_xml(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the origin to an XML element.

        Args:
            root: The root element to append the origin to.

        Returns:
            The XML element representing the origin.

        Examples:
            >>> origin = Origin(xyz=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.0))
            >>> origin.to_xml()
            <Element 'origin' at 0x7f8b3c0b4c70>
        """

        origin = ET.Element("origin") if root is None else ET.SubElement(root, "origin")
        origin.set("xyz", " ".join(format_number(v) for v in self.xyz))
        origin.set("rpy", " ".join(format_number(v) for v in self.rpy))
        return origin

    def to_mjcf(self, root: ET.Element) -> None:
        """
        Convert the origin to a MuJoCo compatible XML element.

        Args:
            root (ET.Element): The root element to add the origin attributes to.
                             Adds 'pos' and 'euler' attributes to this element.

        Examples:
            >>> origin = Origin(xyz=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.0))
            >>> element = ET.Element('body')
            >>> origin.to_mjcf(element)
            >>> element.get('pos')
            '1.0 2.0 3.0'
            >>> element.get('euler')
            '0.0 0.0 0.0'
        """
        root.set("pos", " ".join(format_number(v) for v in self.xyz))
        root.set("euler", " ".join(format_number(v) for v in self.rpy))

    @classmethod
    def from_xml(cls, xml: ET.Element) -> "Origin":
        """
        Create an origin from an XML element.

        Args:
            xml: The XML element to create the origin from.

        Returns:
            The origin created from the XML element.

        Examples:
            >>> xml = ET.Element('origin')
            >>> Origin.from_xml(xml)
            Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0))
        """

        xyz = tuple(map(float, xml.get("xyz").split()))
        rpy = tuple(map(float, xml.get("rpy").split()))
        return cls(xyz, rpy)

    def quat(self, sequence: str = "xyz") -> np.ndarray:
        """
        Convert the origin's rotation to a quaternion.

        Args:
            sequence (str): The sequence of rotations used for Euler angles. Defaults to "xyz".

        Returns:
            np.ndarray: A quaternion [x, y, z, w] representing the rotation.

        Examples:
            >>> origin = Origin(xyz=(0.0, 0.0, 0.0), rpy=(np.pi/2, 0.0, 0.0))
            >>> origin.quat()
            array([0.70710678, 0.        , 0.        , 0.70710678])
        """
        return Rotation.from_euler(sequence, self.rpy).as_quat()

    @classmethod
    def from_matrix(cls, matrix: np.matrix) -> "Origin":
        """
        Create an origin from a transformation matrix.

        Args:
            matrix: The transformation matrix.

        Returns:
            The origin created from the transformation matrix.

        Examples:
            >>> matrix = np.matrix([
            ...     [1, 0, 0, 0],
            ...     [0, 1, 0, 0],
            ...     [0, 0, 1, 0],
            ...     [0, 0, 0, 1],
            ... ])
            >>> Origin.from_matrix(matrix)
            Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0))
        """

        x = float(matrix[0, 3])
        y = float(matrix[1, 3])
        z = float(matrix[2, 3])
        roll, pitch, yaw = Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")
        return cls((x, y, z), (roll, pitch, yaw))

    @classmethod
    def zero_origin(cls) -> "Origin":
        """
        Create an origin at the origin (0, 0, 0) with no rotation.

        Returns:
            The origin at the origin (0, 0, 0) with no rotation.

        Examples:
            >>> Origin.zero_origin()
            Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0))
        """

        return cls((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


@dataclass
class Axis:
    """
    Represents the axis of rotation or translation for a joint in the robot model.

    Attributes:
        xyz (tuple[float, float, float]): The direction vector of the axis.
            Should be a unit vector (normalized).

    Methods:
        to_xml: Converts the axis to an XML element.
        to_mjcf: Converts the axis to a MuJoCo compatible XML element.

    Class Methods:
        from_xml: Creates an axis from an XML element.

    Examples:
        >>> axis = Axis(xyz=(1.0, 0.0, 0.0))  # X-axis rotation
        >>> axis.to_xml()
        <Element 'axis' at 0x...>

        >>> xml_str = '<axis xyz="0 1 0"/>'
        >>> xml_element = ET.fromstring(xml_str)
        >>> axis = Axis.from_xml(xml_element)
        >>> axis.xyz
        (0.0, 1.0, 0.0)
    """

    xyz: tuple[float, float, float]

    def to_xml(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the axis to an XML element.

        Args:
            root: The root element to append the axis to.

        Returns:
            The XML element representing the axis.

        Examples:
            >>> axis = Axis(xyz=(1.0, 0.0, 0.0))
            >>> axis.to_xml()
            <Element 'axis' at 0x7f8b3c0b4c70>
        """

        axis = ET.Element("axis") if root is None else ET.SubElement(root, "axis")
        axis.set("xyz", " ".join(format_number(v) for v in self.xyz))
        return axis

    def to_mjcf(self, root: ET.Element) -> None:
        """
        Convert the axis to an MuJoCo compatible XML element.

        Args:
            root: The root element to append the axis to.

        Returns:
            The XML element representing the axis.

        Examples:
            >>> axis = Axis(xyz=(1.0, 0.0, 0.0))
            >>> axis.to_mjcf()
            <Element 'axis' at 0x7f8b3c0b4c70>
        """
        root.set("axis", " ".join(format_number(v) for v in self.xyz))

    @classmethod
    def from_xml(cls, xml: ET.Element) -> "Axis":
        """
        Create an axis from an XML element.

        Args:
            xml: The XML element to create the axis from.

        Returns:
            The axis created from the XML element.

        Examples:
            >>> xml = ET.Element('axis')
            >>> Axis.from_xml(xml)
            Axis(xyz=(0.0, 0.0, 0.0))
        """
        xyz = tuple(map(float, xml.get("xyz").split()))
        return cls(xyz)


@dataclass
class Inertia:
    """
    Represents the inertia tensor of a link in the robot model.

    The inertia tensor is a 3x3 symmetric matrix that describes how the body's mass
    is distributed relative to its center of mass.

    Attributes:
        ixx (float): Moment of inertia about the x-axis.
        iyy (float): Moment of inertia about the y-axis.
        izz (float): Moment of inertia about the z-axis.
        ixy (float): Product of inertia about the xy-plane.
        ixz (float): Product of inertia about the xz-plane.
        iyz (float): Product of inertia about the yz-plane.

    Methods:
        to_xml: Converts the inertia tensor to an XML element.
        to_mjcf: Converts the inertia tensor to a MuJoCo compatible XML element.
        to_matrix: Returns the inertia tensor as a 3x3 numpy array.

    Class Methods:
        from_xml: Creates an inertia tensor from an XML element.
        zero_inertia: Creates an inertia tensor with all zero values.

    Examples:
        >>> inertia = Inertia(ixx=1.0, iyy=1.0, izz=1.0, ixy=0.0, ixz=0.0, iyz=0.0)
        >>> inertia.to_matrix
        array([[1., 0., 0.],
               [0., 1., 0.],
               [0., 0., 1.]])
    """

    ixx: float
    iyy: float
    izz: float
    ixy: float
    ixz: float
    iyz: float

    def to_xml(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the inertia tensor to an XML element.

        Args:
            root: The root element to append the inertia tensor to.

        Returns:
            The XML element representing the inertia tensor.

        Examples:
            >>> inertia = Inertia(ixx=1.0, iyy=2.0, izz=3.0, ixy=0.0, ixz=0.0, iyz=0.0)
            >>> inertia.to_xml()
            <Element 'inertia' at 0x7f8b3c0b4c70>
        """

        inertia = ET.Element("inertia") if root is None else ET.SubElement(root, "inertia")
        inertia.set("ixx", format_number(self.ixx))
        inertia.set("iyy", format_number(self.iyy))
        inertia.set("izz", format_number(self.izz))
        inertia.set("ixy", format_number(self.ixy))
        inertia.set("ixz", format_number(self.ixz))
        inertia.set("iyz", format_number(self.iyz))
        return inertia

    def to_mjcf(self, root: ET.Element) -> None:
        """
        Convert the inertia tensor to an MuJoCo compatible XML element.

        Args:
            root: The root element to append the inertia tensor to.

        Returns:
            The XML element representing the inertia tensor.

        Examples:
            >>> inertia = Inertia(ixx=1.0, iyy=2.0, izz=3.0, ixy=0.0, ixz=0.0, iyz=0.0)
            >>> inertia.to_mjcf()
            <Element 'inertia' at 0x7f8b3c0b4c70>
        """
        inertial = root if root.tag == "inertial" else ET.SubElement(root, "inertial")
        inertial.set("diaginertia", " ".join(format_number(v) for v in [self.ixx, self.iyy, self.izz]))

    @classmethod
    def from_xml(cls, xml: ET.Element) -> "Inertia":
        """
        Create an inertia tensor from an XML element.

        Args:
            xml: The XML element to create the inertia tensor from.

        Returns:
            The inertia tensor created from the XML element.

        Examples:
            >>> xml = ET.Element('inertia')
            >>> Inertia.from_xml(xml)
            Inertia(ixx=0.0, iyy=0.0, izz=0.0, ixy=0.0, ixz=0.0, iyz=0.0)
        """
        ixx = float(xml.get("ixx"))
        iyy = float(xml.get("iyy"))
        izz = float(xml.get("izz"))
        ixy = float(xml.get("ixy"))
        ixz = float(xml.get("ixz"))
        iyz = float(xml.get("iyz"))
        return cls(ixx, iyy, izz, ixy, ixz, iyz)

    @classmethod
    def zero_inertia(cls) -> "Inertia":
        """
        Create an inertia tensor with zero values.

        Returns:
            The inertia tensor with zero values.

        Examples:
            >>> Inertia.zero_inertia()
            Inertia(ixx=0.0, iyy=0.0, izz=0.0, ixy=0.0, ixz=0.0, iyz=0.0)
        """
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    @property
    def to_matrix(self) -> np.array:
        """
        Returns the inertia tensor as a 3x3 numpy array.

        Returns:
            The inertia tensor as a 3x3 numpy array.
        """
        return np.array([
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz],
        ])


@dataclass
class Material:
    """
    Represents the material properties of a link in the robot model.

    Materials define the visual appearance of links in the robot model,
    primarily through their color properties.

    Attributes:
        name (str): The name identifier for the material.
        color (tuple[float, float, float, float]): The RGBA color values,
            each component in range [0.0, 1.0].

    Methods:
        to_xml: Converts the material properties to an XML element.
        to_mjcf: Converts the material to a MuJoCo compatible XML element.

    Class Methods:
        from_xml: Creates a material from an XML element.
        from_color: Creates a material from a predefined Colors enum value.

    Examples:
        >>> material = Material(name="red_material", color=(1.0, 0.0, 0.0, 1.0))
        >>> material.to_xml()
        <Element 'material' at 0x...>

        >>> material = Material.from_color("steel_material", Colors.BLUE)
        >>> material.color
        (0.0, 0.0, 1.0, 1.0)
    """

    name: str
    color: tuple[float, float, float, float]

    def to_xml(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the material properties to an XML element.

        Args:
            root: The root element to append the material properties to.

        Returns:
            The XML element representing the material properties.

        Examples:
            >>> material = Material(name="material", color=(1.0, 0.0, 0.0, 1.0))
            >>> material.to_xml()
            <Element 'material' at 0x7f8b3c0b4c70>
        """

        material = ET.Element("material") if root is None else ET.SubElement(root, "material")
        material.set("name", self.name)
        ET.SubElement(material, "color", rgba=" ".join(format_number(v) for v in self.color))
        return material

    def to_mjcf(self, root: ET.Element) -> None:
        """
        Convert the material properties to an MuJoCo compatible XML element.

        Args:
            root: The root element to append the material properties to.

        Returns:
            The XML element representing the material properties.

        Examples:
            >>> material = Material(name="material", color=(1.0, 0.0, 0.0, 1.0))
            >>> material.to_mjcf()
            <Element 'material' at 0x7f8b3c0b4c70>
        """
        geom = root if root is not None and root.tag == "geom" else ET.SubElement(root, "geom")
        geom.set("rgba", " ".join(format_number(v) for v in self.color))

    @classmethod
    def from_xml(cls, xml: ET.Element) -> "Material":
        """
        Create a material from an XML element.

        Args:
            xml: The XML element to create the material from.

        Returns:
            The material created from the XML element.

        Examples:
            >>> xml = ET.Element('material')
            >>> Material.from_xml(xml)
            Material(name='material', color=(1.0, 0.0, 0.0, 1.0))
        """

        name = xml.get("name")
        color = tuple(map(float, xml.find("color").get("rgba").split()))
        return cls(name, color)

    @classmethod
    def from_color(cls, name: str, color: Colors) -> "Material":
        """
        Create a material from a color.

        Args:
            name: The name of the material.
            color: The color of the material.

        Returns:
            The material created from the color.

        Examples:
            >>> Material.from_color(name="material", color=Colors.RED)
            Material(name='material', color=(1.0, 0.0, 0.0, 1.0))
        """
        return cls(name, color)


@dataclass
class InertialLink:
    """
    Represents the inertial properties of a link in the robot model.

    This class combines mass, center of mass (via origin), and inertia tensor
    properties to fully describe a link's inertial characteristics.

    Attributes:
        mass (float): The mass of the link in kilograms.
        inertia (Inertia): The inertia tensor of the link.
        origin (Origin): The center of mass position and orientation.

    Methods:
        to_xml: Converts the inertial properties to an XML element.
        to_mjcf: Converts the inertial properties to a MuJoCo compatible XML element.
        transform: Applies a transformation matrix to the inertial properties.

    Class Methods:
        from_xml: Creates an InertialLink from an XML element.

    Examples:
        >>> inertial = InertialLink(
        ...     mass=1.0,
        ...     inertia=Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
        ...     origin=Origin.zero_origin()
        ... )
        >>> inertial.to_xml()
        <Element 'inertial' at 0x...>
    """

    mass: float
    inertia: Inertia
    origin: Origin

    def to_xml(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the inertial properties to an XML element.

        Args:
            root: The root element to append the inertial properties to.

        Returns:
            The XML element representing the inertial properties.

        Examples:
            >>> inertial = InertialLink(
            ...     mass=1.0,
            ...     inertia=Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
            ...     origin=Origin.zero_origin()
            ... )
            >>> inertial.to_xml()
            <Element 'inertial' at 0x...>
        """
        inertial = ET.Element("inertial") if root is None else ET.SubElement(root, "inertial")
        ET.SubElement(inertial, "mass", value=format_number(self.mass))
        self.inertia.to_xml(inertial)
        self.origin.to_xml(inertial)
        return inertial

    def to_mjcf(self, root: ET.Element) -> None:
        """
        Convert the inertial properties to an MuJoCo compatible XML element.

        Example XML:
        ```xml
        <inertial pos="0 0 -0.0075" euler="0.5 0.5 -0.5" mass="0.624"
                  diaginertia="0.073541512 0.07356916 0.073543931" />
        ```
        Args:
            root: The root element to append the inertial properties to.
        """
        inertial = root if root.tag == "inertial" else ET.SubElement(root, "inertial")
        inertial.set("mass", format_number(self.mass))
        self.origin.to_mjcf(inertial)
        self.inertia.to_mjcf(inertial)

    def transform(self, tf_matrix: np.matrix, inplace: bool = False) -> Union["InertialLink", None]:
        """
        Apply a transformation matrix to the Inertial Properties of the a link.

        Args:
            matrix: The transformation matrix to apply to the origin.
            inplace: Whether to apply the transformation in place.

        Returns:
            An updated Inertial Link with the transformation applied to both:
            * the inertia tensor (giving a transformed "inertia tensor prime" = [ixx', iyy', izz', ixy', ixz', iyz'])
            * AND to the origin too (via the Origin class's transform logic [~line 100])

        Examples {@}:
            >>> origin = Origin(xyz=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.0))
            >>> matrix = np.eye(4)
            >>> inertial.transform(matrix)

        Analysis and References:
            The essence is to convert the Inertia tensor to a matrix and then transform the matrix via the equation
            I_prime = R·I·Transpose[R] + m(||d||^2·I - d·Transpose[d])
            Then we put the components into the resultant Inertial Link
            An analysis (on 100k runs) suggests that this is 3 x faster than a direct approach on the tensor elements
            likely because numpy's libraries are optimized for matrix operations.
            (https://chatgpt.com/share/6781b6ac-772c-8006-b1a9-7f2dc3e3ef4d)
            Ref: https://chatgpt.com/share/6781b6ac-772c-8006-b1a9-7f2dc3e3ef4d
        """

        R = tf_matrix[:3, :3]  # Top-left 3x3 block is the rotation matrix
        p = tf_matrix[:3, 3]  # Top-right 3x1 block is the translation vector

        inertia_matrix = self.inertia.to_matrix
        I_rot = R @ inertia_matrix @ R.T

        # Compute the parallel axis theorem adjustment
        parallel_axis_adjustment = self.mass * (np.dot(p, p) * np.eye(3) - np.outer(p, p))

        # Final transformed inertia matrix
        I_transformed = I_rot + parallel_axis_adjustment

        ixx_prime = I_transformed[0, 0]
        iyy_prime = I_transformed[1, 1]
        izz_prime = I_transformed[2, 2]
        ixy_prime = I_transformed[0, 1]
        ixz_prime = I_transformed[0, 2]
        iyz_prime = I_transformed[1, 2]

        # Transform the Origin (Don't replace the original in case the user keeps the inplace flag false)
        Origin_prime = self.origin.transform(tf_matrix)

        # Update values and (if requested) put the extracted values into a new_InertialLink
        if inplace:
            # mass stays the same :-) ==> self.mass = new_InertialLink.mass
            self.inertia.ixx = ixx_prime
            self.inertia.iyy = iyy_prime
            self.inertia.izz = izz_prime
            self.inertia.ixy = ixy_prime
            self.inertia.ixz = ixz_prime
            self.inertia.iyz = iyz_prime
            self.origin = Origin_prime
            return None
        else:
            new_InertialLink = InertialLink(
                mass=self.mass,
                inertia=Inertia(ixx_prime, iyy_prime, izz_prime, ixy_prime, ixz_prime, iyz_prime),
                origin=Origin_prime,
            )
            return new_InertialLink

    @classmethod
    def from_xml(cls, xml: ET.Element) -> "InertialLink":
        """
        Create inertial properties from an XML element.

        Args:
            xml: The XML element to create the inertial properties from.

        Returns:
            The inertial properties created from the XML element.

        Examples:
            >>> xml = ET.Element('inertial')
            >>> InertialLink.from_xml(xml)
            InertialLink(mass=0.0, inertia=None, origin=None)
        """
        mass = float(xml.find("mass").get("value"))

        inertia_element = xml.find("inertia")
        inertia = Inertia.from_xml(inertia_element) if inertia_element is not None else None

        origin_element = xml.find("origin")
        origin = Origin.from_xml(origin_element) if origin_element is not None else None

        return cls(mass=mass, inertia=inertia, origin=origin)


def set_geometry_from_xml(geometry: ET.Element) -> BaseGeometry | None:
    """
    Set the geometry from an XML element.

    Args:
        geometry: The XML element to create the geometry from.

    Returns:
        The geometry created from the XML element.
    """
    if geometry.find("mesh") is not None:
        return MeshGeometry.from_xml(geometry)
    elif geometry.find("box"):
        return BoxGeometry.from_xml(geometry)
    elif geometry.find("length") and geometry.find("radius"):
        return CylinderGeometry.from_xml(geometry)
    elif geometry.find("radius"):
        return SphereGeometry.from_xml(geometry)

    return None


@dataclass
class VisualLink:
    """
    Represents the visual properties of a link in the robot model.

    This class defines how a link appears in visualization tools and simulators,
    including its position, geometry, and material properties.

    Attributes:
        name (Union[str, None]): Optional name identifier for the visual element.
        origin (Origin): The position and orientation of the visual geometry.
        geometry (BaseGeometry): The shape of the visual element (box, cylinder, mesh, etc.).
        material (Material): The material properties (color, texture) of the visual element.

    Methods:
        to_xml: Converts the visual properties to an XML element.
        to_mjcf: Converts the visual properties to a MuJoCo compatible XML element.
        transform: Applies a transformation matrix to the visual geometry's origin.

    Class Methods:
        from_xml: Creates a VisualLink from an XML element.

    Examples:
        >>> visual = VisualLink(
        ...     name="link_visual",
        ...     origin=Origin.zero_origin(),
        ...     geometry=BoxGeometry(size=(1.0, 1.0, 1.0)),
        ...     material=Material.from_color("red", Colors.RED)
        ... )
        >>> visual.to_xml()
        <Element 'visual' at 0x...>
    """

    name: Union[str, None]
    origin: Origin
    geometry: BaseGeometry
    material: Material

    def transform(self, transformation_matrix: np.ndarray) -> None:
        """
        Apply a transformation to the visual link's origin.

        Args:
            transformation_matrix (np.ndarray): A 4x4 transformation matrix (homogeneous).
        """
        # Apply translation and rotation to the origin position
        pos = np.array([self.origin.xyz[0], self.origin.xyz[1], self.origin.xyz[2], 1])
        new_pos = transformation_matrix @ pos
        self.origin.xyz = tuple(new_pos[:3])  # Update position

        # Extract the rotation from the transformation matrix
        rotation_matrix = transformation_matrix[:3, :3]
        current_rotation = R.from_euler("xyz", self.origin.rpy)
        new_rotation = R.from_matrix(rotation_matrix @ current_rotation.as_matrix())
        self.origin.rpy = new_rotation.as_euler("xyz").tolist()

    def to_xml(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the visual properties to an XML element.

        Args:
            root: The root element to append the visual properties to.

        Returns:
            The XML element representing the visual properties.

        Examples:
            >>> visual = VisualLink(
            ...     name="link_visual",
            ...     origin=Origin.zero_origin(),
            ...     geometry=BoxGeometry(size=(1.0, 1.0, 1.0)),
            ...     material=Material.from_color("red", Colors.RED)
            ... )
            >>> visual.to_xml()
            <Element 'visual' at 0x...>
        """
        visual = ET.Element("visual") if root is None else ET.SubElement(root, "visual")
        if self.name:
            visual.set("name", self.name)
        self.origin.to_xml(visual)
        self.geometry.to_xml(visual)
        self.material.to_xml(visual)
        return visual

    def to_mjcf(self, root: ET.Element) -> None:
        """
        Convert the visual properties to an MuJoCo compatible XML element.

        Args:
            root: The root element to append the visual properties to.

        Returns:
            The XML element representing the visual properties.

        Examples:
            >>> visual = VisualLink(
            ...     name="link_visual",
            ...     origin=Origin.zero_origin(),
            ...     geometry=BoxGeometry(size=(1.0, 1.0, 1.0)),
            ...     material=Material.from_color("red", Colors.RED)
            ... )
            >>> visual.to_mjcf()
            <Element 'visual' at 0x...>
        """
        visual = root if root.tag == "geom" else ET.SubElement(root, "geom")
        if self.name:
            visual.set("name", self.name)
        # TODO: Parent body uses visual origin, these share the same?
        self.origin.to_mjcf(visual)

        if self.geometry:
            self.geometry.to_mjcf(visual)

        self.material.to_mjcf(visual)

        visual.set("conaffinity", "0")
        visual.set("condim", "1")
        visual.set("contype", "0")
        visual.set("density", "0")
        visual.set("group", "1")

    @classmethod
    def from_xml(cls, xml: ET.Element) -> "VisualLink":
        """
        Create a visual link from an XML element.

        Args:
            xml: The XML element to create the visual link from.

        Returns:
            The visual link created from the XML element.

        Examples:
            >>> xml = ET.Element('visual')
            >>> VisualLink.from_xml(xml)
            VisualLink(name='visual', origin=None, geometry=None, material=None)
        """
        name = xml.get("name")
        origin_element = xml.find("origin")
        origin = Origin.from_xml(origin_element) if origin_element is not None else None

        geometry_element = xml.find("geometry")
        geometry = set_geometry_from_xml(geometry_element) if geometry_element is not None else None

        material_element = xml.find("material")
        material = Material.from_xml(material_element) if material_element is not None else None
        return cls(name=name, origin=origin, geometry=geometry, material=material)


@dataclass
class CollisionLink:
    """
    Represents the collision properties of a link in the robot model.

    This class defines the geometry used for collision detection in physics
    simulations, which may be different from the visual geometry.

    Attributes:
        name (Union[str, None]): Optional name identifier for the collision element.
        origin (Origin): The position and orientation of the collision geometry.
        geometry (BaseGeometry): The shape used for collision detection.
        friction (Optional[tuple[float, float, float]]): Optional friction coefficients
            (static, dynamic, rolling).

    Methods:
        to_xml: Converts the collision properties to an XML element.
        to_mjcf: Converts the collision properties to a MuJoCo compatible XML element.
        transform: Applies a transformation matrix to the collision geometry's origin.

    Class Methods:
        from_xml: Creates a CollisionLink from an XML element.

    Examples:
        >>> collision = CollisionLink(
        ...     name="link_collision",
        ...     origin=Origin.zero_origin(),
        ...     geometry=BoxGeometry(size=(1.0, 1.0, 1.0))
        ... )
        >>> collision.to_xml()
        <Element 'collision' at 0x...>
    """

    name: Union[str, None]
    origin: Origin
    geometry: BaseGeometry

    friction: Optional[tuple[float, float, float]] = None

    def transform(self, transformation_matrix: np.ndarray) -> None:
        """
        Apply a transformation to the visual link's origin.

        Args:
            transformation_matrix (np.ndarray): A 4x4 transformation matrix (homogeneous).
        """
        # Apply translation and rotation to the origin position
        pos = np.array([self.origin.xyz[0], self.origin.xyz[1], self.origin.xyz[2], 1])
        new_pos = transformation_matrix @ pos
        self.origin.xyz = tuple(new_pos[:3])  # Update position

        # Extract the rotation from the transformation matrix
        rotation_matrix = transformation_matrix[:3, :3]
        current_rotation = R.from_euler("xyz", self.origin.rpy)
        new_rotation = R.from_matrix(rotation_matrix @ current_rotation.as_matrix())
        self.origin.rpy = new_rotation.as_euler("xyz").tolist()

    def to_xml(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the collision properties to an XML element.

        Args:
            root: The root element to append the collision properties to.

        Returns:
            The XML element representing the collision properties.

        Examples:
            >>> collision = CollisionLink(origin=Origin(...), geometry=BoxGeometry(...))
            >>> collision.to_xml()
            <Element 'collision' at 0x7f8b3c0b4c70>
        """
        collision = ET.Element("collision") if root is None else ET.SubElement(root, "collision")
        if self.name:
            collision.set("name", self.name)
        self.origin.to_xml(collision)
        self.geometry.to_xml(collision)
        return collision

    def to_mjcf(self, root: ET.Element) -> None:
        """
        Convert the collision properties to an MuJoCo compatible XML element.

        Example XML:
        ```xml
              <geom name="Assembly-2-1-SUB-Part-5-1-collision"
                    pos="0.0994445 -0.000366963 0.0171076"
                    quat="-0.92388 -4.28774e-08 0.382683 0"
                    type="mesh"
                    rgba="1 0.5 0 1"
                    mesh="Assembly-2-1-SUB-Part-5-1"
                    contype="1"
                    conaffinity="0"
                    density="0"
                    group="1"/>
        ```
        Args:
            root: The root element to append the collision properties to.

        Returns:
            The XML element representing the collision properties.

        Examples:
            >>> collision = CollisionLink(origin=Origin(...), geometry=BoxGeometry(...))
            >>> collision.to_mjcf()
            <Element 'collision' at 0x7f8b3c0b4c70>
        """
        collision = root if root.tag == "geom" else ET.SubElement(root, "geom")
        if self.name:
            collision.set("name", self.name)
        collision.set("contype", "1")
        collision.set("conaffinity", "1")
        self.origin.to_mjcf(collision)

        if self.geometry:
            self.geometry.to_mjcf(collision)

        collision.set("group", "0")

        if self.friction:
            collision.set("friction", " ".join(format_number(v) for v in self.friction))

    @classmethod
    def from_xml(cls, xml: ET.Element) -> "CollisionLink":
        """
        Create a collision link from an XML element.

        Args:
            xml: The XML element to create the collision link from.

        Returns:
            The collision link created from the XML element.

        Examples:
            >>> xml = ET.Element('collision')
            >>> CollisionLink.from_xml(xml)
            CollisionLink(name='collision', origin=None, geometry=None)
        """
        name = xml.get("name")

        origin_element = xml.find("origin")
        origin = Origin.from_xml(origin_element) if origin_element is not None else None

        geometry_element = xml.find("geometry")
        geometry = set_geometry_from_xml(geometry_element) if geometry_element is not None else None

        return cls(name=name, origin=origin, geometry=geometry)


@dataclass
class Link:
    """
    Represents a complete link in the robot model.

    A link is a rigid body in the robot model that can contain visual, collision,
    and inertial properties. Links are connected by joints to form the complete
    robot structure.

    Attributes:
        name (str): The unique identifier for the link.
        visual (VisualLink | None): Optional visual properties for rendering.
        collision (CollisionLink | None): Optional collision properties for physics simulation.
        inertial (InertialLink | None): Optional inertial properties for dynamics.

    Methods:
        to_xml: Converts the link to an XML element.
        to_mjcf: Converts the link to a MuJoCo compatible XML element.

    Class Methods:
        from_xml: Creates a Link from an XML element.

    Examples:
        >>> link = Link(
        ...     name="base_link",
        ...     visual=VisualLink(...),
        ...     collision=CollisionLink(...),
        ...     inertial=InertialLink(...)
        ... )
        >>> link.to_xml()
        <Element 'link' at 0x...>

        >>> xml_str = '''
        ...     <link name="base_link">
        ...         <visual>...</visual>
        ...         <collision>...</collision>
        ...         <inertial>...</inertial>
        ...     </link>
        ... '''
        >>> xml_element = ET.fromstring(xml_str)
        >>> link = Link.from_xml(xml_element)
    """

    name: str
    visual: VisualLink | None = None
    collision: CollisionLink | None = None
    inertial: InertialLink | None = None

    def to_xml(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the link to an XML element.

        Args:
            root: The root element to append the link to.

        Returns:
            The XML element representing the link.

        Examples:
            >>> link = Link(
            ...     name="base_link",
            ...     visual=VisualLink(...),
            ...     collision=CollisionLink(...),
            ...     inertial=InertialLink(...)
            ... )
            >>> link.to_xml()
            <Element 'link' at 0x...>
        """
        link = ET.Element("link") if root is None else ET.SubElement(root, "link")
        link.set("name", self.name)
        if self.visual is not None:
            self.visual.to_xml(link)
        if self.collision is not None:
            self.collision.to_xml(link)
        if self.inertial is not None:
            self.inertial.to_xml(link)
        return link

    def to_mjcf(self, root: Optional[ET.Element] = None) -> ET.Element:
        """
        Convert the link to an MuJoCo compatible XML element.

        Args:
            root: The root element to append the link to.

        Returns:
            The XML element representing the link.

        Examples:
            >>> link = Link(
            ...     name="base_link",
            ...     visual=VisualLink(...),
            ...     collision=CollisionLink(...),
            ...     inertial=InertialLink(...)
            ... )
            >>> link.to_mjcf()
            <Element 'link' at 0x...>
        """
        link = ET.Element("body") if root is None else ET.SubElement(root, "body")
        link.set("name", self.name)

        if self.visual:
            link.set("pos", " ".join(map(str, self.visual.origin.xyz)))
            link.set("euler", " ".join(map(str, self.visual.origin.rpy)))

        if self.collision:
            self.collision.to_mjcf(link)

        if self.visual:
            self.visual.to_mjcf(link)

        if self.inertial:
            self.inertial.to_mjcf(link)

        return link

    @classmethod
    def from_xml(cls, xml: ET.Element) -> "Link":
        """
        Create a link from an XML element.

        Args:
            xml: The XML element to create the link from.

        Returns:
            The link created from the XML element.

        Examples:
            >>> xml = ET.Element('link')
            >>> Link.from_xml(xml)
            Link(name='link', visual=None, collision=None, inertial=None)
        """
        name = xml.get("name")

        visual_element = xml.find("visual")
        visual = VisualLink.from_xml(visual_element) if visual_element is not None else None

        collision_element = xml.find("collision")
        collision = CollisionLink.from_xml(collision_element) if collision_element is not None else None

        inertial_element = xml.find("inertial")
        inertial = InertialLink.from_xml(inertial_element) if inertial_element is not None else None

        return cls(name=name, visual=visual, collision=collision, inertial=inertial)

    # TODO: Implement from part method
    # @classmethod
    # def from_part(cls, part: Part) -> "Link":
    #     """
    #     Create a link from a part.

    #     Args:
    #         part: The part to create the link from.

    #     Returns:
    #         The link created from the part.

    #     Examples:
    #         >>> part = Part(...)
    #         >>> Link.from_part(part)
    #         Link(name='partId', visual=None, collision=None, inertial=None)
    #     """
    #     _cls = cls(name=part.partId)
    #     return _cls


if __name__ == "__main__":
    origin = Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0))
    print(origin.quat())
