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

    Attributes:
        RED (tuple[float, float, float]): Color red.
        GREEN (tuple[float, float, float]): Color green.
        BLUE (tuple[float, float, float]): Color blue.
        YELLOW (tuple[float, float, float]): Color yellow.
        CYAN (tuple[float, float, float]): Color cyan.
        MAGENTA (tuple[float, float, float]): Color magenta.
        WHITE (tuple[float, float, float]): Color white.
        BLACK (tuple[float, float, float]): Color black.
        ORANGE (tuple[float, float, float]): Color orange.
        PINK (tuple[float, float, float]): Color pink.

    Examples:
        >>> Colors.RED
        <Colors.RED: (1.0, 0.0, 0.0)>
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
        to_xml: Converts the origin to an XML element.

    Class Methods:
        from_matrix: Creates an origin from a transformation matrix.
        zero_origin: Creates an origin at the origin (0, 0, 0) with no rotation.

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

        >>> Origin.zero_origin()
        Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0))
    """

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]

    def transform(self, matrix: np.matrix, inplace: bool = False) -> Union["Origin", None]:
        """
        Apply a transformation matrix to the origin.

        Args:
            matrix: The transformation matrix to apply to the origin.
            inplace: Whether to apply the transformation in place.

        Returns:
            The origin with the transformation applied.

        Examples:
            >>> origin = Origin(xyz=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.0))
            >>> matrix = np.eye(4)
            >>> origin.transform(matrix)
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
        Convert the origin to an MuJoCo compatible XML element.

        Args:
            root: The root element to append the origin to.

        Returns:
            The XML element representing the origin.

        Examples:
            >>> origin = Origin(xyz=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.0))
            >>> origin.to_mjcf()
            <Element 'origin' at 0x7f8b3c0b4c70>
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

    def quat(self, sequence: str = "xyz") -> str:
        """
        Convert the origin to a quaternion.

        Args:
            sequence: The sequence of the Euler angles.

        Returns:
            The quaternion representing the origin.
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
    Represents the axis of a link in the robot model.

    Attributes:
        xyz (tuple[float, float, float]): The x, y, z coordinates of the axis.

    Methods:
        to_xml: Converts the axis to an XML element.

    Examples:
        >>> axis = Axis(xyz=(1.0, 0.0, 0.0))
        >>> axis.to_xml()
        <Element 'axis' at 0x7f8b3c0b4c70>
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

    Attributes:
        ixx (float): The inertia tensor element Ixx.
        iyy (float): The inertia tensor element Iyy.
        izz (float): The inertia tensor element Izz.
        ixy (float): The inertia tensor element Ixy.
        ixz (float): The inertia tensor element Ixz.
        iyz (float): The inertia tensor element Iyz.

    Methods:
        to_xml: Converts the inertia tensor to an XML element.

    Examples:
        >>> inertia = Inertia(ixx=1.0, iyy=2.0, izz=3.0, ixy=0.0, ixz=0.0, iyz=0.0)
        >>> inertia.to_xml()
        <Element 'inertia' at 0x7f8b3c0b4c70>
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
        return np.array([
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz]
        ])



@dataclass
class Material:
    """
    Represents the material properties of a link in the robot model.

    Attributes:
        name (str): The name of the material.
        color (tuple[float, float, float, float]): The color of the material in RGBA format.

    Methods:
        to_xml: Converts the material properties to an XML element.

    Class Methods:
        from_color: Creates a material from a color.

    Examples:
        >>> material = Material(name="material", color=(1.0, 0.0, 0.0, 1.0))
        >>> material.to_xml()
        <Element 'material' at 0x7f8b3c0b4c70>

        >>> Material.from_color(name="material", color=Colors.RED)
        Material(name='material', color=(1.0, 0.0, 0.0, 1.0))
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

    Attributes:
        mass (float): The mass of the link.
        inertia (Inertia): The inertia properties of the link.
        origin (Origin): The origin of the link.

    Methods:
        to_xml: Converts the inertial properties to an XML element.

    Examples:
        >>> inertial = InertialLink(mass=1.0, inertia=Inertia(...), origin=Origin(...))
        >>> inertial.to_xml()
        <Element 'inertial' at 0x7f8b3c0b4c70>
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
            >>> inertial = InertialLink(mass=1.0, inertia=Inertia(...), origin=Origin(...))
            >>> inertial.to_xml()
            <Element 'inertial' at 0x7f8b3c0b4c70>
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

    # Adding a method to transform (rotate and translate) an Inertia Matrix
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
            An analysis (on 100k runs) suggests that this is 3× faster than a direct approach on the tensor elements likely because numpy's libraries are optimized for matrix operations.
            Ref: https://chatgpt.com/share/6781b6ac-772c-8006-b1a9-7f2dc3e3ef4d
        """
        # Extract the rotation matrix R and translation vector d from T
        R = tf_matrix[:3, :3]  # Top-left 3x3 block is the rotation matrix
        p = tf_matrix[:3, 3]   # Top-right 3x1 block is the translation vector

        # Unpack the inertia tensor components
        # Example is ixx=1.0, iyy=2.0, izz=3.0, ixy=0.0, ixz=0.0, iyz=0.

        # Construct the original inertia matrix
        inertia_matrix = self.inertia.to_matrix

        # Rotate the inertia matrix
        I_rot = R @ inertia_matrix @ R.T

        # Compute the parallel axis theorem adjustment
        parallel_axis_adjustment = self.mass * (np.dot(p, p) * np.eye(3) - np.outer(p, p))

        # Final transformed inertia matrix
        I_transformed = I_rot + parallel_axis_adjustment

        # Extract the components of the transformed inertia tensor
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
            new_InertialLink = InertialLink(mass=self.mass, inertia=Inertia(ixx_prime, iyy_prime, izz_prime, ixy_prime, ixz_prime, iyz_prime), origin=Origin_prime)
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

    Attributes:
        origin (Origin): The origin of the link.
        geometry (BaseGeometry): The geometry of the link.
        material (Material): The material properties of the link.

    Methods:
        to_xml: Converts the visual properties to an XML element.

    Examples:
        >>> visual = VisualLink(origin=Origin(...), geometry=BoxGeometry(...), material=Material(...))
        >>> visual.to_xml()
        <Element 'visual' at 0x7f8b3c0b4c70>
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
            >>> visual = VisualLink(origin=Origin(...), geometry=BoxGeometry(...), material=Material(...))
            >>> visual.to_xml()
            <Element 'visual' at 0x7f8b3c0b4c70>
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
            >>> visual = VisualLink(origin=Origin(...), geometry=BoxGeometry(...), material=Material(...))
            >>> visual.to_mjcf()
            <Element 'visual' at 0x7f8b3c0b4c70>
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

    Attributes:
        origin (Origin): The origin of the link.
        geometry (BaseGeometry): The geometry of the link.

    Methods:
        to_xml: Converts the collision properties to an XML element.

    Examples:
        >>> collision = CollisionLink(origin=Origin(...), geometry=BoxGeometry(...))
        >>> collision.to_xml()
        <Element 'collision' at 0x7f8b3c0b4c70>
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
    Represents a link in the robot model.

    Attributes:
        name (str): The name of the link.
        visual (VisualLink): The visual properties of the link.
        collision (CollisionLink): The collision properties of the link.
        inertial (InertialLink): The inertial properties of the link.

    Methods:
        to_xml: Converts the link to an XML element.

    Class Methods:
        from_xml: Creates a link from an XML element.

    Examples:
        >>> link = Link(name="link", visual=VisualLink(...), collision=CollisionLink(...), inertial=InertialLink(...))
        >>> link.to_xml()
        <Element 'link' at 0x7f8b3c0b4c70>

        >>> part = Part(...)
        >>> Link.from_part(part)
        Link(name='partId', visual=None, collision=None, inertial=None)
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
            ...     name="link",
            ...     visual=VisualLink(...),
            ...     collision=CollisionLink(...),
            ...     inertial=InertialLink(...),
            ... )
            >>> link.to_xml()
            <Element 'link' at 0x7f8b3c0b4c70>
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
            ...     name="link",
            ...     visual=VisualLink(...),
            ...     collision=CollisionLink(...),
            ...     inertial=InertialLink(...),
            ... )
            >>> link.to_mjcf()
            <Element 'link' at 0x7f8b3c0b4c70>
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
