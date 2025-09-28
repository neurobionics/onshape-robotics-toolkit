"""
This module contains classes for representing geometry in Onshape.

Class:
    - **BaseGeometry**: Abstract base class for geometry objects.
    - **BoxGeometry**: Represents a box geometry.
    - **CylinderGeometry**: Represents a cylinder geometry.
    - **SphereGeometry**: Represents a sphere geometry.
    - **MeshGeometry**: Represents a mesh geometry.
"""

import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from lxml.etree import Element, SubElement, _Element

from onshape_robotics_toolkit.utilities import format_number, xml_escape


class GeometryType(str, Enum):
    """
    Enumerates the possible geometry types in Onshape.

    Attributes:
        BOX (str): Box geometry.
        CYLINDER (str): Cylinder geometry.
        SPHERE (str): Sphere geometry.
        MESH (str): Mesh geometry.

    Examples:
        >>> GeometryType.BOX
        'BOX'
        >>> GeometryType.CYLINDER
        'CYLINDER'
    """

    BOX = "box"
    CYLINDER = "cylinder"
    SPHERE = "sphere"
    MESH = "mesh"


@dataclass
class BaseGeometry(ABC):
    """
    Abstract base class for geometry objects.

    Abstract Methods:
        to_xml: Converts the geometry object to an XML element.
    """

    @abstractmethod
    def to_xml(self, root: Optional[_Element] = None) -> _Element: ...

    @abstractmethod
    def to_mjcf(self, root: _Element) -> None: ...

    @classmethod
    @abstractmethod
    def from_xml(cls, element: _Element) -> "BaseGeometry": ...

    @property
    @abstractmethod
    def geometry_type(self) -> str: ...


@dataclass
class BoxGeometry(BaseGeometry):
    """
    Represents a box geometry.

    Attributes:
        size (tuple[float, float, float]): The size of the box in the x, y, and z dimensions.

    Methods:
        to_xml: Converts the box geometry to an XML element.

    Examples:
        >>> box = BoxGeometry(size=(1.0, 2.0, 3.0))
        >>> box.to_xml()
        <Element 'geometry' at 0x7f8b3c0b4c70>
    """

    size: tuple[float, float, float]

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the box geometry to an XML element.

        Args:
            root: The root element to append the box geometry to.

        Returns:
            The XML element representing the box geometry.

        Examples:
            >>> box = BoxGeometry(size=(1.0, 2.0, 3.0))
            >>> box.to_xml()
            <Element 'geometry' at 0x7f8b3c0b4c70>
        """
        geometry = Element("geometry") if root is None else SubElement(root, "geometry")
        SubElement(geometry, "box", size=" ".join(format_number(v) for v in self.size))
        return geometry

    def to_mjcf(self, root: _Element) -> None:
        """
        Convert the box geometry to an MJCF element.

        Args:
            root: The root element to append the box geometry to.

        Returns:
            The MJCF element representing the box geometry.

        Examples:
            >>> box = BoxGeometry(size=(1.0, 2.0, 3.0))
            >>> box.to_mjcf()
            <Element 'geom' at 0x7f8b3c0b4c70>
        """
        geom = root if root.tag == "geom" else SubElement(root, "geom")
        geom.set("type", GeometryType.BOX)
        geom.set("size", " ".join(format_number(v) for v in self.size))

    @classmethod
    def from_xml(cls, element: _Element) -> "BoxGeometry":
        """
        Create a box geometry from an XML element.

        Args:
            element: The XML element to create the box geometry from.

        Returns:
            The box geometry created from the XML element.

        Examples:
            >>> element = Element("geometry")
            >>> SubElement(element, "box", size="1.0 2.0 3.0")
            >>> BoxGeometry.from_xml(element)
            BoxGeometry(size=(1.0, 2.0, 3.0))
        """
        box_element = element.find("box")
        if box_element is None:
            raise ValueError("No box element found")
        size_values = [float(v) for v in box_element.attrib["size"].split()]
        if len(size_values) != 3:
            raise ValueError("Box size must have exactly 3 values")
        size = (size_values[0], size_values[1], size_values[2])
        return cls(size)

    @property
    def geometry_type(self) -> str:
        return GeometryType.BOX


@dataclass
class CylinderGeometry(BaseGeometry):
    """
    Represents a cylinder geometry.

    Attributes:
        radius (float): The radius of the cylinder.
        length (float): The length of the cylinder.

    Methods:
        to_xml: Converts the cylinder geometry to an XML element.

    Examples:
        >>> cylinder = CylinderGeometry(radius=1.0, length=2.0)
        >>> cylinder.to_xml()
        <Element 'geometry' at 0x7f8b3c0b4c70>
    """

    radius: float
    length: float

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the cylinder geometry to an XML element.

        Args:
            root: The root element to append the cylinder geometry to.

        Returns:
            The XML element representing the cylinder geometry.

        Examples:
            >>> cylinder = CylinderGeometry(radius=1.0, length=2.0)
            >>> cylinder.to_xml()
            <Element 'geometry' at 0x7f8b3c0b4c70>
        """
        geometry = Element("geometry") if root is None else SubElement(root, "geometry")
        SubElement(
            geometry,
            "cylinder",
            radius=format_number(self.radius),
            length=format_number(self.length),
        )
        return geometry

    def to_mjcf(self, root: _Element) -> None:
        """
        Convert the cylinder geometry to an MJCF element.

        Args:
            root: The root element to append the cylinder geometry to.

        Returns:
            The MJCF element representing the cylinder geometry.

        Examples:
            >>> cylinder = CylinderGeometry(radius=1.0, length=2.0)
            >>> cylinder.to_mjcf()
            <Element 'geom' at 0x7f8b3c0b4c70>
        """
        geom = root if root.tag == "geom" else SubElement(root, "geom")
        geom.set("type", GeometryType.CYLINDER)
        geom.set("size", f"{format_number(self.radius)} {format_number(self.length)}")

    @classmethod
    def from_xml(cls, element: _Element) -> "CylinderGeometry":
        """
        Create a cylinder geometry from an XML element.

        Args:
            element: The XML element to create the cylinder geometry from.

        Returns:
            The cylinder geometry created from the XML element.

        Examples:
            >>> element = Element("geometry")
            >>> SubElement(element, "cylinder", radius="1.0", length="2.0")
            >>> CylinderGeometry.from_xml(element)
            CylinderGeometry(radius=1.0, length=2.0)
        """
        cylinder_element = element.find("cylinder")
        if cylinder_element is None:
            raise ValueError("No cylinder element found")
        radius = float(cylinder_element.attrib["radius"])
        length = float(cylinder_element.attrib["length"])
        return cls(radius, length)

    @property
    def geometry_type(self) -> str:
        return GeometryType.CYLINDER


@dataclass
class SphereGeometry(BaseGeometry):
    """
    Represents a sphere geometry.

    Attributes:
        radius (float): The radius of the sphere.

    Methods:
        to_xml: Converts the sphere geometry to an XML element.

    Examples:
        >>> sphere = SphereGeometry(radius=1.0)
        >>> sphere.to_xml()
        <Element 'geometry' at 0x7f8b3c0b4c70>
    """

    radius: float

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the sphere geometry to an XML element.

        Args:
            root: The root element to append the sphere geometry to.

        Returns:
            The XML element representing the sphere geometry.

        Examples:
            >>> sphere = SphereGeometry(radius=1.0)
            >>> sphere.to_xml()
            <Element 'geometry' at 0x7f8b3c0b4c70>
        """
        geometry = Element("geometry") if root is None else SubElement(root, "geometry")
        SubElement(geometry, "sphere", radius=format_number(self.radius))
        return geometry

    def to_mjcf(self, root: _Element) -> None:
        """
        Convert the sphere geometry to an MJCF element.

        Args:
            root: The root element to append the sphere geometry to.

        Returns:
            The MJCF element representing the sphere geometry.

        Examples:
            >>> sphere = SphereGeometry(radius=1.0)
            >>> sphere.to_mjcf()
            <Element 'geom' at 0x7f8b3c0b4c70>
        """
        geom = root if root is not None and root.tag == "geom" else SubElement(root, "geom")
        geom.set("type", GeometryType.SPHERE)
        geom.set("size", format_number(self.radius))

    @classmethod
    def from_xml(cls, element: _Element) -> "SphereGeometry":
        """
        Create a sphere geometry from an XML element.

        Args:
            element: The XML element to create the sphere geometry from.

        Returns:
            The sphere geometry created from the XML element.

        Examples:
            >>> element = Element("geometry")
            >>> SubElement(element, "sphere", radius="1.0")
            >>> SphereGeometry.from_xml(element)
            SphereGeometry(radius=1.0)
        """
        sphere_element = element.find("sphere")
        if sphere_element is None:
            raise ValueError("No sphere element found")
        radius = float(sphere_element.attrib["radius"])
        return cls(radius)

    @property
    def geometry_type(self) -> str:
        return GeometryType.SPHERE


@dataclass
class MeshGeometry(BaseGeometry):
    """
    Represents a mesh geometry.

    Attributes:
        filename (str): The filename of the mesh.

    Methods:
        to_xml: Converts the mesh geometry to an XML element.

    Examples:
        >>> mesh = MeshGeometry(filename="mesh.stl")
        >>> mesh.to_xml()
        <Element 'geometry' at 0x7f8b3c0b4c70>
    """

    filename: str

    def to_xml(self, root: Optional[_Element] = None) -> _Element:
        """
        Convert the mesh geometry to an XML element.

        Args:
            root: The root element to append the mesh geometry to.

        Returns:
            The XML element representing the mesh geometry.

        Examples:
            >>> mesh = MeshGeometry(filename="mesh.stl")
            >>> mesh.to_xml()
            <Element 'geometry' at 0x7f8b3c0b4c70>
        """
        geometry = Element("geometry") if root is None else SubElement(root, "geometry")
        SubElement(geometry, "mesh", filename=self.filename)
        return geometry

    def to_mjcf(self, root: _Element) -> None:
        """
        Convert the mesh geometry to an MJCF element.

        Args:
            root: The root element to append the mesh geometry to.

        Returns:
            The MJCF element representing the mesh geometry.

        Examples:
            >>> mesh = MeshGeometry(filename="mesh.stl")
            >>> mesh.to_mjcf()
            <Element 'geom' at 0x7f8b3c0b4c70>
        """
        geom = root if root is not None and root.tag == "geom" else SubElement(root, "geom")
        geom.set("type", GeometryType.MESH)
        geom.set("mesh", self.mesh_name)

    @classmethod
    def from_xml(cls, element: _Element) -> "MeshGeometry":
        """
        Create a mesh geometry from an XML element.

        Args:
            element: The XML element to create the mesh geometry from.

        Returns:
            The mesh geometry created from the XML element.

        Examples:
            >>> element = Element("geometry")
            >>> SubElement(element, "mesh", filename="mesh.stl")
            >>> MeshGeometry.from_xml(element)
            MeshGeometry(filename="mesh.stl")
        """
        mesh_element = element.find("mesh")
        if mesh_element is None:
            raise ValueError("No mesh element found")
        filename = mesh_element.attrib["filename"]
        return cls(filename)

    def __post_init__(self) -> None:
        self.filename = xml_escape(self.filename)

    @property
    def geometry_type(self) -> str:
        return GeometryType.MESH

    @property
    def mesh_name(self) -> str:
        file_name_w_ext = os.path.basename(self.filename)
        return os.path.splitext(file_name_w_ext)[0]
