import xml.etree.ElementTree as ET
from dataclasses import dataclass
from enum import Enum

import numpy as np
from scipy.spatial.transform import Rotation

from onshape_api.models.geometry import BaseGeometry
from onshape_api.utilities import format_number


class COLORS(tuple[float, float, float], Enum):
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
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        origin = ET.Element("origin") if root is None else ET.SubElement(root, "origin")
        origin.set("xyz", " ".join(format_number(v) for v in self.xyz))
        origin.set("rpy", " ".join(format_number(v) for v in self.rpy))
        return origin

    @classmethod
    def from_matrix(cls, matrix: np.matrix):
        x = float(matrix[0, 3])
        y = float(matrix[1, 3])
        z = float(matrix[2, 3])
        roll, pitch, yaw = Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")
        return cls((x, y, z), (roll, pitch, yaw))

    @classmethod
    def zero_origin(cls):
        return cls((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


@dataclass
class Axis:
    xyz: tuple[float, float, float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        axis = ET.Element("axis") if root is None else ET.SubElement(root, "axis")
        axis.set("xyz", " ".join(format_number(v) for v in self.xyz))
        return axis


@dataclass
class Inertia:
    ixx: float
    iyy: float
    izz: float
    ixy: float
    ixz: float
    iyz: float

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        inertia = ET.Element("inertia") if root is None else ET.SubElement(root, "inertia")
        inertia.set("ixx", format_number(self.ixx))
        inertia.set("iyy", format_number(self.iyy))
        inertia.set("izz", format_number(self.izz))
        inertia.set("ixy", format_number(self.ixy))
        inertia.set("ixz", format_number(self.ixz))
        inertia.set("iyz", format_number(self.iyz))
        return inertia


@dataclass
class InertialLink:
    mass: float
    inertia: Inertia
    origin: Origin

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        inertial = ET.Element("inertial") if root is None else ET.SubElement(root, "inertial")
        ET.SubElement(inertial, "mass", value=format_number(self.mass))
        self.inertia.to_xml(inertial)
        self.origin.to_xml(inertial)
        return inertial


@dataclass
class Material:
    name: str
    color: tuple[float, float, float, float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        material = ET.Element("material") if root is None else ET.SubElement(root, "material")
        material.set("name", self.name)
        ET.SubElement(material, "color", rgba=" ".join(format_number(v) for v in self.color))
        return material

    @classmethod
    def from_color(cls, name: str, color: tuple[float, float, float, float]) -> "Material":
        return cls(name, color)


@dataclass
class VisualLink:
    origin: Origin
    geometry: BaseGeometry
    material: Material

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        visual = ET.Element("visual") if root is None else ET.SubElement(root, "visual")
        self.origin.to_xml(visual)
        self.geometry.to_xml(visual)
        self.material.to_xml(visual)
        return visual


@dataclass
class CollisionLink:
    origin: Origin
    geometry: BaseGeometry

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        collision = ET.Element("collision") if root is None else ET.SubElement(root, "collision")
        self.origin.to_xml(collision)
        self.geometry.to_xml(collision)
        return collision


@dataclass
class Link:
    name: str
    visual: VisualLink | None = None
    collision: CollisionLink | None = None
    inertial: InertialLink | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        link = ET.Element("link") if root is None else ET.SubElement(root, "link")
        link.set("name", self.name)
        if self.visual is not None:
            self.visual.to_xml(link)
        if self.collision is not None:
            self.collision.to_xml(link)
        if self.inertial is not None:
            self.inertial.to_xml(link)
        return link

    @classmethod
    def from_part(cls, part):
        _cls = cls(name=part.partId)
        return _cls
