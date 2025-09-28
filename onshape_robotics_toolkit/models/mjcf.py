from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, cast

from lxml import etree as ET
from lxml.etree import _Element


@dataclass
class Light:
    """
    Represents a light source in a mujoco model.

    Example Light XML:
    ```xml
    <light directional="true" diffuse="0.4 0.4 0.4"
           specular="0.1 0.1 0.1" pos="0 0 5.0"
           dir="0 0 -1" castshadow="false" />
    ```

    Attributes:
        directional: Whether the light is directional.
        diffuse: The diffuse color of the light.
        specular: The specular color of the light.
        pos: The position of the light.
        dir: The direction of the light.
        castshadow: Whether the light casts shadows.
    """

    directional: bool
    diffuse: tuple[float, float, float]
    specular: tuple[float, float, float]
    pos: tuple[float, float, float]
    direction: tuple[float, float, float]
    castshadow: bool

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the light to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the light to.
        """
        light = cast(_Element, ET.Element("light") if root is None else ET.SubElement(root, "light"))
        light.set("directional", str(self.directional).lower())
        light.set("diffuse", " ".join(map(str, self.diffuse)))
        light.set("specular", " ".join(map(str, self.specular)))
        light.set("pos", " ".join(map(str, self.pos)))
        light.set("dir", " ".join(map(str, self.direction)))
        light.set("castshadow", str(self.castshadow).lower())


@dataclass
class Camera:
    """
    Represents a camera in a mujoco model.

    Example Camera XML:
    ```xml
    <camera name="track" mode="trackcom" pos="0 -1 0.25" xyaxes="1 0 0 0 0 1" />
    ```

    Attributes:
        name: The name of the camera.
        mode: The mode of the camera.
        pos: The position of the camera.
        xyaxes: The xyaxes of the camera.
    """

    name: str
    mode: str
    pos: tuple[float, float, float]
    xyaxes: tuple[float, float, float, float, float, float]

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the camera to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the camera to.
        """
        camera = cast(_Element, ET.Element("camera") if root is None else ET.SubElement(root, "camera"))
        camera.set("name", self.name)
        camera.set("mode", self.mode)
        camera.set("pos", " ".join(map(str, self.pos)))
        camera.set("xyaxes", " ".join(map(str, self.xyaxes)))


@dataclass
class Actuator:
    """
    Represents an actuator in a mujoco model.

    Example Actuator XML:
    ```xml
    <motor name="Revolute-1" joint="Revolute-1" ctrllimited="false" gear="70" />
    <motor name="Revolute-1" joint="Revolute-1" ctrllimited="true" ctrlrange="-1 1" gear="70" />
    ```

    Attributes:
        name: The name of the actuator.
        joint: The joint of the actuator.
        ctrllimited: Whether the actuator is control limited.
        ctrlrange: The control range of the actuator.
        gear: The gear of the actuator.
    """

    name: str
    joint: str
    ctrllimited: bool = False
    gear: float = 1.0
    ctrlrange: tuple[float, float] = (0, 0)

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the actuator to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the actuator to.
        """
        motor = cast(_Element, ET.Element("motor") if root is None else ET.SubElement(root, "motor"))
        motor.set("name", self.name)
        motor.set("joint", self.joint)
        motor.set("ctrllimited", str(self.ctrllimited).lower())

        if self.ctrllimited:
            motor.set("ctrlrange", " ".join(map(str, self.ctrlrange)))

        motor.set("gear", str(self.gear))


class Sensor(ABC):
    """
    Represents a sensor in a mujoco model.
    """

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def to_mjcf(self, root: _Element) -> None: ...


class IMU(Sensor):
    """
    Represents an IMU sensor in a mujoco model.
    """

    def __init__(
        self,
        name: str,
        objtype: str,
        objname: str,
        noise: Optional[float] = None,
        reftype: Optional[str] = None,
        refname: Optional[str] = None,
    ):
        super().__init__(name)
        self.objtype = objtype
        self.objname = objname
        self.noise = noise
        self.reftype = reftype
        self.refname = refname

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the IMU to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the IMU to.
        """
        framequat = cast(_Element, ET.Element("framequat") if root is None else ET.SubElement(root, "framequat"))
        framequat.set("name", self.name)
        framequat.set("objtype", self.objtype)
        framequat.set("objname", self.objname)

        if self.noise is not None:
            framequat.set("noise", str(self.noise))

        if self.reftype is not None and self.refname is not None:
            framequat.set("reftype", self.reftype)
            framequat.set("refname", self.refname)


class Gyro(Sensor):
    """
    Represents a gyro sensor in a mujoco model.
    """

    def __init__(self, name: str, site: str, noise: Optional[float] = None, cutoff: Optional[float] = None):
        super().__init__(name)
        self.site = site
        self.noise = noise
        self.cutoff = cutoff

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the gyro to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the gyro to.
        """
        gyro = cast(_Element, ET.Element("gyro") if root is None else ET.SubElement(root, "gyro"))
        gyro.set("name", self.name)
        gyro.set("site", self.site)

        if self.noise is not None:
            gyro.set("noise", str(self.noise))

        if self.cutoff is not None:
            gyro.set("cutoff", str(self.cutoff))


class Encoder(Sensor):
    """
    Represents an encoder sensor in a mujoco model.
    """

    def __init__(self, name: str, actuator: str, noise: Optional[float] = None):
        super().__init__(name)
        self.actuator = actuator
        self.noise = noise

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the encoder to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the encoder to.
        """
        encoder_pos = cast(_Element, ET.Element("actuatorpos") if root is None else ET.SubElement(root, "actuatorpos"))
        encoder_pos.set("name", self.name + "-pos")
        encoder_pos.set("actuator", self.actuator)
        if self.noise is not None:
            encoder_pos.set("noise", str(self.noise))

        encoder_vel = cast(_Element, ET.Element("actuatorvel") if root is None else ET.SubElement(root, "actuatorvel"))
        encoder_vel.set("name", self.name + "-vel")
        encoder_vel.set("actuator", self.actuator)
        if self.noise is not None:
            encoder_vel.set("noise", str(self.noise))


class ForceSensor(Sensor):
    """
    Represents a force sensor in a mujoco model.
    """

    def __init__(self, name: str, actuator: str, noise: Optional[float] = None):
        super().__init__(name)
        self.actuator = actuator
        self.noise = noise

    def to_mjcf(self, root: _Element) -> None:
        """
        Converts the force sensor to an XML element and appends it to the given root element.

        Args:
            root: The root element to append the force sensor to.
        """
        force_sensor = cast(_Element, ET.Element("actuatorfrc") if root is None else ET.SubElement(root, "actuatorfrc"))
        force_sensor.set("name", self.name)
        force_sensor.set("actuator", self.actuator)
        if self.noise is not None:
            force_sensor.set("noise", str(self.noise))
