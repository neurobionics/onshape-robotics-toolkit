"""
Robot description format serializers and deserializers.

This module provides classes for converting Robot objects to/from various
file formats like URDF and MJCF (MuJoCo).

**Main Classes:**
    - URDFSerializer: Convert Robot → URDF XML
    - URDFDeserializer: Convert URDF XML → Robot
    - MJCFSerializer: Convert Robot → MJCF XML
    - MJCFDeserializer: Convert MJCF XML → Robot

**Example:**
    >>> from onshape_robotics_toolkit.formats import URDFSerializer, MJCFSerializer
    >>> from onshape_robotics_toolkit.robot import Robot
    >>>
    >>> # Generate robot from CAD
    >>> robot = Robot.from_graph(...)
    >>>
    >>> # Export to URDF
    >>> urdf = URDFSerializer()
    >>> urdf.save(robot, "robot.urdf", download_assets=True)
    >>>
    >>> # Export to MJCF
    >>> mjcf = MJCFSerializer()
    >>> mjcf.save(robot, "robot.xml", download_assets=True)
"""

from onshape_robotics_toolkit.formats.base import RobotDeserializer, RobotSerializer
from onshape_robotics_toolkit.formats.mjcf import MJCFConfig, MJCFSerializer
from onshape_robotics_toolkit.formats.urdf import URDFSerializer

__all__ = [
    "MJCFConfig",
    "MJCFSerializer",
    "RobotDeserializer",
    "RobotSerializer",
    "URDFSerializer",
]
