"""
Abstract base classes for robot description serializers and deserializers.

This module defines the interfaces that all format-specific serializers
and deserializers must implement.
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    from onshape_robotics_toolkit.robot import Robot


class RobotSerializer(ABC):
    """
    Abstract base class for serializing Robot objects to various formats.

    Subclasses must implement serialize() to convert a Robot object into
    a string representation (e.g., URDF XML, MJCF XML).

    **Example Implementation:**
        >>> class MyFormatSerializer(RobotSerializer):
        ...     def serialize(self, robot: Robot, **options) -> str:
        ...         # Convert robot to my format
        ...         return xml_string
    """

    @abstractmethod
    def serialize(self, robot: "Robot", **options: Any) -> str:
        """
        Convert a Robot object to a string representation.

        Args:
            robot: The Robot object to serialize
            **options: Format-specific serialization options

        Returns:
            str: The serialized robot description (usually XML)

        Examples:
            >>> serializer = URDFSerializer()
            >>> xml_string = serializer.serialize(robot)
        """
        pass

    def save(
        self,
        robot: "Robot",
        file_path: str,
        download_assets: bool = True,
        mesh_dir: Optional[str] = None,
        **options: Any,
    ) -> None:
        """
        Serialize a Robot object and save it to a file.

        This method handles:
        1. Asset downloading (if requested)
        2. Serialization to string
        3. Writing to file with proper XML declaration

        Args:
            robot: The Robot object to save
            file_path: Path to the output file
            download_assets: Whether to download STL assets before saving
            mesh_dir: Optional custom directory for mesh files
            **options: Format-specific serialization options

        Examples:
            >>> serializer = URDFSerializer()
            >>> serializer.save(robot, "robot.urdf", download_assets=True)
        """
        import asyncio
        import os
        from pathlib import Path

        from loguru import logger

        from onshape_robotics_toolkit.config import record_export_config

        # Determine the mesh directory with smart defaults
        resolved_mesh_dir: Optional[str] = None
        if mesh_dir is not None:
            # User explicitly provided mesh_dir
            resolved_mesh_dir = mesh_dir
        elif file_path is not None:
            # Smart default: use file_path.parent / "meshes"
            file_parent = Path(file_path).parent
            resolved_mesh_dir = os.path.join(str(file_parent), "meshes")

        if download_assets:
            asyncio.run(robot._download_assets(resolved_mesh_dir))

        # Set robot_file_dir on all assets so relative paths in XML are correct
        robot_file_dir = str(Path(file_path).parent.absolute())
        for _node, data in robot.nodes(data=True):
            asset = data.get("asset")
            link_data = data.get("data")
            if asset:
                asset.robot_file_dir = robot_file_dir
                # Update the mesh paths in the link's geometry objects
                if (
                    link_data
                    and hasattr(link_data, "visual")
                    and link_data.visual
                    and hasattr(link_data.visual.geometry, "filename")
                ):
                    link_data.visual.geometry.filename = asset.relative_path
                if (
                    link_data
                    and hasattr(link_data, "collision")
                    and link_data.collision
                    and hasattr(link_data.collision.geometry, "filename")
                ):
                    link_data.collision.geometry.filename = asset.relative_path

        # Create parent directories if needed
        file_path_obj = Path(file_path)
        file_path_obj.parent.mkdir(parents=True, exist_ok=True)

        # Serialize and write
        xml_declaration = '<?xml version="1.0" ?>\n'
        xml_content = xml_declaration + self.serialize(robot, **options)

        with open(file_path, "w", encoding="utf-8") as f:
            f.write(xml_content)

        record_export_config(file_path=file_path, download_assets=download_assets, mesh_dir=mesh_dir)
        logger.info(f"Robot model saved to {os.path.abspath(file_path)}")


class RobotDeserializer(ABC):
    """
    Abstract base class for deserializing robot descriptions into Robot objects.

    Subclasses must implement deserialize() to convert a string representation
    (e.g., URDF XML, MJCF XML) into a Robot object.

    **Example Implementation:**
        >>> class MyFormatDeserializer(RobotDeserializer):
        ...     def deserialize(self, xml_string: str, **options) -> Robot:
        ...         # Parse xml_string and create Robot
        ...         return robot
    """

    @abstractmethod
    def deserialize(self, content: str, **options: Any) -> "Robot":
        """
        Convert a string representation to a Robot object.

        Args:
            content: The robot description content (usually XML)
            **options: Format-specific deserialization options

        Returns:
            Robot: The deserialized Robot object

        Examples:
            >>> deserializer = URDFDeserializer()
            >>> robot = deserializer.deserialize(urdf_xml_string)
        """
        pass

    def load(self, file_path: str, **options: Any) -> "Robot":
        """
        Load a robot description from a file and deserialize it.

        Args:
            file_path: Path to the robot description file
            **options: Format-specific deserialization options

        Returns:
            Robot: The deserialized Robot object

        Examples:
            >>> deserializer = URDFDeserializer()
            >>> robot = deserializer.load("robot.urdf")
        """
        with open(file_path, encoding="utf-8") as f:
            content = f.read()
        return self.deserialize(content, **options)
