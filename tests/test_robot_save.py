"""Tests for serializer file extension handling and path logic."""

from __future__ import annotations

from unittest.mock import MagicMock, Mock

import pytest

from onshape_robotics_toolkit.formats import MJCFSerializer, URDFSerializer
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.robot import Robot


@pytest.fixture
def mock_kinematic_graph():
    """Create a minimal mock kinematic graph."""
    mock_graph = Mock(spec=KinematicGraph)
    mock_graph.nodes = {}
    mock_graph.edges = []
    mock_graph.root = None
    return mock_graph


@pytest.fixture
def mock_robot(mock_kinematic_graph):
    """Create a mock robot for testing."""
    robot = Robot(kinematic_graph=mock_kinematic_graph, name="test_robot")
    # Mock the nodes method to return empty iterator for serialization
    robot.nodes = MagicMock(return_value=[])
    robot.edges = []
    return robot


def test_urdf_serializer_saves_file(tmp_path, mock_robot):
    """URDFSerializer should save robot to .urdf file."""
    file_path = str(tmp_path / "robot.urdf")
    serializer = URDFSerializer()
    serializer.save(mock_robot, file_path, download_assets=False)

    # Check that file was created
    expected_path = tmp_path / "robot.urdf"
    assert expected_path.exists()


def test_mjcf_serializer_saves_file(tmp_path, mock_robot):
    """MJCFSerializer should save robot to .xml file."""
    file_path = str(tmp_path / "robot.xml")
    serializer = MJCFSerializer()
    serializer.save(mock_robot, file_path, download_assets=False)

    # Check that file was created
    expected_path = tmp_path / "robot.xml"
    assert expected_path.exists()


def test_urdf_serializer_creates_nested_directories(tmp_path, mock_robot):
    """URDFSerializer should create parent directories if they don't exist."""
    nested_path = tmp_path / "output" / "robots" / "robot.urdf"
    serializer = URDFSerializer()
    serializer.save(mock_robot, str(nested_path), download_assets=False)

    # Check that file was saved in nested directory
    assert nested_path.exists()


def test_mjcf_serializer_creates_nested_directories(tmp_path, mock_robot):
    """MJCFSerializer should create parent directories if they don't exist."""
    nested_path = tmp_path / "output" / "robots" / "robot.xml"
    serializer = MJCFSerializer()
    serializer.save(mock_robot, str(nested_path), download_assets=False)

    # Check that file was saved in nested directory
    assert nested_path.exists()


def test_urdf_serializer_with_custom_mesh_dir(tmp_path, mock_robot):
    """URDFSerializer should accept custom mesh directory."""
    file_path = str(tmp_path / "robot.urdf")
    mesh_dir = str(tmp_path / "custom_meshes")
    serializer = URDFSerializer()
    serializer.save(mock_robot, file_path, download_assets=False, mesh_dir=mesh_dir)

    # Check that file was created
    expected_path = tmp_path / "robot.urdf"
    assert expected_path.exists()


def test_mjcf_serializer_with_custom_mesh_dir(tmp_path, mock_robot):
    """MJCFSerializer should accept custom mesh directory."""
    file_path = str(tmp_path / "robot.xml")
    mesh_dir = str(tmp_path / "custom_meshes")
    serializer = MJCFSerializer()
    serializer.save(mock_robot, file_path, download_assets=False, mesh_dir=mesh_dir)

    # Check that file was created
    expected_path = tmp_path / "robot.xml"
    assert expected_path.exists()
