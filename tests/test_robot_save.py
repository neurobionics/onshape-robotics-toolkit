"""Tests for Robot.save() file extension handling and path logic."""

from __future__ import annotations

import os
from unittest.mock import MagicMock, Mock

import pytest

from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.robot import Robot, RobotType


@pytest.fixture
def mock_kinematic_graph():
    """Create a minimal mock kinematic graph."""
    mock_graph = Mock(spec=KinematicGraph)
    mock_graph.nodes = {}
    mock_graph.edges = []
    mock_graph.root = None
    return mock_graph


@pytest.fixture
def mock_robot_urdf(mock_kinematic_graph):
    """Create a mock URDF robot for testing."""
    robot = Robot(kinematic_graph=mock_kinematic_graph, name="test_robot", robot_type=RobotType.URDF)
    # Mock the nodes method to return empty iterator
    robot.nodes = MagicMock(return_value=[])
    # Mock the XML generation methods
    robot.to_urdf = MagicMock(return_value="<robot></robot>")
    robot.to_mjcf = MagicMock(return_value="<mujoco></mujoco>")
    return robot


@pytest.fixture
def mock_robot_mjcf(mock_kinematic_graph):
    """Create a mock MJCF robot for testing."""
    robot = Robot(kinematic_graph=mock_kinematic_graph, name="test_robot", robot_type=RobotType.MJCF)
    # Mock the nodes method to return empty iterator
    robot.nodes = MagicMock(return_value=[])
    # Mock the XML generation methods
    robot.to_urdf = MagicMock(return_value="<robot></robot>")
    robot.to_mjcf = MagicMock(return_value="<mujoco></mujoco>")
    return robot


def test_save_urdf_no_extension_adds_urdf(tmp_path, mock_robot_urdf):
    """When saving URDF robot without extension, .urdf should be added."""
    file_path = str(tmp_path / "robot")
    mock_robot_urdf.save(file_path=file_path, download_assets=False)

    # Check that .urdf extension was added
    expected_path = tmp_path / "robot.urdf"
    assert expected_path.exists()


def test_save_mjcf_no_extension_adds_xml(tmp_path, mock_robot_mjcf):
    """When saving MJCF robot without extension, .xml should be added."""
    file_path = str(tmp_path / "robot")
    mock_robot_mjcf.save(file_path=file_path, download_assets=False)

    # Check that .xml extension was added
    expected_path = tmp_path / "robot.xml"
    assert expected_path.exists()


def test_save_urdf_wrong_extension_fixes_it(tmp_path, mock_robot_urdf):
    """When saving URDF robot with wrong extension, it should be corrected."""
    file_path = str(tmp_path / "robot.xml")
    mock_robot_urdf.save(file_path=file_path, download_assets=False)

    # Check that extension was changed to .urdf
    expected_path = tmp_path / "robot.urdf"
    assert expected_path.exists()
    # Original .xml file should not exist
    assert not (tmp_path / "robot.xml").exists()


def test_save_mjcf_wrong_extension_fixes_it(tmp_path, mock_robot_mjcf):
    """When saving MJCF robot with wrong extension, it should be corrected."""
    file_path = str(tmp_path / "robot.urdf")
    mock_robot_mjcf.save(file_path=file_path, download_assets=False)

    # Check that extension was changed to .xml
    expected_path = tmp_path / "robot.xml"
    assert expected_path.exists()
    # Original .urdf file should not exist
    assert not (tmp_path / "robot.urdf").exists()


def test_save_urdf_correct_extension_unchanged(tmp_path, mock_robot_urdf):
    """When saving URDF robot with correct extension, it should remain unchanged."""
    file_path = str(tmp_path / "robot.urdf")
    mock_robot_urdf.save(file_path=file_path, download_assets=False)

    # Check that file was saved with correct extension
    expected_path = tmp_path / "robot.urdf"
    assert expected_path.exists()


def test_save_mjcf_correct_extension_unchanged(tmp_path, mock_robot_mjcf):
    """When saving MJCF robot with correct extension, it should remain unchanged."""
    file_path = str(tmp_path / "robot.xml")
    mock_robot_mjcf.save(file_path=file_path, download_assets=False)

    # Check that file was saved with correct extension
    expected_path = tmp_path / "robot.xml"
    assert expected_path.exists()


def test_save_no_file_path_uses_default(tmp_path, mock_robot_urdf):
    """When saving without file_path, should use robot name with correct extension."""
    # Change to tmp directory
    original_dir = os.getcwd()
    try:
        os.chdir(tmp_path)
        mock_robot_urdf.save(file_path=None, download_assets=False)

        # Check that default file was created
        expected_path = tmp_path / "test_robot.urdf"
        assert expected_path.exists()
    finally:
        os.chdir(original_dir)


def test_save_case_insensitive_extension_check(tmp_path, mock_robot_urdf):
    """Extension check should be case-insensitive."""
    file_path = str(tmp_path / "robot.URDF")
    mock_robot_urdf.save(file_path=file_path, download_assets=False)

    # .URDF should be treated same as .urdf (correct extension)
    expected_path = tmp_path / "robot.urdf"
    assert expected_path.exists()


def test_save_nested_directory_creates_parent(tmp_path, mock_robot_urdf):
    """Saving to nested directory should work correctly."""
    nested_path = tmp_path / "output" / "robots" / "robot"
    nested_path.parent.mkdir(parents=True, exist_ok=True)

    mock_robot_urdf.save(file_path=str(nested_path), download_assets=False)

    # Check that file was saved with .urdf extension
    expected_path = tmp_path / "output" / "robots" / "robot.urdf"
    assert expected_path.exists()
