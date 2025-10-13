"""Tests for lightweight Asset helpers without touching the network."""

from __future__ import annotations

import os

from onshape_robotics_toolkit import connect


def test_asset_absolute_path_creates_mesh_directory(tmp_path, monkeypatch) -> None:
    """Asset.absolute_path should create the mesh directory if it does not yet exist."""
    mesh_dir = "unit_test_meshes"
    monkeypatch.setattr(connect, "CURRENT_DIR", str(tmp_path))
    monkeypatch.setattr(connect, "MESHES_DIR", mesh_dir)

    asset = connect.Asset(file_name="part.stl")
    absolute_path = asset.absolute_path

    expected_dir = tmp_path / mesh_dir
    assert expected_dir.is_dir()
    assert absolute_path == os.path.join(str(expected_dir), "part.stl")

    # Relative path should use forward slashes for cross-platform compatibility
    expected_relative_path = os.path.relpath(absolute_path, str(tmp_path)).replace(os.sep, "/")
    assert asset.relative_path == expected_relative_path


def test_asset_custom_mesh_directory(tmp_path) -> None:
    """Asset should respect custom mesh_dir parameter."""
    custom_mesh_dir = str(tmp_path / "custom_meshes")

    asset = connect.Asset(file_name="part.stl", mesh_dir=custom_mesh_dir)
    absolute_path = asset.absolute_path

    # Custom mesh directory should be created
    assert os.path.exists(custom_mesh_dir)
    assert os.path.isdir(custom_mesh_dir)

    # Asset should use the custom directory
    assert absolute_path == os.path.join(custom_mesh_dir, "part.stl")


def test_asset_mesh_directory_can_be_updated(tmp_path, monkeypatch) -> None:
    """Asset mesh_dir attribute should be mutable for late binding."""
    default_mesh_dir = "default_meshes"
    monkeypatch.setattr(connect, "CURRENT_DIR", str(tmp_path))
    monkeypatch.setattr(connect, "MESHES_DIR", default_mesh_dir)

    # Create asset without custom mesh_dir
    asset = connect.Asset(file_name="part.stl")

    # Initially uses default
    initial_path = asset.absolute_path
    assert str(tmp_path / default_mesh_dir) in initial_path

    # Update mesh_dir
    custom_mesh_dir = str(tmp_path / "updated_meshes")
    asset.mesh_dir = custom_mesh_dir

    # Should now use updated directory
    updated_path = asset.absolute_path
    assert updated_path == os.path.join(custom_mesh_dir, "part.stl")
    assert os.path.exists(custom_mesh_dir)


def test_asset_backwards_compatibility_without_mesh_dir(tmp_path, monkeypatch) -> None:
    """Asset without mesh_dir parameter should use default behavior."""
    default_mesh_dir = "meshes"
    monkeypatch.setattr(connect, "CURRENT_DIR", str(tmp_path))
    monkeypatch.setattr(connect, "MESHES_DIR", default_mesh_dir)

    # Create asset without mesh_dir (backwards compatibility)
    asset = connect.Asset(file_name="part.stl")
    absolute_path = asset.absolute_path

    # Should use default CURRENT_DIR/MESHES_DIR
    expected_path = os.path.join(str(tmp_path), default_mesh_dir, "part.stl")
    assert absolute_path == expected_path


def test_asset_relative_path_respects_robot_file_dir(tmp_path, monkeypatch) -> None:
    """Asset relative_path should be calculated from robot_file_dir when set."""
    monkeypatch.setattr(connect, "CURRENT_DIR", str(tmp_path))

    # Create a custom mesh directory structure
    robot_dir = tmp_path / "output" / "robot"
    robot_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir = robot_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    # Create asset with custom mesh_dir
    asset = connect.Asset(file_name="part.stl", mesh_dir=str(mesh_dir))

    # Set robot_file_dir to the robot output directory
    asset.robot_file_dir = str(robot_dir)

    # Relative path should be "meshes/part.stl" (relative to robot_dir)
    relative_path = asset.relative_path
    assert relative_path == "meshes/part.stl"

    # Absolute path should still point to the correct location
    assert asset.absolute_path == os.path.join(str(mesh_dir), "part.stl")
