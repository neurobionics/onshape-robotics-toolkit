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
