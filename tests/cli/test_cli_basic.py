"""Basic CLI tests."""

import pytest
from typer.testing import CliRunner

from onshape_robotics_toolkit.cli.main import app

runner = CliRunner()


def test_version_command():
    """Test the version command."""
    result = runner.invoke(app, ["version"])
    assert result.exit_code == 0
    assert "Onshape Robotics Toolkit" in result.stdout
    assert "version" in result.stdout


def test_help_command():
    """Test the help command."""
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "Onshape Robotics Toolkit" in result.stdout
    assert "export" in result.stdout


def test_export_help():
    """Test export command help."""
    result = runner.invoke(app, ["export", "--help"])
    assert result.exit_code == 0
    assert "Export" in result.stdout
    assert "URL" in result.stdout or "url" in result.stdout


def test_var_help():
    """Test var subcommand help."""
    result = runner.invoke(app, ["var", "--help"])
    assert result.exit_code == 0
    assert "Variable" in result.stdout or "variable" in result.stdout


def test_assembly_help():
    """Test assembly subcommand help."""
    result = runner.invoke(app, ["assembly", "--help"])
    assert result.exit_code == 0
    assert "Assembly" in result.stdout or "assembly" in result.stdout


def test_graph_help():
    """Test graph subcommand help."""
    result = runner.invoke(app, ["graph", "--help"])
    assert result.exit_code == 0
    assert "graph" in result.stdout or "kinematic" in result.stdout


def test_robot_help():
    """Test robot subcommand help."""
    result = runner.invoke(app, ["robot", "--help"])
    assert result.exit_code == 0
    assert "Robot" in result.stdout or "robot" in result.stdout


def test_config_help():
    """Test config subcommand help."""
    result = runner.invoke(app, ["config", "--help"])
    assert result.exit_code == 0
    assert "Configuration" in result.stdout or "config" in result.stdout


def test_export_missing_arguments():
    """Test export command with missing arguments."""
    result = runner.invoke(app, ["export"])
    assert result.exit_code != 0  # Should fail without arguments


def test_invalid_url():
    """Test with an invalid URL format."""
    result = runner.invoke(app, ["export", "invalid-url", "output.urdf"])
    assert result.exit_code != 0  # Should fail with invalid URL


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
