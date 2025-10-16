"""Tests for config commands."""

import tempfile
from pathlib import Path

import pytest
from typer.testing import CliRunner

from onshape_robotics_toolkit.cli.main import app

runner = CliRunner()


def test_config_init_with_temp_file():
    """Test config init command with temporary file."""
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "test_config.yaml"

        # Run non-interactively with all defaults
        runner.invoke(app, ["config", "init", "--output", str(config_path)], input="\n" * 10)

        # Should create the file (exit code 0 or 1 depending on missing .env, but file should exist)
        assert config_path.exists(), "Config file should be created"

        # Verify content
        content = config_path.read_text()
        assert "client" in content or "cad" in content


def test_config_show_nonexistent():
    """Test config show with nonexistent file."""
    result = runner.invoke(app, ["config", "show", "nonexistent.yaml"])
    assert result.exit_code != 0
    assert "not found" in result.stdout or "Error" in result.stdout


def test_config_validate_nonexistent():
    """Test config validate with nonexistent file."""
    result = runner.invoke(app, ["config", "validate", "nonexistent.yaml"])
    assert result.exit_code != 0
    assert "not found" in result.stdout or "Error" in result.stdout


def test_config_edit_nonexistent():
    """Test config edit with nonexistent file."""
    result = runner.invoke(app, ["config", "edit", "nonexistent.yaml"])
    assert result.exit_code != 0
    assert "not found" in result.stdout or "Error" in result.stdout


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
