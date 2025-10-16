"""Utility functions for the CLI."""

import os
from pathlib import Path
from typing import Any, Optional

import typer
from loguru import logger
from rich.console import Console
from rich.table import Table

from onshape_robotics_toolkit.config import ORTConfig
from onshape_robotics_toolkit.utilities.helpers import setup_default_logging, setup_quiet_logging

console = Console()


def setup_cli_logging(verbose: bool = False, quiet: bool = False) -> None:
    """
    Configure logging based on CLI flags.

    Args:
        verbose: Enable DEBUG level logging
        quiet: Only log to file, no console output
    """
    if quiet:
        setup_quiet_logging(file_path="ORT.log", level="DEBUG")
    elif verbose:
        setup_default_logging(console_level="DEBUG", file_level="DEBUG", file_path="ORT.log")
    else:
        # In normal CLI mode, only show WARNING and above on console
        # INFO and DEBUG logs go to file only
        setup_default_logging(console_level="WARNING", file_level="DEBUG", file_path="ORT.log")


def load_config_or_defaults(config_path: Optional[Path] = None) -> Optional[ORTConfig]:
    """
    Load configuration from file or return None for defaults.

    Args:
        config_path: Path to configuration file (default: ORT.yaml)

    Returns:
        ORTConfig object or None
    """
    # Check environment variable
    if config_path is None:
        env_config = os.getenv("ORT_CONFIG")
        config_path = Path(env_config) if env_config else Path("ORT.yaml")

    if config_path and config_path.exists():
        try:
            config = ORTConfig.load(config_path)
            # Only show this in verbose mode (logger will handle it)
            logger.debug(f"Loaded configuration from {config_path}")
            return config
        except Exception as e:
            console.print(f"[yellow]Warning:[/yellow] Failed to load config: {e}")
            return None
    return None


def validate_url(url: str) -> str:
    """
    Validate an Onshape document URL.

    Args:
        url: Onshape document URL

    Returns:
        Validated URL

    Raises:
        typer.BadParameter: If URL is invalid
    """
    if not url.startswith("https://cad.onshape.com/documents/"):
        raise typer.BadParameter("URL must start with 'https://cad.onshape.com/documents/'")
    return url


def validate_env(env_path: Optional[str] = None) -> str:
    """
    Validate that .env file exists and contains required keys.

    Args:
        env_path: Path to .env file (default: .env)

    Returns:
        Path to .env file

    Raises:
        typer.Exit: If .env file is missing or invalid
    """
    if env_path is None:
        env_path = os.getenv("ORT_ENV", ".env")

    env_file = Path(env_path)
    if not env_file.exists():
        console.print(f"[red]Error:[/red] .env file not found at: {env_path}")
        console.print("\n[yellow]Tip:[/yellow] Create a .env file with your Onshape credentials:")
        console.print("  ONSHAPE_ACCESS_KEY=your_access_key")
        console.print("  ONSHAPE_SECRET_KEY=your_secret_key")
        console.print("\n[dim]Or run:[/dim] ort config init")
        raise typer.Exit(1)

    # Check for required keys
    import dotenv

    env_vars = dotenv.dotenv_values(env_path)
    if "ONSHAPE_ACCESS_KEY" not in env_vars or "ONSHAPE_SECRET_KEY" not in env_vars:
        console.print("[red]Error:[/red] .env file is missing required keys")
        console.print("\n[yellow]Required keys:[/yellow]")
        console.print("  ONSHAPE_ACCESS_KEY")
        console.print("  ONSHAPE_SECRET_KEY")
        raise typer.Exit(1)

    return env_path


def handle_error(e: Exception, verbose: bool = False) -> None:
    """
    Handle CLI errors with helpful messages.

    Args:
        e: Exception that was raised
        verbose: Show full traceback
    """
    if verbose:
        console.print_exception()
    else:
        console.print(f"\n[red]Error:[/red] {e!s}")

    # Provide context-specific help
    error_msg = str(e).lower()

    if "connection" in error_msg or "network" in error_msg:
        console.print("\n[yellow]Tip:[/yellow] Check your internet connection and Onshape API access")
    elif "credentials" in error_msg or "authentication" in error_msg:
        console.print("\n[yellow]Tip:[/yellow] Verify your Onshape credentials in .env file")
    elif "empty kinematic graph" in error_msg:
        console.print("\n[yellow]Tip:[/yellow] Mark at least one part as fixed in your Onshape assembly")
    elif "file not found" in error_msg or "no such file" in error_msg:
        console.print("\n[yellow]Tip:[/yellow] Check that all file paths are correct")

    console.print("\n[dim]Run with --verbose for full traceback[/dim]")


def create_table(title: str, columns: list[str]) -> Table:
    """
    Create a Rich table with standard styling.

    Args:
        title: Table title
        columns: Column names

    Returns:
        Rich Table object
    """
    table = Table(title=title, show_header=True, header_style="bold cyan")
    for col in columns:
        table.add_column(col)
    return table


def get_config_value(config: Optional[ORTConfig], key: str, default: Any = None) -> Any:
    """
    Get a value from config with fallback to environment variable and default.

    Args:
        config: ORTConfig object or None
        key: Configuration key (e.g., "cad.max_depth")
        default: Default value if not found

    Returns:
        Configuration value
    """
    # Check environment variable first
    env_key = f"ORT_{key.upper().replace('.', '_')}"
    env_value = os.getenv(env_key)
    if env_value is not None:
        # Try to convert to appropriate type
        if default is not None:
            if isinstance(default, bool):
                return env_value.lower() in ("true", "1", "yes")
            elif isinstance(default, int):
                return int(env_value)
            elif isinstance(default, float):
                return float(env_value)
        return env_value

    # Check config file
    if config is not None:
        parts = key.split(".")
        obj: Any = config
        for part in parts:
            if hasattr(obj, part):
                obj = getattr(obj, part)
            else:
                return default
        return obj if obj is not None else default

    return default
