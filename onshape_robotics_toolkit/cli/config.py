"""Configuration management commands."""

import os
import subprocess
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.prompt import Confirm, Prompt
from rich.syntax import Syntax

from onshape_robotics_toolkit.cli.utils import handle_error, setup_cli_logging
from onshape_robotics_toolkit.config import ORTConfig

console = Console()
app = typer.Typer()


@app.command(name="init")
def init_config(
    output: Path = typer.Option("ORT.yaml", "--output", "-o", help="Output configuration file path"),
    force: bool = typer.Option(False, "--force", "-f", help="Overwrite existing file"),
) -> None:
    """
    Initialize a new configuration file interactively.

    This command will ask you a series of questions to set up your
    ORT.yaml configuration file with sensible defaults.
    """
    setup_cli_logging()

    # Check if file exists
    if (
        output.exists()
        and not force
        and not Confirm.ask(f"[yellow]Warning:[/yellow] {output} already exists. Overwrite?")
    ):
        console.print("[yellow]Cancelled[/yellow]")
        raise typer.Exit(0)

    console.print("\n[bold cyan]ORT Configuration Setup[/bold cyan]")
    console.print("[dim]Press Enter to use default values shown in brackets[/dim]\n")

    try:
        # Client configuration
        console.print("[bold]Client Configuration:[/bold]")
        env_path = Prompt.ask("Path to .env file", default=".env")

        # CAD configuration
        console.print("\n[bold]CAD Processing:[/bold]")
        max_depth = int(Prompt.ask("Default max depth (0=all rigid, higher=more flexible)", default="1"))

        # Kinematics configuration
        console.print("\n[bold]Kinematics:[/bold]")
        use_user_defined_root = Confirm.ask("Use user-defined root part from Onshape?", default=True)

        # Robot configuration
        console.print("\n[bold]Robot Generation:[/bold]")
        robot_name = Prompt.ask("Default robot name", default="robot")
        fetch_mass = Confirm.ask("Fetch mass properties by default?", default=True)

        # Export configuration
        console.print("\n[bold]Export Settings:[/bold]")
        mesh_dir = Prompt.ask("Default mesh directory", default="meshes")
        download_assets = Confirm.ask("Download STL assets by default?", default=True)

        # Create configuration
        from onshape_robotics_toolkit.config import (
            CADConfig,
            ClientConfig,
            ExportConfig,
            KinematicsConfig,
            LoggingConfig,
            RobotBuildConfig,
        )

        config = ORTConfig(
            logging=LoggingConfig(mode="default", console_level="INFO", file_level="DEBUG"),
            client=ClientConfig(env=env_path),
            cad=CADConfig(max_depth=max_depth),
            kinematics=KinematicsConfig(use_user_defined_root=use_user_defined_root),
            robot=RobotBuildConfig(name=robot_name, fetch_mass_properties=fetch_mass),
            export=ExportConfig(mesh_dir=mesh_dir, download_assets=download_assets),
        )

        # Save configuration
        config.save(output)

        console.print(f"\n[green]✓[/green] Configuration saved to [cyan]{output}[/cyan]")
        console.print("\n[dim]You can now use this config with:[/dim]")
        console.print(f"[dim]  ort export <URL> output.urdf --config {output}[/dim]")

    except KeyboardInterrupt:
        console.print("\n[yellow]Cancelled[/yellow]")
        raise typer.Exit(0)
    except Exception as e:
        handle_error(e, verbose=True)
        raise typer.Exit(1)


@app.command(name="validate")
def validate_config(
    config_path: Path = typer.Argument("ORT.yaml", help="Path to configuration file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose output"),
) -> None:
    """Validate an existing configuration file."""
    setup_cli_logging(verbose=verbose)

    if not config_path.exists():
        console.print(f"[red]Error:[/red] Configuration file not found: {config_path}")
        raise typer.Exit(1)

    try:
        console.print(f"[bold]Validating configuration:[/bold] {config_path}\n")

        # Try to load the configuration
        config = ORTConfig.load(config_path)

        # Validation checks
        issues = []
        warnings = []

        # Check client configuration
        if config.client:
            env_path = config.client.env or ".env"
            if not Path(env_path).exists():
                issues.append(f".env file not found: {env_path}")
            else:
                # Check for required keys
                import dotenv

                env_vars = dotenv.dotenv_values(env_path)
                if "ONSHAPE_ACCESS_KEY" not in env_vars:
                    issues.append(f"Missing ONSHAPE_ACCESS_KEY in {env_path}")
                if "ONSHAPE_SECRET_KEY" not in env_vars:
                    issues.append(f"Missing ONSHAPE_SECRET_KEY in {env_path}")

        # Check export configuration
        if config.export and config.export.mesh_dir:
            mesh_path = Path(config.export.mesh_dir)
            if not mesh_path.exists():
                warnings.append(f"Mesh directory will be created: {mesh_path}")

        # Print results
        if issues:
            console.print("[red]✗ Validation failed[/red]\n")
            console.print("[bold]Issues:[/bold]")
            for issue in issues:
                console.print(f"  [red]✗[/red] {issue}")
            if warnings:
                console.print("\n[bold]Warnings:[/bold]")
                for warning in warnings:
                    console.print(f"  [yellow]![/yellow] {warning}")
            raise typer.Exit(1)
        else:
            console.print("[green]✓ Configuration is valid[/green]")
            if warnings:
                console.print("\n[bold]Warnings:[/bold]")
                for warning in warnings:
                    console.print(f"  [yellow]![/yellow] {warning}")

            # Print summary
            console.print("\n[bold]Configuration Summary:[/bold]")
            if config.client:
                console.print(f"  .env path: [cyan]{config.client.env or '.env'}[/cyan]")
            if config.cad:
                console.print(f"  Max depth: [cyan]{config.cad.max_depth}[/cyan]")
            if config.robot:
                console.print(f"  Robot name: [cyan]{config.robot.name}[/cyan]")
            if config.export:
                console.print(f"  Mesh dir: [cyan]{config.export.mesh_dir or 'meshes'}[/cyan]")

    except Exception as e:
        console.print(f"\n[red]✗ Validation failed[/red]: {e!s}")
        if verbose:
            console.print_exception()
        raise typer.Exit(1)


@app.command(name="show")
def show_config(
    config_path: Path = typer.Argument("ORT.yaml", help="Path to configuration file"),
    no_color: bool = typer.Option(False, "--no-color", help="Disable syntax highlighting"),
) -> None:
    """Display configuration file with syntax highlighting."""
    setup_cli_logging()

    if not config_path.exists():
        console.print(f"[red]Error:[/red] Configuration file not found: {config_path}")
        raise typer.Exit(1)

    try:
        with open(config_path) as f:
            content = f.read()

        if no_color:
            console.print(content)
        else:
            syntax = Syntax(content, "yaml", theme="monokai", line_numbers=True)
            console.print(syntax)

    except Exception as e:
        handle_error(e, verbose=False)
        raise typer.Exit(1)


@app.command(name="edit")
def edit_config(
    config_path: Path = typer.Argument("ORT.yaml", help="Path to configuration file"),
    editor: Optional[str] = typer.Option(None, "--editor", "-e", help="Editor command (default: $EDITOR or notepad)"),
) -> None:
    """Open configuration file in your default editor."""
    setup_cli_logging()

    if not config_path.exists():
        console.print(f"[red]Error:[/red] Configuration file not found: {config_path}")
        console.print("\n[yellow]Tip:[/yellow] Create one with: ort config init")
        raise typer.Exit(1)

    try:
        # Determine editor
        editor_cmd = editor or os.getenv("EDITOR") or "notepad"

        console.print(f"[dim]Opening {config_path} with {editor_cmd}...[/dim]")

        # Open editor
        subprocess.run([editor_cmd, str(config_path)])  # noqa: S603

    except FileNotFoundError:
        console.print(f"[red]Error:[/red] Editor not found: {editor_cmd}")
        console.print("\n[yellow]Tip:[/yellow] Specify an editor with --editor <command>")
        raise typer.Exit(1)
    except Exception as e:
        handle_error(e, verbose=False)
        raise typer.Exit(1)
