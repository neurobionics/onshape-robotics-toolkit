"""Export command for the CLI."""

from enum import Enum
from pathlib import Path
from typing import Optional, Union

import typer
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, TimeElapsedColumn

from onshape_robotics_toolkit.cli.utils import (
    get_config_value,
    handle_error,
    load_config_or_defaults,
    setup_cli_logging,
    validate_env,
    validate_url,
)
from onshape_robotics_toolkit.config import record_session
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.formats.mjcf import MJCFConfig, MJCFSerializer
from onshape_robotics_toolkit.formats.urdf import URDFSerializer
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.parse import CAD

console = Console()


class ExportFormat(str, Enum):
    """Export format enum."""

    urdf = "urdf"
    mjcf = "mjcf"


def export_command(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    output: Path = typer.Argument(..., help="Output file path (e.g., robot.urdf or robot.xml)"),
    format: ExportFormat = typer.Option(
        ExportFormat.urdf, "--format", "-f", help="Export format: urdf or mjcf", case_sensitive=False
    ),
    name: Optional[str] = typer.Option(None, "--name", "-n", help="Robot name (default: derived from filename)"),
    max_depth: Optional[int] = typer.Option(
        None, "--max-depth", "-d", help="Maximum assembly depth (0=all rigid, higher=more flexible)"
    ),
    mesh_dir: Optional[str] = typer.Option(None, "--mesh-dir", "-m", help="Directory for mesh files"),
    download_assets: bool = typer.Option(True, "--download-assets/--no-download-assets", help="Download STL assets"),
    fetch_mass: bool = typer.Option(True, "--fetch-mass/--no-fetch-mass", help="Fetch mass properties"),
    use_user_root: bool = typer.Option(True, "--use-user-root/--no-use-user-root", help="Use user-defined root part"),
    position: Optional[tuple[float, float, float]] = typer.Option(
        None, "--position", "-p", help="Robot position (x y z), MJCF only"
    ),
    add_ground_plane: bool = typer.Option(
        False, "--ground-plane/--no-ground-plane", help="Add ground plane (MJCF only)"
    ),
    config_path: Optional[Path] = typer.Option(
        None, "--config", "-c", help="Path to ORT.yaml configuration file", exists=True
    ),
    env_path: Optional[str] = typer.Option(
        None, "--env", "-e", help="Path to .env file with Onshape credentials (default: .env)"
    ),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose (DEBUG) logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode (file logging only)"),
) -> None:
    """
    Export an Onshape assembly to URDF or MJCF format.

    This command performs the full pipeline:
    URL → Assembly → CAD → KinematicGraph → Robot → Export

    Examples:

        # Basic URDF export
        ort export <URL> robot.urdf

        # MJCF with custom options
        ort export <URL> robot.xml --format mjcf --max-depth 2 --ground-plane

        # With configuration file
        ort export <URL> robot.urdf --config my-config.yaml

    The export process automatically saves session configuration to ORT.yaml
    for reproducibility.
    """
    # Setup logging
    setup_cli_logging(verbose=verbose, quiet=quiet)

    # Load configuration
    config = load_config_or_defaults(config_path)

    # Resolve parameters with priority: CLI args > Config > Defaults
    resolved_name = name or (config and config.robot and config.robot.name) or output.stem
    resolved_max_depth = max_depth if max_depth is not None else get_config_value(config, "cad.max_depth", 1)
    resolved_mesh_dir = mesh_dir or get_config_value(config, "export.mesh_dir", "meshes")
    resolved_env_path = env_path or (config and config.client and config.client.env) or ".env"

    # Validate environment
    validate_env(resolved_env_path)

    if not quiet:
        console.print(f"\n[bold cyan]Exporting Onshape assembly to {format.value.upper()}[/bold cyan]")
        console.print(f"[dim]URL:[/dim] {url}")
        console.print(f"[dim]Output:[/dim] {output}")
        console.print(f"[dim]Name:[/dim] {resolved_name}")
        console.print(f"[dim]Max Depth:[/dim] {resolved_max_depth}")
        console.print(f"[dim]Mesh Dir:[/dim] {resolved_mesh_dir}\n")

    try:
        with (
            Progress(
                SpinnerColumn(),
                TextColumn("[progress.description]{task.description}"),
                TimeElapsedColumn(),
                console=console,
                transient=True,
            ) as progress,
            record_session(save_path="ORT.yaml"),
        ):
            # Step 1: Create client
            task = progress.add_task("Connecting to Onshape...", total=None)
            client = Client(env=resolved_env_path)
            progress.update(task, completed=True)

            # Step 2: Fetch assembly
            task = progress.add_task("Fetching assembly data...", total=None)
            from onshape_robotics_toolkit.models.document import Document

            doc = Document.from_url(url)
            assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)
            progress.update(task, completed=True)

            # Step 3: Process CAD
            task = progress.add_task("Processing CAD assembly...", total=None)
            cad = CAD.from_assembly(assembly, max_depth=resolved_max_depth, client=client)
            progress.update(task, completed=True)

            # Step 4: Build kinematic graph
            task = progress.add_task("Building kinematic graph...", total=None)
            kinematic_graph = KinematicGraph.from_cad(cad, use_user_defined_root=use_user_root)
            progress.update(task, completed=True)

            # Step 5: Generate robot
            task = progress.add_task("Generating robot model...", total=None)
            from onshape_robotics_toolkit.robot import Robot

            robot = Robot.from_graph(
                kinematic_graph=kinematic_graph, client=client, name=resolved_name, fetch_mass_properties=fetch_mass
            )
            progress.update(task, completed=True)

            # Step 6: Export
            task = progress.add_task(f"Exporting to {format.value.upper()}...", total=None)
            serializer: Union[URDFSerializer, MJCFSerializer]
            if format == ExportFormat.urdf:
                serializer = URDFSerializer()
                serializer.save(robot, str(output), download_assets=download_assets, mesh_dir=resolved_mesh_dir)
            else:  # MJCF
                mjcf_config = MJCFConfig(position=position or (0, 0, 0), add_ground_plane=add_ground_plane)
                serializer = MJCFSerializer(mjcf_config)
                serializer.save(robot, str(output), download_assets=download_assets, mesh_dir=resolved_mesh_dir)
            progress.update(task, completed=True)

        if not quiet:
            console.print(f"\n[green]✓[/green] Successfully exported to [cyan]{output}[/cyan]")

            # Print statistics
            console.print("\n[bold]Statistics:[/bold]")
            console.print(f"  Links: {len(robot.nodes)}")
            console.print(f"  Joints: {len(robot.edges)}")
            if download_assets:
                console.print(f"  Meshes: Saved to [cyan]{resolved_mesh_dir}/[/cyan]")

            if verbose:
                console.print("\n[dim]Session configuration saved to ORT.yaml[/dim]")

    except KeyboardInterrupt:
        console.print("\n[yellow]Export cancelled by user[/yellow]")
        raise typer.Exit(1)
    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)
