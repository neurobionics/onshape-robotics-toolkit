"""Main CLI application for the Onshape Robotics Toolkit."""

import typer
from rich.console import Console

from onshape_robotics_toolkit import __version__

# Import subcommand modules (will be created)
from onshape_robotics_toolkit.cli import assembly, config, export, graph, robot, variables

console = Console()

# Create main Typer app
app = typer.Typer(
    name="ort",
    help="Onshape Robotics Toolkit - Convert Onshape CAD assemblies to robot description formats (URDF/MJCF)",
    add_completion=False,
    no_args_is_help=True,
    rich_markup_mode="rich",
)

# Add export command directly (it's the most common operation)
app.command(name="export", help="Export Onshape assembly to URDF or MJCF format")(export.export_command)

# Add subcommand groups
app.add_typer(variables.app, name="var", help="Variable manipulation commands")
app.add_typer(assembly.app, name="assembly", help="Assembly inspection commands")
app.add_typer(graph.app, name="graph", help="Kinematic graph utilities")
app.add_typer(robot.app, name="robot", help="Robot manipulation commands")
app.add_typer(config.app, name="config", help="Configuration management")


@app.command()
def version() -> None:
    """Show toolkit version."""
    console.print(f"[bold]Onshape Robotics Toolkit[/bold] version [cyan]{__version__}[/cyan]")


if __name__ == "__main__":
    app()
