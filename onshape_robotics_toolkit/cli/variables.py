"""Variable manipulation commands."""

import json
from typing import Optional

import typer
from rich.console import Console

from onshape_robotics_toolkit.cli.utils import (
    create_table,
    handle_error,
    setup_cli_logging,
    validate_env,
    validate_url,
)
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.models.document import Document

console = Console()
app = typer.Typer()


@app.command(name="list")
def list_variables(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    env_path: Optional[str] = typer.Option(None, "--env", "-e", help="Path to .env file"),
    output_json: bool = typer.Option(False, "--json", help="Output as JSON"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """List all variables in an Onshape document."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        # Get elements and find variable studio
        elements = client.get_elements(doc.did, doc.wtype, doc.wid)
        if "variables" not in elements:
            console.print("[yellow]Warning:[/yellow] No variable studio found in document")
            return

        # Get variables
        variables = client.get_variables(doc.did, doc.wid, elements["variables"].id)

        if output_json:
            # JSON output
            var_data = {
                name: {"expression": var.expression, "value": str(var.value)} for name, var in variables.items()
            }
            console.print(json.dumps(var_data, indent=2))
        else:
            # Table output
            table = create_table("Variables", ["Name", "Expression", "Value"])
            for name, var in variables.items():
                table.add_row(name, var.expression, str(var.value))
            console.print(table)

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)


@app.command(name="get")
def get_variable(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    name: str = typer.Option(..., "--name", "-n", help="Variable name"),
    env_path: Optional[str] = typer.Option(None, "--env", "-e", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """Get a specific variable value from an Onshape document."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        elements = client.get_elements(doc.did, doc.wtype, doc.wid)
        if "variables" not in elements:
            console.print("[red]Error:[/red] No variable studio found in document")
            raise typer.Exit(1)

        variables = client.get_variables(doc.did, doc.wid, elements["variables"].id)

        if name not in variables:
            console.print(f"[red]Error:[/red] Variable '{name}' not found")
            console.print("\n[yellow]Available variables:[/yellow]")
            for var_name in variables:
                console.print(f"  - {var_name}")
            raise typer.Exit(1)

        var = variables[name]
        if not quiet:
            console.print(f"[bold]{name}[/bold]")
            console.print(f"  Expression: [cyan]{var.expression}[/cyan]")
            console.print(f"  Value: [green]{var.value}[/green]")
        else:
            console.print(var.expression)

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)


@app.command(name="set")
def set_variable(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    name: str = typer.Option(..., "--name", "-n", help="Variable name"),
    expression: str = typer.Option(
        ..., "--expression", "-e", "--value", help="Variable expression (e.g., '180 mm', '20 deg')"
    ),
    env_path: Optional[str] = typer.Option(None, "--env", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """Set a variable value in an Onshape document."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        elements = client.get_elements(doc.did, doc.wtype, doc.wid)
        if "variables" not in elements:
            console.print("[red]Error:[/red] No variable studio found in document")
            raise typer.Exit(1)

        variables = client.get_variables(doc.did, doc.wid, elements["variables"].id)

        if name not in variables:
            console.print(f"[red]Error:[/red] Variable '{name}' not found")
            raise typer.Exit(1)

        # Update variable
        variables[name].expression = expression
        variables_to_set = {name: expression}

        client.set_variables(doc.did, doc.wid, elements["variables"].id, variables_to_set)

        if not quiet:
            console.print(f"[green]✓[/green] Variable '{name}' updated to: [cyan]{expression}[/cyan]")

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)


@app.command(name="export")
def export_with_variables(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    output: str = typer.Argument(..., help="Output file path"),
    set_vars: list[str] = typer.Option([], "--set", "-s", help="Set variable (format: name=expression)"),
    format: str = typer.Option("urdf", "--format", "-f", help="Export format: urdf or mjcf"),
    max_depth: int = typer.Option(1, "--max-depth", "-d", help="Maximum assembly depth"),
    env_path: Optional[str] = typer.Option(None, "--env", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """
    Update variables and export in one command.

    Example:
        ort var export <URL> robot.urdf --set wheelDiameter="180 mm" --set forkAngle="20 deg"
    """
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        # Parse variable assignments
        vars_to_set = {}
        for var_assignment in set_vars:
            if "=" not in var_assignment:
                console.print(f"[red]Error:[/red] Invalid variable assignment: {var_assignment}")
                console.print('[yellow]Format:[/yellow] name=expression (e.g., wheelDiameter="180 mm")')
                raise typer.Exit(1)

            var_name, var_expr = var_assignment.split("=", 1)
            vars_to_set[var_name.strip()] = var_expr.strip().strip('"').strip("'")

        if vars_to_set:
            elements = client.get_elements(doc.did, doc.wtype, doc.wid)
            if "variables" not in elements:
                console.print("[red]Error:[/red] No variable studio found in document")
                raise typer.Exit(1)

            # Update variables
            if not quiet:
                console.print("\n[bold]Updating variables:[/bold]")
                for name, expr in vars_to_set.items():
                    console.print(f"  {name} = [cyan]{expr}[/cyan]")

            client.set_variables(doc.did, doc.wid, elements["variables"].id, vars_to_set)

        # Now export (delegate to export command)
        if not quiet:
            console.print("\n[bold]Exporting assembly...[/bold]")

        from pathlib import Path

        from onshape_robotics_toolkit.cli.export import export_command

        export_command(
            url=url,
            output=Path(output),
            format=format,  # type: ignore[arg-type]
            name=None,
            max_depth=max_depth,
            mesh_dir=None,
            download_assets=True,
            fetch_mass=True,
            use_user_root=True,
            position=None,
            add_ground_plane=False,
            config_path=None,
            env_path=resolved_env_path,
            verbose=verbose,
            quiet=quiet,
        )

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)
