"""Assembly inspection commands."""

from typing import Optional

import typer
from rich.console import Console

from onshape_robotics_toolkit.cli.utils import create_table, handle_error, setup_cli_logging, validate_env, validate_url
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.assembly import MateType
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD

console = Console()
app = typer.Typer()


@app.command(name="info")
def assembly_info(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    max_depth: int = typer.Option(1, "--max-depth", "-d", help="Maximum assembly depth"),
    env_path: Optional[str] = typer.Option(None, "--env", "-e", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """Show detailed information about an Onshape assembly."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        if not quiet:
            console.print("\n[bold cyan]Fetching assembly data...[/bold cyan]")

        assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)

        if not quiet:
            console.print("[dim]Processing assembly...[/dim]\n")

        cad = CAD.from_assembly(assembly, max_depth=max_depth, client=client)

        # Document information
        console.print("[bold]Document Information:[/bold]")
        console.print(f"  URL: [cyan]{url}[/cyan]")
        console.print(f"  Document ID: [dim]{doc.did}[/dim]")
        console.print(f"  Element ID: [dim]{doc.eid}[/dim]")

        # Assembly statistics
        console.print("\n[bold]Assembly Statistics:[/bold]")
        console.print(f"  Total Parts: [cyan]{len(cad.parts)}[/cyan]")
        console.print(f"  Total Instances: [cyan]{len(cad.instances)}[/cyan]")
        console.print(f"  Subassemblies: [cyan]{len(cad.subassemblies)}[/cyan]")

        # Count flexible vs rigid subassemblies
        flexible_count = sum(1 for sa in cad.subassemblies.values() if not sa.isRigid)
        rigid_count = sum(1 for sa in cad.subassemblies.values() if sa.isRigid)
        console.print(f"    - Flexible: [green]{flexible_count}[/green]")
        console.print(f"    - Rigid: [yellow]{rigid_count}[/yellow]")

        # Mate statistics
        all_mates = cad.get_all_mates_flattened()
        console.print(f"  Total Mates: [cyan]{len(all_mates)}[/cyan]")

        # Count mates by type
        mate_type_counts: dict[MateType, int] = {}
        for mate_data in all_mates.values():
            mate_type = mate_data.mateType
            mate_type_counts[mate_type] = mate_type_counts.get(mate_type, 0) + 1

        if mate_type_counts:
            table = create_table("Mate Types", ["Type", "Count"])
            for mate_type, count in sorted(mate_type_counts.items(), key=lambda x: x[1], reverse=True):
                table.add_row(mate_type.value, str(count))
            console.print()
            console.print(table)

        # Try to build kinematic graph for more insights
        try:
            graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
            console.print("\n[bold]Kinematic Structure:[/bold]")
            console.print(f"  Graph Nodes (Links): [cyan]{len(graph.nodes)}[/cyan]")
            console.print(f"  Graph Edges (Joints): [cyan]{len(graph.edges)}[/cyan]")
            if graph.root:
                console.print(f"  Root Node: [cyan]{graph.root}[/cyan]")

            # Calculate graph depth
            if graph.root:
                import networkx as nx

                depths = nx.single_source_shortest_path_length(graph, graph.root)
                max_graph_depth = max(depths.values()) if depths else 0
                console.print(f"  Maximum Depth: [cyan]{max_graph_depth}[/cyan]")

        except Exception as e:
            console.print(f"\n[yellow]Warning:[/yellow] Could not build kinematic graph: {e}")

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)


@app.command(name="elements")
def list_elements(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    env_path: Optional[str] = typer.Option(None, "--env", "-e", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """List all elements in an Onshape document."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        if not quiet:
            console.print("\n[bold cyan]Fetching document elements...[/bold cyan]\n")

        elements = client.get_elements(doc.did, doc.wtype, doc.wid)

        table = create_table("Document Elements", ["Type", "Name", "Element ID"])
        for elem_type, element in elements.items():
            table.add_row(elem_type, element.name or "[unnamed]", element.id)

        console.print(table)

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)
