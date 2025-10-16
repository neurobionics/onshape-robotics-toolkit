"""Kinematic graph utilities."""

from pathlib import Path
from typing import Any, Optional

import typer
from rich.console import Console
from rich.tree import Tree

from onshape_robotics_toolkit.cli.utils import handle_error, setup_cli_logging, validate_env, validate_url
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD

console = Console()
app = typer.Typer()


@app.command(name="show")
def show_graph(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    max_depth: int = typer.Option(1, "--max-depth", "-d", help="Maximum assembly depth"),
    use_user_root: bool = typer.Option(True, "--use-user-root/--no-use-user-root", help="Use user-defined root part"),
    env_path: Optional[str] = typer.Option(None, "--env", "-e", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """Display the kinematic graph structure as a tree."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        if not quiet:
            console.print("\n[bold cyan]Building kinematic graph...[/bold cyan]")

        assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)
        cad = CAD.from_assembly(assembly, max_depth=max_depth, client=client)
        graph = KinematicGraph.from_cad(cad, use_user_defined_root=use_user_root)

        if not graph.root:
            console.print("[red]Error:[/red] No root node found in kinematic graph")
            raise typer.Exit(1)

        # Build tree visualization
        tree = Tree("[bold cyan]Kinematic Graph[/bold cyan]")

        def add_children(node: Any, parent_tree: Tree) -> None:
            """Recursively add children to the tree."""
            for child in graph.successors(node):
                # Get edge data (mate information)
                edge_data = graph.get_edge_data(node, child)
                mate_data = edge_data.get("data") if edge_data else None
                mate_type = mate_data.mateType.value if mate_data else "unknown"

                child_label = f"{child} [dim]({mate_type})[/dim]"
                child_tree = parent_tree.add(child_label)
                add_children(child, child_tree)

        # Start from root
        root_tree = tree.add(f"[green]{graph.root}[/green] [dim](root)[/dim]")
        add_children(graph.root, root_tree)

        console.print()
        console.print(tree)

        # Print statistics
        console.print("\n[bold]Graph Statistics:[/bold]")
        console.print(f"  Nodes (Links): [cyan]{len(graph.nodes)}[/cyan]")
        console.print(f"  Edges (Joints): [cyan]{len(graph.edges)}[/cyan]")
        console.print(f"  Root: [cyan]{graph.root}[/cyan]")

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)


@app.command(name="export")
def export_graph(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    output: Path = typer.Argument(..., help="Output file path (e.g., graph.png, graph.dot)"),
    max_depth: int = typer.Option(1, "--max-depth", "-d", help="Maximum assembly depth"),
    use_user_root: bool = typer.Option(True, "--use-user-root/--no-use-user-root", help="Use user-defined root part"),
    env_path: Optional[str] = typer.Option(None, "--env", "-e", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """Export the kinematic graph visualization to a file."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        if not quiet:
            console.print("\n[bold cyan]Building kinematic graph...[/bold cyan]")

        assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)
        cad = CAD.from_assembly(assembly, max_depth=max_depth, client=client)
        graph = KinematicGraph.from_cad(cad, use_user_defined_root=use_user_root)

        # Determine format from extension
        suffix = output.suffix.lower()

        if suffix == ".dot":
            # Export as DOT format
            import networkx as nx

            nx.drawing.nx_pydot.write_dot(graph, str(output))
            if not quiet:
                console.print(f"\n[green]✓[/green] Graph exported to [cyan]{output}[/cyan] (DOT format)")

        elif suffix in [".png", ".svg", ".pdf"]:
            # Export as image using matplotlib
            try:
                import matplotlib.pyplot as plt
                import networkx as nx

                plt.figure(figsize=(12, 8))
                pos = nx.spring_layout(graph, seed=42)

                # Draw nodes
                nx.draw_networkx_nodes(graph, pos, node_color="lightblue", node_size=500)

                # Draw edges
                nx.draw_networkx_edges(graph, pos, edge_color="gray", arrows=True)

                # Draw labels
                labels = {node: str(node).split("/")[-1] for node in graph.nodes}  # Shorten labels
                nx.draw_networkx_labels(graph, pos, labels, font_size=8)

                plt.axis("off")
                plt.tight_layout()
                plt.savefig(str(output), format=suffix[1:], dpi=300, bbox_inches="tight")
                plt.close()

                if not quiet:
                    console.print(f"\n[green]✓[/green] Graph exported to [cyan]{output}[/cyan]")

            except ImportError:
                console.print("[red]Error:[/red] matplotlib is required for image export")
                console.print("[yellow]Tip:[/yellow] Install with: pip install matplotlib")
                raise typer.Exit(1)

        else:
            console.print(f"[red]Error:[/red] Unsupported file format: {suffix}")
            console.print("[yellow]Supported formats:[/yellow] .dot, .png, .svg, .pdf")
            raise typer.Exit(1)

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)
