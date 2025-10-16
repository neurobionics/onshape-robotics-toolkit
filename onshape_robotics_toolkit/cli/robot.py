"""Robot manipulation commands."""

from typing import Any, Optional

import typer
from rich.console import Console
from rich.tree import Tree

from onshape_robotics_toolkit.cli.utils import handle_error, setup_cli_logging, validate_env, validate_url
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot as RobotModel

console = Console()
app = typer.Typer()


@app.command(name="info")
def robot_info(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    max_depth: int = typer.Option(1, "--max-depth", "-d", help="Maximum assembly depth"),
    name: str = typer.Option("robot", "--name", "-n", help="Robot name"),
    env_path: Optional[str] = typer.Option(None, "--env", "-e", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """Show detailed information about the robot model."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        if not quiet:
            console.print("\n[bold cyan]Building robot model...[/bold cyan]")

        assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)
        cad = CAD.from_assembly(assembly, max_depth=max_depth, client=client)
        graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
        robot = RobotModel.from_graph(kinematic_graph=graph, client=client, name=name, fetch_mass_properties=False)

        # Robot information
        console.print("\n[bold]Robot Information:[/bold]")
        console.print(f"  Name: [cyan]{robot.name}[/cyan]")
        console.print(f"  Links: [cyan]{len(robot.nodes)}[/cyan]")
        console.print(f"  Joints: [cyan]{len(robot.edges)}[/cyan]")

        # Find root link
        root_links = [n for n in robot.nodes if robot.in_degree(n) == 0]
        if root_links:
            console.print(f"  Root Link: [cyan]{root_links[0]}[/cyan]")

        # Calculate depth
        if root_links:
            import networkx as nx

            depths = nx.single_source_shortest_path_length(robot, root_links[0])
            max_robot_depth = max(depths.values()) if depths else 0
            console.print(f"  Maximum Depth: [cyan]{max_robot_depth}[/cyan]")

        # Joint type breakdown

        joint_type_counts: dict[str, int] = {}
        for _, _, data in robot.edges(data=True):
            joint_data = data.get("data")
            if joint_data:
                joint_type = joint_data.type
                joint_type_counts[joint_type] = joint_type_counts.get(joint_type, 0) + 1

        if joint_type_counts:
            console.print("\n[bold]Joint Types:[/bold]")
            for joint_type, count in sorted(joint_type_counts.items(), key=lambda x: x[1], reverse=True):
                console.print(f"  {joint_type}: [cyan]{count}[/cyan]")

        # Link details
        links_with_mass = sum(
            1 for _, data in robot.nodes(data=True) if data.get("data") and hasattr(data["data"], "inertial")
        )
        console.print("\n[bold]Link Details:[/bold]")
        console.print(f"  Links with inertial data: [cyan]{links_with_mass}[/cyan]")

        # Asset information
        assets_count = sum(1 for _, data in robot.nodes(data=True) if data.get("asset") is not None)
        console.print("\n[bold]Assets:[/bold]")
        console.print(f"  Mesh assets: [cyan]{assets_count}[/cyan]")

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)


@app.command(name="show-tree")
def show_robot_tree(
    url: str = typer.Argument(..., help="Onshape document URL", callback=validate_url),
    max_depth: int = typer.Option(1, "--max-depth", "-d", help="Maximum assembly depth"),
    name: str = typer.Option("robot", "--name", "-n", help="Robot name"),
    env_path: Optional[str] = typer.Option(None, "--env", "-e", help="Path to .env file"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Quiet mode"),
) -> None:
    """Display the robot structure as a tree."""
    setup_cli_logging(verbose=verbose, quiet=quiet)
    resolved_env_path = validate_env(env_path)

    try:
        client = Client(env=resolved_env_path)
        doc = Document.from_url(url)

        if not quiet:
            console.print("\n[bold cyan]Building robot model...[/bold cyan]")

        assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)
        cad = CAD.from_assembly(assembly, max_depth=max_depth, client=client)
        graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
        robot = RobotModel.from_graph(kinematic_graph=graph, client=client, name=name, fetch_mass_properties=False)

        # Build tree visualization
        tree = Tree(f"[bold cyan]Robot: {robot.name}[/bold cyan]")

        def add_robot_children(node: Any, parent_tree: Tree) -> None:
            """Recursively add children to the tree."""
            for child in robot.successors(node):
                # Get edge data (joint information)
                edge_data = robot.get_edge_data(node, child)
                joint_data = edge_data.get("data") if edge_data else None
                joint_name = joint_data.name if joint_data and hasattr(joint_data, "name") else "unknown"
                joint_type = joint_data.type if joint_data and hasattr(joint_data, "type") else "unknown"

                # Get link name
                link_data = robot.nodes[child].get("data")
                link_name = link_data.name if link_data and hasattr(link_data, "name") else str(child)

                child_label = f"{link_name} [dim]via[/dim] [yellow]{joint_name}[/yellow] [dim]({joint_type})[/dim]"
                child_tree = parent_tree.add(child_label)
                add_robot_children(child, child_tree)

        # Find root
        root_links = [n for n in robot.nodes if robot.in_degree(n) == 0]
        if not root_links:
            console.print("[red]Error:[/red] No root link found")
            raise typer.Exit(1)

        root = root_links[0]
        root_data = robot.nodes[root].get("data")
        root_name = root_data.name if root_data and hasattr(root_data, "name") else str(root)

        root_tree = tree.add(f"[green]{root_name}[/green] [dim](root)[/dim]")
        add_robot_children(root, root_tree)

        console.print()
        console.print(tree)

        # Print statistics
        console.print("\n[bold]Robot Statistics:[/bold]")
        console.print(f"  Links: [cyan]{len(robot.nodes)}[/cyan]")
        console.print(f"  Joints: [cyan]{len(robot.edges)}[/cyan]")

    except Exception as e:
        handle_error(e, verbose=verbose)
        raise typer.Exit(1)
