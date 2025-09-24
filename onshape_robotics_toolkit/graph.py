"""
This module contains functions to create and manipulate graphs from Onshape assembly data.

"""

import random
from typing import Optional, Union

import matplotlib.pyplot as plt
import networkx as nx

from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import (
    AssemblyInstance,
    InstanceType,
    MateFeatureData,
    Occurrence,
    Part,
    PartInstance,
)
from onshape_robotics_toolkit.parse import MATE_JOINER, SUBASSEMBLY_JOINER


def plot_graph(graph: Union[nx.Graph, nx.DiGraph], file_name: Optional[str] = None) -> None:
    """
    Display the graph using networkx and matplotlib, or save it as an image file.

    Args:
        graph: The graph to display or save.
        file_name: The name of the image file to save. If None, the graph will be displayed.

    Examples:
        >>> graph = nx.Graph()
        >>> plot_graph(graph)
        >>> plot_graph(graph, "graph.png")
    """
    colors = [f"#{random.randint(0, 0xFFFFFF):06x}" for _ in range(len(graph.nodes))]  # noqa: S311
    plt.figure(figsize=(8, 8))
    pos = nx.shell_layout(graph)

    if file_name:
        nx.draw(
            graph,
            pos,
            with_labels=True,
            arrows=True,
            node_color=colors,
            edge_color="white",
            font_color="white",
        )
        plt.savefig(file_name, transparent=True)
        plt.close()
    else:
        nx.draw(
            graph,
            pos,
            with_labels=True,
            arrows=True,
            node_color=colors,
        )
        plt.show()


def get_root_node(graph: nx.DiGraph) -> str:
    """
    Get the root node of a directed graph.

    Args:
        graph: The directed graph.

    Returns:
        The root node of the graph.

    Examples:
        >>> graph = nx.DiGraph()
        >>> get_root_node(graph)
    """
    return next(nx.topological_sort(graph))


def convert_to_digraph(graph: nx.Graph, user_defined_root: Union[str, None] = None) -> nx.DiGraph:
    """
    Convert a graph to a directed graph and calculate the root node using closeness centrality.

    Args:
        graph: The graph to convert.
        user_defined_root: The node to use as the root node.

    Returns:
        The directed graph and the root node of the graph, calculated using closeness centrality.

    Examples:
        >>> graph = nx.Graph()
        >>> convert_to_digraph(graph)
        (digraph, root_node)
    """

    centrality = nx.closeness_centrality(graph)
    root_node = user_defined_root if user_defined_root else max(centrality, key=centrality.get)

    bfs_graph = nx.bfs_tree(graph, root_node)
    di_graph = nx.DiGraph(bfs_graph)

    for u, v, data in graph.edges(data=True):
        if not di_graph.has_edge(u, v) and not di_graph.has_edge(v, u):
            # decide which edge to keep
            if centrality[u] > centrality[v]:
                di_graph.add_edge(u, v, **data)
            else:
                di_graph.add_edge(v, u, **data)

    # TODO: Edges and nodes lose their data during this conversion, fix this
    return di_graph, root_node


def get_topological_order(graph: nx.DiGraph) -> tuple[str]:
    """
    Get the topological order of a directed graph.

    Args:
        graph: The directed graph.

    Returns:
        The topological order of the graph.

    Examples:
        >>> graph = nx.DiGraph()
        >>> get_topological_order(graph)
    """
    try:
        order = tuple(nx.topological_sort(graph))
    except nx.NetworkXUnfeasible:
        LOGGER.warning("Graph has one or more cycles")
        order = None

    return order


def get_parts_involved_in_mates(mates: dict[str, Union[MateFeatureData]]) -> set[str]:
    """
    Extract all part IDs that are involved in mates.

    Args:
        mates: Dictionary of mates in the assembly.

    Returns:
        Set of part IDs that are involved in mates.
    """
    involved_parts = set()
    for mate_key in mates:
        try:
            child, parent = mate_key.split(MATE_JOINER)
            involved_parts.add(child)
            involved_parts.add(parent)
        except ValueError:
            LOGGER.warning(f"Mate key: {mate_key} is not in the correct format")
            continue
    return involved_parts


def create_graph(
    occurrences: dict[str, Occurrence],
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
    parts: dict[str, Part],
    mates: dict[str, Union[MateFeatureData]],
    directed: bool = True,
    use_user_defined_root: bool = True,
) -> tuple[nx.DiGraph, str]:
    """
    Create a graph from onshape assembly data.

    Args:
        occurrences: Dictionary of occurrences in the assembly.
        instances: Dictionary of instances in the assembly.
        parts: Dictionary of parts in the assembly.
        mates: Dictionary of mates in the assembly.
        directed: Whether to create a directed graph.
        use_user_defined_root: Whether to use the user defined root node.

    Returns:
        A tuple containing:
        - The graph created from the assembly data.
        - The root node of the graph.

    Examples:
        >>> occurrences = get_occurrences(assembly)
        >>> instances = get_instances(assembly)
        >>> parts = get_parts(assembly, client)
        >>> mates = get_mates(assembly)
        >>> create_graph(occurrences, instances, parts, mates, directed=True)
    """

    graph = nx.Graph()

    # First, get all parts involved in mates
    involved_parts = get_parts_involved_in_mates(mates)

    user_defined_root = add_nodes_to_graph(graph, occurrences, instances, parts, involved_parts, use_user_defined_root)

    if user_defined_root and user_defined_root.split(SUBASSEMBLY_JOINER)[0] in parts:
        # this means that the user defined root is a rigid subassembly
        user_defined_root = user_defined_root.split(SUBASSEMBLY_JOINER)[0]

    add_edges_to_graph(graph, mates)

    cur_graph = remove_unconnected_subgraphs(graph)

    if directed:
        output_graph, root_node = convert_to_digraph(cur_graph, user_defined_root)
    else:
        output_graph = cur_graph
        root_node = None

    LOGGER.info(
        f"Graph created with {len(output_graph.nodes)} nodes and "
        f"{len(output_graph.edges)} edges with root node: {root_node}"
    )

    return output_graph, root_node


def add_nodes_to_graph(
    graph: nx.Graph,
    occurrences: dict[str, Occurrence],
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
    parts: dict[str, Part],
    involved_parts: set[str],
    use_user_defined_root: bool,
) -> str:
    """
    Add nodes to the graph for parts that are involved in mates.

    Args:
        graph: The graph to add nodes to.
        occurrences: Dictionary of occurrences in the assembly.
        instances: Dictionary of instances in the assembly.
        parts: Dictionary of parts in the assembly.
        involved_parts: Set of part IDs that are involved in mates.
        use_user_defined_root: Whether to use the user defined root node.

    Returns:
        The user defined root node if it exists.

    Examples:
        >>> add_nodes_to_graph(graph, occurrences, instances, parts, involved_parts, use_user_defined_root=True)
    """
    user_defined_root = None
    for occurrence in occurrences:
        if use_user_defined_root and occurrences[occurrence].fixed:
            user_defined_root = occurrence

        # Only add nodes for parts that are involved in mates
        if instances[occurrence].type == InstanceType.PART and occurrence in involved_parts:
            try:
                if occurrences[occurrence].hidden:
                    continue

                graph.add_node(occurrence, **parts[occurrence].model_dump())
            except KeyError:
                LOGGER.warning(f"Part {occurrence} not found")
    return user_defined_root


def add_edges_to_graph(graph: nx.Graph, mates: dict[str, Union[MateFeatureData]]) -> None:
    """
    Add edges to the graph.

    Args:
        graph: The graph to add edges to.
        mates: Dictionary of mates in the assembly.

    Examples:
        >>> add_edges_to_graph(graph, mates)
    """
    for mate in mates:
        try:
            child, parent = mate.split(MATE_JOINER)
            graph.add_edge(
                parent,
                child,
            )
        except KeyError:
            LOGGER.warning(f"Mate {mate} not found")
        except ValueError:
            LOGGER.warning(f"Mate key: {mate} is not in the correct format")
            exit(1)


def _print_graph_tree(graph: nx.Graph) -> None:
    """Print a text-based tree representation of the graph structure."""
    if not graph.nodes:
        print("  (empty graph)")
        return

    # Show connected components
    components = list(nx.connected_components(graph))
    for i, component in enumerate(components):
        print(f"  Component {i + 1} ({len(component)} nodes):")
        for j, node in enumerate(sorted(component)):
            prefix = "    ├── " if j < len(component) - 1 else "    └── "
            neighbors = list(graph.neighbors(node))
            neighbor_info = f" -> {len(neighbors)} connections" if neighbors else ""
            try:
                print(f"{prefix}{node}{neighbor_info}")
            except UnicodeEncodeError:
                # Handle Unicode characters in node names that can't be encoded
                # Use LOGGER instead of print to avoid console encoding issues
                from onshape_robotics_toolkit.log import LOGGER

                LOGGER.info(f"Node: {node} ({len(neighbors)} connections)")
        if i < len(components) - 1:
            print()


def remove_unconnected_subgraphs(graph: nx.Graph) -> nx.Graph:
    """
    Remove unconnected subgraphs from the graph.

    Args:
        graph: The graph to remove unconnected subgraphs from.

    Returns:
        The main connected subgraph of the graph, which is the largest connected subgraph.
    """
    if not nx.is_connected(graph):
        LOGGER.warning("Graph has one or more unconnected subgraphs")

        # Show tree visualization of original graph
        LOGGER.info("Original graph structure:")
        _print_graph_tree(graph)

        sub_graphs = list(nx.connected_components(graph))
        main_graph_nodes = max(sub_graphs, key=len)
        main_graph = graph.subgraph(main_graph_nodes).copy()

        # Show tree visualization of reduced graph
        LOGGER.info("Reduced graph structure:")
        _print_graph_tree(main_graph)

        LOGGER.warning(f"Reduced graph nodes from {len(graph.nodes)} to {len(main_graph.nodes)}")
        LOGGER.warning(f"Reduced graph edges from {len(graph.edges)} to {len(main_graph.edges)}")
        return main_graph
    return graph
