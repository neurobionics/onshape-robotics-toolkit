"""
This module contains functions and classes to create and manipulate kinematic graphs from Onshape assembly data.

The main class is KinematicGraph, which uses the PathKey-based CAD system to build a directed graph
representing the kinematic structure of a robot assembly. The graph nodes are parts involved in mates,
and edges represent mate relationships.

Classes:
    KinematicGraph: Build and navigate kinematic graph from CAD assembly

Functions:
    plot_graph: Visualize graphs using matplotlib
    get_root_node: Get root node of directed graph
    convert_to_digraph: Convert undirected graph to directed with root detection
    get_topological_order: Calculate topological ordering
    remove_unconnected_subgraphs: Remove disconnected components from graph
"""

import copy
import random
from typing import Any, Optional, Union

import matplotlib.pyplot as plt
import networkx as nx

from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import MateFeatureData
from onshape_robotics_toolkit.parse import CAD, CHILD, PARENT, PathKey


class KinematicGraph:
    """
    kinematic graph representation of an assembly using PathKey-based system.

    This class creates a directed graph from CAD assembly data where:
    - Nodes: Parts involved in mates (PathKey identifiers)
    - Edges: Mate relationships between parts
    - Root: Determined by closeness centrality or user-defined fixed part

    The tree supports:
    - Topological ordering for kinematic chains
    - Root node detection via centrality or user preference
    - Disconnected subgraph removal
    - Visualization

    Attributes:
        cad: CAD assembly data with PathKey-based registries
        graph: NetworkX directed graph representation
        root_node: PathKey of the root node in the kinematic graph
        topological_order: Ordered sequence of nodes from root to leaves
    """

    def __init__(self, cad: CAD, use_user_defined_root: bool = True):
        """
        Initialize and build kinematic graph from CAD data.

        Note: The preferred way to create a KinematicGraph is via the `from_cad()`
        classmethod, which makes the construction more explicit.

        Args:
            cad: CAD assembly with PathKey-based registries
            use_user_defined_root: Whether to use user-marked fixed part as root

        Examples:
            >>> # Preferred (explicit):
            >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)

            >>> # Also works (backward compatible):
            >>> graph = KinematicGraph(cad, use_user_defined_root=True)
        """
        self.cad = cad
        self.use_user_defined_root = use_user_defined_root
        self.user_defined_root: Union[PathKey, None] = None

        self.graph: Union[nx.Graph, nx.DiGraph] = nx.Graph()
        self.root_node: Optional[PathKey] = None
        self.topological_order: Optional[tuple[PathKey, ...]] = None

        # Build the kinematic graph
        self._build_graph()

    @classmethod
    def from_cad(cls, cad: CAD, use_user_defined_root: bool = True) -> "KinematicGraph":
        """
        Create and build kinematic graph from CAD assembly.

        This is the recommended way to create a KinematicGraph. It constructs
        the graph by processing mates, validating PathKeys, and determining
        the kinematic structure.

        Args:
            cad: CAD assembly with PathKey-based registries
            use_user_defined_root: Whether to use user-marked fixed part as root

        Returns:
            Fully constructed KinematicGraph with nodes, edges, and root

        Examples:
            >>> cad = CAD.from_assembly(assembly, max_depth=1)
            >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
            >>> print(f"Root: {graph.root_node}")
            >>> print(f"Nodes: {len(graph.graph.nodes)}")
        """
        kinematic_graph = cls(cad, use_user_defined_root=use_user_defined_root)
        kinematic_graph._process_graph()

        return kinematic_graph

    def _build_graph(self) -> None:
        """
        Build kinematic graph from CAD assembly data.

        Process:
        1. Collect all mates from root and subassemblies
        2. Validate and filter mates (remove invalid PathKeys)
        3. Get parts involved in valid mates
        4. Add nodes for involved parts (with metadata)
        5. Add edges from mate relationships

        Args:
            use_user_defined_root: Whether to use user-defined fixed part as root
        """
        # remap the mates to switch out any parts that belong to rigid subassemblies
        remapped_mates = self._remap_mates(self.cad)

        # Get all parts involved in valid mates
        involved_parts = self._get_parts_involved_in_mates(remapped_mates)
        LOGGER.debug(f"Parts involved in mates: {len(involved_parts)}")

        # Find user-defined root if requested
        self.user_defined_root = self._find_user_defined_root(involved_parts) if self.use_user_defined_root else None

        # Add nodes for parts involved in mates
        self._add_nodes(involved_parts)

        # Add edges from mates
        self._add_edges(remapped_mates)

        # Check if graph is empty
        if len(self.graph.nodes) == 0:
            LOGGER.warning("KinematicGraph is empty - no valid parts found in mates")
            return

        LOGGER.debug(f"KinematicGraph created: {len(self.graph.nodes)} nodes, " f"{len(self.graph.edges)} edges")

    def _process_graph(self) -> None:
        """
        Process the graph:
            1. Remove disconnected subgraphs
            2. Convert to directed graph with root detection
            3. Calculate topological order
        """

        # Remove disconnected subgraphs
        main_graph = remove_disconnected_subgraphs(self.graph)

        # Check if user_defined_root is still in graph after filtering
        if self.user_defined_root and self.user_defined_root not in main_graph.nodes:
            LOGGER.warning(
                f"User-defined root {self.user_defined_root} not in main graph after filtering. "
                f"Will use centrality-based root detection."
            )
            self.user_defined_root = None

        # Convert to directed graph with root node
        self.graph, self.root_node = convert_to_digraph(main_graph, self.user_defined_root)

        # Calculate topological order
        self.topological_order = get_topological_order(self.graph)

        LOGGER.info(
            f"KinematicGraph processed: {len(self.graph.nodes)} nodes, "
            f"{len(self.graph.edges)} edges with root node: {self.root_node}"
        )

    def _remap_mates(self, cad: CAD) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Remap mates to replace parts that belong to rigid subassemblies with the rigid assembly part

        Args:
            cad (CAD): The CAD assembly generated from Onshape data

        Returns:
            dict[tuple[PathKey, PathKey], MateFeatureData]: A mapping of original mate paths
            (w/o assembly keys) to their remapped counterparts
        """

        def remap_mate(key: PathKey, index: int, mate: MateFeatureData) -> PathKey:
            """
            Return the rigid assembly root key if the part is inside a rigid assembly,
            otherwise return the original key.
            """
            # TODO: Currently we only remap the keys of the mates
            # the MatedCS should also be modified to include the rigid assembly TF
            r_key: PathKey = key
            part = cad.parts[key]
            if part.rigidAssemblyKey is not None:
                r_key = part.rigidAssemblyKey
                # NOTE: this is where we remap the matedOccurrence as well
                mate.matedEntities[index].matedOccurrence = list(r_key.path)
            return r_key

        remapped_mates: dict[tuple[PathKey, PathKey], MateFeatureData] = {}
        # CAD's mates are already filtered and validated, they only include mates that
        # need to be processed for robot generation
        for (_, *entities), mate in cad.mates.items():
            _mate_data = copy.deepcopy(mate)
            remapped_keys = []

            for i, key in enumerate(entities):
                # NOTE: mate data's matedEntities have matedOccurrences that need to
                # be remapped as well in addition to the keys
                remapped_key = remap_mate(key, i, _mate_data)
                remapped_keys.append(remapped_key)

            remapped_mate_key: tuple[PathKey, PathKey] = tuple(remapped_keys)  # type: ignore[assignment]
            if remapped_mate_key in remapped_mates:
                LOGGER.warning(
                    "Duplicate mate detected after remapping: %s -> %s. "
                    "This can happen if multiple parts in a rigid assembly are mated to the same part. "
                    "Only the first mate will be kept.",
                    remapped_mate_key[PARENT],
                    remapped_mate_key[CHILD],
                )
                continue

            remapped_mates[remapped_mate_key] = _mate_data
        return remapped_mates

    def _get_parts_involved_in_mates(self, mates: dict[tuple[PathKey, PathKey], MateFeatureData]) -> set[PathKey]:
        """
        Extract all part PathKeys that are involved in mates.

        Args:
            mates: Dictionary of mate relationships

        Returns:
            Set of PathKeys for parts involved in mates
        """
        involved_parts: set[PathKey] = set()
        for parent_key, child_key in mates:
            involved_parts.add(parent_key)
            involved_parts.add(child_key)

        LOGGER.debug(f"Found {len(involved_parts)} parts involved in mates")
        return involved_parts

    def _find_user_defined_root(self, involved_parts: set[PathKey]) -> Optional[PathKey]:
        """
        Find user-defined root part (marked as fixed in Onshape).

        Args:
            involved_parts: Set of parts to search within

        Returns:
            PathKey of fixed part, or None if not found
        """
        for part_key in involved_parts:
            occurrence = self.cad.occurrences.get(part_key)
            if occurrence and occurrence.fixed:
                LOGGER.debug(f"Found user-defined root: {part_key}")
                return part_key
        return None

    def _add_nodes(self, involved_parts: set[PathKey]) -> None:
        """
        Add nodes to graph for parts involved in mates.

        With eager parts population, all valid mate targets are guaranteed to be
        in cad.parts. We only need to filter hidden occurrences.

        Args:
            involved_parts: Set of PathKeys to add as nodes
            user_defined_root: Optional user-defined root node
        """
        nodes_added = 0
        skipped_hidden = 0

        LOGGER.debug(f"Processing {len(involved_parts)} involved parts")
        for part_key in involved_parts:
            # Check if instance is suppressed
            instance = self.cad.instances.get(part_key)
            if instance and instance.suppressed:
                # TODO: Should this happen here or within the CAD class?
                # Skipping parts here is problematic since the mate registry still
                # refers to them, leading to dangling edges.
                LOGGER.debug(f"Skipping suppressed part: {part_key}")
                skipped_hidden += 1
                continue

            # Add node with part metadata
            part = self.cad.parts[part_key]
            self.graph.add_node(
                part_key,
                **part.model_dump(mode="python", exclude_unset=False),
                is_root=(part_key == self.user_defined_root),
            )
            nodes_added += 1

        LOGGER.debug(f"Added {nodes_added} nodes to graph (skipped {skipped_hidden} hidden parts)")

    def _add_edges(self, mates: dict[tuple[PathKey, PathKey], MateFeatureData]) -> None:
        """
        Add edges to graph from mate relationships.

        Args:
            mates: Dictionary of mate relationships
        """
        edges_added = 0
        for (parent_key, child_key), mate in mates.items():
            # Only add edge if both nodes exist in graph
            if parent_key in self.graph.nodes and child_key in self.graph.nodes:
                self.graph.add_edge(
                    parent_key,
                    child_key,
                    mate_data=mate,
                )
                edges_added += 1
                LOGGER.debug(f"Added edge: {parent_key} -> {child_key}")
            else:
                LOGGER.debug(f"Skipping edge {parent_key} -> {child_key} (nodes not in graph)")

        LOGGER.debug(f"Added {edges_added} edges to graph")

    def show(self, file_name: Optional[str] = "robot") -> None:
        """
        Visualize the kinematic graph with part names as labels instead of PathKey IDs.

        Creates a more readable visualization by mapping PathKeys to their corresponding
        part or assembly names from the CAD instance registry.

        Args:
            file_name: Optional filename to save visualization. If None, displays interactively.

        Examples:
            >>> tree.show()  # Display interactively with names
            >>> tree.show("kinematic_tree.png")  # Save to file with names
        """
        # Create a mapping from PathKey to name
        label_mapping: dict[PathKey, str] = {}
        for node in self.graph.nodes:
            label_mapping[node] = str(node)
        # Create a copy of the graph with relabeled nodes
        labeled_graph = nx.relabel_nodes(self.graph, label_mapping, copy=True)

        # Plot the labeled graph
        plot_graph(labeled_graph, file_name)

    def get_node_data(self, node_key: PathKey) -> dict[str, Any]:
        """
        Get metadata for a specific node.

        Args:
            node_key: PathKey of the node

        Returns:
            Dictionary of node attributes

        Examples:
            >>> metadata = tree.get_node_data(some_part_key)
            >>> print(metadata['name'])
        """
        return dict(self.graph.nodes[node_key])

    def get_mate_data(self, parent_key: PathKey, child_key: PathKey) -> Optional[MateFeatureData]:
        """
        Get mate data for an edge between two nodes.

        Args:
            parent_key: PathKey of parent node
            child_key: PathKey of child node

        Returns:
            MateFeatureData if edge exists, None otherwise

        Examples:
            >>> mate = tree.get_mate_data(parent_key, child_key)
            >>> if mate:
            >>>     print(mate.featureType)
        """
        if self.graph.has_edge(parent_key, child_key):
            return self.graph.edges[parent_key, child_key].get("mate_data")
        return None

    def __repr__(self) -> str:
        """String representation of the kinematic graph."""
        return (
            f"KinematicGraph(nodes={len(self.graph.nodes)}, " f"edges={len(self.graph.edges)}, root={self.root_node})"
        )


def plot_graph(graph: Union[nx.Graph, nx.DiGraph], file_name: Optional[str] = "robot") -> None:
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
    pos = nx.planar_layout(graph)

    nx.draw(
        graph,
        pos,
        with_labels=True,
        node_color=colors,
        edge_color="white",
        font_color="white",
    )
    plt.savefig(file_name, transparent=True)
    plt.close()


def get_root_node(graph: nx.DiGraph) -> Any:
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


def convert_to_digraph(graph: nx.Graph, user_defined_root: Any = None) -> tuple[nx.DiGraph, Any]:
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
    root_node = user_defined_root if user_defined_root else max(centrality, key=lambda x: centrality[x])

    # Create BFS tree from root (this loses edge data!)
    bfs_graph = nx.bfs_tree(graph, root_node)
    di_graph = nx.DiGraph(bfs_graph)

    # Restore edge data for BFS tree edges from original graph
    for u, v in list(di_graph.edges()):
        if graph.has_edge(u, v):
            # Copy edge data from original undirected graph
            di_graph[u][v].update(graph[u][v])
        elif graph.has_edge(v, u):
            # Edge might be reversed in undirected graph
            # TODO: if there are scenarios where converting the graph to a digraph,
            # alters the direction of the mates, we should reorient the mate data too
            # this will then become a method tied to the class as it would then operate
            # on mate data and requires some knowledge of the CAD structure
            di_graph[u][v].update(graph[v][u])

    # Add back any edges not in BFS tree (loops, etc.)
    for u, v, data in graph.edges(data=True):
        if not di_graph.has_edge(u, v) and not di_graph.has_edge(v, u):
            # Decide which direction to keep based on centrality
            if centrality[u] > centrality[v]:
                di_graph.add_edge(u, v, **data)
            else:
                di_graph.add_edge(v, u, **data)

    return di_graph, root_node


def get_topological_order(graph: nx.DiGraph) -> Optional[tuple[Any, ...]]:
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


def remove_disconnected_subgraphs(graph: nx.Graph) -> nx.Graph:
    """
    Remove disconnected subgraphs from the graph.

    Args:
        graph: The graph to remove disconnected subgraphs from.

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
