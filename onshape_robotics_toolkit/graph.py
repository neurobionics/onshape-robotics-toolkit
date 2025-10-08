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
        self.graph: nx.DiGraph = nx.DiGraph()
        self.root_node: Optional[PathKey] = None
        self.topological_order: Optional[tuple[PathKey, ...]] = None

        # Build the kinematic graph
        self._build_graph(use_user_defined_root)

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
        return cls(cad, use_user_defined_root=use_user_defined_root)

    def _build_graph(self, use_user_defined_root: bool) -> None:
        """
        Build kinematic graph from CAD assembly data.

        Process:
        1. Collect all mates from root and subassemblies
        2. Validate and filter mates (remove invalid PathKeys)
        3. Get parts involved in valid mates
        4. Add nodes for involved parts (with metadata)
        5. Add edges from mate relationships
        6. Remove disconnected subgraphs
        7. Convert to directed graph with root detection
        8. Calculate topological order

        Args:
            use_user_defined_root: Whether to use user-defined fixed part as root
        """
        # Collect all mates from root assembly and subassemblies
        all_mates = self._collect_all_mates()

        # Validate and filter mates to remove invalid PathKeys
        valid_mates = self._validate_mates(all_mates)

        # Get all parts involved in valid mates
        involved_parts = self._get_parts_involved_in_mates(valid_mates)
        LOGGER.info(f"Parts involved in mates: {len(involved_parts)}")

        # Find user-defined root if requested
        user_defined_root = self._find_user_defined_root(involved_parts) if use_user_defined_root else None

        # Add nodes for parts involved in mates
        self._add_nodes(involved_parts, user_defined_root)

        # Add edges from mates
        self._add_edges(valid_mates)

        # Check if graph is empty
        if len(self.graph.nodes) == 0:
            LOGGER.warning("KinematicGraph is empty - no valid parts found in mates")
            return

        # Remove disconnected subgraphs
        undirected_graph = self.graph.to_undirected()
        main_graph = remove_unconnected_subgraphs(undirected_graph)

        # Check if user_defined_root is still in graph after filtering
        if user_defined_root and user_defined_root not in main_graph.nodes:
            LOGGER.warning(
                f"User-defined root {user_defined_root} not in main graph after filtering. "
                f"Will use centrality-based root detection."
            )
            user_defined_root = None

        # Convert to directed graph with root node
        self.graph, self.root_node = convert_to_digraph(main_graph, user_defined_root)

        # Align mate entity ordering with the directed edge orientation
        self._align_mated_entities_with_edges()

        # Calculate topological order
        self.topological_order = get_topological_order(self.graph)

        LOGGER.info(
            f"KinematicGraph created: {len(self.graph.nodes)} nodes, "
            f"{len(self.graph.edges)} edges, root={self.root_node}"
        )

    def _validate_mates(
        self, mates: dict[tuple[PathKey, PathKey], MateFeatureData]
    ) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Validate mates and filter out invalid PathKeys.

        A mate is valid if both PathKeys correspond to either:
        - Parts in the parts registry
        - Part instances in the instances registry
        - Rigid assembly instances in the instances registry

        Args:
            mates: Dictionary of all mates to validate

        Returns:
            Dictionary of valid mates only
        """
        valid_mates: dict[tuple[PathKey, PathKey], MateFeatureData] = {}
        invalid_count = 0

        for (parent_key, child_key), mate_data in mates.items():
            # Check if both keys are valid
            parent_valid = self._is_valid_mate_target(parent_key)
            child_valid = self._is_valid_mate_target(child_key)

            if parent_valid and child_valid:
                valid_mates[(parent_key, child_key)] = mate_data
            else:
                invalid_keys = []
                if not parent_valid:
                    invalid_keys.append(f"parent={parent_key} (not in parts)")
                if not child_valid:
                    invalid_keys.append(f"child={child_key} (not in parts)")
                LOGGER.debug(f"Filtering mate '{mate_data.name}': {', '.join(invalid_keys)}")
                invalid_count += 1

        if invalid_count > 0:
            LOGGER.warning(
                f"Filtered out {invalid_count} mates with invalid PathKeys "
                f"({len(valid_mates)}/{len(mates)} mates remain)"
            )

        LOGGER.debug(f"Valid mates: {len(valid_mates)}")
        return valid_mates

    def _align_mated_entities_with_edges(self) -> None:
        """
        Align mate entity ordering to match the directed graph edge orientation.

        During graph construction, edges may be reoriented when converting from the undirected
        representation to a directed tree rooted at the chosen base link. The MateFeatureData
        attached to each edge retains its original matedEntities ordering, which can result in
        swapped parent/child indices. Because downstream URDF generation relies on the global
        CHILD/PARENT constants, we realign the list so that:

            mate.matedEntities[CHILD]  -> child node of the directed edge
            mate.matedEntities[PARENT] -> parent node of the directed edge
        """

        for parent_key, child_key, data in self.graph.edges(data=True):
            mate = data["mate_data"]
            if not isinstance(mate, MateFeatureData):
                continue

            expected_parent_path = parent_key.path
            expected_child_path = child_key.path

            def entity_path(index: int, mate: MateFeatureData = mate) -> tuple[str, ...]:
                try:
                    return tuple(mate.matedEntities[index].matedOccurrence)
                except (AttributeError, IndexError):
                    return ()

            current_child_path = entity_path(CHILD)
            current_parent_path = entity_path(PARENT)

            if current_child_path == expected_child_path and current_parent_path == expected_parent_path:
                continue

            parent_idx = child_idx = None
            for idx, entity in enumerate(mate.matedEntities):
                occurrence_path = tuple(getattr(entity, "matedOccurrence", ()))
                if occurrence_path == expected_parent_path:
                    parent_idx = idx
                if occurrence_path == expected_child_path:
                    child_idx = idx

            if parent_idx is None or child_idx is None:
                LOGGER.warning(
                    "Mate '%s' entities do not match edge %s -> %s (parent_idx=%s, child_idx=%s).",
                    getattr(mate, "name", "<unnamed mate>"),
                    parent_key,
                    child_key,
                    parent_idx,
                    child_idx,
                )
                continue

            if parent_idx == PARENT and child_idx == CHILD:
                # Order already matches constants; nothing to do.
                continue

            ordered_entities = list(mate.matedEntities)
            ordered_entities[CHILD] = mate.matedEntities[child_idx]
            ordered_entities[PARENT] = mate.matedEntities[parent_idx]
            mate.matedEntities = ordered_entities

            LOGGER.debug(
                "Aligned mate '%s' entity order for edge %s -> %s (parent_idx=%s, child_idx=%s).",
                getattr(mate, "name", "<unnamed mate>"),
                parent_key,
                child_key,
                parent_idx,
                child_idx,
            )

    def _is_valid_mate_target(self, key: PathKey) -> bool:
        """
        Check if a PathKey is a valid target for a mate relationship.

        Valid targets exist in the parts registry. With eager parts population,
        this includes all PartInstances and rigid AssemblyInstances.

        However, mates from flexible subassemblies may reference parts that
        aren't in the root parts registry yet (they're in fetched_subassemblies).
        We still validate primarily against parts registry for consistency.

        Args:
            key: PathKey to validate

        Returns:
            True if key is valid mate target (exists in parts registry)
        """
        # Primary check: parts registry (all PartInstances + rigid assemblies)
        return key in self.cad.parts

    def _collect_all_mates(self) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Collect all mates from CAD (already flattened with absolute PathKeys).

        The CAD class now stores all mates flattened from root + subassemblies.
        We just need to get them without assembly provenance for graph construction.

        Returns:
            Dictionary mapping (parent_key, child_key) to MateFeatureData with absolute PathKeys
        """
        # Get all mates flattened (removes assembly provenance from keys)
        all_mates = self.cad.get_all_mates_flattened()

        LOGGER.debug(f"Collected {len(all_mates)} total mates from CAD")
        return all_mates

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

    def _add_nodes(self, involved_parts: set[PathKey], user_defined_root: Optional[PathKey]) -> None:
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
            # With eager parts population, part MUST exist if it passed validation
            if part_key not in self.cad.parts:
                LOGGER.warning(f"Part {part_key} not in parts registry despite passing validation")
                continue

            # Check if occurrence is hidden
            occurrence = self.cad.occurrences.get(part_key)
            if occurrence and occurrence.hidden:
                LOGGER.debug(f"Skipping hidden part: {part_key}")
                skipped_hidden += 1
                continue

            # Add node with part metadata
            part = self.cad.parts[part_key]
            self.graph.add_node(
                part_key,
                **part.model_dump(mode="python", exclude_unset=False),
                is_root=(part_key == user_defined_root),
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
        skipped_edges = []
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
                skipped_edges.append((parent_key, child_key, mate.name))

        if skipped_edges:
            LOGGER.warning(f"Skipped {len(skipped_edges)} edges (nodes not in graph):")
            for p, c, name in skipped_edges:
                in_nodes_p = p in self.graph.nodes
                in_nodes_c = c in self.graph.nodes
                LOGGER.warning(f"  {name}: {p} (in_graph:{in_nodes_p}) <-> {c} (in_graph:{in_nodes_c})")

        LOGGER.debug(f"Added {edges_added} edges to graph")

    def plot(self, file_name: Optional[str] = None) -> None:
        """
        Visualize the kinematic graph.

        Args:
            file_name: Optional filename to save visualization. If None, displays interactively.

        Examples:
            >>> tree.plot()  # Display interactively
            >>> tree.plot("kinematic_tree.png")  # Save to file
        """
        plot_graph(self.graph, file_name)

    def show(self, file_name: Optional[str] = None) -> None:
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

    def get_node_metadata(self, node_key: PathKey) -> dict[str, Any]:
        """
        Get metadata for a specific node.

        Args:
            node_key: PathKey of the node

        Returns:
            Dictionary of node attributes

        Examples:
            >>> metadata = tree.get_node_metadata(some_part_key)
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

    def get_children(self, node_key: PathKey) -> list[PathKey]:
        """
        Get all children of a node in the kinematic graph.

        Args:
            node_key: PathKey of parent node

        Returns:
            List of child PathKeys

        Examples:
            >>> children = tree.get_children(root_key)
            >>> for child in children:
            >>>     print(f"Child: {child}")
        """
        return list(self.graph.successors(node_key))

    def get_parent(self, node_key: PathKey) -> Optional[PathKey]:
        """
        Get parent of a node in the kinematic graph.

        Args:
            node_key: PathKey of child node

        Returns:
            Parent PathKey, or None if node is root

        Examples:
            >>> parent = tree.get_parent(some_part_key)
            >>> if parent:
            >>>     print(f"Parent: {parent}")
        """
        predecessors = list(self.graph.predecessors(node_key))
        return predecessors[0] if predecessors else None

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
    pos = nx.shell_layout(graph)

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
