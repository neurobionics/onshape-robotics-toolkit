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
    create_graph: Legacy function for creating graphs from old string-based system
"""

import random
from typing import Any, Optional, Union

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
from onshape_robotics_toolkit.parse import CAD, MATE_JOINER, SUBASSEMBLY_JOINER, PathKey


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
        Initialize and build kinematic graph from CAD assembly.

        Args:
            cad: CAD assembly with PathKey-based registries
            use_user_defined_root: Whether to use user-marked fixed part as root

        Examples:
            >>> cad = CAD.from_assembly(assembly, max_depth=1)
            >>> tree = KinematicGraph(cad, use_user_defined_root=True)
            >>> print(f"Root: {tree.root_node}")
            >>> print(f"Nodes: {len(tree.graph.nodes)}")
        """
        self.cad = cad
        self.graph: nx.DiGraph = nx.DiGraph()
        self.root_node: Optional[PathKey] = None
        self.topological_order: Optional[tuple[PathKey, ...]] = None

        # Build the kinematic graph
        self._build_graph(use_user_defined_root)

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
        LOGGER.debug(f"Parts involved in mates: {len(involved_parts)}")

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
                    invalid_keys.append(f"parent={parent_key}")
                if not child_valid:
                    invalid_keys.append(f"child={child_key}")
                LOGGER.debug(f"Filtering out mate with invalid PathKeys: {', '.join(invalid_keys)}")
                invalid_count += 1

        if invalid_count > 0:
            LOGGER.warning(
                f"Filtered out {invalid_count} mates with invalid PathKeys "
                f"({len(valid_mates)}/{len(mates)} mates remain)"
            )

        LOGGER.debug(f"Valid mates: {len(valid_mates)}")
        return valid_mates

    def _is_valid_mate_target(self, key: PathKey) -> bool:
        """
        Check if a PathKey is a valid target for a mate relationship.

        Valid targets are:
        - Parts in the parts registry (includes rigid assemblies)
        - Part instances in the instances registry
        - Rigid assembly instances in the instances registry

        Args:
            key: PathKey to validate

        Returns:
            True if key is valid mate target
        """
        # Check parts registry (includes rigid assemblies)
        if key in self.cad.parts:
            return True

        # Check part instances
        if key in self.cad.root_assembly.instances.parts:
            return True

        # Check assembly instances (only rigid ones are valid)
        if key in self.cad.root_assembly.instances.assemblies:
            assembly_instance = self.cad.root_assembly.instances.assemblies[key]
            return assembly_instance.isRigid

        return False

    def _collect_all_mates(self) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Collect all mates from root assembly and all subassemblies.

        This includes:
        - Mates from root assembly (absolute PathKeys)
        - Mates from subassemblies (converted to absolute PathKeys)
        - Mates from fetched flexible subassemblies (recursively)

        Subassembly mates use relative PathKeys (just leaf IDs), so we need to
        prefix them with the subassembly's path to create absolute PathKeys.

        Returns:
            Dictionary mapping (parent_key, child_key) to MateFeatureData with absolute PathKeys
        """
        all_mates: dict[tuple[PathKey, PathKey], MateFeatureData] = {}

        # Add mates from root assembly (already have absolute PathKeys)
        all_mates.update(self.cad.root_assembly.mates.mates)
        LOGGER.debug(f"Collected {len(self.cad.root_assembly.mates.mates)} mates from root assembly")

        # Add mates from all subassemblies in the JSON (need to convert to absolute PathKeys)
        for sub_key, assembly_data in self.cad.sub_assemblies.items():
            sub_mates_count = len(assembly_data.mates.mates)
            if sub_mates_count > 0:
                LOGGER.debug(f"Processing {sub_mates_count} mates from subassembly {sub_key}")
                absolute_mates = self._convert_subassembly_mates_to_absolute(assembly_data.mates.mates, sub_key)
                all_mates.update(absolute_mates)

        # Add mates from fetched flexible subassemblies (if any)
        for fetched_cad in self.cad.fetched_subassemblies.values():
            # Recursively collect mates from fetched subassemblies
            all_mates.update(fetched_cad.root_assembly.mates.mates)
            for sub_assembly_data in fetched_cad.sub_assemblies.values():
                all_mates.update(sub_assembly_data.mates.mates)

        LOGGER.debug(
            f"Collected {len(all_mates)} total mates "
            f"(root: {len(self.cad.root_assembly.mates.mates)}, "
            f"subassemblies: {len(self.cad.sub_assemblies)}, "
            f"fetched: {len(self.cad.fetched_subassemblies)})"
        )
        return all_mates

    def _convert_subassembly_mates_to_absolute(
        self, mates: dict[tuple[PathKey, PathKey], MateFeatureData], subassembly_key: PathKey
    ) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Convert subassembly mates with relative PathKeys to absolute PathKeys.

        Subassembly mates use relative PathKeys (leaf IDs only), but we need
        absolute PathKeys (full path from root) to match the instance registries.

        Also filters out mates that are completely within rigid subassemblies,
        since those are internal to the rigid assembly and shouldn't be in the tree.

        Args:
            mates: Dictionary of mates with relative PathKeys
            subassembly_key: PathKey of the subassembly (used as prefix)

        Returns:
            Dictionary of mates with absolute PathKeys (filtered)
        """
        absolute_mates: dict[tuple[PathKey, PathKey], MateFeatureData] = {}
        filtered_count = 0

        for (parent_key, child_key), mate_data in mates.items():
            # Convert relative PathKeys to absolute by prepending subassembly path
            absolute_parent = self._make_absolute_pathkey(parent_key, subassembly_key)
            absolute_child = self._make_absolute_pathkey(child_key, subassembly_key)

            # Check if entities are within rigid subassemblies
            parent_rigid_root = self._find_rigid_assembly_ancestor(absolute_parent)
            child_rigid_root = self._find_rigid_assembly_ancestor(absolute_child)

            # Case 1: Both within same rigid assembly - filter out (internal mate)
            if parent_rigid_root and child_rigid_root and parent_rigid_root == child_rigid_root:
                LOGGER.debug(
                    f"Filtering internal mate: {absolute_parent} <-> {absolute_child} "
                    f"(both within {parent_rigid_root})"
                )
                filtered_count += 1
                continue

            # Case 2: One or both entities inside rigid assembly - remap to rigid root
            final_parent = parent_rigid_root if parent_rigid_root else absolute_parent
            final_child = child_rigid_root if child_rigid_root else absolute_child

            if final_parent != absolute_parent or final_child != absolute_child:
                LOGGER.debug(
                    f"Remapping mate to rigid roots: {absolute_parent} -> {final_parent}, "
                    f"{absolute_child} -> {final_child}"
                )

            absolute_mates[(final_parent, final_child)] = mate_data

            LOGGER.debug(f"Converted mate: {parent_key} -> {absolute_parent}, " f"{child_key} -> {absolute_child}")

        if filtered_count > 0:
            LOGGER.info(f"Filtered {filtered_count} internal mates from subassembly {subassembly_key}")

        return absolute_mates

    def _find_rigid_assembly_ancestor(self, key: PathKey) -> Optional[PathKey]:
        """
        Find the rigid assembly that contains this PathKey, if any.

        Args:
            key: PathKey to check

        Returns:
            PathKey of containing rigid assembly, or None if not within one
        """
        # Check each ancestor (parent key) to see if it's a rigid assembly
        current = key
        while current and current.parent_key:
            parent = current.parent_key
            # Check if parent is a rigid assembly
            if parent in self.cad.root_assembly.instances.assemblies:
                assembly_instance = self.cad.root_assembly.instances.assemblies[parent]
                if assembly_instance.isRigid:
                    return parent
            current = parent
        return None

    def _make_absolute_pathkey(self, relative_key: PathKey, prefix_key: PathKey) -> PathKey:
        """
        Convert a relative PathKey to absolute by prepending a prefix.

        Args:
            relative_key: PathKey with relative path (may be partial)
            prefix_key: PathKey to prepend (subassembly path)

        Returns:
            Absolute PathKey with full path from root
        """
        # If relative_key already contains the prefix, return as-is
        prefix_len = len(prefix_key.path)
        if len(relative_key.path) > prefix_len and relative_key.path[:prefix_len] == prefix_key.path:
            return relative_key

        # Otherwise, prepend the prefix
        absolute_path = prefix_key.path + relative_key.path
        return PathKey(absolute_path)

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
            occurrence = self.cad.root_assembly.occurrences.occurrences.get(part_key)
            if occurrence and occurrence.fixed:
                LOGGER.debug(f"Found user-defined root: {part_key}")
                return part_key
        return None

    def _add_nodes(self, involved_parts: set[PathKey], user_defined_root: Optional[PathKey]) -> None:
        """
        Add nodes to graph for parts/assemblies involved in mates.

        Nodes are added with part metadata as attributes. Some mates may reference
        assembly instances (for rigid subassemblies), which we treat as nodes if
        they're in the parts registry (rigid assemblies are stored as parts).

        Args:
            involved_parts: Set of PathKeys to add as nodes
            user_defined_root: Optional user-defined root node
        """
        nodes_added = 0
        skipped_assemblies = 0

        LOGGER.debug(f"Processing {len(involved_parts)} involved parts/assemblies")
        for part_key in involved_parts:
            # Check if this is a part or rigid assembly in parts registry
            if part_key not in self.cad.parts:
                # Check if it's a flexible assembly instance (not in parts registry)
                if part_key in self.cad.root_assembly.instances.assemblies:
                    assembly_instance = self.cad.root_assembly.instances.assemblies[part_key]
                    if not assembly_instance.isRigid:
                        # Flexible assembly - skip it, mates should reference parts inside it
                        LOGGER.debug(
                            f"Skipping flexible assembly {part_key} - " f"mates should reference parts inside it"
                        )
                        skipped_assemblies += 1
                        continue
                    else:
                        # Rigid assembly but not in parts registry - this shouldn't happen
                        LOGGER.warning(
                            f"Rigid assembly {part_key} in mate but not in parts registry. "
                            f"This may indicate mate processing didn't complete properly."
                        )
                        continue

                # Not in parts registry and not an assembly - check if it's a part instance
                if part_key in self.cad.root_assembly.instances.parts:
                    LOGGER.warning(
                        f"Part instance {part_key} involved in mate but corresponding Part object "
                        f"not found in parts registry. This may indicate the part wasn't populated."
                    )
                else:
                    LOGGER.warning(
                        f"PathKey {part_key} involved in mate but not found in any registry "
                        f"(not in parts, part instances, or assembly instances)"
                    )
                continue

            # Check if occurrence is hidden
            occurrence = self.cad.root_assembly.occurrences.occurrences.get(part_key)
            if occurrence and occurrence.hidden:
                LOGGER.debug(f"Skipping hidden part: {part_key}")
                continue

            # Add node with part metadata
            part = self.cad.parts[part_key]
            self.graph.add_node(
                part_key,
                **part.model_dump(mode="python", exclude_unset=False),
                is_root=(part_key == user_defined_root),
            )
            nodes_added += 1

        LOGGER.debug(f"Added {nodes_added} nodes to graph " f"(skipped {skipped_assemblies} flexible assemblies)")

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
        label_mapping = {}
        for node in self.graph.nodes:
            # Try to get name from part instances
            if node in self.cad.root_assembly.instances.parts:
                part_instance = self.cad.root_assembly.instances.parts[node]
                label_mapping[node] = part_instance.name
            # Try to get name from assembly instances (for rigid assemblies)
            elif node in self.cad.root_assembly.instances.assemblies:
                assembly_instance = self.cad.root_assembly.instances.assemblies[node]
                label_mapping[node] = assembly_instance.name
            else:
                # Fallback to PathKey representation
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
            node_color=colors,
        )
        plt.show()


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
    use_user_defined_root: bool = True,
) -> tuple[nx.DiGraph, Optional[Any]]:
    """
    Create a graph from onshape assembly data.

    Args:
        occurrences: Dictionary of occurrences in the assembly.
        instances: Dictionary of instances in the assembly.
        parts: Dictionary of parts in the assembly.
        mates: Dictionary of mates in the assembly.
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
        >>> create_graph(occurrences, instances, parts, mates)
    """

    graph: nx.DiGraph = nx.DiGraph()

    # First, get all parts involved in mates
    involved_parts = get_parts_involved_in_mates(mates)

    user_defined_root = add_nodes_to_graph(graph, occurrences, instances, parts, involved_parts, use_user_defined_root)

    if user_defined_root and user_defined_root.split(SUBASSEMBLY_JOINER)[0] in parts:
        # this means that the user defined root is a rigid subassembly
        user_defined_root = user_defined_root.split(SUBASSEMBLY_JOINER)[0]

    add_edges_to_graph(graph, mates)

    cur_graph = remove_unconnected_subgraphs(graph)
    output_graph, root_node = convert_to_digraph(cur_graph, user_defined_root)

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
) -> Optional[str]:
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
