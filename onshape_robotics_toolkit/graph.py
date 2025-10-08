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
from typing import Optional

import matplotlib.pyplot as plt
import networkx as nx

from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import MateFeatureData
from onshape_robotics_toolkit.parse import CAD, CHILD, PARENT, PathKey
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name


class KinematicGraph(nx.DiGraph):
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
        root_node: PathKey of the root node in the kinematic graph
        topological_order: Ordered sequence of nodes from root to leaves
    """

    def __init__(self, cad: CAD):
        """
        Initialize and build kinematic graph from CAD data.

        Note: The preferred way to create a KinematicGraph is via the `from_cad()`
        classmethod, which makes the construction more explicit.

        Args:
            name: Name of the kinematic graph
            cad: CAD assembly with PathKey-based registries
            use_user_defined_root: Whether to use user-marked fixed part as root

        Examples:
            >>> # Preferred (explicit):
            >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)

            >>> # Also works (backward compatible):
            >>> graph = KinematicGraph(cad, use_user_defined_root=True)
        """
        self.cad = cad
        self.root: Optional[PathKey] = None

        super().__init__()

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
        kinematic_graph = cls(cad=cad)
        kinematic_graph._build_graph(use_user_defined_root)
        kinematic_graph._process_graph()

        return kinematic_graph

    def _build_graph(self, use_user_defined_root: bool) -> None:
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
        involved_parts = self._get_parts_involved_in_mates(remapped_mates)

        self._add_nodes(involved_parts)
        self._add_edges(remapped_mates)
        self._find_root_node(involved_parts, use_user_defined_root)

        if len(self.nodes) == 0:
            LOGGER.warning("KinematicGraph is empty - no valid parts found in mates")
            return

        LOGGER.info(
            f"KinematicGraph processed: {len(self.nodes)} nodes, "
            f"{len(self.edges)} edges with root node: {self.root}"
        )

    def _process_graph(self) -> None:
        """
        Process the graph:
            1. Remove disconnected subgraphs
            2. Convert to directed graph with root detection
            3. Calculate topological order
        """
        # TODO: add processing methods to:
        # 1. Identify if the graph has disconnected subgraphs
        # 2. Identify and process any loops present in the graph
        pass

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

    def _find_root_node(self, involved_parts: set[PathKey], use_user_defined_root: bool = True) -> None:
        """
        Find user-defined root part (marked as fixed in Onshape).

        Args:
            involved_parts: Set of parts to search within

        Returns:
            PathKey of fixed part, or None if not found
        """
        root = None
        if use_user_defined_root:
            for part_key in involved_parts:
                occurrence = self.cad.occurrences.get(part_key)
                if occurrence and occurrence.fixed:
                    LOGGER.debug(f"Found user-defined root: {part_key}")
                    root = part_key
                    self.root = root
                    self.nodes[root]["is_root"] = True
                    break

            if root is None:
                LOGGER.warning("No user-defined root part found (marked as fixed in Onshape), auto-detecting root")
                self._find_root_node(involved_parts, use_user_defined_root=False)
        else:
            try:
                root = next(nx.topological_sort(self))
            except nx.NetworkXUnfeasible:
                LOGGER.warning("DiGraph has one or more cycles")

            if root:
                self.root = root
                self.nodes[root]["is_root"] = True
                LOGGER.debug(f"Auto-detected root node: {root}")
            else:
                LOGGER.warning("Could not determine root node via topological sort")

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

            part = self.cad.parts[part_key]
            self.add_node(
                part_key,
                **part.model_dump(mode="python", exclude_unset=False),
                is_root=False,
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
            if parent_key in self.nodes and child_key in self.nodes:
                self.add_edge(
                    parent_key,
                    child_key,
                    mate_data=mate,
                )
                edges_added += 1
                LOGGER.debug(f"Added edge: {parent_key} -> {child_key}")
            else:
                LOGGER.debug(f"Skipping edge {parent_key} -> {child_key} (nodes not in graph)")

        LOGGER.debug(f"Added {edges_added} edges to graph")

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
        if file_name is None:
            file_name = get_sanitized_name(self.cad.name if self.cad.name else "kinematic_graph")

        colors = [f"#{random.randint(0, 0xFFFFFF):06x}" for _ in range(len(self.nodes))]  # noqa: S311
        plt.figure(figsize=(8, 8))
        pos = nx.planar_layout(self)

        nx.draw(
            self,
            pos,
            with_labels=True,
            node_color=colors,
            edge_color="white",
            font_color="white",
        )
        plt.savefig(file_name, transparent=True)
        plt.close()
