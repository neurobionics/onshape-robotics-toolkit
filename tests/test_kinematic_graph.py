"""Tests for KinematicGraph class."""

from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.parse import CAD


class TestKinematicGraph:
    """Tests for KinematicGraph class with PathKey-based system."""

    def test_create_kinematic_tree(self, cad_doc: CAD):
        """Test creating a kinematic graph from CAD document."""
        tree = KinematicGraph(cad_doc, use_user_defined_root=True)

        assert tree is not None
        assert tree.graph is not None
        assert tree.cad == cad_doc

    def test_tree_has_nodes_and_edges(self, cad_doc: CAD):
        """Test that kinematic graph has nodes and edges."""
        tree = KinematicGraph(cad_doc)

        # Tree should have nodes from parts involved in mates
        assert len(tree.graph.nodes) > 0
        # Tree should have edges from mate relationships
        assert len(tree.graph.edges) > 0

        print(f"\nKinematicGraph: {len(tree.graph.nodes)} nodes, {len(tree.graph.edges)} edges")

    def test_tree_has_root_node(self, cad_doc: CAD):
        """Test that kinematic graph has a root node."""
        tree = KinematicGraph(cad_doc)

        assert tree.root_node is not None
        assert tree.root_node in tree.graph.nodes

        print(f"\nRoot node: {tree.root_node}")

    def test_tree_has_topological_order(self, cad_doc: CAD):
        """Test that kinematic graph has topological ordering."""
        tree = KinematicGraph(cad_doc)

        # If graph is acyclic, should have topological order
        if tree.topological_order is not None:
            assert len(tree.topological_order) == len(tree.graph.nodes)
            # First node should be root
            assert tree.topological_order[0] == tree.root_node
            print(f"\nTopological order: {len(tree.topological_order)} nodes")
        else:
            # If graph has cycles, topological order will be None
            print("\nGraph has cycles, no topological order")

    def test_get_children(self, cad_doc: CAD):
        """Test getting children of a node."""
        tree = KinematicGraph(cad_doc)

        if tree.root_node:
            children = tree.get_children(tree.root_node)
            assert isinstance(children, list)
            # Root should typically have children
            if children:
                print(f"\nRoot has {len(children)} children")

    def test_get_parent(self, cad_doc: CAD):
        """Test getting parent of a node."""
        tree = KinematicGraph(cad_doc)

        # Root should have no parent
        if tree.root_node:
            parent = tree.get_parent(tree.root_node)
            assert parent is None

            # Children should have root as parent
            children = tree.get_children(tree.root_node)
            if children:
                child = children[0]
                parent = tree.get_parent(child)
                assert parent == tree.root_node

    def test_get_node_metadata(self, cad_doc: CAD):
        """Test getting metadata for a node."""
        tree = KinematicGraph(cad_doc)

        if tree.root_node:
            metadata = tree.get_node_metadata(tree.root_node)
            assert isinstance(metadata, dict)
            # Metadata dict may be empty if part has no additional attributes stored
            # Just verify the method works without error
            print(f"\nRoot metadata: {metadata}")

    def test_get_mate_data(self, cad_doc: CAD):
        """Test getting mate data for an edge."""
        tree = KinematicGraph(cad_doc)

        if tree.root_node:
            children = tree.get_children(tree.root_node)
            if children:
                child = children[0]
                mate_data = tree.get_mate_data(tree.root_node, child)
                # Mate data may or may not exist depending on graph structure
                if mate_data:
                    assert hasattr(mate_data, "mateType")
                    print(f"\nFound mate data: {mate_data.mateType}")

    def test_tree_repr(self, cad_doc: CAD):
        """Test string representation of kinematic graph."""
        tree = KinematicGraph(cad_doc)

        repr_str = repr(tree)
        assert "KinematicGraph" in repr_str
        assert "nodes=" in repr_str
        assert "edges=" in repr_str
        print(f"\n{repr_str}")

    def test_tree_with_different_depths(self, cad_doc_all_depths: CAD):
        """Test kinematic graph creation with different max_depth values."""
        tree = KinematicGraph(cad_doc_all_depths)

        # Tree should be created successfully regardless of max_depth
        assert tree is not None

        # For max_depth=0, all assemblies are rigid, so graph may be empty or minimal
        # For max_depth > 0, should have nodes and edges from flexible assemblies
        if cad_doc_all_depths.max_depth == 0:
            # At depth 0, only root-level mates are processed
            # May have nodes if root-level parts are mated
            print(
                f"\nmax_depth={cad_doc_all_depths.max_depth}: "
                f"{len(tree.graph.nodes)} nodes, {len(tree.graph.edges)} edges (all rigid)"
            )
        else:
            # At higher depths, should have kinematic structure
            assert tree.root_node is not None
            assert len(tree.graph.nodes) > 0
            print(
                f"\nmax_depth={cad_doc_all_depths.max_depth}: "
                f"{len(tree.graph.nodes)} nodes, {len(tree.graph.edges)} edges"
            )

    def test_tree_without_user_defined_root(self, cad_doc: CAD):
        """Test kinematic graph with centrality-based root detection."""
        tree = KinematicGraph(cad_doc, use_user_defined_root=False)

        # Should still have a root node (detected via centrality)
        assert tree.root_node is not None
        assert tree.root_node in tree.graph.nodes
        print(f"\nCentrality-based root: {tree.root_node}")

    def test_all_nodes_are_pathkeys(self, cad_doc: CAD):
        """Test that all nodes in the tree are PathKey objects."""
        from onshape_robotics_toolkit.parse import PathKey

        tree = KinematicGraph(cad_doc)

        for node in tree.graph.nodes:
            assert isinstance(node, PathKey), f"Node {node} is not a PathKey"

    def test_nodes_match_parts_in_cad(self, cad_doc: CAD):
        """Test that all nodes in tree correspond to parts in CAD."""
        tree = KinematicGraph(cad_doc)

        for node in tree.graph.nodes:
            # Node should be in parts registry
            assert node in cad_doc.parts, f"Node {node} not found in CAD parts registry"


class TestKinematicGraphInternals:
    """Tests for internal methods of KinematicGraph class."""

    def test_collect_all_mates(self, cad_doc_depth_1: CAD):
        """Test collecting mates from root and all subassemblies."""
        tree = KinematicGraph(cad_doc_depth_1)

        # Access internal method via tree instance
        all_mates = tree._collect_all_mates()

        assert isinstance(all_mates, dict)
        # Should have mates from root assembly
        assert len(all_mates) > 0

        # All mate keys should be tuples of PathKeys
        for (parent_key, child_key), mate_data in all_mates.items():
            from onshape_robotics_toolkit.parse import PathKey

            assert isinstance(parent_key, PathKey)
            assert isinstance(child_key, PathKey)
            assert mate_data is not None

        print(f"\nCollected {len(all_mates)} mates total")

    def test_validate_mates(self, cad_doc_depth_1: CAD):
        """Test mate validation filters out invalid PathKeys."""
        tree = KinematicGraph(cad_doc_depth_1)

        # Collect all mates
        all_mates = tree._collect_all_mates()

        # Validate mates
        valid_mates = tree._validate_mates(all_mates)

        # Valid mates should be subset of all mates
        assert len(valid_mates) <= len(all_mates)

        # All valid mate PathKeys should exist in registries
        for parent_key, child_key in valid_mates:
            assert tree._is_valid_mate_target(parent_key), f"Invalid parent: {parent_key}"
            assert tree._is_valid_mate_target(child_key), f"Invalid child: {child_key}"

        print(f"\nValidated {len(valid_mates)}/{len(all_mates)} mates")

    def test_get_parts_involved_in_mates(self, cad_doc_depth_1: CAD):
        """Test extracting parts involved in mates."""
        tree = KinematicGraph(cad_doc_depth_1)

        # Collect and validate mates
        all_mates = tree._collect_all_mates()
        valid_mates = tree._validate_mates(all_mates)

        # Get involved parts
        involved_parts = tree._get_parts_involved_in_mates(valid_mates)

        assert isinstance(involved_parts, set)
        assert len(involved_parts) > 0

        # All involved parts should be PathKeys
        from onshape_robotics_toolkit.parse import PathKey

        for part_key in involved_parts:
            assert isinstance(part_key, PathKey)

        print(f"\nFound {len(involved_parts)} parts involved in mates")


class TestKinematicGraphVisualization:
    """Tests for kinematic graph visualization methods."""

    def test_show_method_exists(self, cad_doc: CAD):
        """Test that show method exists and is callable."""
        tree = KinematicGraph(cad_doc)
        assert hasattr(tree, "show")
        assert callable(tree.show)

    def test_show_creates_label_mapping(self, cad_doc_depth_1: CAD):
        """Test that show method creates proper name mappings."""
        tree = KinematicGraph(cad_doc_depth_1)

        # Manually create label mapping like show() does using new API
        label_mapping = {}
        for node in tree.graph.nodes:
            # Get instance from flat instances dict
            instance = cad_doc_depth_1.instances.get(node)
            if instance:
                label_mapping[node] = instance.name
            else:
                # Fallback to PathKey string representation
                label_mapping[node] = str(node)

        # All nodes should have labels
        assert len(label_mapping) == len(tree.graph.nodes)

        # Labels should be non-empty strings
        for _, label in label_mapping.items():
            assert isinstance(label, str)
            assert len(label) > 0

        print(f"\nCreated {len(label_mapping)} node labels")
        for node, label in list(label_mapping.items())[:5]:  # Print first 5
            print(f"  {node} -> {label}")


class TestKinematicGraphConnectivity:
    """Tests for graph connectivity and component analysis."""

    def test_single_connected_component(self, cad_doc_depth_1: CAD):
        """Test that kinematic graph has single connected component."""
        import networkx as nx

        tree = KinematicGraph(cad_doc_depth_1)

        # Convert to undirected for connectivity analysis
        undirected = tree.graph.to_undirected()
        components = list(nx.connected_components(undirected))

        # Should have exactly one connected component
        assert len(components) == 1, (
            f"Expected 1 connected component, found {len(components)}: " f"{[len(c) for c in components]}"
        )

        print(f"\nGraph is fully connected: {len(tree.graph.nodes)} nodes in single component")

    def test_no_self_loops(self, cad_doc_depth_1: CAD):
        """Test that graph has no self-loops."""
        tree = KinematicGraph(cad_doc_depth_1)

        for parent, child in tree.graph.edges:
            assert parent != child, f"Self-loop found: {parent} -> {parent}"

    def test_all_nodes_reachable_from_root(self, cad_doc_depth_1: CAD):
        """Test that all nodes are reachable from root."""
        import networkx as nx

        tree = KinematicGraph(cad_doc_depth_1)

        if tree.root_node and len(tree.graph.nodes) > 1:
            # Get descendants of root
            descendants = nx.descendants(tree.graph, tree.root_node)
            descendants.add(tree.root_node)  # Include root itself

            # All nodes should be reachable from root
            assert len(descendants) == len(tree.graph.nodes), (
                f"Not all nodes reachable from root. " f"Reachable: {len(descendants)}, Total: {len(tree.graph.nodes)}"
            )
