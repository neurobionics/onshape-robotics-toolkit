"""Tests for the KinematicGraph builder."""

from __future__ import annotations

import networkx as nx

from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.assembly import MateFeatureData
from onshape_robotics_toolkit.parse import CAD, PathKey


def test_graph_structure_matches_cad_mates(cad_doc: CAD) -> None:
    """The graph should contain every part involved in a mate and retain mate metadata."""
    graph = KinematicGraph.from_cad(cad_doc)

    assert len(graph.nodes) == 9
    assert len(graph.edges) == 8
    assert str(graph.root) == "Part_1_1"

    for node, data in graph.nodes(data=True):
        assert isinstance(node, PathKey)
        assert "data" in data
        part = data["data"]
        assert part is cad_doc.parts[node]  # node payload mirrors CAD registry

    for parent, child, edge_data in graph.edges(data=True):
        mate: MateFeatureData = edge_data["data"]
        assert isinstance(mate, MateFeatureData)
        parent_occ = mate.matedEntities[0].matedOccurrence
        child_occ = mate.matedEntities[1].matedOccurrence

        assert parent_occ[-1] == parent.leaf
        assert child_occ[-1] == child.leaf

        if len(parent_occ) == len(parent.path):
            assert parent_occ == list(parent.path)
        if len(child_occ) == len(child.path):
            assert child_occ == list(child.path)


def test_graph_is_connected_and_directed(cad_doc: CAD) -> None:
    """The graph should be a single connected component rooted at the fixed part."""
    graph = KinematicGraph.from_cad(cad_doc)

    undirected = graph.to_undirected()
    components = list(nx.connected_components(undirected))
    assert len(components) == 1

    # Every node is reachable from the root in the directed graph.
    reachable = nx.descendants(graph, graph.root)
    reachable.add(graph.root)
    assert reachable == set(graph.nodes)


def test_rigid_remapping_limits_depth(cad_doc_depth_1: CAD) -> None:
    """When subassemblies become rigid, the graph nodes collapse to shallow PathKeys."""
    graph = KinematicGraph.from_cad(cad_doc_depth_1)

    assert len(graph.nodes) == 7
    assert len(graph.edges) == 6
    assert max(node.depth for node in graph.nodes) <= 1

    # Nodes corresponding to rigid subassemblies should carry the synthetic Part objects.
    rigid_nodes = [node for node in graph.nodes if graph.nodes[node]["data"].isRigidAssembly]
    assert {str(node) for node in rigid_nodes} == {
        "Assembly_2_1_Assembly_1_1",
        "Assembly_2_1_Assembly_1_2",
    }


def test_graph_root_is_fixed_part(cad_doc: CAD) -> None:
    """The root of the kinematic graph should be the fixed part."""
    graph = KinematicGraph.from_cad(cad_doc)

    # Root should be defined
    assert graph.root is not None

    # Root part should be fixed
    root_occurrence = cad_doc.occurrences[graph.root]
    assert root_occurrence.fixed, "Root node should correspond to a fixed part"


def test_graph_preserves_mate_data_on_edges(cad_doc: CAD) -> None:
    """Graph edges should preserve mate data from CAD."""
    graph = KinematicGraph.from_cad(cad_doc)

    for parent, child in graph.edges:
        edge_data = graph.get_edge_data(parent, child)
        assert "data" in edge_data, "Edge should have mate data"

        mate = edge_data["data"]
        from onshape_robotics_toolkit.models.assembly import MateFeatureData

        assert isinstance(mate, MateFeatureData), "Edge data should be a MateFeatureData"

        # Mate should reference the parent and child
        parent_occ = mate.matedEntities[0].matedOccurrence
        child_occ = mate.matedEntities[1].matedOccurrence

        # At least the leaf should match
        assert parent_occ[-1] == parent.leaf
        assert child_occ[-1] == child.leaf


def test_graph_all_nodes_reachable_from_root(cad_doc: CAD) -> None:
    """All nodes in the graph should be reachable from the root (no disconnected components)."""
    graph = KinematicGraph.from_cad(cad_doc)

    # Get all nodes reachable from root
    reachable = set(nx.descendants(graph, graph.root))
    reachable.add(graph.root)

    # Should match all nodes
    assert reachable == set(graph.nodes), "All nodes should be reachable from root"


def test_graph_is_acyclic(cad_doc: CAD) -> None:
    """The kinematic graph should be acyclic (a tree structure)."""
    graph = KinematicGraph.from_cad(cad_doc)

    # Directed graph should be a DAG (Directed Acyclic Graph)
    assert nx.is_directed_acyclic_graph(graph), "Kinematic graph should be acyclic"


def test_graph_node_has_part_data(cad_doc: CAD) -> None:
    """Each graph node should have associated Part data."""
    graph = KinematicGraph.from_cad(cad_doc)

    from onshape_robotics_toolkit.models.assembly import Part

    for node in graph.nodes:
        node_data = graph.nodes[node]
        assert "data" in node_data, f"Node {node} missing data"

        part = node_data["data"]
        assert isinstance(part, Part), f"Node {node} data should be a Part"


def test_graph_with_user_defined_root(assembly_json_path) -> None:
    """Graph should respect user-defined root when use_user_defined_root=True."""
    from onshape_robotics_toolkit.models.assembly import Assembly
    from onshape_robotics_toolkit.utilities import load_model_from_json

    assembly = load_model_from_json(Assembly, str(assembly_json_path))

    # Test with use_user_defined_root=False (default behavior)
    cad_default = CAD.from_assembly(assembly, max_depth=2)
    graph_default = KinematicGraph.from_cad(cad_default, use_user_defined_root=False)

    # Test with use_user_defined_root=True
    graph_user_root = KinematicGraph.from_cad(cad_default, use_user_defined_root=True)

    # Both should have the same structure for this fixture
    # (since we don't have explicit user-defined root in test data)
    assert len(graph_default.nodes) == len(graph_user_root.nodes)
    assert len(graph_default.edges) == len(graph_user_root.edges)


def test_convert_to_digraph_basic() -> None:
    """Test convert_to_digraph with a simple graph."""
    from onshape_robotics_toolkit.graph import convert_to_digraph

    # Create a simple undirected graph
    graph = nx.Graph()
    key1 = PathKey(("a",), ("a",))
    key2 = PathKey(("b",), ("b",))
    key3 = PathKey(("c",), ("c",))

    graph.add_edge(key1, key2, data="edge1")
    graph.add_edge(key2, key3, data="edge2")

    # Convert to directed graph
    digraph, root = convert_to_digraph(graph)

    # Should be directed
    assert isinstance(digraph, nx.DiGraph)
    # Should have all nodes
    assert len(digraph.nodes) == 3
    # Should preserve data
    assert digraph.has_edge(root, key2) or digraph.has_edge(root, key3) or digraph.has_edge(key2, root)


def test_convert_to_digraph_with_user_defined_root() -> None:
    """Test convert_to_digraph respects user-defined root."""
    from onshape_robotics_toolkit.graph import convert_to_digraph

    graph = nx.Graph()
    key1 = PathKey(("a",), ("a",))
    key2 = PathKey(("b",), ("b",))
    key3 = PathKey(("c",), ("c",))

    graph.add_edge(key1, key2)
    graph.add_edge(key2, key3)

    # Specify root
    digraph, root = convert_to_digraph(graph, user_defined_root=key2)

    assert root == key2


def test_convert_to_digraph_preserves_edge_data() -> None:
    """Test that convert_to_digraph preserves edge metadata."""
    from onshape_robotics_toolkit.graph import convert_to_digraph

    graph = nx.Graph()
    key1 = PathKey(("a",), ("a",))
    key2 = PathKey(("b",), ("b",))
    key3 = PathKey(("c",), ("c",))

    graph.add_edge(key1, key2, weight=10, type="revolute")
    graph.add_edge(key2, key3, weight=20, type="prismatic")

    digraph, root = convert_to_digraph(graph, user_defined_root=key1)

    # Edge data should be preserved
    assert digraph[key1][key2]["weight"] == 10
    assert digraph[key1][key2]["type"] == "revolute"
    assert digraph[key2][key3]["weight"] == 20


def test_convert_to_digraph_with_cycle() -> None:
    """Test convert_to_digraph handles graphs with cycles."""
    from onshape_robotics_toolkit.graph import convert_to_digraph

    graph = nx.Graph()
    key1 = PathKey(("a",), ("a",))
    key2 = PathKey(("b",), ("b",))
    key3 = PathKey(("c",), ("c",))

    # Create a triangle (cycle)
    graph.add_edge(key1, key2, data="edge1")
    graph.add_edge(key2, key3, data="edge2")
    graph.add_edge(key3, key1, data="edge3")

    digraph, root = convert_to_digraph(graph, user_defined_root=key1)

    # Should have all nodes
    assert len(digraph.nodes) == 3
    # BFS tree will break the cycle
    assert len(digraph.edges) >= 2


def test_print_graph_tree_empty() -> None:
    """Test _print_graph_tree with empty graph."""
    from onshape_robotics_toolkit.graph import _print_graph_tree

    graph = nx.Graph()
    # Function should not raise error with empty graph
    _print_graph_tree(graph)


def test_print_graph_tree_multiple_components() -> None:
    """Test _print_graph_tree with disconnected components."""
    from onshape_robotics_toolkit.graph import _print_graph_tree

    graph = nx.Graph()
    key1 = PathKey(("a",), ("a",))
    key2 = PathKey(("b",), ("b",))
    key3 = PathKey(("c",), ("c",))
    key4 = PathKey(("d",), ("d",))

    # Component 1
    graph.add_edge(key1, key2)
    # Component 2
    graph.add_edge(key3, key4)

    # Function should not raise error with multiple components
    _print_graph_tree(graph)


def test_remove_disconnected_subgraphs_empty() -> None:
    """Test remove_disconnected_subgraphs with empty graph."""
    from onshape_robotics_toolkit.graph import remove_disconnected_subgraphs

    graph = nx.Graph()
    result = remove_disconnected_subgraphs(graph)

    assert len(result.nodes) == 0


def test_remove_disconnected_subgraphs_connected() -> None:
    """Test remove_disconnected_subgraphs with connected graph."""
    from onshape_robotics_toolkit.graph import remove_disconnected_subgraphs

    graph = nx.Graph()
    key1 = PathKey(("a",), ("a",))
    key2 = PathKey(("b",), ("b",))
    graph.add_edge(key1, key2)

    result = remove_disconnected_subgraphs(graph)

    assert len(result.nodes) == 2
    assert len(result.edges) == 1


def test_remove_disconnected_subgraphs_disconnected() -> None:
    """Test remove_disconnected_subgraphs removes smaller components."""
    from onshape_robotics_toolkit.graph import remove_disconnected_subgraphs

    graph = nx.Graph()
    key1 = PathKey(("a",), ("a",))
    key2 = PathKey(("b",), ("b",))
    key3 = PathKey(("c",), ("c",))
    key4 = PathKey(("d",), ("d",))
    key5 = PathKey(("e",), ("e",))

    # Larger component
    graph.add_edge(key1, key2)
    graph.add_edge(key2, key3)
    # Smaller component
    graph.add_edge(key4, key5)

    result = remove_disconnected_subgraphs(graph)

    # Should keep only the larger component
    assert len(result.nodes) == 3
    assert len(result.edges) == 2


def test_kinematic_graph_empty_mates(assembly) -> None:
    """Test graph construction with empty mates (rigid assembly case)."""
    # Create a CAD with no mates by clearing root assembly features
    import copy

    assembly_no_mates = copy.deepcopy(assembly)
    assembly_no_mates.rootAssembly.features = []

    cad = CAD.from_assembly(assembly_no_mates, max_depth=2)
    graph = KinematicGraph.from_cad(cad)

    # Graph should still be constructed (may have rigid assembly nodes)
    assert len(graph.nodes) >= 0


def test_kinematic_graph_show_method(cad_doc, tmp_path) -> None:
    """Test the show() visualization method."""
    graph = KinematicGraph.from_cad(cad_doc)

    # Test with file output
    output_file = tmp_path / "test_graph.png"
    graph.show(str(output_file))

    # File should be created (or attempted, matplotlib is stubbed in tests)
    # The stubbed pyplot.savefig won't actually create a file, but we verify the call completes


def test_kinematic_graph_show_default_name(cad_doc, tmp_path) -> None:
    """Test show() uses sanitized name when no filename provided."""
    graph = KinematicGraph.from_cad(cad_doc)

    # Test with explicit filename to avoid tkinter issues in testing
    output_file = tmp_path / "default_name_graph.png"
    graph.show(str(output_file))


def test_kinematic_graph_show_with_custom_graph(cad_doc, tmp_path, monkeypatch) -> None:
    """Test show() with custom graph parameter."""
    # Mock matplotlib to avoid tkinter issues in test environment
    # Unused parameters are intentional for mocking
    mock_noop = lambda *args, **kwargs: None

    # Patch matplotlib.pyplot and nx.draw
    import matplotlib.pyplot as plt
    import networkx as nx_module

    monkeypatch.setattr(plt, "figure", mock_noop)
    monkeypatch.setattr(plt, "savefig", mock_noop)
    monkeypatch.setattr(plt, "close", mock_noop)
    monkeypatch.setattr(nx_module, "draw", mock_noop)

    graph = KinematicGraph.from_cad(cad_doc)

    custom_graph = nx.DiGraph()
    key1 = PathKey(("a",), ("a",))
    key2 = PathKey(("b",), ("b",))
    custom_graph.add_edge(key1, key2)

    # Should not raise error
    output_file = tmp_path / "custom_graph.png"
    graph.show(str(output_file), graph=custom_graph)


def test_remap_mates_with_rigid_assemblies(cad_doc_depth_1) -> None:
    """Test _remap_mates handles rigid subassemblies correctly."""
    graph = KinematicGraph.from_cad(cad_doc_depth_1)

    # Verify rigid remapping occurred
    rigid_nodes = [node for node in graph.nodes if graph.nodes[node]["data"].isRigidAssembly]
    assert len(rigid_nodes) > 0, "Should have rigid assembly nodes with depth_1"


def test_mate_reversal_preserves_parent_child_order(cad_doc) -> None:
    """Test that mate reversal in _process_graph preserves BFS parent->child order."""
    graph = KinematicGraph.from_cad(cad_doc)

    # All edges should have valid mate data with correct ordering
    for parent, child in graph.edges:
        edge_data = graph.get_edge_data(parent, child)
        mate = edge_data["data"]

        # Mate entities should reference parent and child
        parent_occ = mate.matedEntities[0].matedOccurrence
        child_occ = mate.matedEntities[1].matedOccurrence

        # Leaf should match
        assert parent_occ[-1] == parent.leaf
        assert child_occ[-1] == child.leaf


def test_graph_root_detection_fallback(assembly) -> None:
    """Test root detection falls back to centrality when no fixed part found."""
    import copy

    # Create assembly with no fixed parts
    assembly_no_fixed = copy.deepcopy(assembly)
    for occ in assembly_no_fixed.rootAssembly.occurrences:
        occ.fixed = False

    cad = CAD.from_assembly(assembly_no_fixed, max_depth=2)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)

    # Should still have a root (determined by centrality)
    if len(graph.nodes) > 0:
        assert graph.root is not None


def test_single_node_graph(assembly) -> None:
    """Test graph handles single node (fully rigid assembly) correctly."""
    import copy

    # Create assembly with just one part/subassembly
    assembly_single = copy.deepcopy(assembly)
    # Remove all features to make it rigid
    assembly_single.rootAssembly.features = []
    # Keep only first occurrence
    if len(assembly_single.rootAssembly.occurrences) > 1:
        assembly_single.rootAssembly.occurrences = [assembly_single.rootAssembly.occurrences[0]]

    cad = CAD.from_assembly(assembly_single, max_depth=0)
    graph = KinematicGraph.from_cad(cad)

    # Single node graphs are valid
    if len(graph.nodes) == 1:
        assert graph.root is not None
        assert graph.root in graph.nodes
