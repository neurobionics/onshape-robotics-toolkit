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
