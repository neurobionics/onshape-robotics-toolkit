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
