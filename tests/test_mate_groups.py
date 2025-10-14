"""
Unit tests for mate group feature support.

Tests the behavior of assemblies containing mate groups:
- Assemblies with only mate groups should be treated as rigid
- Assemblies with mate groups + regular mates should remain flexible
- Mate groups should not create kinematic edges in the robot graph
"""

from onshape_robotics_toolkit.models.assembly import (
    AssemblyFeature,
    AssemblyFeatureType,
    MateFeatureData,
    MateGroupFeatureData,
    MateGroupFeatureOccurrence,
    MateType,
    SubAssembly,
)
from onshape_robotics_toolkit.parse import CAD


def test_is_rigid_by_features_empty_assembly():
    """Test that assemblies with no features are considered rigid."""
    # Create a SubAssembly with no features
    subassembly = SubAssembly(
        instances=[],
        patterns=[],
        features=[],  # Empty features
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="b" * 24,
        documentMicroversion="c" * 24,
    )

    # Create a minimal CAD object to access the method
    cad = CAD(
        document_id="a" * 24,
        element_id="b" * 24,
        wtype="w",
        workspace_id="w" * 24,
        document_microversion="c" * 24,
        name="test",
        max_depth=0,
    )

    # Test that empty assembly is rigid
    assert cad._is_rigid_by_features(subassembly) is True


def test_is_rigid_by_features_only_mate_groups():
    """Test that assemblies with only mate groups are considered rigid."""
    # Create mate group features
    mate_group_1 = AssemblyFeature(
        id="MG1",
        suppressed=False,
        featureType=AssemblyFeatureType.MATEGROUP,
        featureData=MateGroupFeatureData(
            occurrences=[MateGroupFeatureOccurrence(occurrence=["part1"])],
            name="Mate group 1",
            id="MG1",
        ),
    )

    mate_group_2 = AssemblyFeature(
        id="MG2",
        suppressed=False,
        featureType=AssemblyFeatureType.MATEGROUP,
        featureData=MateGroupFeatureData(
            occurrences=[MateGroupFeatureOccurrence(occurrence=["part2"])],
            name="Mate group 2",
            id="MG2",
        ),
    )

    # Create a SubAssembly with only mate groups
    subassembly = SubAssembly(
        instances=[],
        patterns=[],
        features=[mate_group_1, mate_group_2],  # Only mate groups
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="b" * 24,
        documentMicroversion="c" * 24,
    )

    # Create a minimal CAD object to access the method
    cad = CAD(
        document_id="a" * 24,
        element_id="b" * 24,
        wtype="w",
        workspace_id="w" * 24,
        document_microversion="c" * 24,
        name="test",
        max_depth=0,
    )

    # Test that mate-group-only assembly is rigid
    assert cad._is_rigid_by_features(subassembly) is True


def test_is_rigid_by_features_mixed_features():
    """Test that assemblies with mate groups + regular mates are flexible."""
    from onshape_robotics_toolkit.models.assembly import MatedCS, MatedEntity

    # Create a mate group
    mate_group = AssemblyFeature(
        id="MG1",
        suppressed=False,
        featureType=AssemblyFeatureType.MATEGROUP,
        featureData=MateGroupFeatureData(
            occurrences=[MateGroupFeatureOccurrence(occurrence=["part1"])],
            name="Mate group 1",
            id="MG1",
        ),
    )

    # Create a regular mate (revolute)
    regular_mate = AssemblyFeature(
        id="M1",
        suppressed=False,
        featureType=AssemblyFeatureType.MATE,
        featureData=MateFeatureData(
            matedEntities=[
                MatedEntity(
                    matedOccurrence=["part1"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[0.0, 0.0, 0.0],
                    ),
                ),
                MatedEntity(
                    matedOccurrence=["part2"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[0.0, 0.0, 0.0],
                    ),
                ),
            ],
            mateType=MateType.REVOLUTE,
            name="Revolute 1",
            id="M1",
        ),
    )

    # Create a SubAssembly with both mate groups and regular mates
    subassembly = SubAssembly(
        instances=[],
        patterns=[],
        features=[mate_group, regular_mate],  # Mixed features
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="b" * 24,
        documentMicroversion="c" * 24,
    )

    # Create a minimal CAD object to access the method
    cad = CAD(
        document_id="a" * 24,
        element_id="b" * 24,
        wtype="w",
        workspace_id="w" * 24,
        document_microversion="c" * 24,
        name="test",
        max_depth=0,
    )

    # Test that mixed assembly is NOT rigid (flexible)
    assert cad._is_rigid_by_features(subassembly) is False


def test_is_rigid_by_features_suppressed_features():
    """Test that suppressed features are ignored when determining rigidity."""
    # Create a suppressed mate group
    suppressed_mate_group = AssemblyFeature(
        id="MG1",
        suppressed=True,  # Suppressed!
        featureType=AssemblyFeatureType.MATEGROUP,
        featureData=MateGroupFeatureData(
            occurrences=[MateGroupFeatureOccurrence(occurrence=["part1"])],
            name="Mate group 1",
            id="MG1",
        ),
    )

    # Create a SubAssembly with only suppressed features
    subassembly = SubAssembly(
        instances=[],
        patterns=[],
        features=[suppressed_mate_group],  # Only suppressed features
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="b" * 24,
        documentMicroversion="c" * 24,
    )

    # Create a minimal CAD object to access the method
    cad = CAD(
        document_id="a" * 24,
        element_id="b" * 24,
        wtype="w",
        workspace_id="w" * 24,
        document_microversion="c" * 24,
        name="test",
        max_depth=0,
    )

    # Test that assembly with only suppressed features is rigid (treated as empty)
    assert cad._is_rigid_by_features(subassembly) is True


def test_rigid_assembly_only_feature_types_constant():
    """Test that the RIGID_ASSEMBLY_ONLY_FEATURE_TYPES constant is properly defined."""
    from onshape_robotics_toolkit.parse import RIGID_ASSEMBLY_ONLY_FEATURE_TYPES

    # Verify the constant includes MATEGROUP
    assert AssemblyFeatureType.MATEGROUP in RIGID_ASSEMBLY_ONLY_FEATURE_TYPES

    # Verify it's a set (for efficient lookups)
    assert isinstance(RIGID_ASSEMBLY_ONLY_FEATURE_TYPES, set)


def test_empty_graph_for_mate_group_only_assembly():
    """Test that assemblies with only mate groups result in empty graphs (no errors)."""
    import networkx as nx

    from onshape_robotics_toolkit.graph import remove_disconnected_subgraphs

    # Create an empty graph (simulating assembly with only mate groups)
    empty_graph = nx.Graph()

    # This should not raise an exception (previously raised NetworkXPointlessConcept)
    result = remove_disconnected_subgraphs(empty_graph)

    # Should return the empty graph unchanged
    assert len(result.nodes) == 0
    assert len(result.edges) == 0
