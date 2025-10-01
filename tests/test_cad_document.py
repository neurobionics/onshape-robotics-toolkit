"""Tests for CADDocument class and basic functionality."""

import pytest

from onshape_robotics_toolkit.models.assembly import Assembly
from onshape_robotics_toolkit.parse import CADDocument, RootAssemblyData


class TestCADDocument:
    """Tests for CADDocument class."""

    def test_assembly_data_structure(self, cad_doc: CADDocument):
        """Document the test assembly data structure."""
        # This test documents what's in the test data to help understand skip conditions
        print("\n=== Test Assembly Structure ===")
        print(f"Total parts: {len(cad_doc.parts)}")
        print(f"Part instances: {len(cad_doc.root_assembly.instances.parts)}")
        print(f"Assembly instances: {len(cad_doc.root_assembly.instances.assemblies)}")

        # Part depths
        part_depths = {}
        for key in cad_doc.root_assembly.instances.parts:
            part_depths[key.depth] = part_depths.get(key.depth, 0) + 1
        print(f"Part depths: {dict(sorted(part_depths.items()))}")

        # Assembly depths
        assembly_depths = {}
        for key in cad_doc.root_assembly.instances.assemblies:
            assembly_depths[key.depth] = assembly_depths.get(key.depth, 0) + 1
        print(f"Assembly depths: {dict(sorted(assembly_depths.items()))}")

        # This will always pass - it's just for documentation
        assert True

    def test_create_from_assembly(self, cad_doc: CADDocument, assembly: Assembly):
        """Test creating CADDocument from Assembly."""
        assert cad_doc is not None
        assert cad_doc.document_id == assembly.rootAssembly.documentId
        assert cad_doc.element_id == assembly.rootAssembly.elementId
        assert cad_doc.current_depth == 0
        assert cad_doc.max_depth == 0

    def test_root_assembly_populated(self, cad_doc: CADDocument):
        """Test that root assembly data is populated."""
        assert cad_doc.root_assembly is not None
        assert isinstance(cad_doc.root_assembly, RootAssemblyData)

        # Check that registries exist
        assert cad_doc.root_assembly.instances is not None
        assert cad_doc.root_assembly.occurrences is not None
        assert cad_doc.root_assembly.mates is not None
        assert cad_doc.root_assembly.patterns is not None

    def test_parts_populated(self, cad_doc: CADDocument, assembly: Assembly):
        """Test that parts dictionary is populated."""
        assert cad_doc.parts is not None
        assert len(cad_doc.parts) == len(assembly.parts)

        # Check that all part IDs are present
        for part in assembly.parts:
            assert part.partId in cad_doc.parts

    def test_instances_populated(self, cad_doc: CADDocument):
        """Test that instances are populated in the registry."""
        # Should have both parts and assemblies
        total_instances = len(cad_doc.root_assembly.instances.parts) + len(cad_doc.root_assembly.instances.assemblies)

        assert total_instances > 0
        print(f"\nFound {len(cad_doc.root_assembly.instances.parts)} part instances")
        print(f"Found {len(cad_doc.root_assembly.instances.assemblies)} assembly instances")

    def test_occurrences_populated(self, cad_doc: CADDocument):
        """Test that occurrences are populated in the registry."""
        occurrences_count = len(cad_doc.root_assembly.occurrences.occurrences)
        assert occurrences_count > 0
        print(f"\nFound {occurrences_count} occurrences")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
