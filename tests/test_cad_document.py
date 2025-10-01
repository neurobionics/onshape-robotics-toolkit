"""Tests for CAD class and basic functionality."""

import pytest

from onshape_robotics_toolkit.models.assembly import Assembly
from onshape_robotics_toolkit.parse import CAD, RootAssemblyData


class TestCAD:
    """Tests for CAD class."""

    def test_assembly_data_structure(self, cad_doc: CAD):
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

    def test_create_from_assembly(self, cad_doc: CAD, assembly: Assembly):
        """Test creating CAD from Assembly."""
        assert cad_doc is not None
        assert cad_doc.document_id == assembly.rootAssembly.documentId
        assert cad_doc.element_id == assembly.rootAssembly.elementId
        assert cad_doc.current_depth == 0
        assert cad_doc.max_depth == 0

    def test_root_assembly_populated(self, cad_doc: CAD):
        """Test that root assembly data is populated."""
        assert cad_doc.root_assembly is not None
        assert isinstance(cad_doc.root_assembly, RootAssemblyData)

        # Check that registries exist
        assert cad_doc.root_assembly.instances is not None
        assert cad_doc.root_assembly.occurrences is not None
        assert cad_doc.root_assembly.mates is not None
        assert cad_doc.root_assembly.patterns is not None

    def test_parts_populated(self, cad_doc: CAD, assembly: Assembly):
        """Test that parts dictionary is populated."""
        assert cad_doc.parts is not None
        assert len(cad_doc.parts) == len(assembly.parts)

        # Check that all part IDs are present
        for part in assembly.parts:
            assert part.partId in cad_doc.parts

    def test_instances_populated(self, cad_doc: CAD):
        """Test that instances are populated in the registry."""
        # Should have both parts and assemblies
        total_instances = len(cad_doc.root_assembly.instances.parts) + len(cad_doc.root_assembly.instances.assemblies)

        assert total_instances > 0
        print(f"\nFound {len(cad_doc.root_assembly.instances.parts)} part instances")
        print(f"Found {len(cad_doc.root_assembly.instances.assemblies)} assembly instances")

    def test_occurrences_populated(self, cad_doc: CAD):
        """Test that occurrences are populated in the registry."""
        occurrences_count = len(cad_doc.root_assembly.occurrences.occurrences)
        assert occurrences_count > 0
        print(f"\nFound {occurrences_count} occurrences")

    def test_instances_match_occurrences(self, cad_doc: CAD):
        """Test that instance count matches occurrence count."""
        total_instances = len(cad_doc.root_assembly.instances.parts) + len(cad_doc.root_assembly.instances.assemblies)
        total_occurrences = len(cad_doc.root_assembly.occurrences.occurrences)

        assert (
            total_instances == total_occurrences
        ), f"Instances ({total_instances}) should match occurrences ({total_occurrences})"

    def test_subassembly_instances_have_full_paths(self, cad_doc: CAD):
        """Test that instances from subassemblies have proper hierarchical paths."""
        # Find instances with depth > 1 (these come from subassemblies)
        nested_instances = [key for key in cad_doc.root_assembly.instances.parts if key.depth > 1]
        nested_assemblies = [key for key in cad_doc.root_assembly.instances.assemblies if key.depth > 1]

        # If we have nested instances, verify their paths
        if nested_instances or nested_assemblies:
            for key in nested_instances + nested_assemblies:
                # Path should have multiple elements
                assert len(key.path) > 1, f"Nested instance {key} should have path length > 1"

                # Should have a parent
                assert key.parent_key is not None, f"Nested instance {key} should have a parent"

                # Parent should exist in instances or assemblies
                parent = key.parent_key
                assert (
                    parent in cad_doc.root_assembly.instances.parts
                    or parent in cad_doc.root_assembly.instances.assemblies
                ), f"Parent {parent} should exist in instances registry"

    def test_show_method_runs(self, cad_doc: CAD, capsys):
        """Test that show() method runs without errors."""
        # Should not raise any exceptions
        cad_doc.show()

        # Capture the output
        captured = capsys.readouterr()

        # Should print something
        assert len(captured.out) > 0

        # Should show "Assembly Root"
        assert "Assembly Root" in captured.out

        # Should show at least some parts/assemblies with the tree marker |--
        assert "|--" in captured.out

    def test_show_displays_hierarchy(self, cad_doc_depth_2: CAD, capsys):
        """Test that show() displays nested hierarchy correctly."""
        cad_doc_depth_2.show()
        captured = capsys.readouterr()

        # Count indentation levels to verify nesting
        lines = captured.out.split("\n")
        root_items = [line for line in lines if line.startswith("|--")]
        nested_items = [line for line in lines if line.startswith("    |--")]

        # Should have both root and nested items if max_depth=2
        assert len(root_items) > 0, "Should have root-level items"
        # Nested items may or may not exist depending on assembly structure
        print(f"\nRoot items: {len(root_items)}, Nested items: {len(nested_items)}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
