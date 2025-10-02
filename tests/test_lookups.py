"""Tests for lookup functionality in CAD and registries."""

import pytest

# Import helpers from conftest
from conftest import (
    get_first_assembly_key,
    get_first_occurrence_key,
    get_first_part_id,
    get_first_part_key,
    get_flexible_assembly_keys,
    get_nested_part_key,
    get_root_part_key,
)

from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name


class TestLookups:
    """Tests for looking up instances, occurrences, parts, and subassemblies."""

    def test_lookup_part_instance_by_key(self, cad_doc: CAD):
        """Test looking up a part instance by PathKey."""
        key = get_first_part_key(cad_doc)
        assert key is not None, "No part instances found"

        part_instance = cad_doc.get_part_instance(key)

        assert part_instance is not None
        assert hasattr(part_instance, "name")
        assert hasattr(part_instance, "partId")
        print(f"\nFound part instance: {part_instance.name}")

    def test_lookup_assembly_instance_by_key(self, cad_doc: CAD):
        """Test looking up an assembly instance by PathKey."""
        key = get_first_assembly_key(cad_doc)
        if key is None:
            pytest.skip("No assembly instances found in test data")

        assembly_instance = cad_doc.get_assembly_instance(key)

        assert assembly_instance is not None
        assert hasattr(assembly_instance, "name")
        assert hasattr(assembly_instance, "documentId")
        print(f"\nFound assembly instance: {assembly_instance.name}")

    def test_lookup_occurrence_by_key(self, cad_doc: CAD):
        """Test looking up an occurrence by PathKey."""
        key = get_first_occurrence_key(cad_doc)
        assert key is not None, "No occurrences found"

        occurrence = cad_doc.get_occurrence(key)

        assert occurrence is not None
        assert hasattr(occurrence, "path")
        assert hasattr(occurrence, "transform")
        print(f"\nFound occurrence with path: {occurrence.path}")

    def test_lookup_part_by_pathkey(self, cad_doc: CAD):
        """Test looking up a part definition by PathKey."""
        part_key = get_first_part_id(cad_doc)  # This now returns PathKey, not part_id
        assert part_key is not None, "No parts found"

        part = cad_doc.get_part(part_key)

        assert part is not None
        print(f"\nFound part: {part.name if hasattr(part, 'name') else str(part_key)}")

    def test_lookup_transform_by_key(self, cad_doc: CAD):
        """Test looking up a transform matrix by PathKey."""
        key = get_first_occurrence_key(cad_doc)
        assert key is not None, "No occurrences found"

        transform = cad_doc.get_transform(key)

        assert transform is not None
        assert transform.shape == (4, 4), "Transform should be 4x4 matrix"
        print(f"\nFound transform:\n{transform}")

    def test_lookup_instance_by_name(self, cad_doc: CAD):
        """Test looking up instances by sanitized name."""
        key = get_first_part_key(cad_doc)
        if key is None:
            pytest.skip("No part instances found")

        part_instance = cad_doc.root_assembly.instances.get_part(key)
        assert part_instance is not None

        # Lookup by name
        sanitized_name = get_sanitized_name(part_instance.name)
        found_keys = cad_doc.root_assembly.instances.lookup_by_name(sanitized_name)

        assert len(found_keys) > 0, f"Should find at least one instance with name {sanitized_name}"
        assert key in found_keys, "Original key should be in found keys"
        print(f"\nFound {len(found_keys)} instance(s) with name: {sanitized_name}")

    def test_lookup_instance_by_name_and_depth(self, cad_doc: CAD):
        """Test looking up instances by name filtered by depth."""
        key = get_root_part_key(cad_doc)
        if key is None:
            pytest.skip("No root-level part instances found")

        part_instance = cad_doc.root_assembly.instances.get_part(key)
        assert part_instance is not None

        # Lookup by name and depth
        sanitized_name = get_sanitized_name(part_instance.name)
        found_keys = cad_doc.root_assembly.instances.lookup_by_name(sanitized_name, depth=1)

        assert len(found_keys) > 0, f"Should find at least one root instance with name {sanitized_name}"
        assert key in found_keys, "Original key should be in found keys"

        # Verify all found keys have depth 1
        for found_key in found_keys:
            assert found_key.depth == 1, "All found keys should have depth 1"

        print(f"\nFound {len(found_keys)} root-level instance(s) with name: {sanitized_name}")

    def test_lookup_hierarchical_name(self, cad_doc: CAD):
        """Test getting hierarchical name for an instance."""
        # Try to get a nested part first
        key = get_nested_part_key(cad_doc)

        if key is not None:
            # Test nested part
            hierarchical_name = cad_doc.root_assembly.instances.get_hierarchical_name(key)

            # Should contain separator ":"
            assert ":" in hierarchical_name, "Hierarchical name should contain separator"
            # Number of separators should be depth - 1
            assert hierarchical_name.count(":") == key.depth - 1
            print(f"\nHierarchical name for depth {key.depth}: {hierarchical_name}")
        else:
            # Fall back to root-level part
            key = get_first_part_key(cad_doc)
            assert key is not None, "No part instances found"

            hierarchical_name = cad_doc.root_assembly.instances.get_hierarchical_name(key)
            assert hierarchical_name != "", "Should have a name"
            print(f"\nRoot-level name: {hierarchical_name}")

    def test_get_all_subassemblies_non_recursive(self, cad_doc_depth_1: CAD):
        """Test getting all direct subassemblies (non-recursive)."""
        # Get all subassemblies (non-recursive)
        subassemblies = cad_doc_depth_1.get_all_subassemblies(recursive=False)

        # Should return the same as cad_doc.fetched_subassemblies
        assert (
            subassemblies is cad_doc_depth_1.fetched_subassemblies
            or subassemblies == cad_doc_depth_1.fetched_subassemblies
        )
        print(f"\nFound {len(subassemblies)} direct fetched subassemblies")

    def test_get_subassembly_by_key(self, cad_doc_depth_2: CAD):
        """Test getting a specific subassembly by PathKey."""
        # With max depth 2, assemblies at depth 1 should be flexible
        flexible_keys = get_flexible_assembly_keys(cad_doc_depth_2)
        if not flexible_keys:
            pytest.skip("No assembly instances found in test data")

        key = flexible_keys[0]

        # Initially should be None (not fetched yet)
        subassembly = cad_doc_depth_2.get_subassembly(key)
        assert subassembly is None, "Subassembly should not be fetched yet"
        print(f"\nSubassembly at key {key.path} not fetched (as expected)")

    def test_distinguish_rigid_vs_flexible_assemblies(self, cad_doc_depth_1: CAD):
        """Test distinguishing between rigid and flexible assemblies."""
        assembly_keys = list(cad_doc_depth_1.root_assembly.instances.assemblies.keys())
        if not assembly_keys:
            pytest.skip("No assembly instances found")

        rigid_count = sum(1 for key in assembly_keys if cad_doc_depth_1.root_assembly.instances.is_rigid_assembly(key))
        flexible_count = sum(
            1 for key in assembly_keys if cad_doc_depth_1.root_assembly.instances.is_flexible_assembly(key)
        )

        print(f"\nFound {rigid_count} rigid assemblies and {flexible_count} flexible assemblies (max_depth=1)")
        # Each assembly should be either rigid or flexible, not both
        assert rigid_count + flexible_count == len(assembly_keys)


class TestLookupsAllDepths:
    """Test lookups across all max_depth configurations (0, 1, 2).

    These tests use the parametrized cad_doc_all_depths fixture,
    so each test runs 3 times (once for each max_depth).
    """

    def test_part_instance_lookup_all_depths(self, cad_doc_all_depths: CAD):
        """Test part instance lookup works at all depth configurations."""
        key = get_first_part_key(cad_doc_all_depths)
        assert key is not None, "Should always have parts"

        part_instance = cad_doc_all_depths.get_part_instance(key)
        assert part_instance is not None
        assert hasattr(part_instance, "name")
        assert hasattr(part_instance, "partId")

    def test_occurrence_lookup_all_depths(self, cad_doc_all_depths: CAD):
        """Test occurrence lookup works at all depth configurations."""
        key = get_first_occurrence_key(cad_doc_all_depths)
        assert key is not None, "Should always have occurrences"

        occurrence = cad_doc_all_depths.get_occurrence(key)
        assert occurrence is not None
        assert hasattr(occurrence, "path")
        assert hasattr(occurrence, "transform")

    def test_transform_lookup_all_depths(self, cad_doc_all_depths: CAD):
        """Test transform lookup works at all depth configurations."""
        key = get_first_occurrence_key(cad_doc_all_depths)
        assert key is not None

        transform = cad_doc_all_depths.get_transform(key)
        assert transform is not None
        assert transform.shape == (4, 4)

    def test_rigid_flexible_classification_all_depths(self, cad_doc_all_depths: CAD):
        """Test rigid/flexible classification at all depth configurations."""
        assembly_keys = list(cad_doc_all_depths.root_assembly.instances.assemblies.keys())
        if not assembly_keys:
            pytest.skip("No assembly instances in test data")

        # Every assembly should be classified as either rigid or flexible, not both
        for key in assembly_keys:
            is_rigid = cad_doc_all_depths.root_assembly.instances.is_rigid_assembly(key)
            is_flexible = cad_doc_all_depths.root_assembly.instances.is_flexible_assembly(key)

            # Exactly one should be true
            assert is_rigid != is_flexible, f"Assembly at {key} should be either rigid XOR flexible"

            # Classification depends on depth vs max_depth
            expected_flexible = key.depth <= cad_doc_all_depths.max_depth
            assert (
                is_flexible == expected_flexible
            ), f"Assembly at depth {key.depth} with max_depth={cad_doc_all_depths.max_depth}"

    def test_name_lookup_all_depths(self, cad_doc_all_depths: CAD):
        """Test name-based lookup works at all depth configurations."""
        key = get_first_part_key(cad_doc_all_depths)
        if key is None:
            pytest.skip("No part instances")

        part_instance = cad_doc_all_depths.root_assembly.instances.get_part(key)
        assert part_instance is not None

        sanitized_name = get_sanitized_name(part_instance.name)
        found_keys = cad_doc_all_depths.root_assembly.instances.lookup_by_name(sanitized_name)

        assert len(found_keys) > 0
        assert key in found_keys


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
