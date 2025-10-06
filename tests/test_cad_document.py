"""Tests for CAD class and basic functionality."""

import pytest

from onshape_robotics_toolkit.models.assembly import Assembly, AssemblyInstance, MatedCS, PartInstance
from onshape_robotics_toolkit.parse import CAD, PathKey


class TestCAD:
    """Tests for CAD class with flat structure."""

    def test_create_from_assembly(self, cad_doc: CAD, assembly: Assembly):
        """Test creating CAD from Assembly."""
        assert cad_doc is not None
        assert cad_doc.document_id == assembly.rootAssembly.documentId
        assert cad_doc.element_id == assembly.rootAssembly.elementId
        assert cad_doc.max_depth == 2  # Using max_depth=2 for test fixture

    def test_flat_structure_populated(self, cad_doc: CAD):
        """Test that flat registries are populated."""
        # Check that flat registries exist
        assert cad_doc.instances is not None
        assert cad_doc.occurrences is not None
        assert cad_doc.mates is not None
        assert cad_doc.patterns is not None

        # Should have some data
        assert len(cad_doc.instances) > 0
        assert len(cad_doc.occurrences) > 0

    def test_instances_are_pathkey_indexed(self, cad_doc: CAD):
        """Test that instances are indexed by PathKey."""
        for key, instance in cad_doc.instances.items():
            assert isinstance(key, PathKey), "Instance keys should be PathKey"
            assert isinstance(instance, (PartInstance, AssemblyInstance))

    def test_occurrences_are_pathkey_indexed(self, cad_doc: CAD):
        """Test that occurrences are indexed by PathKey."""
        for key in cad_doc.occurrences:
            assert isinstance(key, PathKey), "Occurrence keys should be PathKey"

    def test_mates_have_assembly_provenance(self, cad_doc: CAD):
        """Test that mates have assembly provenance in keys."""
        # Mate keys should be 3-tuples: (assembly_key, parent_key, child_key)
        for mate_key in cad_doc.mates:
            assert isinstance(mate_key, tuple)
            assert len(mate_key) == 3
            assembly_key, parent_key, child_key = mate_key
            # Assembly key is None for root, PathKey for subassembly
            assert assembly_key is None or isinstance(assembly_key, PathKey)
            assert isinstance(parent_key, PathKey)
            assert isinstance(child_key, PathKey)

    def test_occurrences_include_nested_parts(self, cad_doc: CAD):
        """Test that occurrences include nested parts from subassemblies."""
        # Occurrences includes ALL parts in the tree (including nested)
        # Instances includes only root-level instances
        # So occurrences >= instances
        assert len(cad_doc.occurrences) >= len(
            cad_doc.instances
        ), f"Occurrences ({len(cad_doc.occurrences)}) should >= instances ({len(cad_doc.instances)})"

        # All instance keys should be in occurrences
        for key in cad_doc.instances:
            assert key in cad_doc.occurrences, f"Instance {key} should have corresponding occurrence"

    def test_parts_registry_populated(self, cad_doc: CAD):
        """Test that parts dict is populated from assembly.parts."""
        # Parts should be populated during from_assembly()
        # Should have parts for all part instances + rigid assemblies
        assert len(cad_doc.parts) > 0, "Parts should be populated from assembly"

        # Every part should have a corresponding instance
        for part_key in cad_doc.parts:
            assert part_key in cad_doc.instances, f"Part {part_key} should have instance"

    def test_pathkey_dual_indexing(self, cad_doc: CAD):
        """Test that PathKey dual indexing works."""
        assert len(cad_doc.keys) > 0
        assert len(cad_doc.keys_by_name) > 0

        # Both indexes should have same number of entries
        assert len(cad_doc.keys) == len(cad_doc.keys_by_name)

    def test_get_path_key_lookup(self, cad_doc: CAD):
        """Test PathKey lookup by ID."""
        # Get a sample PathKey
        sample_key = next(iter(cad_doc.instances.keys()))

        # Should be able to look it up by ID tuple
        found_key = cad_doc.get_path_key(sample_key.path)
        assert found_key == sample_key

    def test_get_path_key_by_name_lookup(self, cad_doc: CAD):
        """Test PathKey lookup by name."""
        # Get a sample PathKey
        sample_key = next(iter(cad_doc.instances.keys()))

        # Should be able to look it up by name tuple
        found_key = cad_doc.get_path_key_by_name(sample_key.name_path)
        assert found_key == sample_key

    def test_is_part_and_is_rigid_assembly(self, cad_doc: CAD):
        """Test instance type checking methods."""
        for key in cad_doc.instances:
            is_part = cad_doc.is_part(key)
            is_rigid = cad_doc.is_rigid_assembly(key)
            is_flex = cad_doc.is_flexible_assembly(key)

            # Should be one of the three types
            assert is_part or is_rigid or is_flex

    def test_get_mates_from_root(self, cad_doc: CAD):
        """Test getting only root-level mates."""
        root_mates = cad_doc.get_mates_from_root()

        # All mates in result should have None as assembly key
        for parent_key, child_key in root_mates:
            assert isinstance(parent_key, PathKey)
            assert isinstance(child_key, PathKey)

            # Verify this mate exists in full mates dict with None assembly
            assert (None, parent_key, child_key) in cad_doc.mates

    def test_get_all_mates_flattened(self, cad_doc: CAD):
        """Test getting all mates without assembly provenance."""
        flat_mates = cad_doc.get_all_mates_flattened()

        # Should have (parent, child) tuples as keys
        for mate_key in flat_mates:
            assert isinstance(mate_key, tuple)
            assert len(mate_key) == 2
            parent_key, child_key = mate_key
            assert isinstance(parent_key, PathKey)
            assert isinstance(child_key, PathKey)

    def test_assembly_data_structure(self, cad_doc: CAD):
        """Document the test assembly data structure."""
        print("\n=== Test Assembly Structure ===")
        print(f"Total instances: {len(cad_doc.instances)}")

        # Count part vs assembly instances
        part_count = sum(1 for inst in cad_doc.instances.values() if isinstance(inst, PartInstance))
        asm_count = sum(1 for inst in cad_doc.instances.values() if isinstance(inst, AssemblyInstance))
        print(f"Part instances: {part_count}")
        print(f"Assembly instances: {asm_count}")

        # Part depths
        part_depths = {}
        for key, inst in cad_doc.instances.items():
            if isinstance(inst, PartInstance):
                part_depths[key.depth] = part_depths.get(key.depth, 0) + 1
        print(f"Part depths: {dict(sorted(part_depths.items()))}")

        # Assembly depths
        assembly_depths = {}
        for key, inst in cad_doc.instances.items():
            if isinstance(inst, AssemblyInstance):
                assembly_depths[key.depth] = assembly_depths.get(key.depth, 0) + 1
        print(f"Assembly depths: {dict(sorted(assembly_depths.items()))}")

        print(f"Total mates: {len(cad_doc.mates)}")
        print(f"Total patterns: {len(cad_doc.patterns)}")

        # This will always pass - it's just for documentation
        assert True

    def test_parts_with_rigid_parent_have_transform(self, cad_doc_depth_1: CAD):
        """Parts inside rigid subassemblies should record rigid transforms."""
        parts_with_transform = [
            (key, part)
            for key, part in cad_doc_depth_1.parts.items()
            if cad_doc_depth_1.is_part(key) and part.rigidAssemblyToPartTF
        ]

        assert parts_with_transform, "Expected at least one part inside a rigid subassembly to have transforms"

        for key, part in parts_with_transform:
            rigid_root = cad_doc_depth_1.get_rigid_assembly_root(key)
            assert rigid_root is not None, "Transform present should imply rigid assembly ancestor"

            transforms = part.rigidAssemblyToPartTF
            assert isinstance(transforms, dict)
            assert len(transforms) == 1

            transform_key, mated_cs = next(iter(transforms.items()))
            assert transform_key == "|".join(rigid_root.path)
            assert isinstance(mated_cs, MatedCS)

    def test_unique_names_with_duplicate_inputs(self, assembly: Assembly):
        """Duplicated instance names should yield unique hierarchical paths."""
        # Force duplicate names across root and subassembly instances
        for inst in assembly.rootAssembly.instances:
            inst.name = "duplicate"
        for sub in assembly.subAssemblies:
            for inst in sub.instances:
                inst.name = "duplicate"

        cad = CAD.from_assembly(assembly, max_depth=2)

        name_paths = [key.name_path for key in cad.keys.values()]
        assert len(name_paths) == len({tuple(path) for path in name_paths}), "Name paths must be unique"

        hierarchical_names = [key.hierarchical_name(separator="-") for key in cad.keys.values()]
        assert len(hierarchical_names) == len(set(hierarchical_names)), "Hierarchical names must be unique"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
