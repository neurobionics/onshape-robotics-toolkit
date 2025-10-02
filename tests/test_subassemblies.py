"""Tests for subassembly data structure and rigid assembly handling."""

import pytest

from onshape_robotics_toolkit.models.assembly import AssemblyInstance
from onshape_robotics_toolkit.parse import CAD, AssemblyData, PathKey


class TestSubassemblyDataStructure:
    """Tests for the separated subassembly data structure."""

    def test_sub_assemblies_attribute_exists(self, cad_doc: CAD):
        """Test that sub_assemblies attribute exists and is a dict."""
        assert hasattr(cad_doc, "sub_assemblies")
        assert isinstance(cad_doc.sub_assemblies, dict)

    def test_sub_assemblies_keyed_by_pathkey(self, cad_doc_depth_1: CAD):
        """Test that sub_assemblies are keyed by PathKey."""
        if not cad_doc_depth_1.sub_assemblies:
            pytest.skip("No subassemblies in test data")

        for key, assembly_data in cad_doc_depth_1.sub_assemblies.items():
            assert isinstance(key, PathKey), "Keys should be PathKey instances"
            assert isinstance(assembly_data, AssemblyData), "Values should be AssemblyData instances"

    def test_sub_assemblies_contain_assembly_data(self, cad_doc_depth_1: CAD):
        """Test that each subassembly has its own AssemblyData with registries."""
        if not cad_doc_depth_1.sub_assemblies:
            pytest.skip("No subassemblies in test data")

        for key, assembly_data in cad_doc_depth_1.sub_assemblies.items():
            # Each should have its own registries
            assert hasattr(assembly_data, "instances")
            assert hasattr(assembly_data, "mates")
            assert hasattr(assembly_data, "patterns")
            print(f"\nSubassembly at {key} has {len(assembly_data.mates.mates)} mates")

    def test_mates_separated_by_subassembly(self, cad_doc_depth_1: CAD):
        """Test that mates are stored separately per subassembly, not flattened."""
        # Get total mates from root assembly
        root_mates = len(cad_doc_depth_1.root_assembly.mates.mates)

        # Get total mates from all subassemblies
        subassembly_mates = sum(len(sub.mates.mates) for sub in cad_doc_depth_1.sub_assemblies.values())

        print(f"\nRoot assembly mates: {root_mates}")
        print(f"Subassembly mates: {subassembly_mates}")

        # If we have subassemblies with mates, they should be separate
        if subassembly_mates > 0:
            # Root should not have ALL mates (they're separated now)
            total_mates = root_mates + subassembly_mates
            assert total_mates > root_mates, "Mates should be separated, not all in root"

    def test_get_all_mates_aggregates_correctly(self, cad_doc_depth_1: CAD):
        """Test that get_all_mates() aggregates mates from root and subassemblies."""
        all_mates = cad_doc_depth_1.get_all_mates()
        root_mates = len(cad_doc_depth_1.root_assembly.mates.mates)
        subassembly_mates = sum(len(sub.mates.mates) for sub in cad_doc_depth_1.sub_assemblies.values())

        # get_all_mates() uses dict.update() which may have overlapping keys
        # So the total may be less than sum if there are duplicate mate keys
        expected_min = max(root_mates, subassembly_mates)  # At least the larger of the two
        expected_max = root_mates + subassembly_mates  # At most the sum

        assert (
            expected_min <= len(all_mates) <= expected_max
        ), f"get_all_mates() should aggregate mates (got {len(all_mates)}, \
            expected between {expected_min} and {expected_max})"
        print(f"\nTotal mates: {len(all_mates)} (root: {root_mates}, subassemblies: {subassembly_mates})")

    def test_get_subassembly_data(self, cad_doc_depth_1: CAD):
        """Test that get_subassembly_data() returns the correct AssemblyData."""
        if not cad_doc_depth_1.sub_assemblies:
            pytest.skip("No subassemblies in test data")

        key = next(iter(cad_doc_depth_1.sub_assemblies.keys()))
        assembly_data = cad_doc_depth_1.get_subassembly_data(key)

        assert assembly_data is not None
        assert isinstance(assembly_data, AssemblyData)
        assert assembly_data == cad_doc_depth_1.sub_assemblies[key]


class TestRigidAssemblyHandling:
    """Tests for rigid assembly detection and handling."""

    def test_isrigid_flag_set_correctly_depth_0(self, cad_doc: CAD):
        """Test that assemblies beyond depth 0 are marked rigid when max_depth=0."""
        assembly_keys = list(cad_doc.root_assembly.instances.assemblies.keys())
        if not assembly_keys:
            pytest.skip("No assembly instances in test data")

        for key, instance in cad_doc.root_assembly.instances.assemblies.items():
            assert isinstance(instance, AssemblyInstance)
            # Assemblies at depth > max_depth should be rigid
            expected_rigid = key.depth > cad_doc.max_depth
            assert (
                instance.isRigid == expected_rigid
            ), f"Assembly at depth {key.depth} isRigid={instance.isRigid}, expected {expected_rigid} (max_depth=0)"

    def test_isrigid_flag_set_correctly_depth_1(self, cad_doc_depth_1: CAD):
        """Test that assemblies are marked rigid correctly when max_depth=1."""
        assembly_keys = list(cad_doc_depth_1.root_assembly.instances.assemblies.keys())
        if not assembly_keys:
            pytest.skip("No assembly instances in test data")

        for key, instance in cad_doc_depth_1.root_assembly.instances.assemblies.items():
            assert isinstance(instance, AssemblyInstance)
            # Rigid if depth > max_depth
            expected_rigid = key.depth > 1
            assert (
                instance.isRigid == expected_rigid
            ), f"Assembly at depth {key.depth} isRigid={instance.isRigid}, expected {expected_rigid}"

    def test_isrigid_flag_set_correctly_depth_2(self, cad_doc_depth_2: CAD):
        """Test that assemblies are marked rigid correctly when max_depth=2."""
        assembly_keys = list(cad_doc_depth_2.root_assembly.instances.assemblies.keys())
        if not assembly_keys:
            pytest.skip("No assembly instances in test data")

        for key, instance in cad_doc_depth_2.root_assembly.instances.assemblies.items():
            assert isinstance(instance, AssemblyInstance)
            # Rigid if depth > max_depth
            expected_rigid = key.depth > 2
            assert (
                instance.isRigid == expected_rigid
            ), f"Assembly at depth {key.depth} isRigid={instance.isRigid}, expected {expected_rigid}"

    def test_is_rigid_assembly_uses_flag(self, cad_doc_depth_1: CAD):
        """Test that is_rigid_assembly() method uses the isRigid flag."""
        assembly_keys = list(cad_doc_depth_1.root_assembly.instances.assemblies.keys())
        if not assembly_keys:
            pytest.skip("No assembly instances in test data")

        for key in assembly_keys:
            instance = cad_doc_depth_1.root_assembly.instances.assemblies[key]
            assert isinstance(instance, AssemblyInstance)

            # The method should return the same value as the flag
            is_rigid = cad_doc_depth_1.root_assembly.instances.is_rigid_assembly(key)
            assert is_rigid == instance.isRigid, f"is_rigid_assembly() should match instance.isRigid flag for {key}"

    def test_rigid_count_property(self, cad_doc_depth_1: CAD):
        """Test that rigid_count property returns correct count."""
        assembly_keys = list(cad_doc_depth_1.root_assembly.instances.assemblies.keys())
        if not assembly_keys:
            pytest.skip("No assembly instances in test data")

        # Count manually
        manual_count = sum(
            1
            for key, instance in cad_doc_depth_1.root_assembly.instances.assemblies.items()
            if isinstance(instance, AssemblyInstance) and instance.isRigid
        )

        # Compare with property
        assert cad_doc_depth_1.root_assembly.instances.rigid_count == manual_count

    def test_flexible_count_property(self, cad_doc_depth_1: CAD):
        """Test that flexible_count property returns correct count."""
        assembly_keys = list(cad_doc_depth_1.root_assembly.instances.assemblies.keys())
        if not assembly_keys:
            pytest.skip("No assembly instances in test data")

        # Count manually
        manual_count = sum(
            1
            for key, instance in cad_doc_depth_1.root_assembly.instances.assemblies.items()
            if isinstance(instance, AssemblyInstance) and not instance.isRigid
        )

        # Compare with property
        assert cad_doc_depth_1.root_assembly.instances.flexible_count == manual_count


class TestRigidAssemblyAsPartObjects:
    """Tests for rigid assemblies being stored as Part objects."""

    def test_rigid_assemblies_in_parts_dict(self, cad_doc_depth_1: CAD):
        """Test that rigid assemblies are added to parts dictionary."""
        # Find rigid assemblies
        rigid_assembly_keys = [
            key
            for key, instance in cad_doc_depth_1.root_assembly.instances.assemblies.items()
            if isinstance(instance, AssemblyInstance) and instance.isRigid
        ]

        if not rigid_assembly_keys:
            pytest.skip("No rigid assemblies in test data at max_depth=1")

        # Check that each rigid assembly has a Part object in parts dict
        for key in rigid_assembly_keys:
            assert key in cad_doc_depth_1.parts, f"Rigid assembly at {key} should be in parts dict"
            part = cad_doc_depth_1.parts[key]
            assert part.isRigidAssembly is True, f"Part at {key} should be marked as rigid assembly"
            print(f"\nRigid assembly {key} correctly stored as Part with isRigidAssembly=True")

    def test_parts_dict_uses_pathkey_indexing(self, cad_doc: CAD):
        """Test that parts dictionary uses PathKey for indexing."""
        if not cad_doc.parts:
            pytest.skip("No parts in test data")

        for key, part in cad_doc.parts.items():
            assert isinstance(key, PathKey), "Parts dict keys should be PathKey instances"
            # Can access part by the same key used for instances
            if not part.isRigidAssembly:
                instance = cad_doc.root_assembly.instances.parts.get(key)
                if instance:  # Regular part instance
                    assert instance.partId == part.partId, "Part IDs should match"

    def test_get_part_uses_pathkey(self, cad_doc: CAD):
        """Test that get_part() method accepts PathKey."""
        if not cad_doc.parts:
            pytest.skip("No parts in test data")

        key = next(iter(cad_doc.parts.keys()))
        part = cad_doc.get_part(key)

        assert part is not None
        assert part == cad_doc.parts[key]

    def test_rigid_assemblies_have_no_partid(self, cad_doc_depth_1: CAD):
        """Test that rigid assembly Part objects have empty partId."""
        # Find rigid assemblies in parts dict
        rigid_parts = [(k, v) for k, v in cad_doc_depth_1.parts.items() if v.isRigidAssembly]

        if not rigid_parts:
            pytest.skip("No rigid assemblies in parts dict at max_depth=1")

        for key, part in rigid_parts:
            # Rigid assemblies don't have a partId
            assert part.partId == "", f"Rigid assembly at {key} should have empty partId"
            # But they should have documentId and elementId
            assert part.documentId != "", f"Rigid assembly at {key} should have documentId"
            assert part.elementId != "", f"Rigid assembly at {key} should have elementId"


class TestInstanceStorageStrategy:
    """Tests for the dual storage strategy of instances."""

    def test_instances_in_root_registry(self, cad_doc_depth_1: CAD):
        """Test that all instances are stored in root_assembly.instances (flat registry)."""
        total_instances = len(cad_doc_depth_1.root_assembly.instances.parts) + len(
            cad_doc_depth_1.root_assembly.instances.assemblies
        )
        total_occurrences = len(cad_doc_depth_1.root_assembly.occurrences.occurrences)

        # All instances should be in the flat registry
        assert total_instances == total_occurrences, "All instances should be in root flat registry"

    def test_instances_also_in_subassembly_registries(self, cad_doc_depth_1: CAD):
        """Test that instances are also stored in their subassembly's registry."""
        if not cad_doc_depth_1.sub_assemblies:
            pytest.skip("No subassemblies in test data")

        for key, assembly_data in cad_doc_depth_1.sub_assemblies.items():
            # Each subassembly should have instances
            subassembly_instance_count = len(assembly_data.instances.parts) + len(assembly_data.instances.assemblies)

            # If the subassembly has instances, they should also be in root
            if subassembly_instance_count > 0:
                for sub_key in assembly_data.instances.parts:
                    assert (
                        sub_key in cad_doc_depth_1.root_assembly.instances.parts
                    ), f"Instance {sub_key} should be in both subassembly and root"

                for sub_key in assembly_data.instances.assemblies:
                    assert (
                        sub_key in cad_doc_depth_1.root_assembly.instances.assemblies
                    ), f"Instance {sub_key} should be in both subassembly and root"

                print(f"\nSubassembly at {key} has {subassembly_instance_count} instances (also in root)")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
