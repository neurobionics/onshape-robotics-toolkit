"""Tests for PathKey class."""

import pytest

from onshape_robotics_toolkit.parse import PathKey


class TestPathKey:
    """Tests for PathKey class."""

    def test_create_from_tuple(self):
        """Test creating PathKey from tuple."""
        key = PathKey(("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG"), ("assembly_1", "part_1"))
        assert key.path == ("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG")
        assert key.name_path == ("assembly_1", "part_1")
        assert key.depth == 2

    def test_create_from_single_id(self):
        """Test creating PathKey from single instance ID."""
        id_to_name = {"MqRDHdbA0tAm2ygBR": "test_part"}
        key = PathKey.from_path("MqRDHdbA0tAm2ygBR", id_to_name)
        assert key.path == ("MqRDHdbA0tAm2ygBR",)
        assert key.name_path == ("test_part",)
        assert key.depth == 1

    def test_create_from_list(self):
        """Test creating PathKey from list of IDs."""
        id_to_name = {"MoN/4FhyvQ92+I8TU": "asm", "MM10pxoGk/3TUSoYG": "part"}
        key = PathKey.from_path(["MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG"], id_to_name)
        assert key.path == ("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG")
        assert key.name_path == ("asm", "part")
        assert key.depth == 2

    def test_leaf_id_and_name(self):
        """Test getting leaf ID and name from PathKey."""
        key = PathKey(
            ("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG", "MrpOYQ6mQsyqwPVz0"), ("assembly_1", "subassembly_1", "part_1")
        )
        assert key.leaf == "MrpOYQ6mQsyqwPVz0"
        assert key.name == "part_1"

    def test_parent_key(self):
        """Test getting parent PathKey."""
        key = PathKey(
            ("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG", "MrpOYQ6mQsyqwPVz0"), ("assembly_1", "subassembly_1", "part_1")
        )
        parent = key.parent
        assert parent is not None
        assert parent.path == ("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG")
        assert parent.name_path == ("assembly_1", "subassembly_1")

    def test_parent_key_root(self):
        """Test parent of root-level PathKey is None."""
        key = PathKey(("MqRDHdbA0tAm2ygBR",), ("test_part",))
        parent = key.parent
        assert parent is None

    def test_hierarchical_name(self):
        """Test hierarchical name generation."""
        key = PathKey(("id1", "id2", "id3"), ("assembly_1", "subassembly_1", "part_1"))
        assert key.hierarchical_name(separator="-") == "assembly_1-subassembly_1-part_1"
        assert key.hierarchical_name(separator="_") == "assembly_1_subassembly_1_part_1"

    def test_immutability(self):
        """Test that PathKey is immutable (frozen dataclass)."""
        key = PathKey(("test",), ("test_name",))
        # PathKey is a frozen dataclass, so attributes cannot be modified
        with pytest.raises((AttributeError, TypeError)):
            key._path = ("modified",)  # type: ignore[misc]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
