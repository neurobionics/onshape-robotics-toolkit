"""Tests for PathKey class."""

import pytest

from onshape_robotics_toolkit.parse import PathKey


class TestPathKey:
    """Tests for PathKey class."""

    def test_create_from_tuple(self):
        """Test creating PathKey from tuple."""
        key = PathKey(("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG"))
        assert key.path == ("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG")
        assert key.depth == 2

    def test_create_from_single_id(self):
        """Test creating PathKey from single instance ID."""
        key = PathKey.from_path("MqRDHdbA0tAm2ygBR")
        assert key.path == ("MqRDHdbA0tAm2ygBR",)
        assert key.depth == 1

    def test_create_from_list(self):
        """Test creating PathKey from list of IDs."""
        key = PathKey.from_path(["MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG"])
        assert key.path == ("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG")
        assert key.depth == 2

    def test_leaf_id(self):
        """Test getting leaf ID from PathKey."""
        key = PathKey(("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG", "MrpOYQ6mQsyqwPVz0"))
        assert key.leaf_id == "MrpOYQ6mQsyqwPVz0"

    def test_parent_key(self):
        """Test getting parent PathKey."""
        key = PathKey(("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG", "MrpOYQ6mQsyqwPVz0"))
        parent = key.parent_key
        assert parent is not None
        assert parent.path == ("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG")

    def test_parent_key_root(self):
        """Test parent of root-level PathKey is None."""
        key = PathKey(("MqRDHdbA0tAm2ygBR",))
        parent = key.parent_key
        assert parent is None

    def test_immutability(self):
        """Test that PathKey is immutable."""
        key = PathKey(("test",))
        with pytest.raises(AttributeError):
            key._path = ("modified",)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
