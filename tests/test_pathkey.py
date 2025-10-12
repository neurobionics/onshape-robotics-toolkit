"""Unit tests for the PathKey helper."""

from __future__ import annotations

import pytest

from onshape_robotics_toolkit.parse import PathKey


def test_pathkey_basic_properties():
    key = PathKey(("root", "child"), ("asm", "wheel"))

    assert key.path == ("root", "child")
    assert key.name_path == ("asm", "wheel")
    assert key.leaf == "child"
    assert key.name == "wheel"
    assert key.root == "root"
    assert key.depth == 1


def test_pathkey_parent_and_root_detection():
    key = PathKey(("a", "b", "c"), ("one", "two", "three"))
    parent = key.parent

    assert parent is not None
    assert parent.path == ("a", "b")
    assert parent.depth == 1
    assert parent.parent == PathKey(("a",), ("one",))
    assert parent.parent.parent is None


def test_pathkey_from_path_validates_id_mapping():
    mapping = {"a": "first", "b": "second"}
    key = PathKey.from_path(["a", "b"], mapping)
    assert key.name_path == ("first", "second")

    # Single ID input is allowed and returns a tuple internally
    root_key = PathKey.from_path("a", mapping)
    assert root_key.path == ("a",)
    assert root_key.name_path == ("first",)

    with pytest.raises(KeyError):
        PathKey.from_path(["a", "missing"], mapping)


def test_pathkey_sorting_is_depth_then_lexicographic():
    shallow = PathKey(("a",), ("A",))
    deeper = PathKey(("a", "b"), ("A", "B"))
    sibling = PathKey(("a", "c"), ("A", "C"))

    ordered = sorted([sibling, deeper, shallow])
    assert ordered == [shallow, deeper, sibling]


def test_pathkey_string_representations():
    key = PathKey(("root", "child"), ("asm", "wheel"))
    assert str(key) == "asm_wheel"
    assert "PathKey" in repr(key)
