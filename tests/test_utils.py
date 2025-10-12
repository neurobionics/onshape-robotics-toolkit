"""Utility functions for testing."""

from __future__ import annotations

import re

import numpy as np
from lxml import etree as ET


def normalize_urdf_floats(urdf_str: str) -> str:
    """
    Normalize floating point numbers in URDF string for comparison.

    This handles minor floating point differences by rounding to 7 decimal places.

    Args:
        urdf_str: URDF XML string

    Returns:
        Normalized URDF string
    """

    def replace_float(match: re.Match[str]) -> str:
        value = float(match.group(0))
        # Round to 7 decimal places for comparison
        return f"{value:.7g}"

    # Match floating point numbers (with optional sign, decimals, and scientific notation)
    pattern = r"-?\d+\.?\d*(?:[eE][+-]?\d+)?"
    return re.sub(pattern, replace_float, urdf_str)


def compare_xml_elements(
    elem1: ET._Element,
    elem2: ET._Element,
    tolerance: float = 1e-6,
    ignore_order: bool = True,
) -> tuple[bool, list[str]]:
    """
    Compare two XML elements for semantic equality.

    Args:
        elem1: First XML element
        elem2: Second XML element
        tolerance: Tolerance for floating point comparisons
        ignore_order: Whether to ignore order of child elements

    Returns:
        Tuple of (is_equal, list_of_differences)
    """
    differences: list[str] = []

    # Compare tags
    if elem1.tag != elem2.tag:
        differences.append(f"Different tags: {elem1.tag} != {elem2.tag}")
        return False, differences

    # Compare attributes
    attrs1 = dict(elem1.attrib)
    attrs2 = dict(elem2.attrib)

    if set(attrs1.keys()) != set(attrs2.keys()):
        diff_keys = set(attrs1.keys()) ^ set(attrs2.keys())
        differences.append(f"Different attributes in <{elem1.tag}>: {diff_keys}")

    for key in attrs1:
        if key not in attrs2:
            continue

        val1 = attrs1[key]
        val2 = attrs2[key]

        # Try to compare as floats/vectors if possible
        try:
            # Handle space-separated vectors (like "x y z")
            if " " in val1 or " " in val2:
                vec1 = np.array([float(x) for x in val1.split()])
                vec2 = np.array([float(x) for x in val2.split()])
                if not np.allclose(vec1, vec2, atol=tolerance, rtol=tolerance):
                    differences.append(f"Different attribute values in <{elem1.tag} {key}>: {val1} != {val2}")
            else:
                # Single float
                f1 = float(val1)
                f2 = float(val2)
                if not np.isclose(f1, f2, atol=tolerance, rtol=tolerance):
                    differences.append(f"Different attribute values in <{elem1.tag} {key}>: {val1} != {val2}")
        except ValueError:
            # Not a number, compare as strings
            if val1 != val2:
                differences.append(f"Different attribute values in <{elem1.tag} {key}>: {val1} != {val2}")

    # Compare text content
    text1 = (elem1.text or "").strip()
    text2 = (elem2.text or "").strip()
    if text1 != text2:
        differences.append(f"Different text in <{elem1.tag}>: '{text1}' != '{text2}'")

    # Compare children
    children1 = list(elem1)
    children2 = list(elem2)

    if len(children1) != len(children2):
        differences.append(f"Different number of children in <{elem1.tag}>: {len(children1)} != {len(children2)}")
        return False, differences

    if ignore_order:
        # Group children by tag for unordered comparison
        def group_by_tag(children: list[ET._Element]) -> dict[str, list[ET._Element]]:
            groups: dict[str, list[ET._Element]] = {}
            for child in children:
                groups.setdefault(child.tag, []).append(child)
            return groups

        groups1 = group_by_tag(children1)
        groups2 = group_by_tag(children2)

        if set(groups1.keys()) != set(groups2.keys()):
            differences.append(f"Different child tags in <{elem1.tag}>: {set(groups1.keys())} != {set(groups2.keys())}")

        for tag in groups1:
            if tag not in groups2:
                continue

            g1 = groups1[tag]
            g2 = groups2[tag]

            if len(g1) != len(g2):
                differences.append(f"Different count of <{tag}> in <{elem1.tag}>: {len(g1)} != {len(g2)}")
                continue

            # Try to match elements (simple matching by attributes)
            for child1 in g1:
                matched = False
                for child2 in g2[:]:  # Copy to allow removal
                    is_equal, child_diffs = compare_xml_elements(child1, child2, tolerance, ignore_order)
                    if is_equal:
                        g2.remove(child2)
                        matched = True
                        break

                if not matched:
                    differences.append(f"Unmatched element: {ET.tostring(child1, encoding='unicode')}")
    else:
        # Ordered comparison
        for child1, child2 in zip(children1, children2):
            is_equal, child_diffs = compare_xml_elements(child1, child2, tolerance, ignore_order)
            differences.extend(child_diffs)

    return len(differences) == 0, differences


def compare_urdf_files(
    file1: str,
    file2: str,
    tolerance: float = 1e-6,
    ignore_order: bool = True,
    ignore_colors: bool = True,
) -> tuple[bool, list[str]]:
    """
    Compare two URDF files for semantic equality.

    Args:
        file1: Path to first URDF file
        file2: Path to second URDF file
        tolerance: Tolerance for floating point comparisons
        ignore_order: Whether to ignore order of elements
        ignore_colors: Whether to ignore material color differences

    Returns:
        Tuple of (is_equal, list_of_differences)
    """
    tree1 = ET.parse(file1)  # noqa: S320
    tree2 = ET.parse(file2)  # noqa: S320

    root1 = tree1.getroot()
    root2 = tree2.getroot()

    # If ignoring colors, remove all material/color elements
    if ignore_colors:
        for root in [root1, root2]:
            for material in root.findall(".//material"):
                color = material.find("color")
                if color is not None:
                    material.remove(color)

    return compare_xml_elements(root1, root2, tolerance, ignore_order)


def compare_urdf_strings(
    urdf1: str,
    urdf2: str,
    tolerance: float = 1e-6,
    ignore_order: bool = True,
    ignore_colors: bool = True,
) -> tuple[bool, list[str]]:
    """
    Compare two URDF XML strings for semantic equality.

    Args:
        urdf1: First URDF XML string
        urdf2: Second URDF XML string
        tolerance: Tolerance for floating point comparisons
        ignore_order: Whether to ignore order of elements
        ignore_colors: Whether to ignore material color differences

    Returns:
        Tuple of (is_equal, list_of_differences)
    """
    root1 = ET.fromstring(urdf1)  # noqa: S320
    root2 = ET.fromstring(urdf2)  # noqa: S320

    # If ignoring colors, remove all material/color elements
    if ignore_colors:
        for root in [root1, root2]:
            for material in root.findall(".//material"):
                color = material.find("color")
                if color is not None:
                    material.remove(color)

    return compare_xml_elements(root1, root2, tolerance, ignore_order)
