"""Tests for URDF generation from assembly JSON files."""

from __future__ import annotations

from pathlib import Path

import pytest

from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.assembly import Assembly
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import load_model_from_json
from tests.test_robot import DummyClient
from tests.test_utils import compare_urdf_files


@pytest.fixture
def test_data_dir() -> Path:
    """Return the test data directory."""
    return Path(__file__).parent / "data"


def test_urdf_generation_matches_expected_output(test_data_dir: Path) -> None:
    """Generated URDF should match the expected golden file."""
    # Load assembly
    assembly_path = test_data_dir / "assembly.json"
    expected_urdf_path = test_data_dir / "assembly_expected.urdf"

    assembly = load_model_from_json(Assembly, str(assembly_path))

    # Create CAD with max_depth=2 (all flexible)
    cad = CAD.from_assembly(assembly, max_depth=2)

    # Create graph
    graph = KinematicGraph.from_cad(cad)

    # Create robot without fetching mass properties or downloading assets
    client = DummyClient()
    robot = Robot.from_graph(
        kinematic_graph=graph,
        client=client,
        name="test_robot",
        fetch_mass_properties=False,
    )

    # Generate URDF
    urdf_output = robot.to_urdf()

    # Add XML declaration
    full_urdf = '<?xml version="1.0" ?>\n' + urdf_output

    # Save to temp file for comparison
    temp_urdf = test_data_dir / "temp_generated.urdf"
    temp_urdf.write_text(full_urdf)

    try:
        # Compare with expected output (ignore colors since they're random)
        is_equal, differences = compare_urdf_files(
            str(temp_urdf),
            str(expected_urdf_path),
            tolerance=1e-6,
            ignore_order=True,
            ignore_colors=True,
        )

        if not is_equal:
            print("\n".join(differences))

        assert is_equal, "URDF generation differs from expected:\n" + "\n".join(differences)
    finally:
        # Clean up temp file
        if temp_urdf.exists():
            temp_urdf.unlink()


def test_urdf_has_correct_structure(cad_doc: CAD) -> None:
    """Generated URDF should have the correct number of links and joints."""
    graph = KinematicGraph.from_cad(cad_doc)
    client = DummyClient()

    robot = Robot.from_graph(
        kinematic_graph=graph,
        client=client,
        name="test_robot",
        fetch_mass_properties=False,
    )

    urdf_str = robot.to_urdf()

    # Count links and joints in URDF
    link_count = urdf_str.count("<link name=")
    joint_count = urdf_str.count("<joint name=")

    assert link_count == len(graph.nodes), f"Expected {len(graph.nodes)} links, got {link_count}"
    assert joint_count == len(graph.edges), f"Expected {len(graph.edges)} joints, got {joint_count}"


def test_urdf_all_links_have_required_elements(cad_doc: CAD) -> None:
    """Each link in URDF should have visual, collision, and inertial elements."""
    from lxml import etree as ET

    graph = KinematicGraph.from_cad(cad_doc)
    client = DummyClient()

    robot = Robot.from_graph(
        kinematic_graph=graph,
        client=client,
        name="test_robot",
        fetch_mass_properties=False,
    )

    urdf_str = robot.to_urdf()
    root = ET.fromstring(urdf_str)  # noqa: S320

    links = root.findall("link")
    assert len(links) > 0, "No links found in URDF"

    for link in links:
        link_name = link.get("name")

        # Check for visual element
        visual = link.find("visual")
        assert visual is not None, f"Link {link_name} missing visual element"

        # Check for collision element
        collision = link.find("collision")
        assert collision is not None, f"Link {link_name} missing collision element"

        # Check for inertial element
        inertial = link.find("inertial")
        assert inertial is not None, f"Link {link_name} missing inertial element"

        # Check inertial has required sub-elements
        mass = inertial.find("mass")
        assert mass is not None, f"Link {link_name} inertial missing mass"

        inertia = inertial.find("inertia")
        assert inertia is not None, f"Link {link_name} inertial missing inertia"

        origin = inertial.find("origin")
        assert origin is not None, f"Link {link_name} inertial missing origin"


def test_urdf_all_joints_have_required_elements(cad_doc: CAD) -> None:
    """Each joint in URDF should have parent, child, origin, and type-specific elements."""
    from lxml import etree as ET

    graph = KinematicGraph.from_cad(cad_doc)
    client = DummyClient()

    robot = Robot.from_graph(
        kinematic_graph=graph,
        client=client,
        name="test_robot",
        fetch_mass_properties=False,
    )

    urdf_str = robot.to_urdf()
    root = ET.fromstring(urdf_str)  # noqa: S320

    joints = root.findall("joint")
    assert len(joints) > 0, "No joints found in URDF"

    for joint in joints:
        joint_name = joint.get("name")
        joint_type = joint.get("type")

        # Check for parent link
        parent = joint.find("parent")
        assert parent is not None, f"Joint {joint_name} missing parent"
        assert parent.get("link"), f"Joint {joint_name} parent missing link attribute"

        # Check for child link
        child = joint.find("child")
        assert child is not None, f"Joint {joint_name} missing child"
        assert child.get("link"), f"Joint {joint_name} child missing link attribute"

        # Check for origin
        origin = joint.find("origin")
        assert origin is not None, f"Joint {joint_name} missing origin"

        # Type-specific checks
        if joint_type in ["revolute", "continuous", "prismatic"]:
            # Should have axis
            axis = joint.find("axis")
            assert axis is not None, f"Joint {joint_name} of type {joint_type} missing axis"

        if joint_type in ["revolute", "prismatic"]:
            # Should have limits
            limits = joint.find("limit")
            assert limits is not None, f"Joint {joint_name} of type {joint_type} missing limits"


def test_urdf_joint_parent_child_references_valid(cad_doc: CAD) -> None:
    """Joint parent/child references should match existing links."""
    from lxml import etree as ET

    graph = KinematicGraph.from_cad(cad_doc)
    client = DummyClient()

    robot = Robot.from_graph(
        kinematic_graph=graph,
        client=client,
        name="test_robot",
        fetch_mass_properties=False,
    )

    urdf_str = robot.to_urdf()
    root = ET.fromstring(urdf_str)  # noqa: S320

    # Get all link names
    link_names = {link.get("name") for link in root.findall("link")}

    # Check all joint references
    for joint in root.findall("joint"):
        joint_name = joint.get("name")

        parent_link = joint.find("parent").get("link")  # type: ignore[union-attr]
        child_link = joint.find("child").get("link")  # type: ignore[union-attr]

        assert parent_link in link_names, f"Joint {joint_name} references unknown parent: {parent_link}"
        assert child_link in link_names, f"Joint {joint_name} references unknown child: {child_link}"
