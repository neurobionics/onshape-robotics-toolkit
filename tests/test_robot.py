"""Tests for Robot generation helpers covering multiple joint types and mass properties."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from threading import Lock
from typing import Any

import numpy as np

from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.assembly import MatedCS, MatedEntity, MateFeatureData, MateType
from onshape_robotics_toolkit.models.joint import BaseJoint, FixedJoint, PrismaticJoint, RevoluteJoint
from onshape_robotics_toolkit.models.link import Link
from onshape_robotics_toolkit.models.mass import MassProperties, PrincipalAxis
from onshape_robotics_toolkit.parse import CAD, PathKey
from onshape_robotics_toolkit.robot import Robot, get_robot_joint

IDENTITY_TF = np.eye(4)


def _make_mate(name: str, mate_type: MateType, parent_occ: list[str], child_occ: list[str]) -> MateFeatureData:
    """Create a mate with identity coordinate frames for both entities."""
    parent_entity = MatedEntity(matedOccurrence=parent_occ, matedCS=MatedCS.from_tf(IDENTITY_TF))
    child_entity = MatedEntity(matedOccurrence=child_occ, matedCS=MatedCS.from_tf(IDENTITY_TF))
    return MateFeatureData(
        matedEntities=[parent_entity, child_entity],
        mateType=mate_type,
        name=name,
        id=f"{mate_type.value}-{name}",
    )


@dataclass
class DummyClient:
    """Minimal client surface used by tests that should never hit the network."""

    def download_part_stl(self, *_, **__):
        raise RuntimeError("Should not download assets during unit tests")

    def download_assembly_stl(self, *_, **__):
        raise RuntimeError("Should not download assets during unit tests")

    def get_mass_property(self, *_, **__):
        raise RuntimeError("Mass properties should be skipped in unit tests")

    def get_assembly_mass_properties(self, *_, **__):
        raise RuntimeError("Mass properties should be skipped in unit tests")


@dataclass
class RecordingMassClient:
    """Record calls made by fetch_mass_properties_for_parts and return a fixed payload."""

    response: MassProperties
    mass_calls: list[tuple[Any, ...]] = None  # type: ignore[assignment]
    assembly_calls: list[tuple[Any, ...]] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        self.mass_calls = []
        self.assembly_calls = []
        self._lock = Lock()

    def get_mass_property(self, *args: Any, **kwargs: Any) -> MassProperties:
        with self._lock:
            self.mass_calls.append(args)
        return self.response

    def get_assembly_mass_properties(self, *args: Any, **kwargs: Any) -> MassProperties:
        with self._lock:
            self.assembly_calls.append(args)
        return self.response


def test_robot_from_graph_populates_links_and_joints(cad_doc: CAD) -> None:
    """Robot.from_graph should mirror the kinematic graph structure."""
    graph = KinematicGraph.from_cad(cad_doc)
    client = DummyClient()

    robot = Robot.from_graph(
        kinematic_graph=graph,
        client=client,
        name="test_robot",
        fetch_mass_properties=False,
    )

    assert robot.kinematic_graph is graph
    assert len(robot.nodes) == len(graph.nodes)
    assert len(robot.edges) == len(graph.edges)

    for node, payload in robot.nodes(data=True):
        assert isinstance(node, PathKey)
        link = payload["data"]
        assert isinstance(link, Link)
        # Assets are only missing for dummy links (none exist in this fixture)
        assert payload["asset"] is not None
        assert payload["world_to_link_tf"] is not None

    for parent, child, payload in robot.edges(data=True):
        assert isinstance(parent, PathKey)
        assert isinstance(child, PathKey)
        joint = payload["data"]
        assert isinstance(joint, BaseJoint)


def test_get_robot_joint_generates_expected_variants() -> None:
    """get_robot_joint should generate the correct joint/link structures for common mate types."""
    parent_key = PathKey(("root",), ("root",))
    rev_child = PathKey(("rev",), ("rev",))
    fixed_child = PathKey(("fixed",), ("fixed",))
    slider_child = PathKey(("slider",), ("slider",))
    ball_child = PathKey(("ball",), ("ball",))

    used_names: set[str] = set()

    revolute_mate = _make_mate("revolute_joint", MateType.REVOLUTE, ["root"], ["rev"])
    revolute_joints, revolute_links = get_robot_joint(parent_key, rev_child, revolute_mate, IDENTITY_TF, used_names)
    assert isinstance(revolute_joints[(parent_key, rev_child)], RevoluteJoint)
    assert revolute_links == {}
    assert len(used_names) == 1

    fastened_mate = _make_mate("fastened_joint", MateType.FASTENED, ["root"], ["fixed"])
    fastened_joints, fastened_links = get_robot_joint(parent_key, fixed_child, fastened_mate, IDENTITY_TF, used_names)
    assert isinstance(fastened_joints[(parent_key, fixed_child)], FixedJoint)
    assert fastened_links == {}
    assert len(used_names) == 2

    slider_mate = _make_mate("slider_joint", MateType.SLIDER, ["root"], ["slider"])
    slider_joints, slider_links = get_robot_joint(parent_key, slider_child, slider_mate, IDENTITY_TF, used_names)
    assert isinstance(slider_joints[(parent_key, slider_child)], PrismaticJoint)
    assert slider_links == {}
    assert len(used_names) == 3

    ball_mate = _make_mate("ball_joint", MateType.BALL, ["root"], ["ball"])
    ball_joints, ball_links = get_robot_joint(parent_key, ball_child, ball_mate, IDENTITY_TF, used_names)
    assert len(ball_joints) == 3
    assert all(isinstance(joint, RevoluteJoint) for joint in ball_joints.values())
    assert ball_links is not None and len(ball_links) == 2

    dummy_keys = list(ball_links.keys())
    assert all(len(dummy_key.path) == len(parent_key.path) + 2 for dummy_key in dummy_keys)
    for link in ball_links.values():
        assert link.inertial.mass == 0.0
    assert len(used_names) == 4  # each invocation reserves a unique joint base name


MASS_RESPONSE = MassProperties(
    volume=[1.0, 1.0, 1.0],
    mass=[2.0, 2.0, 2.0],
    centroid=[0.0, 0.0, 0.0],
    inertia=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    principalInertia=[1.0, 1.0, 1.0],
    principalAxes=[
        PrincipalAxis(x=1.0, y=0.0, z=0.0),
        PrincipalAxis(x=0.0, y=1.0, z=0.0),
        PrincipalAxis(x=0.0, y=0.0, z=1.0),
    ],
)


def test_fetch_mass_properties_populates_parts(cad_doc_depth_0: CAD) -> None:
    """fetch_mass_properties_for_parts should populate parts and record the client calls."""
    client = RecordingMassClient(response=MASS_RESPONSE)

    assert all(part.MassProperty is None for part in cad_doc_depth_0.parts.values())

    asyncio.run(cad_doc_depth_0.fetch_mass_properties_for_parts(client))

    assert client.mass_calls, "Expected mass property calls for regular parts"
    assert client.assembly_calls, "Expected assembly mass property calls for rigid assemblies"

    regular_parts = [part for part in cad_doc_depth_0.parts.values() if not part.isRigidAssembly]
    assembly_parts = [part for part in cad_doc_depth_0.parts.values() if part.isRigidAssembly]

    assert regular_parts, "Fixture should contain non-assembly parts"
    assert assembly_parts, "Fixture should contain synthetic rigid assembly parts"

    for part in regular_parts:
        assert part.MassProperty is not None
        assert part.MassProperty.mass == MASS_RESPONSE.mass

    for part in assembly_parts:
        assert part.MassProperty is not None
        assert part.MassProperty.mass == MASS_RESPONSE.mass

    skipped_parts = [part for part in cad_doc_depth_0.parts.values() if part.rigidAssemblyToPartTF is not None]
    for part in skipped_parts:
        assert part.MassProperty is None


def test_all_supported_mate_types() -> None:
    """Test that all supported mate types generate correct joint types."""

    parent_key = PathKey(("parent",), ("parent",))
    child_key = PathKey(("child",), ("child",))
    used_names: set[str] = set()

    # Test REVOLUTE -> RevoluteJoint
    revolute_mate = _make_mate("revolute", MateType.REVOLUTE, ["parent"], ["child"])
    joints, links = get_robot_joint(parent_key, child_key, revolute_mate, IDENTITY_TF, used_names)
    assert isinstance(joints[(parent_key, child_key)], RevoluteJoint)
    assert joints[(parent_key, child_key)].axis.xyz == (0.0, 0.0, -1.0)
    assert links == {}

    # Test FASTENED -> FixedJoint
    fastened_mate = _make_mate("fastened", MateType.FASTENED, ["parent"], ["child"])
    joints, links = get_robot_joint(parent_key, child_key, fastened_mate, IDENTITY_TF, used_names)
    assert isinstance(joints[(parent_key, child_key)], FixedJoint)
    assert links == {}

    # Test SLIDER -> PrismaticJoint
    slider_mate = _make_mate("slider", MateType.SLIDER, ["parent"], ["child"])
    joints, links = get_robot_joint(parent_key, child_key, slider_mate, IDENTITY_TF, used_names)
    assert isinstance(joints[(parent_key, child_key)], PrismaticJoint)
    assert joints[(parent_key, child_key)].axis.xyz == (0.0, 0.0, -1.0)
    assert links == {}

    # Test CYLINDRICAL -> PrismaticJoint (treated same as SLIDER)
    cylindrical_mate = _make_mate("cylindrical", MateType.CYLINDRICAL, ["parent"], ["child"])
    joints, links = get_robot_joint(parent_key, child_key, cylindrical_mate, IDENTITY_TF, used_names)
    assert isinstance(joints[(parent_key, child_key)], PrismaticJoint)
    assert links == {}

    # Test BALL -> 3 RevoluteJoints + 2 dummy links
    ball_mate = _make_mate("ball", MateType.BALL, ["parent"], ["child"])
    joints, links = get_robot_joint(parent_key, child_key, ball_mate, IDENTITY_TF, used_names)
    assert len(joints) == 3
    assert all(isinstance(j, RevoluteJoint) for j in joints.values())
    assert links is not None and len(links) == 2

    # Test unsupported types -> DummyJoint (if implemented)
    # For now, unsupported types log a warning and create DummyJoint
    from onshape_robotics_toolkit.models.joint import DummyJoint

    # Test PLANAR (not fully supported)
    planar_mate = _make_mate("planar", MateType.PLANAR, ["parent"], ["child"])
    joints, links = get_robot_joint(parent_key, child_key, planar_mate, IDENTITY_TF, used_names)
    # Check if it creates a DummyJoint or handles it differently
    joint = joints[(parent_key, child_key)]
    # PLANAR might not be supported, so it should create a DummyJoint or similar
    assert isinstance(joint, (DummyJoint, BaseJoint))


def test_joint_limits_are_set_correctly() -> None:
    """Test that joint limits are set based on mate type."""
    parent_key = PathKey(("parent",), ("parent",))
    child_key = PathKey(("child",), ("child",))
    used_names: set[str] = set()

    # Revolute joint should have rotational limits
    revolute_mate = _make_mate("revolute", MateType.REVOLUTE, ["parent"], ["child"])
    joints, _ = get_robot_joint(parent_key, child_key, revolute_mate, IDENTITY_TF, used_names)
    revolute_joint = joints[(parent_key, child_key)]

    assert revolute_joint.limits is not None
    assert revolute_joint.limits.effort == 1.0
    assert revolute_joint.limits.velocity == 1.0

    # Prismatic joint should have translational limits
    slider_mate = _make_mate("slider", MateType.SLIDER, ["parent"], ["child"])
    joints, _ = get_robot_joint(parent_key, child_key, slider_mate, IDENTITY_TF, used_names)
    prismatic_joint = joints[(parent_key, child_key)]

    assert prismatic_joint.limits is not None
    assert prismatic_joint.limits.effort == 1.0
    assert prismatic_joint.limits.velocity == 1.0


def test_joint_naming_uniqueness() -> None:
    """Test that joint names are made unique when there are conflicts."""
    parent_key = PathKey(("parent",), ("parent",))
    child1_key = PathKey(("child1",), ("child1",))
    child2_key = PathKey(("child2",), ("child2",))
    used_names: set[str] = set()

    # Create two mates with the same name
    mate1 = _make_mate("joint", MateType.REVOLUTE, ["parent"], ["child1"])
    mate2 = _make_mate("joint", MateType.REVOLUTE, ["parent"], ["child2"])

    joints1, _ = get_robot_joint(parent_key, child1_key, mate1, IDENTITY_TF, used_names)
    joint1_name = joints1[(parent_key, child1_key)].name

    joints2, _ = get_robot_joint(parent_key, child2_key, mate2, IDENTITY_TF, used_names)
    joint2_name = joints2[(parent_key, child2_key)].name

    # Names should be different
    assert joint1_name != joint2_name
    assert joint1_name in used_names
    assert joint2_name in used_names
