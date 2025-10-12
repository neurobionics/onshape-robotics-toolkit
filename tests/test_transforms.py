"""Tests for coordinate transform logic in the robot generation pipeline."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from onshape_robotics_toolkit.models.assembly import MatedCS, MatedEntity, MateFeatureData, MateType
from onshape_robotics_toolkit.models.link import Origin
from onshape_robotics_toolkit.parse import PathKey
from onshape_robotics_toolkit.robot import get_robot_joint, get_robot_link
from tests.test_robot import DummyClient


class TestMatedCSTransformations:
    """Test MatedCS coordinate system transformations."""

    def test_identity_transform(self) -> None:
        """MatedCS with identity orientation should produce identity transform."""
        cs = MatedCS(
            xAxis=[1.0, 0.0, 0.0],
            yAxis=[0.0, 1.0, 0.0],
            zAxis=[0.0, 0.0, 1.0],
            origin=[0.0, 0.0, 0.0],
        )

        tf = cs.to_tf
        expected = np.eye(4)

        assert np.allclose(tf, expected, atol=1e-10), f"Transform should be identity:\n{tf}"

    def test_translation_only_transform(self) -> None:
        """MatedCS with only translation should produce pure translation matrix."""
        cs = MatedCS(
            xAxis=[1.0, 0.0, 0.0],
            yAxis=[0.0, 1.0, 0.0],
            zAxis=[0.0, 0.0, 1.0],
            origin=[1.0, 2.0, 3.0],
        )

        tf = cs.to_tf
        expected = np.eye(4)
        expected[:3, 3] = [1.0, 2.0, 3.0]

        assert np.allclose(tf, expected, atol=1e-10), f"Transform should be pure translation:\n{tf}"

    def test_rotation_90_deg_z_axis(self) -> None:
        """MatedCS with 90° rotation about Z should produce correct transform."""
        cs = MatedCS(
            xAxis=[0.0, 1.0, 0.0],  # X points in +Y direction
            yAxis=[-1.0, 0.0, 0.0],  # Y points in -X direction
            zAxis=[0.0, 0.0, 1.0],  # Z unchanged
            origin=[0.0, 0.0, 0.0],
        )

        tf = cs.to_tf
        expected = np.eye(4)
        expected[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]

        assert np.allclose(tf, expected, atol=1e-10), f"Transform should be 90° Z rotation:\n{tf}"

    def test_combined_rotation_and_translation(self) -> None:
        """MatedCS with rotation and translation should combine correctly."""
        cs = MatedCS(
            xAxis=[0.0, 1.0, 0.0],
            yAxis=[-1.0, 0.0, 0.0],
            zAxis=[0.0, 0.0, 1.0],
            origin=[1.0, 2.0, 3.0],
        )

        tf = cs.to_tf
        expected = np.eye(4)
        expected[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        expected[:3, 3] = [1.0, 2.0, 3.0]

        assert np.allclose(tf, expected, atol=1e-10), f"Transform should combine rotation and translation:\n{tf}"

    def test_from_tf_roundtrip(self) -> None:
        """Converting transform to MatedCS and back should preserve the transform."""
        original_tf = np.eye(4)
        original_tf[:3, :3] = Rotation.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix()
        original_tf[:3, 3] = [1.0, 2.0, 3.0]

        cs = MatedCS.from_tf(original_tf)
        reconstructed_tf = cs.to_tf

        assert np.allclose(reconstructed_tf, original_tf, atol=1e-10), "Roundtrip should preserve transform"


class TestOriginTransformations:
    """Test Origin class transformations."""

    def test_origin_from_identity_matrix(self) -> None:
        """Origin from identity matrix should have zero translation and rotation."""
        origin = Origin.from_matrix(np.eye(4))

        assert np.allclose(origin.xyz, [0, 0, 0], atol=1e-10)
        assert np.allclose(origin.rpy, [0, 0, 0], atol=1e-10)

    def test_origin_from_translation_matrix(self) -> None:
        """Origin from translation matrix should extract translation correctly."""
        tf = np.eye(4)
        tf[:3, 3] = [1.0, 2.0, 3.0]

        origin = Origin.from_matrix(tf)

        assert np.allclose(origin.xyz, [1.0, 2.0, 3.0], atol=1e-10)
        assert np.allclose(origin.rpy, [0, 0, 0], atol=1e-10)

    def test_origin_from_rotation_matrix(self) -> None:
        """Origin from rotation matrix should extract Euler angles correctly."""
        # 90° rotation about Z axis
        tf = np.eye(4)
        tf[:3, :3] = Rotation.from_euler("z", np.pi / 2).as_matrix()

        origin = Origin.from_matrix(tf)

        assert np.allclose(origin.xyz, [0, 0, 0], atol=1e-10)
        # Should be close to [0, 0, π/2]
        expected_rpy = Rotation.from_matrix(tf[:3, :3]).as_euler("xyz")
        assert np.allclose(origin.rpy, expected_rpy, atol=1e-6)

    def test_origin_from_combined_transform(self) -> None:
        """Origin from combined transform should extract both translation and rotation."""
        tf = np.eye(4)
        tf[:3, :3] = Rotation.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix()
        tf[:3, 3] = [1.0, 2.0, 3.0]

        origin = Origin.from_matrix(tf)

        assert np.allclose(origin.xyz, [1.0, 2.0, 3.0], atol=1e-10)
        expected_rpy = Rotation.from_matrix(tf[:3, :3]).as_euler("xyz")
        assert np.allclose(origin.rpy, expected_rpy, atol=1e-6)


class TestJointOriginCalculation:
    """Test joint origin calculation from mate transforms."""

    def test_joint_origin_with_identity_parent_transform(self) -> None:
        """Joint with identity parent transform should use mate transform directly."""
        parent_key = PathKey(("parent",), ("parent",))
        child_key = PathKey(("child",), ("child",))

        mate = MateFeatureData(
            matedEntities=[
                MatedEntity(
                    matedOccurrence=["parent"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[1.0, 0.0, 0.0],
                    ),
                ),
                MatedEntity(
                    matedOccurrence=["child"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[0.0, 0.0, 0.0],
                    ),
                ),
            ],
            mateType=MateType.REVOLUTE,
            name="test_joint",
            id="test_id",
        )

        world_to_parent_tf = np.eye(4)  # Identity
        used_names: set[str] = set()

        joints_dict, _ = get_robot_joint(parent_key, child_key, mate, world_to_parent_tf, used_names)

        joint = joints_dict[(parent_key, child_key)]
        # Joint origin should be at parent's mate location (1, 0, 0)
        assert np.allclose(joint.origin.xyz, [1.0, 0.0, 0.0], atol=1e-10)

    def test_joint_origin_with_translated_parent(self) -> None:
        """Joint with translated parent should compose transforms correctly."""
        parent_key = PathKey(("parent",), ("parent",))
        child_key = PathKey(("child",), ("child",))

        mate = MateFeatureData(
            matedEntities=[
                MatedEntity(
                    matedOccurrence=["parent"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[0.5, 0.0, 0.0],  # Mate at +0.5 in parent frame
                    ),
                ),
                MatedEntity(
                    matedOccurrence=["child"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[0.0, 0.0, 0.0],
                    ),
                ),
            ],
            mateType=MateType.REVOLUTE,
            name="test_joint",
            id="test_id",
        )

        world_to_parent_tf = np.eye(4)
        world_to_parent_tf[:3, 3] = [1.0, 2.0, 3.0]  # Parent at (1, 2, 3)
        used_names: set[str] = set()

        joints_dict, _ = get_robot_joint(parent_key, child_key, mate, world_to_parent_tf, used_names)

        joint = joints_dict[(parent_key, child_key)]
        # Joint origin should be at world (1, 2, 3) + parent frame (0.5, 0, 0) = (1.5, 2, 3)
        assert np.allclose(joint.origin.xyz, [1.5, 2.0, 3.0], atol=1e-10)

    def test_joint_origin_with_rotated_parent(self) -> None:
        """Joint with rotated parent should rotate mate position correctly."""
        parent_key = PathKey(("parent",), ("parent",))
        child_key = PathKey(("child",), ("child",))

        mate = MateFeatureData(
            matedEntities=[
                MatedEntity(
                    matedOccurrence=["parent"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[1.0, 0.0, 0.0],  # Mate at +X in parent frame
                    ),
                ),
                MatedEntity(
                    matedOccurrence=["child"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[0.0, 0.0, 0.0],
                    ),
                ),
            ],
            mateType=MateType.REVOLUTE,
            name="test_joint",
            id="test_id",
        )

        # Parent rotated 90° about Z
        world_to_parent_tf = np.eye(4)
        world_to_parent_tf[:3, :3] = Rotation.from_euler("z", np.pi / 2).as_matrix()
        used_names: set[str] = set()

        joints_dict, _ = get_robot_joint(parent_key, child_key, mate, world_to_parent_tf, used_names)

        joint = joints_dict[(parent_key, child_key)]
        # Parent +X becomes world +Y, so mate at (0, 1, 0) in world frame
        assert np.allclose(joint.origin.xyz, [0.0, 1.0, 0.0], atol=1e-10)


class TestLinkOriginCalculation:
    """Test link origin calculation from mate transforms."""

    def test_root_link_with_no_mate(self) -> None:
        """Root link with no mate should be at world origin."""
        from onshape_robotics_toolkit.models.assembly import Part

        part = Part(
            isStandardContent=False,
            partId="test",
            bodyType="solid",
            mateConnectors=[],
            fullConfiguration="default",
            configuration="default",
            documentId="000000000000000000000000",
            elementId="000000000000000000000000",
            documentMicroversion="000000000000000000000000",
        )

        client = DummyClient()
        link, world_to_link_tf, _ = get_robot_link("root", part, client, mate=None)

        # World to link should be identity (link at world origin)
        assert np.allclose(world_to_link_tf, np.eye(4), atol=1e-10)

    def test_child_link_origin_from_mate(self) -> None:
        """Child link origin should be at mate location."""
        from onshape_robotics_toolkit.models.assembly import Part

        part = Part(
            isStandardContent=False,
            partId="test",
            bodyType="solid",
            mateConnectors=[],
            fullConfiguration="default",
            configuration="default",
            documentId="000000000000000000000000",
            elementId="000000000000000000000000",
            documentMicroversion="000000000000000000000000",
        )

        mate = MateFeatureData(
            matedEntities=[
                MatedEntity(
                    matedOccurrence=["parent"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[1.0, 2.0, 3.0],
                    ),
                ),
                MatedEntity(
                    matedOccurrence=["child"],
                    matedCS=MatedCS(
                        xAxis=[1.0, 0.0, 0.0],
                        yAxis=[0.0, 1.0, 0.0],
                        zAxis=[0.0, 0.0, 1.0],
                        origin=[0.5, 0.5, 0.5],  # Child's mate offset
                    ),
                ),
            ],
            mateType=MateType.REVOLUTE,
            name="test_mate",
            id="test_id",
        )

        client = DummyClient()
        link, world_to_link_tf, _ = get_robot_link("child", part, client, mate=mate)

        # World to link should be inverse of child's mate CS
        expected_link_to_world = mate.matedEntities[1].matedCS.to_tf
        expected_world_to_link = np.linalg.inv(expected_link_to_world)

        assert np.allclose(world_to_link_tf, expected_world_to_link, atol=1e-10)


class TestBallJointTransforms:
    """Test ball joint (3-DOF) transform decomposition."""

    def test_ball_joint_creates_three_revolute_joints(self) -> None:
        """Ball joint should create 3 revolute joints with correct axes."""
        parent_key = PathKey(("parent",), ("parent",))
        child_key = PathKey(("child",), ("child",))

        mate = MateFeatureData(
            matedEntities=[
                MatedEntity(
                    matedOccurrence=["parent"],
                    matedCS=MatedCS.from_tf(np.eye(4)),
                ),
                MatedEntity(
                    matedOccurrence=["child"],
                    matedCS=MatedCS.from_tf(np.eye(4)),
                ),
            ],
            mateType=MateType.BALL,
            name="ball_joint",
            id="ball_id",
        )

        world_to_parent_tf = np.eye(4)
        used_names: set[str] = set()

        joints_dict, links_dict = get_robot_joint(parent_key, child_key, mate, world_to_parent_tf, used_names)

        # Should create 3 joints
        assert len(joints_dict) == 3, "Ball joint should create 3 revolute joints"

        # Should create 2 dummy links
        assert links_dict is not None
        assert len(links_dict) == 2, "Ball joint should create 2 dummy links"

        # Check joint axes
        joint_list = list(joints_dict.values())
        assert np.allclose(joint_list[0].axis.xyz, [1.0, 0.0, 0.0]), "First joint should rotate about X"
        assert np.allclose(joint_list[1].axis.xyz, [0.0, 1.0, 0.0]), "Second joint should rotate about Y"
        assert np.allclose(joint_list[2].axis.xyz, [0.0, 0.0, -1.0]), "Third joint should rotate about -Z"

    def test_ball_joint_dummy_links_have_zero_mass(self) -> None:
        """Ball joint dummy links should have zero mass."""
        parent_key = PathKey(("parent",), ("parent",))
        child_key = PathKey(("child",), ("child",))

        mate = MateFeatureData(
            matedEntities=[
                MatedEntity(matedOccurrence=["parent"], matedCS=MatedCS.from_tf(np.eye(4))),
                MatedEntity(matedOccurrence=["child"], matedCS=MatedCS.from_tf(np.eye(4))),
            ],
            mateType=MateType.BALL,
            name="ball_joint",
            id="ball_id",
        )

        world_to_parent_tf = np.eye(4)
        used_names: set[str] = set()

        _, links_dict = get_robot_joint(parent_key, child_key, mate, world_to_parent_tf, used_names)

        assert links_dict is not None
        for link in links_dict.values():
            assert link.inertial.mass == 0.0, "Dummy links should have zero mass"
