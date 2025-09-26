"""
This module contains functions to generate URDF components from Onshape assembly data.

"""

import os
import random
from typing import Optional, Union

import numpy as np
from networkx import DiGraph

from onshape_robotics_toolkit.connect import Asset, Client
from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import (
    MateFeatureData,
    MateRelationFeatureData,
    MateType,
    Part,
)
from onshape_robotics_toolkit.models.document import WorkspaceType
from onshape_robotics_toolkit.models.geometry import MeshGeometry
from onshape_robotics_toolkit.models.joint import (
    BaseJoint,
    DummyJoint,
    FixedJoint,
    JointLimits,
    # JointDynamics,
    JointMimic,
    PrismaticJoint,
    RevoluteJoint,
)
from onshape_robotics_toolkit.models.link import (
    Axis,
    CollisionLink,
    Colors,
    Inertia,
    InertialLink,
    Link,
    Material,
    Origin,
    VisualLink,
)
from onshape_robotics_toolkit.parse import CHILD, MATE_JOINER, PARENT
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name

SCRIPT_DIR = os.path.dirname(__file__)


def get_joint_name(mate_id: str, mates: dict[str, MateFeatureData]) -> str:
    """
    Get the name of the joint from the mate id.

    Args:
        mate_id: The id of the mate.
        mates: The dictionary of mates in the assembly.

    Returns:
        The name of the joint.
    """
    reverse_mates = {mate.id: key for key, mate in mates.items()}
    return reverse_mates.get(mate_id)


def get_robot_link(
    name: str,
    part: Part,
    wid: str,
    client: Client,
    mate: Optional[Union[MateFeatureData, None]] = None,
) -> tuple[Link, np.matrix, Asset]:
    """
    Generate a URDF link from an Onshape part.

    Args:
        name: The name of the link.
        part: The Onshape part object.
        wid: The unique identifier of the workspace.
        client: The Onshape client object to use for sending API requests.
        mate: MateFeatureData object to use for generating the transformation matrix.

    Returns:
        tuple[Link, np.matrix]: The generated link object
            and the transformation matrix from the STL origin to the link origin.

    Examples:
        >>> get_robot_link("root", part, wid, client)
        (
            Link(name='root', visual=VisualLink(...), collision=CollisionLink(...), inertial=InertialLink(...)),
            np.matrix([[1., 0., 0., 0.],
                [0., 1., 0., 0.],
                [0., 0., 1., 0.],
                [0., 0., 0., 1.]])
        )

    """
    _link_to_stl_tf = np.eye(4)

    if mate is None:
        try:
            _link_to_stl_tf[:3, 3] = np.array(part.MassProperty.center_of_mass).reshape(3)
        except AttributeError:
            LOGGER.warning(f"Part {part.partId} has no mass properties, using identity transform")

    elif mate.matedEntities[CHILD].parentCS:
        LOGGER.debug(f"DEBUG: Link {name} has parentCS, applying hierarchical transform")
        _link_to_stl_tf = mate.matedEntities[CHILD].parentCS.part_tf @ mate.matedEntities[CHILD].matedCS.part_to_mate_tf
    else:
        LOGGER.debug(f"DEBUG: Link {name} using direct part_to_mate_tf")
        _link_to_stl_tf = mate.matedEntities[CHILD].matedCS.part_to_mate_tf

    _stl_to_link_tf = np.matrix(np.linalg.inv(_link_to_stl_tf))
    _origin = Origin.zero_origin()
    _principal_axes_rotation = (0.0, 0.0, 0.0)

    # Check if part has mass properties
    if part.MassProperty is None:
        LOGGER.warning(f"Part {part.partId} has no mass properties, using default values")
        _mass = 1.0  # Default mass
        _com = (0.0, 0.0, 0.0)  # Default center of mass at origin
        _inertia = np.eye(3)  # Default identity inertia matrix
    else:
        _mass = part.MassProperty.mass[0]
        _com = part.MassProperty.center_of_mass_wrt(_stl_to_link_tf)
        _inertia = part.MassProperty.inertia_wrt(np.matrix(_stl_to_link_tf[:3, :3]))

    LOGGER.info(f"Creating robot link for {name}")

    if part.documentVersion:
        wtype = WorkspaceType.V.value
        mvwid = part.documentVersion

    elif part.isRigidAssembly:
        wtype = WorkspaceType.W.value
        mvwid = part.rigidAssemblyWorkspaceId
    else:
        wtype = WorkspaceType.W.value
        mvwid = wid

    _asset = Asset(
        did=part.documentId,
        wtype=wtype,
        wid=mvwid,
        eid=part.elementId,
        partID=part.partId,
        client=client,
        transform=_stl_to_link_tf,
        is_rigid_assembly=part.isRigidAssembly,
        file_name=f"{name}.stl",
    )

    _mesh_path = _asset.relative_path

    _link = Link(
        name=name,
        visual=VisualLink(
            name=f"{name}_visual",
            origin=_origin,
            geometry=MeshGeometry(_mesh_path),
            material=Material.from_color(name=f"{name}-material", color=random.SystemRandom().choice(list(Colors))),
        ),
        inertial=InertialLink(
            origin=Origin(
                xyz=_com,
                rpy=_principal_axes_rotation,
            ),
            mass=_mass,
            inertia=Inertia(
                ixx=_inertia[0, 0],
                ixy=_inertia[0, 1],
                ixz=_inertia[0, 2],
                iyy=_inertia[1, 1],
                iyz=_inertia[1, 2],
                izz=_inertia[2, 2],
            ),
        ),
        collision=CollisionLink(
            name=f"{name}_collision",
            origin=_origin,
            geometry=MeshGeometry(_mesh_path),
        ),
    )

    return _link, _stl_to_link_tf, _asset


def get_robot_joint(
    parent: str,
    child: str,
    mate: MateFeatureData,
    stl_to_parent_tf: np.matrix,
    mimic: Optional[JointMimic] = None,
    is_rigid_assembly: bool = False,
) -> tuple[list[BaseJoint], Optional[list[Link]]]:
    """
    Generate a URDF joint from an Onshape mate feature.

    Args:
        parent: The name of the parent link.
        child: The name of the child link.
        mate: The Onshape mate feature object.
        stl_to_parent_tf: The transformation matrix from the STL origin to the parent link origin.
        mimic: The mimic joint object.
        is_rigid_assembly: Whether the assembly is a rigid assembly.

    Returns:
        tuple[list[BaseJoint], Optional[list[Link]]]: The generated joint object and the links.

    Examples:
        >>> get_robot_joint("root", "link1", mate, np.eye(4))
        (
            [
                RevoluteJoint(
                    name='base_link_to_link1',
                    parent='root',
                    child='link1',
                    origin=Origin(...),
                    limits=JointLimits(...),
                    axis=Axis(...),
                    dynamics=JointDynamics(...)
                )
            ],
            None
        )

    """
    links = []
    if isinstance(mate, MateFeatureData):
        # Check if we have a parentCS (hierarchical transform) regardless of rigid assembly status
        # This handles the case where flexible assemblies contain parts from collapsed rigid subassemblies
        if mate.matedEntities[PARENT].parentCS is not None:
            LOGGER.debug(f"DEBUG: Using parentCS for joint {mate.name} from {parent} to {child}")
            LOGGER.debug(f"DEBUG: parentCS.part_tf = {mate.matedEntities[PARENT].parentCS.part_tf}")
            LOGGER.debug(
                f"DEBUG: parent matedCS.part_to_mate_tf = {mate.matedEntities[PARENT].matedCS.part_to_mate_tf}"
            )
            parent_to_mate_tf = (
                mate.matedEntities[PARENT].parentCS.part_tf @ mate.matedEntities[PARENT].matedCS.part_to_mate_tf
            )
            LOGGER.debug(f"DEBUG: combined parent_to_mate_tf = {parent_to_mate_tf}")
        else:
            LOGGER.debug(f"DEBUG: No parentCS for joint {mate.name} from {parent} to {child}, using direct transform")
            # No hierarchical transform needed, use direct part-to-mate transformation
            parent_to_mate_tf = mate.matedEntities[PARENT].matedCS.part_to_mate_tf
            LOGGER.debug(f"DEBUG: direct parent_to_mate_tf = {parent_to_mate_tf}")

    LOGGER.debug(f"DEBUG: stl_to_parent_tf = {stl_to_parent_tf}")
    stl_to_mate_tf = stl_to_parent_tf @ parent_to_mate_tf
    LOGGER.debug(f"DEBUG: final stl_to_mate_tf = {stl_to_mate_tf}")
    origin = Origin.from_matrix(stl_to_mate_tf)
    LOGGER.debug(f"DEBUG: final origin = {origin.xyz} rpy={origin.rpy}")
    sanitized_name = get_sanitized_name(mate.name)

    LOGGER.info(f"Creating robot joint from {parent} to {child}")

    if mate.mateType == MateType.REVOLUTE:
        return [
            RevoluteJoint(
                name=sanitized_name,
                parent=parent,
                child=child,
                origin=origin,
                limits=JointLimits(
                    effort=1.0,
                    velocity=1.0,
                    # TODO: add support for joint limits from Onshape
                    lower=-2 * np.pi,
                    upper=2 * np.pi,
                ),
                axis=Axis((0.0, 0.0, -1.0)),
                # dynamics=JointDynamics(damping=0.1, friction=0.1),
                mimic=mimic,
            )
        ], links

    elif mate.mateType == MateType.FASTENED:
        return [FixedJoint(name=sanitized_name, parent=parent, child=child, origin=origin)], links

    elif mate.mateType == MateType.SLIDER or mate.mateType == MateType.CYLINDRICAL:
        return [
            PrismaticJoint(
                name=sanitized_name,
                parent=parent,
                child=child,
                origin=origin,
                # limits=JointLimits(
                #     effort=1.0,
                #     velocity=1.0,
                #     lower=-0.1,
                #     upper=0.1,
                # ),
                axis=Axis((0.0, 0.0, -1.0)),
                # dynamics=JointDynamics(damping=0.1, friction=0.1),
                mimic=mimic,
            )
        ], links

    elif mate.mateType == MateType.BALL:
        dummy_x = Link(
            name=f"{parent}_{get_sanitized_name(mate.name)}_x",
            inertial=InertialLink(
                mass=0.0,
                inertia=Inertia.zero_inertia(),
                origin=Origin.zero_origin(),
            ),
        )
        dummy_y = Link(
            name=f"{parent}_{get_sanitized_name(mate.name)}_y",
            inertial=InertialLink(
                mass=0.0,
                inertia=Inertia.zero_inertia(),
                origin=Origin.zero_origin(),
            ),
        )

        links = [dummy_x, dummy_y]

        return [
            RevoluteJoint(
                name=sanitized_name + "_x",
                parent=parent,
                child=dummy_x.name,
                origin=origin,
                limits=JointLimits(
                    effort=1.0,
                    velocity=1.0,
                    lower=-2 * np.pi,
                    upper=2 * np.pi,
                ),
                axis=Axis((1.0, 0.0, 0.0)),
                # dynamics=JointDynamics(damping=0.1, friction=0.1),
                mimic=mimic,
            ),
            RevoluteJoint(
                name=sanitized_name + "_y",
                parent=dummy_x.name,
                child=dummy_y.name,
                origin=Origin.zero_origin(),
                limits=JointLimits(
                    effort=1.0,
                    velocity=1.0,
                    lower=-2 * np.pi,
                    upper=2 * np.pi,
                ),
                axis=Axis((0.0, 1.0, 0.0)),
                # dynamics=JointDynamics(damping=0.1, friction=0.1),
                mimic=mimic,
            ),
            RevoluteJoint(
                name=sanitized_name + "_z",
                parent=dummy_y.name,
                child=child,
                origin=Origin.zero_origin(),
                limits=JointLimits(
                    effort=1.0,
                    velocity=1.0,
                    lower=-2 * np.pi,
                    upper=2 * np.pi,
                ),
                axis=Axis((0.0, 0.0, -1.0)),
                # dynamics=JointDynamics(damping=0.1, friction=0.1),
                mimic=mimic,
            ),
        ], links

    else:
        LOGGER.warning(f"Unsupported joint type: {mate.mateType}")
        return [DummyJoint(name=sanitized_name, parent=parent, child=child, origin=origin)], links


def get_topological_mates(
    graph: DiGraph,
    mates: dict[str, MateFeatureData],
    relations: Optional[dict[str, MateRelationFeatureData]] = None,
) -> tuple[dict[str, MateFeatureData], dict[str, MateRelationFeatureData]]:
    """
    Get the topological mates from the graph. This shuffles the order of the mates to match the directed graph edges.

    Args:
        graph: The graph representation of the assembly.
        mates: The dictionary of mates in the assembly.
        relations: The dictionary of relations in the assembly.

    Returns:
        tuple[dict[str, MateFeatureData], dict[str, MateRelationFeatureData]]: The topological mates and relations.

    Examples:
        >>> get_topological_mates(graph, mates)
        (
            {
                'link1-MATE-body': MateFeatureData(...),
                'subassembly1-SUB-link2-MATE-body': MateFeatureData(...),
            },
            {
                'link1-MATE-body-REL-subassembly1-SUB-link2-MATE-body': MateRelationFeatureData(...),
            }
        )
    """
    topological_mates: dict[str, MateFeatureData] = {}
    topological_relations: dict[str, MateRelationFeatureData] = relations or {}

    mate_keys = {tuple(key.split(MATE_JOINER)) for key in mates}
    graph_edges = set(graph.edges)

    rogue_mates = mate_keys.difference(graph_edges)

    for edge in graph.edges:
        parent, child = edge
        key = f"{parent}{MATE_JOINER}{child}"

        if (child, parent) in rogue_mates:
            # the only way it can be a rogue mate is if the parent and child are swapped
            # LOGGER.info(f"Rogue mate found: {edge}")
            rogue_key = f"{child}{MATE_JOINER}{parent}"
            topological_mates[key] = mates[rogue_key]

            if isinstance(topological_mates[key], MateFeatureData):
                topological_mates[key].matedEntities = topological_mates[key].matedEntities[::-1]

            if relations and rogue_key in topological_relations:
                LOGGER.info(f"Rogue relation found: {rogue_key}")
                topological_relations[key] = topological_relations[rogue_key]
                topological_relations.pop(rogue_key)

        else:
            topological_mates[key] = mates[key]

    return topological_mates, topological_relations
