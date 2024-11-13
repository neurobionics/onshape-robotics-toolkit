"""
This module contains functions to generate URDF components from Onshape assembly data.

"""

import io
import os
import random
from typing import Optional, Union

import numpy as np
import stl
from networkx import DiGraph

from onshape_api.connect import Client
from onshape_api.log import LOGGER
from onshape_api.mesh import transform_mesh
from onshape_api.models.assembly import (
    Assembly,
    MateFeatureData,
    MateRelationFeatureData,
    MateType,
    Part,
    RelationType,
)
from onshape_api.models.geometry import MeshGeometry
from onshape_api.models.joint import (
    BaseJoint,
    DummyJoint,
    FixedJoint,
    JointDynamics,
    JointLimits,
    JointMimic,
    PrismaticJoint,
    RevoluteJoint,
)
from onshape_api.models.link import (
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
from onshape_api.parse import MATE_JOINER, PARENT

SCRIPT_DIR = os.path.dirname(__file__)
CURRENT_DIR = os.getcwd()


def download_stl_mesh(
    did: str, wid: str, eid: str, partID: str, client: Client, transform: np.ndarray, file_name: str
) -> str:
    """
    Download an STL mesh from an Onshape part studio, transform it, and save it to a file.

    Args:
        did: The unique identifier of the document.
        wid: The unique identifier of the workspace.
        eid: The unique identifier of the element.
        partID: The unique identifier of the part.
        client: The Onshape client object to use for sending API requests.
        transform: The transformation matrix to apply to the mesh.
        file_name: The name of the file to save the mesh to.

    Returns:
        str: The relative path to the saved mesh file.

    Raises:
        Exception: If an error occurs while downloading the mesh.

    Examples:
        >>> download_stl_mesh("0b0c209535554345432581fe", "0b0c209535554345432581fe", "0b0c209535554345432581fe",
        ...                   "0b0c209535554345432581fe", client, np.eye(4), "part.stl")
        "meshes/part.stl"
    """

    try:
        with io.BytesIO() as buffer:
            LOGGER.info(f"Downloading mesh for {file_name}...")
            client.download_stl(did, wid, eid, partID, buffer)
            buffer.seek(0)

            raw_mesh = stl.mesh.Mesh.from_file(None, fh=buffer)
            transformed_mesh = transform_mesh(raw_mesh, transform)

            meshes_dir = os.path.join(CURRENT_DIR, "meshes")
            os.makedirs(meshes_dir, exist_ok=True)

            save_path = os.path.join(meshes_dir, file_name)
            transformed_mesh.save(save_path)
            LOGGER.info(f"Saved mesh to {save_path}")

            return os.path.relpath(save_path, CURRENT_DIR)

    except Exception as e:
        LOGGER.warning(f"An error occurred: {e}")
        raise


def get_joint_name(mate_id: str, mates: dict[str, MateFeatureData]) -> str:
    reverse_mates = {mate.id: key for key, mate in mates.items()}
    return reverse_mates.get(mate_id)


def get_robot_link(
    name: str,
    part: Part,
    wid: str,
    client: Client,
    mate: Optional[Union[MateFeatureData, None]] = None,
) -> tuple[Link, np.matrix]:
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
    if mate is None:
        _link_to_stl_tf = np.eye(4)
        _link_to_stl_tf[:3, 3] = np.array(part.MassProperty.center_of_mass).reshape(3)
    else:
        _link_to_stl_tf = mate.matedEntities[0].matedCS.part_to_mate_tf

    _stl_to_link_tf = np.matrix(np.linalg.inv(_link_to_stl_tf))
    _mass = part.MassProperty.mass[0]
    _origin = Origin.zero_origin()
    _com = part.MassProperty.center_of_mass_wrt(_stl_to_link_tf)
    _inertia = part.MassProperty.inertia_wrt(np.matrix(_stl_to_link_tf[:3, :3]))
    _principal_axes_rotation = (0.0, 0.0, 0.0)

    LOGGER.info(f"Creating robot link for {name}")

    _mesh_path = download_stl_mesh(
        part.documentId,
        wid,
        part.elementId,
        part.partId,
        client,
        _stl_to_link_tf,
        f"{name}.stl",
    )

    _link = Link(
        name=name,
        visual=VisualLink(
            origin=_origin,
            geometry=MeshGeometry(_mesh_path),
            material=Material.from_color(name=f"{name}_material", color=random.SystemRandom().choice(list(Colors))),
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
            origin=_origin,
            geometry=MeshGeometry(_mesh_path),
        ),
    )

    return _link, _stl_to_link_tf


def get_robot_joint(
    parent: str,
    child: str,
    mate: MateFeatureData,
    stl_to_parent_tf: np.matrix,
    mimic: Optional[JointMimic] = None,
) -> BaseJoint:
    """
    Generate a URDF joint from an Onshape mate feature.

    Args:
        parent: The name of the parent link.
        child: The name of the child link.
        mate: The Onshape mate feature object.
        stl_to_parent_tf: The transformation matrix from the STL origin to the parent link origin.

    Returns:
        Joint object that represents the URDF joint.

    Examples:
        >>> get_robot_joint("root", "link1", mate, np.eye(4))
        RevoluteJoint(
            name='base_link_to_link1',
            parent='root',
            child='link1',
            origin=Origin(...),
            limits=JointLimits(...),
            axis=Axis(...),
            dynamics=JointDynamics(...)
        )

    """

    parent_to_mate_tf = mate.matedEntities[PARENT].matedCS.part_to_mate_tf
    stl_to_mate_tf = stl_to_parent_tf @ parent_to_mate_tf

    origin = Origin.from_matrix(stl_to_mate_tf)

    LOGGER.info(f"Creating robot joint from {parent} to {child}")
    LOGGER.info(f"Joint mimic: {mimic}")

    if mate.mateType == MateType.REVOLUTE:
        return RevoluteJoint(
            name=f"{parent}{MATE_JOINER}{child}",
            parent=parent,
            child=child,
            origin=origin,
            limits=JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=-np.pi,
                upper=np.pi,
            ),
            axis=Axis((0.0, 0.0, -1.0)),
            dynamics=JointDynamics(damping=0.1, friction=0.1),
            mimic=mimic,
        )

    elif mate.mateType == MateType.FASTENED:
        return FixedJoint(name=f"{parent}{MATE_JOINER}{child}", parent=parent, child=child, origin=origin)

    elif mate.mateType == MateType.SLIDER or mate.mateType == MateType.CYLINDRICAL:
        return PrismaticJoint(
            name=f"{parent}{MATE_JOINER}{child}",
            parent=parent,
            child=child,
            origin=origin,
            limits=JointLimits(
                effort=1.0,
                velocity=1.0,
                lower=-0.1,
                upper=0.1,
            ),
            axis=Axis((0.0, 0.0, -1.0)),
            dynamics=JointDynamics(damping=0.1, friction=0.1),
            mimic=mimic,
        )

    else:
        LOGGER.warning(f"Unsupported joint type: {mate.mateType}")
        return DummyJoint(name=f"{parent}{MATE_JOINER}{child}", parent=parent, child=child, origin=origin)


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

    Returns:
        dict[str, MateFeatureData]: The topological mates.

    Examples:
        >>> get_topological_mates(graph, mates)
        {
            'link1-MATE-body': MateFeatureData(...),
            'subassembly1-SUB-link2-MATE-body': MateFeatureData(...),
        }
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
            topological_mates[key].matedEntities = topological_mates[key].matedEntities[::-1]

            if relations and rogue_key in topological_relations:
                topological_relations[key] = topological_relations[rogue_key]
                topological_relations.pop(rogue_key)

        else:
            topological_mates[key] = mates[key]

    return topological_mates, topological_relations


def get_urdf_components(
    assembly: Assembly,
    graph: DiGraph,
    root_node: str,
    parts: dict[str, Part],
    mates: dict[str, MateFeatureData],
    relations: dict[str, MateRelationFeatureData],
    client: Client,
) -> tuple[list[Link], list[BaseJoint]]:
    """
    Generate URDF links and joints from an Onshape assembly.

    Args:
        assembly: The Onshape assembly object.
        graph: The graph representation of the assembly.
        parts: The dictionary of parts in the assembly.
        mates: The dictionary of mates in the assembly.
        client: The Onshape client object to use for sending API requests.

    Returns:
        tuple[list[Link], list[BaseJoint]]: The generated URDF links and joints.

    Examples:
        >>> get_urdf_components(assembly, graph, parts, mates, client)
        (
            [
                Link(name='root', visual=VisualLink(...), collision=CollisionLink(...), inertial=InertialLink(...)),
                Link(name='link1', visual=VisualLink(...), collision=CollisionLink(...), inertial=InertialLink(...)),
                Link(name='link2', visual=VisualLink(...), collision=CollisionLink(...), inertial=InertialLink(...))
            ],
            [
                RevoluteJoint(...),
                FixedJoint(...),
            ]
        )

    """
    joints = []
    links = []

    topological_mates, topological_relations = get_topological_mates(graph, mates, relations)

    stl_to_link_tf_map = {}

    LOGGER.info(f"Processing root node: {root_node}")

    root_link, stl_to_root_tf = get_robot_link(root_node, parts[root_node], assembly.document.wid, client, None)

    links.append(root_link)
    stl_to_link_tf_map[root_node] = stl_to_root_tf

    LOGGER.info(f"Processing remaining {len(graph.nodes) - 1} nodes using {len(graph.edges)} edges")

    for edge in graph.edges:
        parent, child = edge
        mate_key = f"{parent}{MATE_JOINER}{child}"

        parent_tf = stl_to_link_tf_map[parent]

        if parent and child not in parts:
            LOGGER.warning(f"Part {parent} or {child} not found in parts")
            continue

        relation = topological_relations.get(topological_mates[mate_key].id)

        if relation:
            multiplier = 1.0
            if relation.relationType == RelationType.RACK_AND_PINION:
                multiplier = relation.relationLength

            elif relation.relationType == RelationType.GEAR or relation.relationType == RelationType.LINEAR:
                multiplier = relation.relationRatio

            joint_mimic = JointMimic(
                joint=get_joint_name(relation.mates[PARENT].featureId, topological_mates),
                multiplier=multiplier,
                offset=0.0,
            )
        else:
            joint_mimic = None

        joint = get_robot_joint(parent, child, topological_mates[mate_key], parent_tf, joint_mimic)
        joints.append(joint)

        link, stl_to_link_tf = get_robot_link(
            child,
            parts[child],
            assembly.document.wid,
            client,
            topological_mates[mate_key],
        )
        stl_to_link_tf_map[child] = stl_to_link_tf
        links.append(link)

    return links, joints
