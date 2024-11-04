import io
import os
import random
from typing import Optional, Union

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import stl

from onshape_api.connect import Client
from onshape_api.log import LOGGER
from onshape_api.mesh import transform_mesh
from onshape_api.models.assembly import (
    MATETYPE,
    Instance,
    InstanceType,
    MateFeatureData,
    Occurrence,
    Part,
)
from onshape_api.models.geometry import MeshGeometry
from onshape_api.models.joint import FixedJoint, JointDynamics, JointLimits, RevoluteJoint
from onshape_api.models.link import (
    COLORS,
    Axis,
    CollisionLink,
    Inertia,
    InertialLink,
    Link,
    Material,
    Origin,
    VisualLink,
)
from onshape_api.models.mass import MassModel
from onshape_api.parse import MATE_JOINER

SCRIPT_DIR = os.path.dirname(__file__)
CURRENT_DIR = os.getcwd()


def generate_names(max_length: int) -> list[str]:
    words_file_path = os.path.join(SCRIPT_DIR, "words.txt")

    with open(words_file_path) as file:
        words = file.read().splitlines()

    if max_length > len(words):
        raise ValueError("max_length exceeds the number of available words")

    return random.sample(words, max_length)


def show_graph(graph: nx.Graph) -> None:
    nx.draw_circular(graph, with_labels=True)
    plt.show()


def convert_to_digraph(graph: nx.Graph) -> tuple[nx.DiGraph, str]:
    _centrality = nx.closeness_centrality(graph)
    _root_node = max(_centrality, key=_centrality.get)
    _graph = nx.bfs_tree(graph, _root_node)
    return _graph, _root_node


def create_graph(
    occurences: dict[str, Occurrence],
    instances: dict[str, Instance],
    parts: dict[str, Part],
    mates: dict[str, MateFeatureData],
    directed: bool = True,
) -> Union[nx.Graph, tuple[nx.DiGraph, str]]:
    graph = nx.Graph()

    for occurence in occurences:
        if instances[occurence].type == InstanceType.PART:
            try:
                if occurences[occurence].hidden:
                    continue

                graph.add_node(occurence, **parts[occurence].model_dump())
            except KeyError:
                LOGGER.warning(f"Part {occurence} not found")

    for mate in mates:
        try:
            child, parent = mate.split(MATE_JOINER)
            graph.add_edge(child, parent, **mates[mate].model_dump())
        except KeyError:
            LOGGER.warning(f"Mate {mate} not found")

    if directed:
        graph, root_node = convert_to_digraph(graph)

    LOGGER.info(f"Graph created with {len(graph.nodes)} nodes and {len(graph.edges)} edges")

    return graph


def download_stl_mesh(
    did: str, wid: str, eid: str, partID: str, client: Client, transform: np.ndarray, file_name: str
) -> str:
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
        print(f"An error occurred: {e}")
        raise


def get_robot_link(
    name: str,
    part: Part,
    mass_property: MassModel,
    workspaceId: str,
    client: Client,
    mate: Optional[Union[MateFeatureData, None]] = None,
) -> tuple[Link, np.matrix]:
    LOGGER.info(f"Creating robot link for {name}")

    if mate is None:
        _link_to_stl_tf = np.eye(4)
        _link_to_stl_tf[:3, 3] = np.array(mass_property.center_of_mass).reshape(3)
    else:
        _link_to_stl_tf = mate.matedEntities[0].matedCS.part_to_mate_tf

    _stl_to_link_tf = np.matrix(np.linalg.inv(_link_to_stl_tf))
    _mass = mass_property.mass[0]
    _origin = Origin.zero_origin()
    _com = mass_property.center_of_mass_wrt(_stl_to_link_tf)
    _inertia = mass_property.inertia_wrt(np.matrix(_stl_to_link_tf[:3, :3]))
    _principal_axes_rotation = (0.0, 0.0, 0.0)

    _mesh_path = download_stl_mesh(
        part.documentId,
        workspaceId,
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
            material=Material.from_color(name=f"{name}_material", color=random.SystemRandom().choice(list(COLORS))),
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
) -> Union[RevoluteJoint, FixedJoint]:
    LOGGER.info(f"Creating robot joint from {parent} to {child}")

    parent_to_mate_tf = mate.matedEntities[1].matedCS.part_to_mate_tf
    stl_to_mate_tf = stl_to_parent_tf @ parent_to_mate_tf

    origin = Origin.from_matrix(stl_to_mate_tf)

    match mate.mateType:
        case MATETYPE.REVOLUTE:
            return RevoluteJoint(
                name=f"{parent}_to_{child}",
                parent=parent,
                child=child,
                origin=origin,
                limits=JointLimits(
                    effort=1.0,
                    velocity=1.0,
                    lower=-np.pi,
                    upper=np.pi,
                ),
                axis=Axis((0.0, 0.0, 1.0)),
                dynamics=JointDynamics(damping=0.1, friction=0.1),
            )

        case MATETYPE.FASTENED:
            return FixedJoint(name=f"{parent}_to_{child}", parent=parent, child=child, origin=origin)

        case _:
            raise ValueError(
                f"We only support fastened and revolute joints for now, got {mate.mateType}. "
                "Please check back later."
            )


def get_urdf_components(
    graph: Union[nx.Graph, nx.DiGraph],
    workspaceId: str,
    parts: dict[str, Part],
    mass_properties: dict[str, MassModel],
    mates: dict[str, MateFeatureData],
    client: Client,
) -> tuple[list[Link], list[Union[RevoluteJoint, FixedJoint]]]:
    if not isinstance(graph, nx.DiGraph):
        graph, root_node = convert_to_digraph(graph)

    joints: list[Union[RevoluteJoint, FixedJoint]] = []
    links: list[Link] = []

    _readable_names = generate_names(len(graph.nodes))
    _readable_names_mapping = dict(zip(graph.nodes, _readable_names))
    _stl_to_link_tf_mapping: dict[str, np.matrix] = {}

    LOGGER.info(f"Processing root node: {_readable_names_mapping[root_node]}")

    root_link, stl_to_root_tf = get_robot_link(
        _readable_names_mapping[root_node], parts[root_node], mass_properties[root_node], workspaceId, client, None
    )

    links.append(root_link)
    _stl_to_link_tf_mapping[root_node] = stl_to_root_tf

    LOGGER.info(f"Processing remaining {len(graph.nodes) - 1} nodes using {len(graph.edges)} edges")

    for edge in graph.edges:
        parent, child = edge
        _parent_tf = _stl_to_link_tf_mapping[parent]

        # TODO: should mate keys be parent to child in the parser?
        _mate_key = f"{child}{MATE_JOINER}{parent}"

        if _mate_key not in mates:
            # TODO: subassembly to root mates have a funky convention
            _mate_key = f"{parent}{MATE_JOINER}{child}"

        _joint = get_robot_joint(
            _readable_names_mapping[parent],
            _readable_names_mapping[child],
            mates[_mate_key],
            _parent_tf,
        )
        joints.append(_joint)

        _link, _stl_to_link_tf = get_robot_link(
            _readable_names_mapping[child],
            parts[child],
            mass_properties[child],
            workspaceId,
            client,
            mates[_mate_key],
        )
        _stl_to_link_tf_mapping[child] = _stl_to_link_tf
        links.append(_link)

    return links, joints
