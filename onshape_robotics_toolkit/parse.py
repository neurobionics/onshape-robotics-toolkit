"""
This module contains functions that provide a way to traverse the assembly structure, extract information about parts,
subassemblies, instances, and mates, and generate a hierarchical representation of the assembly.

"""

import asyncio
import copy
import uuid
from typing import Optional, Union

import networkx as nx
import numpy as np

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import (
    Assembly,
    AssemblyFeature,
    AssemblyFeatureType,
    AssemblyInstance,
    InstanceType,
    MatedCS,
    MateFeatureData,
    MateRelationFeatureData,
    Occurrence,
    Part,
    PartInstance,
    Pattern,
    RelationType,
    RootAssembly,
    SubAssembly,
)
from onshape_robotics_toolkit.models.document import WorkspaceType
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name

SUBASSEMBLY_JOINER = f"_{uuid.uuid4().hex[:4].upper()}_"
MATE_JOINER = f"_{uuid.uuid4().hex[:4].upper()}_"

LOGGER.info(f"Generated joiners - SUBASSEMBLY: {SUBASSEMBLY_JOINER}, MATE: {MATE_JOINER}")

CHILD = 0
PARENT = 1

RELATION_CHILD = 1
RELATION_PARENT = 0


# TODO: get_mate_connectors method to parse part mate connectors that may be useful to someone
async def traverse_instances_async(
    root: Union[RootAssembly, SubAssembly],
    prefix: str,
    current_depth: int,
    max_depth: int,
    assembly: Assembly,
    id_to_name_map: dict[str, str],
    instance_map: dict[str, Union[PartInstance, AssemblyInstance]],
    instance_proxy_map: dict[str, str],
    rigid_subassembly_prefix: str = "",
) -> None:
    """
    Asynchronously traverse the assembly structure to get instances.

    Args:
        root: The root assembly or subassembly to traverse.
        prefix: The prefix for the instance ID.
        current_depth: The current depth in the assembly hierarchy.
        max_depth: The maximum depth to traverse.
        assembly: The assembly object to traverse.
        id_to_name_map: A dictionary mapping instance IDs to their sanitized names.
        instance_map: A dictionary mapping instance IDs to their corresponding instances.
    """
    isRigid = False
    if current_depth >= max_depth:
        LOGGER.debug(
            f"Max depth {max_depth} reached. Assuming all sub-assemblies to be rigid at depth {current_depth}."
        )
        isRigid = True

    for instance in root.instances:
        sanitized_name = get_sanitized_name(instance.name)
        LOGGER.debug(f"Parsing instance: {sanitized_name}")
        instance_id = f"{prefix}{SUBASSEMBLY_JOINER}{sanitized_name}" if prefix else sanitized_name
        id_to_name_map[instance.id] = sanitized_name
        instance_map[instance_id] = instance

        # Determine proxy mapping based on rigid subassembly hierarchy
        if rigid_subassembly_prefix:
            # We're inside a rigid subassembly, map to the rigid subassembly root
            instance_proxy_map[instance_id] = rigid_subassembly_prefix
        else:
            # Not in a rigid subassembly, map to self
            instance_proxy_map[instance_id] = instance_id

        if instance.type == InstanceType.ASSEMBLY:
            instance_map[instance_id].isRigid = isRigid

            # Determine the rigid subassembly prefix for child traversal
            child_rigid_prefix = rigid_subassembly_prefix
            if isRigid and not rigid_subassembly_prefix:
                # This is the first rigid subassembly we encounter, set it as the prefix
                child_rigid_prefix = instance_id

            tasks = [
                traverse_instances_async(
                    sub_assembly,
                    instance_id,
                    current_depth + 1,
                    max_depth,
                    assembly,
                    id_to_name_map,
                    instance_map,
                    instance_proxy_map,
                    child_rigid_prefix,
                )
                for sub_assembly in assembly.subAssemblies
                if sub_assembly.uid == instance.uid
            ]
            await asyncio.gather(*tasks)


def get_instances(
    assembly: Assembly, max_depth: int = 0
) -> tuple[dict[str, Union[PartInstance, AssemblyInstance]], dict[str, str], dict[str, Occurrence], dict[str, str]]:
    """
    Optimized synchronous wrapper for `get_instances`.

    Args:
        assembly: The assembly object to traverse.
        max_depth: The maximum depth to traverse.

    Returns:
        A tuple containing:
        - A dictionary mapping instance IDs to their corresponding instances.
        - A dictionary mapping instance IDs to their sanitized names.
    """
    instance_map: dict[str, Union[PartInstance, AssemblyInstance]] = {}
    id_to_name_map: dict[str, str] = {}
    instance_proxy_map: dict[str, str] = {}
    asyncio.run(
        traverse_instances_async(
            assembly.rootAssembly, "", 0, max_depth, assembly, id_to_name_map, instance_map, instance_proxy_map
        )
    )
    occurrence_map = get_occurrences(assembly, id_to_name_map, max_depth)
    return instance_map, instance_proxy_map, occurrence_map, id_to_name_map


def get_instances_sync(
    assembly: Assembly, max_depth: int = 0
) -> tuple[dict[str, Union[PartInstance, AssemblyInstance]], dict[str, Occurrence], dict[str, str]]:
    """
    Get instances and their sanitized names from an Onshape assembly.

    Args:
        assembly: The Onshape assembly object to use for extracting instances.
        max_depth: Maximum depth to traverse in the assembly hierarchy. Default is 0

    Returns:
        A tuple containing:
        - A dictionary mapping instance IDs to their corresponding instances.
        - A dictionary mapping instance IDs to their sanitized names.

    Examples:
        >>> assembly = Assembly(...)
        >>> get_instances(assembly, max_depth=2)
        (
            {
                "part1": PartInstance(...),
                "subassembly1": AssemblyInstance(...),
            },
            {
                "part1": "part1",
                "subassembly1": "subassembly1",
            }
        )
    """

    def traverse_instances(
        root: Union[RootAssembly, SubAssembly], prefix: str = "", current_depth: int = 0
    ) -> tuple[dict[str, Union[PartInstance, AssemblyInstance]], dict[str, str]]:
        """
        Traverse the assembly structure to get instances.

        Args:
            root: Root assembly or subassembly object to traverse.
            prefix: Prefix for the instance ID.
            current_depth: Current depth in the assembly hierarchy.

        Returns:
            A tuple containing:
            - A dictionary mapping instance IDs to their corresponding instances.
            - A dictionary mapping instance IDs to their sanitized names.
        """
        instance_map = {}
        id_to_name_map = {}

        # Stop traversing if the maximum depth is reached
        if current_depth >= max_depth:
            LOGGER.debug(f"Max depth {max_depth} reached. Stopping traversal at depth {current_depth}.")
            return instance_map, id_to_name_map

        for instance in root.instances:
            sanitized_name = get_sanitized_name(instance.name)
            LOGGER.debug(f"Parsing instance: {sanitized_name}")
            instance_id = f"{prefix}{SUBASSEMBLY_JOINER}{sanitized_name}" if prefix else sanitized_name
            id_to_name_map[instance.id] = sanitized_name
            instance_map[instance_id] = instance

            # Recursively process sub-assemblies if applicable
            if instance.type == InstanceType.ASSEMBLY:
                for sub_assembly in assembly.subAssemblies:
                    if sub_assembly.uid == instance.uid:
                        sub_instance_map, sub_id_to_name_map = traverse_instances(
                            sub_assembly, instance_id, current_depth + 1
                        )
                        instance_map.update(sub_instance_map)
                        id_to_name_map.update(sub_id_to_name_map)

        return instance_map, id_to_name_map

    instance_map, id_to_name_map = traverse_instances(assembly.rootAssembly)
    # return occurrences internally as it relies on max_depth
    occurrence_map = get_occurrences(assembly, id_to_name_map, max_depth)

    return instance_map, occurrence_map, id_to_name_map


def get_occurrences(assembly: Assembly, id_to_name_map: dict[str, str], max_depth: int = 0) -> dict[str, Occurrence]:
    """
    Optimized occurrences fetching using comprehensions.

    Args:
        assembly: The assembly object to traverse.
        id_to_name_map: A dictionary mapping instance IDs to their sanitized names.
        max_depth: The maximum depth to traverse. Default is 0

    Returns:
        A dictionary mapping occurrence paths to their corresponding occurrences.
    """
    return {
        SUBASSEMBLY_JOINER.join([
            id_to_name_map[path] for path in occurrence.path if path in id_to_name_map
        ]): occurrence
        for occurrence in assembly.rootAssembly.occurrences
        # if len(occurrence.path) <= max_depth + 1 # let's get all occurrences regardless of depth
    }


async def fetch_rigid_subassemblies_async(
    subassembly: SubAssembly, key: str, client: Client, rigid_subassembly_map: dict[str, RootAssembly]
):
    """
    Fetch rigid subassemblies asynchronously.

    Args:
        subassembly: The subassembly to fetch.
        key: The instance key to fetch.
        client: The client object to use for fetching the subassembly.
        rigid_subassembly_map: A dictionary to store the fetched subassemblies.
    """
    try:
        rigid_subassembly_map[key] = await asyncio.to_thread(
            client.get_root_assembly,
            did=subassembly.documentId,
            wtype=WorkspaceType.M.value,
            wid=subassembly.documentMicroversion,
            eid=subassembly.elementId,
            with_mass_properties=True,
            log_response=False,
        )
    except Exception as e:
        LOGGER.error(f"Failed to fetch rigid subassembly for {key}: {e}")


async def get_subassemblies_async(
    assembly: Assembly,
    client: Client,
    instance_map: dict[str, Union[PartInstance, AssemblyInstance]],
) -> tuple[dict[str, SubAssembly], dict[str, RootAssembly]]:
    """
    Asynchronously fetch subassemblies.

    Args:
        assembly: The assembly object to traverse.
        client: The client object to use for fetching the subassemblies.
        instance_map: A dictionary mapping instance IDs to their corresponding instances.

    Returns:
        A tuple containing:
        - A dictionary mapping instance IDs to their corresponding subassemblies.
        - A dictionary mapping instance IDs to their corresponding rigid subassemblies.
    """
    subassembly_map: dict[str, SubAssembly] = {}
    rigid_subassembly_map: dict[str, RootAssembly] = {}

    # Group by UID
    subassembly_instance_map = {}
    rigid_subassembly_instance_map = {}

    for instance_key, instance in instance_map.items():
        if instance.type == InstanceType.ASSEMBLY:
            if instance.isRigid:
                rigid_subassembly_instance_map.setdefault(instance.uid, []).append(instance_key)
            else:
                subassembly_instance_map.setdefault(instance.uid, []).append(instance_key)

    # Process subassemblies concurrently
    # TODO: currently we parse rigid-subassemblies within a rigid-subassemly too, this could be bad
    # and cause random mates using wrong references and ending up in numberous sub-graphs
    tasks = []
    for subassembly in assembly.subAssemblies:
        uid = subassembly.uid
        if uid in subassembly_instance_map:
            is_rigid = len(subassembly.features) == 0 or all(
                feature.featureType == AssemblyFeatureType.MATEGROUP for feature in subassembly.features
            )
            for key in subassembly_instance_map[uid]:
                if is_rigid:
                    tasks.append(fetch_rigid_subassemblies_async(subassembly, key, client, rigid_subassembly_map))
                else:
                    subassembly_map[key] = subassembly

        elif uid in rigid_subassembly_instance_map:
            for key in rigid_subassembly_instance_map[uid]:
                tasks.append(fetch_rigid_subassemblies_async(subassembly, key, client, rigid_subassembly_map))

    await asyncio.gather(*tasks)
    return subassembly_map, rigid_subassembly_map


def get_subassemblies(
    assembly: Assembly,
    client: Client,
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
) -> tuple[dict[str, SubAssembly], dict[str, RootAssembly]]:
    """
    Synchronous wrapper for `get_subassemblies_async`.

    Args:
        assembly: The assembly object to traverse.
        client: The client object to use for fetching the subassemblies.
        instances: A dictionary mapping instance IDs to their corresponding instances.

    Returns:
        A tuple containing:
        - A dictionary mapping instance IDs to their corresponding subassemblies.
        - A dictionary mapping instance IDs to their corresponding rigid subassemblies.
    """
    return asyncio.run(get_subassemblies_async(assembly, client, instances))


async def _fetch_mass_properties_async(
    part: Part,
    key: str,
    client: Client,
    rigid_subassemblies: dict[str, RootAssembly],
    parts: dict[str, Part],
    include_rigid_subassembly_parts: bool = True,
):
    """
    Asynchronously fetch mass properties for a part.

    Args:
        part: The part for which to fetch mass properties.
        key: The instance key associated with the part.
        client: The Onshape client object.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        parts: The dictionary to store fetched parts.
        include_rigid_subassembly_parts: Whether to include parts from rigid subassemblies.
    """
    if include_rigid_subassembly_parts or key.split(SUBASSEMBLY_JOINER)[0] not in rigid_subassemblies:
        try:
            LOGGER.info(f"Fetching mass properties for part: {part.uid}, {part.partId}")
            part.MassProperty = await asyncio.to_thread(
                client.get_mass_property,
                did=part.documentId,
                wtype=WorkspaceType.M.value,
                wid=part.documentMicroversion,
                eid=part.elementId,
                partID=part.partId,
            )
        except Exception as e:
            LOGGER.error(f"Failed to fetch mass properties for part {part.partId}: {e}")

    parts[key] = part


async def _get_parts_with_selective_mass_properties_async(
    parts: dict[str, Part],
    graph: nx.Graph,
    client: Client,
    rigid_subassemblies: dict[str, RootAssembly],
    include_rigid_subassembly_parts: bool = True,
) -> None:
    """
    Asynchronously fetch mass properties for specific parts only.

    Args:
        parts: Dictionary of parts to potentially update with mass properties.
        graph: Graph of the robot.
        client: The Onshape client object.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        include_rigid_subassembly_parts: Whether to include parts from rigid subassemblies.
    """
    tasks = []
    for key in graph.nodes:
        if key in parts:
            part = parts[key]
            # Check if we should skip parts from rigid subassemblies
            if not include_rigid_subassembly_parts and key.split(SUBASSEMBLY_JOINER)[0] in rigid_subassemblies:
                continue

            # Skip if this is a rigid subassembly (already has mass properties)
            if part.isRigidAssembly:
                continue

            tasks.append(
                _fetch_mass_properties_async(
                    part, key, client, rigid_subassemblies, parts, include_rigid_subassembly_parts
                )
            )

    if tasks:
        await asyncio.gather(*tasks)


async def _get_parts_without_mass_properties_async(
    assembly: Assembly,
    rigid_subassemblies: dict[str, RootAssembly],
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
) -> dict[str, Part]:
    """
    Asynchronously get parts of an Onshape assembly without fetching mass properties.

    Args:
        assembly: The Onshape assembly object to use for extracting parts.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        instances: Mapping of instance IDs to their corresponding instances.

    Returns:
        A dictionary mapping part IDs to their corresponding part objects.
    """
    part_instance_map: dict[str, list[str]] = {}
    part_map: dict[str, Part] = {}

    for key, instance in instances.items():
        if instance.type == InstanceType.PART:
            part_instance_map.setdefault(instance.uid, []).append(key)

    # Create Part objects without mass properties
    for part in assembly.parts:
        if part.uid in part_instance_map:
            for key in part_instance_map[part.uid]:
                part_map[key] = part

    # Convert rigid subassemblies to parts
    for assembly_key, rigid_subassembly in rigid_subassemblies.items():
        part_map[assembly_key] = Part(
            isStandardContent=False,
            fullConfiguration=rigid_subassembly.fullConfiguration,
            configuration=rigid_subassembly.configuration,
            documentId=rigid_subassembly.documentId,
            elementId=rigid_subassembly.elementId,
            documentMicroversion=rigid_subassembly.documentMicroversion,
            documentVersion="",
            partId="",
            bodyType="",
            MassProperty=rigid_subassembly.MassProperty,
            isRigidAssembly=True,
            rigidAssemblyWorkspaceId=rigid_subassembly.documentMetaData.defaultWorkspace.id,
            rigidAssemblyToPartTF={},
        )

    return part_map


async def _get_parts_async(
    assembly: Assembly,
    rigid_subassemblies: dict[str, RootAssembly],
    client: Client,
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
) -> dict[str, Part]:
    """
    Asynchronously get parts of an Onshape assembly using the legacy workflow.

    WARNING: This function fetches mass properties for ALL parts in the assembly,
    which can be very slow for complex assemblies with many parts.

    Consider using the optimized workflow with _get_parts_without_mass_properties_async()
    and _get_parts_with_selective_mass_properties_async() instead.

    Args:
        assembly: The Onshape assembly object to use for extracting parts.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        client: The Onshape client object.
        instances: Mapping of instance IDs to their corresponding instances.

    Returns:
        A dictionary mapping part IDs to their corresponding part objects.
    """
    part_instance_map: dict[str, list[str]] = {}
    part_map: dict[str, Part] = {}

    for key, instance in instances.items():
        if instance.type == InstanceType.PART:
            part_instance_map.setdefault(instance.uid, []).append(key)

    tasks = []
    for part in assembly.parts:
        if part.uid in part_instance_map:
            for key in part_instance_map[part.uid]:
                tasks.append(
                    _fetch_mass_properties_async(
                        part, key, client, rigid_subassemblies, part_map, include_rigid_subassembly_parts=False
                    )
                )

    await asyncio.gather(*tasks)

    # Convert rigid subassemblies to parts
    for assembly_key, rigid_subassembly in rigid_subassemblies.items():
        if rigid_subassembly.MassProperty is None:
            LOGGER.warning(f"Rigid subassembly {assembly_key} has no mass properties")

        part_map[assembly_key] = Part(
            isStandardContent=False,
            fullConfiguration=rigid_subassembly.fullConfiguration,
            configuration=rigid_subassembly.configuration,
            documentId=rigid_subassembly.documentId,
            elementId=rigid_subassembly.elementId,
            documentMicroversion=rigid_subassembly.documentMicroversion,
            documentVersion="",
            partId="",
            bodyType="",
            MassProperty=rigid_subassembly.MassProperty,
            isRigidAssembly=True,
            rigidAssemblyWorkspaceId=rigid_subassembly.documentMetaData.defaultWorkspace.id,
            rigidAssemblyToPartTF={},
        )

    return part_map


def get_parts(
    assembly: Assembly,
    rigid_subassemblies: dict[str, RootAssembly],
    client: Client,
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
    graph: nx.Graph = None,
    include_rigid_subassembly_parts: bool = False,
) -> dict[str, Part]:
    """
    Get parts of an Onshape assembly with optimized mass property fetching.

    This function uses a two-phase approach by default:
    1. Create Part objects without mass properties (fast)
    2. Only fetch mass properties for parts that become URDF links (graph nodes)

    Args:
        assembly: The Onshape assembly object to use for extracting parts.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        client: The Onshape client object to use for sending API requests.
        instances: Mapping of instance IDs to their corresponding instances.
        graph: Graph of the robot. If provided, only fetches mass properties for graph nodes.
               If None, falls back to legacy behavior (fetches all mass properties).
        include_rigid_subassembly_parts: Whether to include parts from rigid subassemblies.

    Returns:
        A dictionary mapping part IDs to their corresponding part objects.
    """
    if graph is not None:
        # Optimized workflow: two-phase approach
        parts = get_parts_without_mass_properties(assembly, rigid_subassemblies, instances)
        get_parts_with_selective_mass_properties(
            parts, graph, client, rigid_subassemblies, include_rigid_subassembly_parts
        )
        return parts
    else:
        # Legacy workflow: fetch all mass properties
        import warnings

        warnings.warn(
            "Calling get_parts() without a graph parameter uses the legacy workflow that fetches "
            "mass properties for ALL parts, which can be very slow for complex assemblies. "
            "Consider passing a graph parameter or use get_parts_legacy() explicitly.",
            DeprecationWarning,
            stacklevel=2,
        )
        return get_parts_legacy(assembly, rigid_subassemblies, client, instances)


def get_parts_legacy(
    assembly: Assembly,
    rigid_subassemblies: dict[str, RootAssembly],
    client: Client,
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
) -> dict[str, Part]:
    """
    Get parts of an Onshape assembly using the legacy workflow.

    WARNING: This function fetches mass properties for ALL parts in the assembly,
    which can be very slow for complex assemblies with many parts.

    Consider using get_parts() with a graph parameter for better performance.

    Args:
        assembly: The Onshape assembly object to use for extracting parts.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        client: The Onshape client object to use for sending API requests.
        instances: Mapping of instance IDs to their corresponding instances.

    Returns:
        A dictionary mapping part IDs to their corresponding part objects.
    """
    import warnings

    warnings.warn(
        "get_parts_legacy() fetches mass properties for ALL parts, which can be very slow. "
        "Consider using get_parts() with a graph parameter for better performance.",
        DeprecationWarning,
        stacklevel=2,
    )
    return asyncio.run(_get_parts_async(assembly, rigid_subassemblies, client, instances))


def get_parts_without_mass_properties(
    assembly: Assembly,
    rigid_subassemblies: dict[str, RootAssembly],
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
) -> dict[str, Part]:
    """
    Get parts of an Onshape assembly without fetching mass properties.

    Args:
        assembly: The Onshape assembly object to use for extracting parts.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        instances: Mapping of instance IDs to their corresponding instances.

    Returns:
        A dictionary mapping part IDs to their corresponding part objects.
    """
    return asyncio.run(_get_parts_without_mass_properties_async(assembly, rigid_subassemblies, instances))


def get_parts_with_selective_mass_properties(
    parts: dict[str, Part],
    graph: nx.Graph,
    client: Client,
    rigid_subassemblies: dict[str, RootAssembly],
    include_rigid_subassembly_parts: bool = False,
) -> None:
    """
    Fetch mass properties for specific parts only.

    Args:
        parts: Dictionary of parts to potentially update with mass properties.
        graph: Graph of the robot.
        client: The Onshape client object.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        include_rigid_subassembly_parts: Whether to include parts from rigid subassemblies.
    """
    return asyncio.run(
        _get_parts_with_selective_mass_properties_async(
            parts, graph, client, rigid_subassemblies, include_rigid_subassembly_parts
        )
    )


def get_parts_optimized(
    assembly: Assembly,
    rigid_subassemblies: dict[str, RootAssembly],
    client: Client,
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
    graph: nx.Graph = None,
    include_rigid_subassembly_parts: bool = False,
) -> dict[str, Part]:
    """
    Get parts of an Onshape assembly with optimized mass property fetching.

    This function uses a two-phase approach:
    1. Create Part objects without mass properties
    2. Only fetch mass properties for parts that become URDF links (graph nodes)

    Args:
        assembly: The Onshape assembly object to use for extracting parts.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        client: The Onshape client object to use for sending API requests.
        instances: Mapping of instance IDs to their corresponding instances.
        graph: Graph of the robot.
        include_rigid_subassembly_parts: Whether to include parts from rigid subassemblies.

    Returns:
        A dictionary mapping part IDs to their corresponding part objects.
    """
    parts = get_parts_without_mass_properties(assembly, rigid_subassemblies, instances)
    get_parts_with_selective_mass_properties(parts, graph, client, rigid_subassemblies, include_rigid_subassembly_parts)

    return parts


def get_occurrence_name(occurrences: list[str], subassembly_prefix: Optional[str] = None) -> str:
    """
    Get the mapping name for an occurrence path.

    Args:
        occurrences: Occurrence path.
        subassembly_prefix: Prefix for the subassembly.

    Returns:
        The mapping name.

    Examples:
        >>> get_occurrence_name(["subassembly1", "part1"], "subassembly1")
        "subassembly1-SUB-part1"

        >>> get_occurrence_name(["part1"], "subassembly1")
        "subassembly1-SUB-part1"
    """
    prefix = f"{subassembly_prefix}{SUBASSEMBLY_JOINER}" if subassembly_prefix else ""
    return f"{prefix}{SUBASSEMBLY_JOINER.join(occurrences)}"


def get_proxy_occurrence_name(
    occurrences: list[str], instance_proxy_map: dict[str, str], subassembly_prefix: Optional[str] = None
) -> str:
    """
    Get the mapping name for an occurrence path using proxy mappings for rigid subassemblies.

    Args:
        occurrences: Occurrence path.
        instance_proxy_map: Mapping of instance IDs to their proxy names (rigid subassembly roots).
        subassembly_prefix: Prefix for the subassembly.

    Returns:
        The mapping name using proxy for rigid subassemblies.

    Examples:
        >>> get_proxy_occurrence_name(["rigid_sub", "part1"], {"rigid_sub_JOINER_part1": "rigid_sub"}, "subassembly1")
        "subassembly1-SUB-rigid_sub"
    """
    # Build the full occurrence path key
    prefix = f"{subassembly_prefix}{SUBASSEMBLY_JOINER}" if subassembly_prefix else ""
    occurrence_key = f"{prefix}{SUBASSEMBLY_JOINER.join(occurrences)}"

    # Use proxy mapping if available, otherwise use the original occurrence name
    proxy_name = instance_proxy_map.get(occurrence_key, SUBASSEMBLY_JOINER.join(occurrences))

    # Handle case where proxy_name includes subassembly prefix already
    if subassembly_prefix and proxy_name.startswith(f"{subassembly_prefix}{SUBASSEMBLY_JOINER}"):
        return proxy_name
    elif subassembly_prefix:
        return f"{subassembly_prefix}{SUBASSEMBLY_JOINER}{proxy_name}"
    else:
        return proxy_name


def find_occurrence_by_id(occurrence_id: str, occurrences: dict[str, Occurrence]) -> Optional[Occurrence]:
    """
    Find occurrence by ID, handling both direct keys and IDs within occurrence paths.

    Args:
        occurrence_id: The occurrence ID to search for
        occurrences: Dictionary of occurrences keyed by occurrence paths

    Returns:
        The occurrence if found, None otherwise
    """
    # First try direct key lookup (most common case for root-level entities)
    if occurrence_id in occurrences:
        return occurrences[occurrence_id]

    # Search through all occurrences to find one that contains this ID in its path
    for occurrence in occurrences.values():
        if occurrence_id in occurrence.path:
            return occurrence

    return None


def join_mate_occurrences(parent: list[str], child: list[str], prefix: Optional[str] = None) -> str:
    """
    Join two occurrence paths with a mate joiner.

    Args:
        parent: Occurrence path of the parent entity.
        child: Occurrence path of the child entity.
        prefix: Prefix to add to the occurrence path.

    Returns:
        The joined occurrence path.

    Examples:
        >>> join_mate_occurrences(["subassembly1", "part1"], ["subassembly2"])
        "subassembly1-SUB-part1-MATE-subassembly2"

        >>> join_mate_occurrences(["part1"], ["part2"])
        "part1-MATE-part2"
    """
    parent_occurrence = get_occurrence_name(parent, prefix)
    child_occurrence = get_occurrence_name(child, prefix)
    return f"{parent_occurrence}{MATE_JOINER}{child_occurrence}"


def join_mate_occurrences_with_proxy(
    parent: list[str], child: list[str], instance_proxy_map: dict[str, str], prefix: Optional[str] = None
) -> str:
    """
    Join two occurrence paths with a mate joiner using proxy mappings for rigid subassemblies.

    Args:
        parent: Occurrence path of the parent entity.
        child: Occurrence path of the child entity.
        instance_proxy_map: Mapping of instance IDs to their proxy names (rigid subassembly roots).
        prefix: Prefix to add to the occurrence path.

    Returns:
        The joined occurrence path using proxy for rigid subassemblies.

    Examples:
        >>> join_mate_occurrences_with_proxy(["rigid_sub", "part1"], ["part2"], {"rigid_sub_JOINER_part1": "rigid_sub"})
        "rigid_sub-MATE-part2"
    """
    parent_occurrence = get_proxy_occurrence_name(parent, instance_proxy_map, prefix)
    child_occurrence = get_proxy_occurrence_name(child, instance_proxy_map, prefix)
    return f"{parent_occurrence}{MATE_JOINER}{child_occurrence}"


def build_hierarchical_transform_for_rigid_subassembly(
    occurrences_list: list[str],
    rigid_subassembly_occurrence_map: dict[str, dict[str, Occurrence]],
) -> np.matrix:
    """
    Build hierarchical transform chain for a part buried within a rigid subassembly.

    Args:
        occurrences_list: List like [rigid_sub_name, level1, level2, ..., part_name]
        rigid_subassembly_occurrence_map: Map of rigid subassembly occurrences

    Returns:
        Transform matrix from rigid subassembly root to the part's coordinate system
    """
    if len(occurrences_list) < 2:
        return np.eye(4)

    rigid_sub_name = occurrences_list[0]
    if rigid_sub_name not in rigid_subassembly_occurrence_map:
        LOGGER.warning(f"Rigid subassembly {rigid_sub_name} not found in occurrence map")
        return np.eye(4)

    parent_tf = np.eye(4)
    rigid_sub_occurrences = rigid_subassembly_occurrence_map[rigid_sub_name]

    # Traverse through each sub-assembly level (excluding the final part)
    for i in range(len(occurrences_list) - 1):
        # Build key relative to this rigid subassembly (exclude the rigid subassembly name itself)
        hierarchical_key = SUBASSEMBLY_JOINER.join(occurrences_list[1 : i + 2])
        _occurrence_data = rigid_sub_occurrences.get(hierarchical_key)
        if _occurrence_data is None:
            LOGGER.warning(f"No occurrence data found for hierarchical key: {hierarchical_key}")
            continue
        _occurrence_tf = np.matrix(_occurrence_data.transform).reshape(4, 4)
        parent_tf = parent_tf @ _occurrence_tf

    return parent_tf


async def build_rigid_subassembly_occurrence_map(
    rigid_subassemblies: dict[str, RootAssembly], id_to_name_map: dict[str, str], parts: dict[str, Part]
) -> dict[str, dict[str, Occurrence]]:
    """
    Asynchronously build a map of rigid subassembly occurrences.

    Args:
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        id_to_name_map: A dictionary mapping instance IDs to their sanitized names.
        parts: A dictionary mapping instance IDs to their corresponding parts.

    Returns:
        A dictionary mapping occurrence paths to their corresponding occurrences.
    """
    occurrence_map: dict[str, dict[str, Occurrence]] = {}
    for assembly_key, rigid_subassembly in rigid_subassemblies.items():
        sub_occurrences: dict[str, Occurrence] = {}
        for occurrence in rigid_subassembly.occurrences:
            try:
                occurrence_path = [id_to_name_map[path] for path in occurrence.path]
                sub_occurrences[SUBASSEMBLY_JOINER.join(occurrence_path)] = occurrence
            except KeyError:
                LOGGER.warning(f"Occurrence path {occurrence.path} not found")

        occurrence_map[assembly_key] = sub_occurrences

    return occurrence_map


async def process_features_async(
    features: list[AssemblyFeature],
    patterns: list[Pattern],
    parts: dict[str, Part],
    id_to_name_map: dict[str, str],
    rigid_subassembly_occurrence_map: dict[str, dict[str, Occurrence]],
    rigid_subassemblies: dict[str, RootAssembly],
    subassembly_prefix: Optional[str],
    occurrences: dict[str, Occurrence],
    instance_proxy_map: dict[str, str],
) -> tuple[dict[str, MateFeatureData], dict[str, MateRelationFeatureData]]:
    """
    Process assembly features asynchronously.

    Args:
        features: The assembly features to process.
        patterns: The assembly patterns to process.
        parts: A dictionary mapping instance IDs to their corresponding parts.
        id_to_name_map: A dictionary mapping instance IDs to their sanitized names.
        rigid_subassembly_occurrence_map: A dictionary mapping occurrence paths to their corresponding occurrences.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        subassembly_prefix: The prefix for the subassembly.
        occurrences: Dictionary mapping instance IDs to their occurrences with transforms.

    Args:
        features: The assembly features to process.
        patterns: The assembly patterns to process.
        parts: A dictionary mapping instance IDs to their corresponding parts.
        id_to_name_map: A dictionary mapping instance IDs to their sanitized names.
        rigid_subassembly_occurrence_map: A dictionary mapping occurrence paths to their corresponding occurrences.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        subassembly_prefix: The prefix for the subassembly.

    Returns:
        A tuple containing:
        - A dictionary mapping occurrence paths to their corresponding mates.
        - A dictionary mapping occurrence paths to their corresponding relations.
    """
    mates_map: dict[str, MateFeatureData] = {}
    relations_map: dict[str, MateRelationFeatureData] = {}
    seed_to_pattern_map: dict[str, list[str]] = {}
    seed_to_mate_map: dict[str, MateFeatureData] = {}
    seed_to_mate_occurrence_map: dict[str, int] = {}

    for pattern in patterns:
        if pattern.suppressed:
            continue
        for seed_id, pattern_instances in pattern.seedToPatternInstances.items():
            seed_to_pattern_map[seed_id] = pattern_instances

    for feature in features:
        feature.featureData.id = feature.id

        if feature.suppressed:
            continue

        if feature.featureType == AssemblyFeatureType.MATE:
            if len(feature.featureData.matedEntities) < 2:
                LOGGER.warning(f"Invalid mate feature: {feature}")
                continue

            child_occurrence = feature.featureData.matedEntities[CHILD].matedOccurrence[0]
            parent_occurrence = feature.featureData.matedEntities[PARENT].matedOccurrence[0]

            # Check if child or parent is in a pattern
            child_in_pattern = child_occurrence in seed_to_pattern_map
            parent_in_pattern = parent_occurrence in seed_to_pattern_map

            if child_in_pattern and parent_in_pattern:
                # Both entities in patterns is unrealistic in practice - log warning and skip
                LOGGER.warning(
                    f"Mate {feature.featureData.name} has both child and parent in patterns. "
                    f"This scenario is not supported. Skipping mate."
                )
                continue
            elif child_in_pattern:
                seed_to_mate_map[child_occurrence] = feature.featureData
                seed_to_mate_occurrence_map[child_occurrence] = CHILD
            elif parent_in_pattern:
                seed_to_mate_map[parent_occurrence] = feature.featureData
                seed_to_mate_occurrence_map[parent_occurrence] = PARENT

            try:
                child_occurrences = [
                    id_to_name_map[path] for path in feature.featureData.matedEntities[CHILD].matedOccurrence
                ]
                parent_occurrences = [
                    id_to_name_map[path] for path in feature.featureData.matedEntities[PARENT].matedOccurrence
                ]
            except KeyError as e:
                LOGGER.warning(e)
                LOGGER.warning(f"Key not found in {id_to_name_map.keys()}")
                continue

            # Handle rigid subassemblies
            if parent_occurrences[0] in rigid_subassemblies:
                # Build hierarchical transform chain from rigid subassembly root to the part
                parent_tf = build_hierarchical_transform_for_rigid_subassembly(
                    parent_occurrences, rigid_subassembly_occurrence_map
                )
                parent_parentCS = MatedCS.from_tf(parent_tf)
                parts[parent_occurrences[0]].rigidAssemblyToPartTF[parent_occurrences[1]] = parent_parentCS.part_tf
                feature.featureData.matedEntities[PARENT].parentCS = parent_parentCS

                parent_occurrences = [parent_occurrences[0]]

            if child_occurrences[0] in rigid_subassemblies:
                # Build hierarchical transform chain from rigid subassembly root to the part
                parent_tf = build_hierarchical_transform_for_rigid_subassembly(
                    child_occurrences, rigid_subassembly_occurrence_map
                )
                child_parentCS = MatedCS.from_tf(parent_tf)
                parts[child_occurrences[0]].rigidAssemblyToPartTF[child_occurrences[1]] = child_parentCS.part_tf
                feature.featureData.matedEntities[CHILD].parentCS = child_parentCS

                child_occurrences = [child_occurrences[0]]

            mates_map[
                join_mate_occurrences_with_proxy(
                    parent=parent_occurrences,
                    child=child_occurrences,
                    instance_proxy_map=instance_proxy_map,
                    prefix=subassembly_prefix,
                )
            ] = feature.featureData

        elif feature.featureType == AssemblyFeatureType.MATERELATION:
            if feature.featureData.relationType == RelationType.SCREW:
                child_joint_id = feature.featureData.mates[0].featureId
            else:
                child_joint_id = feature.featureData.mates[RELATION_CHILD].featureId

            relations_map[child_joint_id] = feature.featureData

    # Process patterns to expand mates for all pattern instances
    pattern_expanded_mates = await _process_assembly_patterns_async(
        patterns=patterns,
        features=features,
        parts=parts,
        seed_to_pattern_map=seed_to_pattern_map,
        seed_to_mate_map=seed_to_mate_map,
        seed_to_mate_occurrence_map=seed_to_mate_occurrence_map,
        id_to_name_map=id_to_name_map,
        rigid_subassembly_occurrence_map=rigid_subassembly_occurrence_map,
        rigid_subassemblies=rigid_subassemblies,
        subassembly_prefix=subassembly_prefix,
        occurrences=occurrences,
        instance_proxy_map=instance_proxy_map,
    )
    mates_map.update(pattern_expanded_mates)
    return mates_map, relations_map


async def _process_assembly_patterns_async(
    patterns: list[Pattern],
    features: list[AssemblyFeature],
    parts: dict[str, Part],
    seed_to_pattern_map: dict[str, list[str]],
    seed_to_mate_map: dict[str, MateFeatureData],
    seed_to_mate_occurrence_map: dict[str, int],
    id_to_name_map: dict[str, str],
    rigid_subassembly_occurrence_map: dict[str, dict[str, Occurrence]],
    rigid_subassemblies: dict[str, RootAssembly],
    subassembly_prefix: Optional[str],
    occurrences: dict[str, Occurrence],
    instance_proxy_map: dict[str, str],
) -> dict[str, MateFeatureData]:
    """
    Process assembly patterns to expand mates for all pattern instances.

    Args:
        patterns: The assembly patterns to process.
        features: The assembly features to process.
        parts: A dictionary mapping instance IDs to their corresponding parts.
        id_to_name_map: A dictionary mapping instance IDs to their sanitized names.
        rigid_subassembly_occurrence_map: A dictionary mapping occurrence paths to their corresponding occurrences.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        subassembly_prefix: The prefix for the subassembly.
        occurrences: Dictionary mapping instance IDs to their occurrences with transforms.

    Returns:
        A dictionary mapping occurrence paths to their corresponding pattern-expanded mates.
    """
    pattern_expanded_mates: dict[str, MateFeatureData] = {}

    if not patterns:
        return pattern_expanded_mates

    for occurrence_id in seed_to_mate_map:
        occurrence_type = seed_to_mate_occurrence_map[occurrence_id]

        # Handle case where one entity (child or parent) is in a pattern
        if occurrence_type in [CHILD, PARENT]:
            for index, pattern_instance_id in enumerate(seed_to_pattern_map[occurrence_id]):
                # Create a deep copy of the original mate to avoid mutation
                pattern_mate = copy.deepcopy(seed_to_mate_map[occurrence_id])
                other_entity_index = PARENT if occurrence_type == CHILD else CHILD

                # Get the original mate coordinate systems for other entity
                # this is the mate coordinate of the seed or original mate wrt to the other entity's coordinate system
                other_entity_cs = pattern_mate.matedEntities[other_entity_index].matedCS

                # Get occurrence transforms for the seed, pattern entity,
                # and other entity, these are in world coordinates
                seed_occurrence = find_occurrence_by_id(occurrence_id, occurrences)
                pattern_occurrence = find_occurrence_by_id(pattern_instance_id, occurrences)
                other_entity_id = pattern_mate.matedEntities[other_entity_index].matedOccurrence[0]
                other_occurrence = find_occurrence_by_id(other_entity_id, occurrences)

                if seed_occurrence is None:
                    LOGGER.warning(f"Could not find occurrence for seed entity: {occurrence_id}")
                    continue
                if pattern_occurrence is None:
                    LOGGER.warning(f"Could not find occurrence for pattern entity: {pattern_instance_id}")
                    continue
                if other_occurrence is None:
                    LOGGER.warning(f"Could not find occurrence for other entity: {other_entity_id}")
                    continue

                seed_entity_occurrence_tf = np.matrix(seed_occurrence.transform).reshape(4, 4)
                pattern_entity_occurrence_tf = np.matrix(pattern_occurrence.transform).reshape(4, 4)
                other_entity_occurrence_tf = np.matrix(other_occurrence.transform).reshape(4, 4)

                # Calculate the relative transform from seed to pattern position: M_rel = M_pattern * M_seed^(-1)
                seed_to_pattern_relative_tf = pattern_entity_occurrence_tf @ np.linalg.inv(seed_entity_occurrence_tf)

                # Clean up floating point precision issues in the relative transform
                tolerance = 1e-10
                seed_to_pattern_relative_tf[np.abs(seed_to_pattern_relative_tf) < tolerance] = 0.0

                # For the other entity (non-pattern), we need to find the new mate CS
                # The joint should remain relative to the other entity, but account for the pattern transformation
                # Transform: other_entity_local -> world -> apply_pattern_relative -> back_to_other_entity_local
                other_entity_mate_world_tf = other_entity_occurrence_tf @ other_entity_cs.part_to_mate_tf
                pattern_transformed_world_tf = seed_to_pattern_relative_tf @ other_entity_mate_world_tf
                other_entity_mate_tf = np.linalg.inv(other_entity_occurrence_tf) @ pattern_transformed_world_tf

                # Clean up floating point precision issues in the final transform
                other_entity_mate_tf[np.abs(other_entity_mate_tf) < tolerance] = 0.0
                other_entity_transformed_cs = MatedCS.from_tf(other_entity_mate_tf)

                # Update the other entity's mate CS
                pattern_mate.matedEntities[seed_to_mate_occurrence_map[occurrence_id]].matedOccurrence = [
                    pattern_instance_id
                ]

                pattern_mate.matedEntities[other_entity_index].matedCS = other_entity_transformed_cs
                pattern_mate.name = f"{seed_to_mate_map[occurrence_id].name}_{index + 1}"

                try:
                    child_occurrences = [
                        id_to_name_map[path] for path in pattern_mate.matedEntities[CHILD].matedOccurrence
                    ]
                    parent_occurrences = [
                        id_to_name_map[path] for path in pattern_mate.matedEntities[PARENT].matedOccurrence
                    ]
                except KeyError as e:
                    LOGGER.warning(e)
                    LOGGER.warning(f"Key not found in {id_to_name_map.keys()}")
                    continue

                # Handle rigid subassemblies for pattern instances
                if parent_occurrences[0] in rigid_subassemblies:
                    parent_tf = build_hierarchical_transform_for_rigid_subassembly(
                        parent_occurrences, rigid_subassembly_occurrence_map
                    )
                    parent_parentCS = MatedCS.from_tf(parent_tf)
                    parts[parent_occurrences[0]].rigidAssemblyToPartTF[parent_occurrences[1]] = parent_parentCS.part_tf
                    pattern_mate.matedEntities[PARENT].parentCS = parent_parentCS
                    parent_occurrences = [parent_occurrences[0]]

                if child_occurrences[0] in rigid_subassemblies:
                    parent_tf = build_hierarchical_transform_for_rigid_subassembly(
                        child_occurrences, rigid_subassembly_occurrence_map
                    )
                    child_parentCS = MatedCS.from_tf(parent_tf)
                    parts[child_occurrences[0]].rigidAssemblyToPartTF[child_occurrences[1]] = child_parentCS.part_tf
                    pattern_mate.matedEntities[CHILD].parentCS = child_parentCS
                    child_occurrences = [child_occurrences[0]]

                pattern_mate_key = join_mate_occurrences_with_proxy(
                    parent=parent_occurrences,
                    child=child_occurrences,
                    instance_proxy_map=instance_proxy_map,
                    prefix=subassembly_prefix,
                )
                pattern_expanded_mates[pattern_mate_key] = pattern_mate

    return pattern_expanded_mates


async def get_mates_and_relations_async(
    assembly: Assembly,
    instance_proxy_map: dict[str, str],
    occurrences: dict[str, Occurrence],
    subassemblies: dict[str, SubAssembly],
    rigid_subassemblies: dict[str, RootAssembly],
    id_to_name_map: dict[str, str],
    parts: dict[str, Part],
) -> tuple[dict[str, MateFeatureData], dict[str, MateRelationFeatureData]]:
    """
    Asynchronously get mates and relations.

    Args:
        assembly: The assembly object to traverse.
        subassemblies: A dictionary mapping instance IDs to their corresponding subassemblies.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        id_to_name_map: A dictionary mapping instance IDs to their sanitized names.
        parts: A dictionary mapping instance IDs to their corresponding parts.

    Returns:
        A tuple containing:
        - A dictionary mapping occurrence paths to their corresponding mates.
        - A dictionary mapping occurrence paths to their corresponding relations.
    """
    rigid_subassembly_occurrence_map = await build_rigid_subassembly_occurrence_map(
        rigid_subassemblies, id_to_name_map, parts
    )

    mates_map, relations_map = await process_features_async(
        assembly.rootAssembly.features,
        assembly.rootAssembly.patterns,
        parts,
        id_to_name_map,
        rigid_subassembly_occurrence_map,
        rigid_subassemblies,
        None,
        occurrences,
        instance_proxy_map,
    )

    for key, subassembly in subassemblies.items():
        sub_mates, sub_relations = await process_features_async(
            subassembly.features,
            subassembly.patterns,
            parts,
            id_to_name_map,
            rigid_subassembly_occurrence_map,
            rigid_subassemblies,
            key,
            occurrences,
            instance_proxy_map,
        )
        mates_map.update(sub_mates)
        relations_map.update(sub_relations)

    return mates_map, relations_map


def get_mates_and_relations(
    assembly: Assembly,
    instance_proxy_map: dict[str, str],
    occurrences: dict[str, Occurrence],
    subassemblies: dict[str, SubAssembly],
    rigid_subassemblies: dict[str, RootAssembly],
    id_to_name_map: dict[str, str],
    parts: dict[str, Part],
) -> tuple[dict[str, MateFeatureData], dict[str, MateRelationFeatureData]]:
    """
    Synchronous wrapper for `get_mates_and_relations_async`.

    Args:
        assembly: The assembly object to traverse.
        subassemblies: A dictionary mapping instance IDs to their corresponding subassemblies.
        rigid_subassemblies: Mapping of instance IDs to rigid subassemblies.
        id_to_name_map: A dictionary mapping instance IDs to their sanitized names.
        parts: A dictionary mapping instance IDs to their corresponding parts.

    Returns:
        A tuple containing:
        - A dictionary mapping occurrence paths to their corresponding mates.
        - A dictionary mapping occurrence paths to their corresponding relations.
    """
    return asyncio.run(
        get_mates_and_relations_async(
            assembly, instance_proxy_map, occurrences, subassemblies, rigid_subassemblies, id_to_name_map, parts
        )
    )
