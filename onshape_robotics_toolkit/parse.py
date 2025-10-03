"""
This module contains functions that provide a way to traverse the assembly structure, extract information about parts,
subassemblies, instances, and mates, and generate a hierarchical representation of the assembly.

"""

import asyncio
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Optional, Union

import numpy as np

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import (
    Assembly,
    AssemblyFeatureType,
    AssemblyInstance,
    MateFeatureData,
    Occurrence,
    Part,
    PartInstance,
    Pattern,
    SubAssembly,
)
from onshape_robotics_toolkit.models.document import WorkspaceType
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name

SUBASSEMBLY_JOINER = f"_{uuid.uuid4().hex[:4].upper()}_"
MATE_JOINER = f"_{uuid.uuid4().hex[:4].upper()}_"

LOGGER.debug(f"Generated joiners - SUBASSEMBLY: {SUBASSEMBLY_JOINER}, MATE: {MATE_JOINER}")

# Path and entity constants
ROOT_PATH_NAME = "root"
SUBASSEMBLY_PATH_PREFIX = "subassembly"

# Assembly classification constants
RIGID_ASSEMBLY_ONLY_FEATURE_TYPES = {AssemblyFeatureType.MATEGROUP}

# Entity relationship constants
CHILD = 0
PARENT = 1

RELATION_CHILD = 1
RELATION_PARENT = 0


# ============================================================================
# PATH-TUPLE BASED KEY SYSTEM (New Simplified Approach)
# ============================================================================


@dataclass(frozen=True, slots=True)
class PathKey:
    """
    Immutable path-based key using Onshape's natural ID hierarchy.

    Stores both ID path (for uniqueness/hashing) and name path (for readability).
    Both paths are built during CAD population and cached in the PathKey.

    Examples:
        # Root-level part instance
        PathKey(("MqRDHdbA0tAm2ygBR",), ("wheel_1",))

        # Nested part in subassembly
        PathKey(
            ("MoN/4FhyvQ92+I8TU", "MZHBlAU4IxmX6u6A0", "MrpOYQ6mQsyqwPVz0"),
            ("assembly_1", "subassembly_2", "part_3")
        )
    """

    _path: tuple[str, ...]
    _name_path: tuple[str, ...]

    def __init__(self, path: tuple[str, ...], name_path: tuple[str, ...]):
        """
        Create a PathKey from tuples of instance IDs and names.

        Args:
            path: Tuple of Onshape instance IDs representing hierarchical position
            name_path: Tuple of sanitized names parallel to path
        """
        if len(path) != len(name_path):
            raise ValueError(f"path and name_path must have same length: {len(path)} != {len(name_path)}")

        object.__setattr__(self, "_path", path)
        object.__setattr__(self, "_name_path", name_path)

    @property
    def path(self) -> tuple[str, ...]:
        """Get the immutable ID path tuple."""
        return self._path

    @property
    def name_path(self) -> tuple[str, ...]:
        """Get the immutable name path tuple."""
        return self._name_path

    @property
    def leaf(self) -> str:
        """Get the last ID in the path (the actual entity ID)."""
        return self._path[-1] if self._path else ""

    @property
    def name(self) -> str:
        """Get the last name in the path (human-readable leaf name)."""
        return self._name_path[-1] if self._name_path else ""

    @property
    def parent(self) -> Optional["PathKey"]:
        """Get parent PathKey by trimming last element."""
        if len(self._path) <= 1:
            return None
        return PathKey(self._path[:-1], self._name_path[:-1])

    @property
    def root(self) -> Optional[str]:
        """Get the root ID (first element in path)."""
        return self._path[0] if self._path else None

    @property
    def root_name(self) -> Optional[str]:
        """Get the root name (first element in name path)."""
        return self._name_path[0] if self._name_path else None

    @property
    def depth(self) -> int:
        """Get depth in hierarchy (0 = root level)."""
        return len(self._path)

    def hierarchical_name(self, separator: str = "-") -> str:
        """
        Get hierarchical name representation.

        Args:
            separator: String to join names (default "-")

        Returns:
            Hierarchical name like "assembly_1-subassembly_2-part_3"
        """
        return separator.join(self._name_path)

    def __repr__(self) -> str:
        return f"PathKey({self._path})"

    def __str__(self) -> str:
        """String representation showing the name path structure."""
        return " > ".join(self._name_path) if self._name_path else "(empty)"

    def __lt__(self, other: "PathKey") -> bool:
        """
        Less-than comparison for sorting PathKeys.

        Compares by depth first, then lexicographically by path elements.
        This ensures consistent ordering for visualization and debugging.

        Args:
            other: Another PathKey to compare with

        Returns:
            True if this PathKey should sort before other
        """
        if not isinstance(other, PathKey):
            return NotImplemented
        # Sort by depth first (shallower first), then by path
        return (self.depth, self._path) < (other.depth, other._path)

    def __le__(self, other: "PathKey") -> bool:
        """Less-than-or-equal comparison."""
        if not isinstance(other, PathKey):
            return NotImplemented
        return (self.depth, self._path) <= (other.depth, other._path)

    def __gt__(self, other: "PathKey") -> bool:
        """Greater-than comparison."""
        if not isinstance(other, PathKey):
            return NotImplemented
        return (self.depth, self._path) > (other.depth, other._path)

    def __ge__(self, other: "PathKey") -> bool:
        """Greater-than-or-equal comparison."""
        if not isinstance(other, PathKey):
            return NotImplemented
        return (self.depth, self._path) >= (other.depth, other._path)

    @classmethod
    def from_path(cls, path: Union[list[str], str], id_to_name: dict[str, str]) -> "PathKey":
        """
        Create PathKey from a path (list) or single instance ID (string).

        This handles both:
        - occurrence.path from JSON (list of IDs)
        - matedOccurrence from mate features (list of IDs)
        - single instance ID for root-level instances (string)

        Args:
            path: Either a list of instance IDs or a single instance ID string
            id_to_name: Mapping from instance ID to sanitized name

        Returns:
            PathKey with both ID and name paths

        Examples:
            # From occurrence JSON (list)
            occ_key = PathKey.from_path(occurrence.path, id_to_name)
            # PathKey(("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG"), ("asm_1", "part_2"))

            # From single instance ID (string)
            part_key = PathKey.from_path("MqRDHdbA0tAm2ygBR", id_to_name)
            # PathKey(("MqRDHdbA0tAm2ygBR",), ("wheel_1",))
        """
        # Single instance ID -> single-element tuple, otherwise convert list to tuple
        id_tuple = (path,) if isinstance(path, str) else tuple(path)

        # Build name tuple from IDs
        name_tuple = tuple(id_to_name.get(instance_id, instance_id) for instance_id in id_tuple)

        return cls(id_tuple, name_tuple)


# ============================================================================
# LEGACY REGISTRY CLASSES REMOVED
# ============================================================================
# The following classes have been removed in favor of flat dict storage in CAD:
# - InstanceRegistry -> CAD.instances (single dict for parts + assemblies)
# - OccurrenceRegistry -> CAD.occurrences (dict)
# - MateRegistry -> CAD.mates (dict)
# - PatternRegistry -> CAD.patterns (dict)
# - AssemblyData -> merged into CAD flat structure
# - RootAssemblyData -> merged into CAD flat structure
# ============================================================================


@dataclass
class CAD:
    """
    Streamlined CAD document with flat dict-based storage.

    All data is stored in simple dicts keyed by PathKey for O(1) lookup.
    No nested registries, no duplicate storage, single source of truth.

    This structure maps to Onshape's assembly JSON schema but flattens
    the hierarchy for easier access:
    {
      "parts": [...],                    -> parts
      "rootAssembly": {
        "instances": [...],              -> instances
        "occurrences": [...],            -> occurrences
        "features": [...],               -> mates (filtered for mate types)
        "patterns": [...]                -> patterns
      },
      "subAssemblies": [...]             -> data merged into root dicts
    }

    Attributes:
        document_id: Onshape document ID
        element_id: Onshape element (assembly) ID
        workspace_id: Onshape workspace ID
        document_microversion: Onshape document microversion
        max_depth: Maximum depth for flexible assemblies (0 = all rigid)
        current_depth: Current depth in hierarchy (0 = root)
        keys: Canonical PathKey index by ID (ID path tuple → PathKey)
        keys_by_name: Reverse PathKey index by name (name path tuple → PathKey)
        instances: All instances (parts AND assemblies) keyed by PathKey
        occurrences: All occurrence transforms keyed by PathKey
        mates: All mate relationships keyed by (parent_key, child_key)
        patterns: All pattern definitions keyed by pattern_id
        parts: Part definitions keyed by PathKey (includes rigid assemblies)
        fetched_subassemblies: Recursively fetched subassembly CAD documents
    """

    # Document metadata
    document_id: str
    element_id: str
    workspace_id: str
    document_microversion: str
    max_depth: int
    current_depth: int

    # Core data (flat dicts)
    keys: dict[tuple[str, ...], PathKey]  # ID path tuple -> PathKey (canonical index)
    keys_by_name: dict[tuple[str, ...], PathKey]  # Name path tuple -> PathKey (reverse index)
    instances: dict[PathKey, Union[PartInstance, AssemblyInstance]]
    occurrences: dict[PathKey, Occurrence]
    mates: dict[tuple[PathKey, PathKey], MateFeatureData]
    patterns: dict[str, Pattern]
    parts: dict[PathKey, Part]

    # Nested subassemblies
    fetched_subassemblies: dict[PathKey, "CAD"]

    def __init__(
        self,
        document_id: str,
        element_id: str,
        workspace_id: str,
        document_microversion: str,
        max_depth: int = 0,
        current_depth: int = 0,
    ):
        """
        Initialize an empty CAD document.

        Args:
            document_id: Onshape document ID
            element_id: Onshape element (assembly) ID
            workspace_id: Onshape workspace ID
            document_microversion: Onshape document microversion
            max_depth: Maximum depth for flexible assemblies
            current_depth: Current depth in hierarchy
        """
        self.document_id = document_id
        self.element_id = element_id
        self.workspace_id = workspace_id
        self.document_microversion = document_microversion
        self.max_depth = max_depth
        self.current_depth = current_depth

        # Initialize empty dicts
        self.keys = {}
        self.keys_by_name = {}
        self.instances = {}
        self.occurrences = {}
        self.mates = {}
        self.patterns = {}
        self.parts = {}
        self.fetched_subassemblies = {}

    # ============================================================================
    # HELPER METHODS
    # ============================================================================

    def get_path_key(self, path: Union[str, list[str], tuple[str, ...]]) -> Optional[PathKey]:
        """
        Get PathKey from an ID path.

        Args:
            path: Instance ID (string) or path (list/tuple of IDs)

        Returns:
            PathKey if found, None otherwise

        Examples:
            # From single ID
            key = cad.get_path_key("M123")

            # From path list
            key = cad.get_path_key(["M123", "M456"])

            # From path tuple
            key = cad.get_path_key(("M123", "M456"))
        """
        path_tuple = (path,) if isinstance(path, str) else tuple(path)
        return self.keys.get(path_tuple)

    def get_path_key_by_name(self, name_path: Union[str, list[str], tuple[str, ...]]) -> Optional[PathKey]:
        """
        Get PathKey from a name path (reverse lookup).

        Args:
            name_path: Instance name (string) or name path (list/tuple of names)

        Returns:
            PathKey if found, None otherwise

        Examples:
            # From single name
            key = cad.get_path_key_by_name("wheel_1")

            # From name path list
            key = cad.get_path_key_by_name(["Assembly_1", "Part_1"])

            # From name path tuple
            key = cad.get_path_key_by_name(("Assembly_1", "Part_1"))
        """
        name_tuple = (name_path,) if isinstance(name_path, str) else tuple(name_path)
        return self.keys_by_name.get(name_tuple)

    def is_rigid_assembly(self, key: PathKey) -> bool:
        """Check if instance is a rigid assembly."""
        instance = self.instances.get(key)
        return isinstance(instance, AssemblyInstance) and instance.isRigid

    def is_flexible_assembly(self, key: PathKey) -> bool:
        """Check if instance is a flexible assembly."""
        instance = self.instances.get(key)
        return isinstance(instance, AssemblyInstance) and not instance.isRigid

    def is_part(self, key: PathKey) -> bool:
        """Check if instance is a part."""
        return isinstance(self.instances.get(key), PartInstance)

    def get_transform(self, key: PathKey) -> Optional[np.ndarray]:
        """Get 4x4 transform matrix for occurrence."""
        occ = self.occurrences.get(key)
        return np.array(occ.transform).reshape(4, 4) if occ else None

    def get_pattern_for_seed(self, seed_id: str) -> Optional[Pattern]:
        """Get pattern containing a seed instance."""
        for pattern in self.patterns.values():
            if seed_id in pattern.seedToPatternInstances:
                return pattern
        return None

    def get_patterned_instances(self, seed_id: str) -> list[str]:
        """Get list of patterned instance IDs for a seed."""
        pattern = self.get_pattern_for_seed(seed_id)
        if pattern and seed_id in pattern.seedToPatternInstances:
            return pattern.seedToPatternInstances[seed_id]
        return []

    def lookup_by_name(self, name: str, depth: Optional[int] = None) -> list[PathKey]:
        """
        Find PathKeys by name.

        Args:
            name: Sanitized instance name
            depth: Optional depth filter

        Returns:
            List of matching PathKeys
        """
        matches = []
        for key in self.instances:
            if key.name == name and (depth is None or key.depth == depth):
                matches.append(key)
        return matches

    def __len__(self) -> int:
        """Total number of instances."""
        return len(self.instances)

    def __repr__(self) -> str:
        part_count = sum(1 for inst in self.instances.values() if isinstance(inst, PartInstance))
        asm_count = sum(1 for inst in self.instances.values() if isinstance(inst, AssemblyInstance))
        return (
            f"CAD("
            f"keys={len(self.keys)}, "
            f"instances={len(self.instances)} (parts={part_count}, asm={asm_count}), "
            f"occurrences={len(self.occurrences)}, "
            f"mates={len(self.mates)}, "
            f"patterns={len(self.patterns)})"
        )

    # ============================================================================
    # CONSTRUCTION METHODS
    # ============================================================================

    @classmethod
    def from_assembly(
        cls,
        assembly: Assembly,
        max_depth: int = 0,
        current_depth: int = 0,
    ) -> "CAD":
        """
        Create a CAD from an Onshape Assembly.

        Args:
            assembly: Onshape assembly data
            max_depth: Maximum depth for flexible assemblies
            current_depth: Current depth in hierarchy

        Returns:
            CAD with populated flat dicts
        """
        # Create CAD instance
        cad = cls(
            document_id=assembly.rootAssembly.documentId,
            element_id=assembly.rootAssembly.elementId,
            workspace_id=assembly.document.wid,
            document_microversion=assembly.rootAssembly.documentMicroversion,
            max_depth=max_depth,
            current_depth=current_depth,
        )

        # Build id_to_name mapping FIRST (needed for PathKey creation)
        id_to_name = cls._build_id_to_name_map(assembly)

        # Build ALL PathKeys upfront from occurrences (single source of truth)
        # Store them on the CAD instance for O(1) lookup throughout lifecycle
        cad.keys, cad.keys_by_name = cls._build_path_keys_from_occurrences(
            assembly.rootAssembly.occurrences, id_to_name
        )

        # Populate all data using the pre-built PathKeys
        cad._populate_from_assembly(assembly)

        # TODO: Handle subassemblies and parts
        # for subassembly in assembly.subAssemblies:
        #     cad._populate_subassembly(subassembly)
        # cad._populate_parts(assembly)

        LOGGER.info(
            f"Created CAD: {len([k for k in cad.instances.values() if isinstance(k, PartInstance)])} parts, "
            f"{len([k for k in cad.instances.values() if isinstance(k, AssemblyInstance)])} assemblies, "
            f"{len(cad.occurrences)} occurrences, {len(cad.mates)} mates"
        )

        return cad

    @staticmethod
    def _build_path_keys_from_occurrences(
        occurrences: list[Occurrence], id_to_name: dict[str, str]
    ) -> tuple[dict[tuple[str, ...], PathKey], dict[tuple[str, ...], PathKey]]:
        """
        Build all PathKeys upfront from occurrences.

        Occurrences are the single source of truth for paths in the assembly.
        This creates PathKeys once and builds two indexes for O(1) lookup:
        - By ID path: for internal lookups
        - By name path: for user-facing lookups

        Args:
            occurrences: List of occurrences from root assembly
            id_to_name: Mapping from instance ID to sanitized name

        Returns:
            Tuple of (keys_by_id, keys_by_name) dictionaries
        """
        LOGGER.debug("Building PathKeys from occurrences...")

        keys_by_id: dict[tuple[str, ...], PathKey] = {}
        keys_by_name: dict[tuple[str, ...], PathKey] = {}

        for occurrence in occurrences:
            # Create PathKey with both ID and name paths
            key = PathKey.from_path(occurrence.path, id_to_name)

            # Index by ID path tuple (immutable, hashable)
            keys_by_id[key.path] = key

            # Index by name path tuple for reverse lookup
            keys_by_name[key.name_path] = key

        LOGGER.debug(f"Built {len(keys_by_id)} PathKeys from {len(occurrences)} occurrences")
        return keys_by_id, keys_by_name

    @staticmethod
    def _build_id_to_name_map(assembly: Assembly) -> dict[str, str]:
        """
        Build mapping from instance ID to sanitized name.

        Processes root assembly and all subassemblies to build complete mapping.
        This must be called FIRST before creating any PathKeys.

        Args:
            assembly: Assembly from Onshape API

        Returns:
            Dictionary mapping instance IDs to sanitized names
        """
        LOGGER.debug("Building instance ID to name mapping...")

        id_to_name: dict[str, str] = {}

        # Root assembly instances
        for instance in assembly.rootAssembly.instances:
            sanitized = get_sanitized_name(instance.name)
            id_to_name[instance.id] = sanitized

        # Subassembly instances
        visited_uids: set[str] = set()
        subassembly_deque: deque[SubAssembly] = deque(assembly.subAssemblies)

        while subassembly_deque:
            subassembly = subassembly_deque.popleft()

            if subassembly.uid in visited_uids:
                continue
            visited_uids.add(subassembly.uid)

            for instance in subassembly.instances:
                sanitized = get_sanitized_name(instance.name)
                id_to_name[instance.id] = sanitized

        LOGGER.debug(f"Mapped {len(id_to_name)} instance IDs to names")
        return id_to_name

    def _populate_from_assembly(self, assembly: Assembly) -> None:
        """
        Populate all CAD data using pre-built PathKeys from self.keys.

        Args:
            assembly: Assembly from Onshape API
        """
        root_assembly = assembly.rootAssembly

        # Populate instances - lookup pre-built PathKey by instance ID
        for instance in root_assembly.instances:
            # Instance ID is a single element, so wrap in tuple
            path_tuple = (instance.id,)
            key = self.keys.get(path_tuple)
            if key:
                self.instances[key] = instance
            else:
                LOGGER.warning(f"No PathKey found for instance {instance.id} ({instance.name})")

        # Populate occurrences - use pre-built PathKeys directly
        for occurrence in root_assembly.occurrences:
            path_tuple = tuple(occurrence.path)
            key = self.keys.get(path_tuple)
            if key:
                self.occurrences[key] = occurrence
            else:
                LOGGER.warning(f"No PathKey found for occurrence path {occurrence.path}")

        # Populate mates - lookup pre-built PathKeys for parent and child
        for feature in root_assembly.features:
            if feature.featureType == AssemblyFeatureType.MATE and isinstance(feature.featureData, MateFeatureData):
                mate_data = feature.featureData
                if len(mate_data.matedEntities) >= 2:
                    parent_path = tuple(mate_data.matedEntities[PARENT].matedOccurrence)
                    child_path = tuple(mate_data.matedEntities[CHILD].matedOccurrence)

                    parent_key = self.keys.get(parent_path)
                    child_key = self.keys.get(child_path)

                    if parent_key and child_key:
                        self.mates[(parent_key, child_key)] = mate_data
                    else:
                        LOGGER.warning(f"Missing PathKey for mate: parent={parent_path}, child={child_path}")

        # Populate patterns (no PathKeys needed)
        for pattern in root_assembly.patterns:
            if not pattern.suppressed:
                self.patterns[pattern.id] = pattern

        LOGGER.debug(
            f"Populated assembly: {len(self.instances)} instances, "
            f"{len(self.occurrences)} occurrences, {len(self.mates)} mates, {len(self.patterns)} patterns"
        )


# ============================================================================
# HELPER FUNCTIONS FOR PathKey-based Robot Construction
# ============================================================================


def fetch_mass_properties_for_kinematic_parts(
    cad: CAD,
    kinematic_graph: any,
    client: Client,
    include_rigid_subassembly_parts: bool = False,
) -> None:
    """
    Fetch mass properties for parts involved in the kinematic graph.

    This function populates the MassProperty field for Part objects that are
    nodes in the kinematic graph. It uses async batch fetching for efficiency.

    Args:
        cad: CAD document with parts registry
        kinematic_graph: Kinematic graph containing parts that need mass properties
        client: Onshape client for API requests
        include_rigid_subassembly_parts: Whether to include rigid subassembly parts
    """

    # Get list of PathKeys that need mass properties (graph nodes)
    part_keys_needing_mass = list(kinematic_graph.graph.nodes)

    LOGGER.info(f"Fetching mass properties for {len(part_keys_needing_mass)} parts in kinematic chain")

    # Batch fetch mass properties
    asyncio.run(
        _fetch_mass_properties_for_pathkeys_async(
            cad=cad,
            part_keys=part_keys_needing_mass,
            client=client,
            include_rigid_subassembly_parts=include_rigid_subassembly_parts,
        )
    )


async def _fetch_mass_properties_for_pathkeys_async(
    cad: CAD,
    part_keys: list[PathKey],
    client: Client,
    include_rigid_subassembly_parts: bool,
) -> None:
    """
    Async helper to fetch mass properties for specific PathKeys.

    Args:
        cad: CAD document with parts registry
        part_keys: List of PathKeys for parts that need mass properties
        client: Onshape client for API requests
        include_rigid_subassembly_parts: Whether to include rigid subassembly parts
    """
    tasks = []

    for key in part_keys:
        # Get part from registry
        part = cad.parts.get(key)
        if part is None:
            LOGGER.warning(f"Part {key} not found in parts registry")
            continue

        # Skip if already has mass properties
        if part.MassProperty is not None:
            LOGGER.debug(f"Part {key} already has mass properties, skipping")
            continue

        # Check if this is a rigid assembly
        if part.isRigidAssembly and not include_rigid_subassembly_parts:
            LOGGER.debug(f"Skipping rigid assembly {key} (include_rigid_subassembly_parts=False)")
            continue

        # Create async task to fetch mass property
        LOGGER.debug(f"Fetching mass properties for {key}: {part.partId}")
        task = asyncio.to_thread(
            client.get_mass_property,
            did=part.documentId,
            wtype=WorkspaceType.M.value,
            wid=part.documentMicroversion if part.documentMicroversion else cad.document_microversion,
            eid=part.elementId,
            partID=part.partId,
        )
        tasks.append((key, task))

    # Execute all tasks concurrently
    if tasks:
        LOGGER.info(f"Fetching mass properties for {len(tasks)} parts concurrently...")
        results = await asyncio.gather(*[task for _, task in tasks], return_exceptions=True)

        # Update parts with fetched mass properties
        for (key, _), result in zip(tasks, results):
            if isinstance(result, Exception):
                LOGGER.error(f"Failed to fetch mass properties for {key}: {result}")
                continue

            # Update the part in the registry
            part = cad.parts[key]
            part.MassProperty = result
            LOGGER.debug(f"Updated mass properties for {key}")

    LOGGER.info("Mass properties fetching complete")
