"""
This module contains functions that provide a way to traverse the assembly structure, extract information about parts,
subassemblies, instances, and mates, and generate a hierarchical representation of the assembly.

"""

import asyncio
import uuid
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Union, cast

if TYPE_CHECKING:
    from onshape_robotics_toolkit.graph import KinematicGraph

import numpy as np

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER
from onshape_robotics_toolkit.models.assembly import (
    Assembly,
    AssemblyFeatureType,
    AssemblyInstance,
    MatedCS,
    MateFeatureData,
    Occurrence,
    Part,
    PartInstance,
    Pattern,
    SubAssembly,
)
from onshape_robotics_toolkit.models.document import WorkspaceType
from onshape_robotics_toolkit.models.mass import MassProperties
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
        return len(self._path) - 1

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


@dataclass
class CAD:
    """
    Streamlined CAD document with flat dict-based storage.

    All data is stored in simple dicts keyed by PathKey for O(1) lookup.
    No nested registries, no duplicate storage, single source of truth.

    This structure maps to Onshape's assembly JSON schema but flattens
    the hierarchy for easier access:
    {
      "parts": [...],                    -> parts (populated eagerly, mass props None)
      "rootAssembly": {
        "instances": [...],              -> instances (flattened)
        "occurrences": [...],            -> occurrences (flattened)
        "features": [...],               -> mates (with assembly provenance)
        "patterns": [...]                -> patterns (flattened)
      },
      "subAssemblies": [...]             -> data merged into root dicts
    }

    Attributes:
        document_id: Onshape document ID
        element_id: Onshape element (assembly) ID
        workspace_id: Onshape workspace ID
        document_microversion: Onshape document microversion
        max_depth: Maximum depth for flexible assemblies (0 = all rigid)
        keys: Canonical PathKey index by ID (ID path tuple → PathKey)
        keys_by_name: Reverse PathKey index by name (name path tuple → PathKey)
        instances: All instances (parts AND assemblies) from root + subassemblies
        occurrences: All occurrence transforms from root + subassemblies
        mates: All mate relationships with assembly provenance
               Key = (assembly_key, parent_key, child_key)
               - assembly_key: None for root, PathKey for subassembly
               - parent_key, child_key: absolute PathKeys
        patterns: All pattern definitions from root + subassemblies
        parts: Part definitions from assembly.parts (mass properties None until fetched)
        fetched_subassemblies: Recursively fetched subassembly CAD documents
    """

    # Document metadata
    document_id: str
    element_id: str
    wtype: str
    workspace_id: str
    document_microversion: str
    name: str
    max_depth: int

    # Core data (flat dicts with absolute PathKeys)
    keys: dict[tuple[str, ...], PathKey]  # ID path tuple -> PathKey (canonical index)
    keys_by_name: dict[tuple[str, ...], PathKey]  # Name path tuple -> PathKey (reverse index)
    instances: dict[PathKey, Union[PartInstance, AssemblyInstance]]
    occurrences: dict[PathKey, Occurrence]
    subassemblies: dict[PathKey, SubAssembly]
    mates: dict[tuple[Optional[PathKey], PathKey, PathKey], MateFeatureData]  # (assembly, parent, child)
    patterns: dict[str, Pattern]
    parts: dict[PathKey, Part]  # Populated eagerly from assembly.parts

    rigid_part_transforms: dict[PathKey, MatedCS]  # Cache of rigid part transforms

    def __init__(
        self,
        document_id: str,
        element_id: str,
        wtype: str,
        workspace_id: str,
        document_microversion: str,
        name: str,
        max_depth: int = 0,
    ):
        """
        Initialize an empty CAD document.

        Args:
            document_id: Onshape document ID
            element_id: Onshape element (assembly) ID
            workspace_id: Onshape workspace ID
            document_microversion: Onshape document microversion
            max_depth: Maximum depth for flexible assemblies
        """
        self.document_id = document_id
        self.element_id = element_id
        self.wtype = wtype
        self.workspace_id = workspace_id
        self.document_microversion = document_microversion
        self.name = name
        self.max_depth = max_depth

        # Initialize empty dicts
        self.keys_by_id = {}
        self.keys_by_name = {}
        self.instances = {}
        self.occurrences = {}
        self.mates = {}
        self.patterns = {}
        self.parts = {}
        self.subassemblies = {}

        self.rigid_part_transforms = {}

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
        return self.keys_by_id.get(path_tuple)

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
        # TODO: this should come out of subassemblies dict
        instance = self.instances.get(key)
        return isinstance(instance, AssemblyInstance) and instance.isRigid

    def is_flexible_assembly(self, key: PathKey) -> bool:
        """Check if instance is a flexible assembly."""
        # TODO: this should come out of subassemblies dict
        instance = self.instances.get(key)
        return isinstance(instance, AssemblyInstance) and not instance.isRigid

    def is_part(self, key: PathKey) -> bool:
        """Check if instance is a part."""
        # TODO: this should come out of parts dict w/ regard to rigid subassemblies
        return isinstance(self.instances.get(key), PartInstance)

    def get_transform(self, key: PathKey, wrt: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """Get 4x4 transform matrix for occurrence."""
        occ = self.occurrences.get(key)

        if occ:
            if wrt is not None:
                return occ.tf_wrt(wrt)
            else:
                return occ.tf
        return None

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

    def get_rigid_assembly_root(self, key: PathKey) -> Optional[PathKey]:
        """
        Find the top-most rigid assembly root for a given PathKey.

        Walks up the hierarchy to find the highest-level rigid assembly.
        This ensures that if an assembly is inside another rigid assembly,
        we return the outermost one.

        If the key itself is a rigid assembly, checks if it's inside another rigid assembly.
        If the key is inside a rigid assembly, returns the top-most rigid assembly's PathKey.
        If the key is not inside any rigid assembly, returns None.

        Args:
            key: PathKey to find rigid assembly root for

        Returns:
            PathKey of top-most rigid assembly root, or None if not inside rigid assembly

        Examples:
            >>> # Part at depth 2 inside rigid assembly at depth 1
            >>> key = PathKey(("asm1", "sub1", "part1"), ("Assembly_1", "Sub_1", "Part_1"))
            >>> rigid_root = cad.get_rigid_assembly_root(key)
            >>> # Returns PathKey(("asm1", "sub1"), ("Assembly_1", "Sub_1"))
        """
        # Walk up the hierarchy from the key to find ALL rigid assemblies
        # Return the top-most one (closest to root)
        rigid_root: Optional[PathKey] = None
        current: Optional[PathKey] = key

        while current is not None:
            instance = self.instances.get(current)
            if isinstance(instance, AssemblyInstance) and instance.isRigid:
                rigid_root = current  # Keep updating to get the top-most
            current = current.parent

        return rigid_root

    def compute_relative_mate_transform(
        self, part_key: PathKey, rigid_root_key: PathKey, part_to_mate_tf: np.ndarray
    ) -> np.ndarray:
        """
        Compute mate transform relative to rigid assembly root.

        Given a part buried inside a rigid assembly and its part_to_mate transform,
        this computes the equivalent transform from the rigid assembly root to the mate.

        Math:
            T_rigid_mate = inv(T_world_rigid) @ T_world_part @ T_part_mate

        Args:
            part_key: PathKey of the buried part
            rigid_root_key: PathKey of the rigid assembly root
            part_to_mate_tf: Original part_to_mate transform (4x4 matrix)

        Returns:
            4x4 transform matrix from rigid root to mate coordinate system

        Examples:
            >>> rigid_key = PathKey(("asm1", "sub1"), ("Assembly_1", "Sub_1"))
            >>> part_key = PathKey(("asm1", "sub1", "part1"), ("Assembly_1", "Sub_1", "Part_1"))
            >>> original_tf = mate.matedEntities[PARENT].matedCS.part_to_mate_tf
            >>> new_tf = cad.compute_relative_mate_transform(part_key, rigid_key, original_tf)
        """
        # Get global transforms
        rigid_tf = self.get_transform(rigid_root_key)
        part_tf = self.get_transform(part_key)

        if rigid_tf is None or part_tf is None:
            raise ValueError(f"Missing transforms: rigid_tf={rigid_tf is not None}, part_tf={part_tf is not None}")

        # Compute relative transform: T_rigid_mate = inv(T_world_rigid) @ T_world_part @ T_part_mate
        rigid_to_mate_tf: np.ndarray = np.linalg.inv(rigid_tf) @ part_tf @ part_to_mate_tf

        return rigid_to_mate_tf

    # ============================================================================
    # MATE QUERY METHODS
    # ============================================================================

    def get_mates_from_root(self) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Get only root-level mates (no assembly provenance).

        Returns:
            Dictionary with (parent, child) keys
        """
        return {(p, c): mate for (asm, p, c), mate in self.mates.items() if asm is None}

    def get_mates_from_subassembly(self, sub_key: PathKey) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Get mates from specific subassembly.

        Args:
            sub_key: PathKey of the subassembly

        Returns:
            Dictionary with (parent, child) keys
        """
        return {(p, c): mate for (asm, p, c), mate in self.mates.items() if asm == sub_key}

    def get_all_mates_flattened(self) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Get all mates without assembly provenance (backward compatible).

        If there are duplicate (parent, child) pairs from different assemblies,
        this will only keep one (last one wins).

        Returns:
            Dictionary with (parent, child) keys
        """
        return {(p, c): mate for (asm, p, c), mate in self.mates.items()}

    def get_mate_data(
        self, parent: PathKey, child: PathKey, assembly: Optional[PathKey] = None
    ) -> Optional[MateFeatureData]:
        """
        Get mate data for specific parent-child pair.

        Args:
            parent: Parent PathKey
            child: Child PathKey
            assembly: Assembly PathKey (None for root, PathKey for subassembly)
                     If None, searches all assemblies (root first)

        Returns:
            MateFeatureData if found, None otherwise
        """
        if assembly is not None:
            # Specific assembly lookup
            return self.mates.get((assembly, parent, child))
        else:
            # Search across all assemblies (try root first)
            mate = self.mates.get((None, parent, child))
            if mate:
                return mate

            # Search subassemblies
            for (_asm, p, c), mate in self.mates.items():
                if p == parent and c == child:
                    return mate
            return None

    def get_mate_assembly(self, parent: PathKey, child: PathKey) -> Optional[Optional[PathKey]]:
        """
        Find which assembly contains this mate.

        Args:
            parent: Parent PathKey
            child: Child PathKey

        Returns:
            None if root assembly, PathKey if subassembly, None if not found
        """
        for asm, p, c in self.mates:
            if p == parent and c == child:
                return asm  # Returns None for root, PathKey for subassembly
        return None

    def __len__(self) -> int:
        """Total number of instances."""
        return len(self.instances)

    def __repr__(self) -> str:
        part_count = sum(1 for inst in self.instances.values() if isinstance(inst, PartInstance))
        asm_count = sum(1 for inst in self.instances.values() if isinstance(inst, AssemblyInstance))
        return (
            f"CAD("
            f"keys={len(self.keys_by_id)}, "
            f"instances={len(self.instances)} (parts={part_count}, asm={asm_count}), "
            f"occurrences={len(self.occurrences)}, "
            f"subassemblies={(len(self.subassemblies))}, "
            f"mates={len(self.mates)}, "
            f"patterns={len(self.patterns)}, "
            f"parts={len(self.parts)})"
        )

    # ============================================================================
    # CONSTRUCTION METHODS
    # ============================================================================

    @classmethod
    def from_assembly(
        cls,
        assembly: Assembly,
        max_depth: int = 0,
    ) -> "CAD":
        """
        Create a CAD from an Onshape Assembly.

        Args:
            assembly: Onshape assembly data
            max_depth: Maximum depth for flexible assemblies

        Returns:
            CAD with populated flat dicts
        """
        # Create CAD instance
        if assembly.document is None:
            raise ValueError("Assembly document is None")

        cad = cls(
            document_id=assembly.rootAssembly.documentId,
            element_id=assembly.rootAssembly.elementId,
            wtype=assembly.document.wtype,
            workspace_id=assembly.document.wid,
            document_microversion=assembly.rootAssembly.documentMicroversion,
            name=assembly.document.name,  # TODO: this is different from assembly.name
            max_depth=max_depth,
        )

        # Build id_to_name mapping FIRST (needed for PathKey creation)
        id_to_name = cls._build_id_to_name_map(assembly)

        # Build ALL PathKeys upfront from occurrences (single source of truth)
        # Store them on the CAD instance for O(1) lookup throughout lifecycle
        cad.keys_by_id, cad.keys_by_name = cls._build_path_keys_from_occurrences(
            assembly.rootAssembly.occurrences, id_to_name
        )

        # Populate all data using the pre-built PathKeys
        # This includes marking rigid assemblies BEFORE populating mates
        cad._populate_from_assembly(assembly)

        LOGGER.info(f"Created {cad}")

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
        Populate all CAD data using pre-built PathKeys from self.keys_by_id.

        Flattens data from root assembly and all subassemblies into flat registries.

        Args:
            assembly: Assembly from Onshape API
        """
        # Step 1: Populate instances (root + subassemblies)
        self._populate_instances(assembly)

        # Step 2: Populate occurrences (root + subassemblies)
        self._populate_occurrences(assembly)

        # Step 3: Populate subassemblies and check/mark if they are rigid
        self._populate_subassemblies(assembly)

        # Step 4: Populate patterns (root + subassemblies)
        self._populate_patterns(assembly)

        # Step 6: Populate parts from assembly.parts (mass properties still None)
        self._populate_parts(assembly)

        # Step 5: Populate mates with assembly provenance
        # Mates will be remapped if they reference parts inside rigid assemblies
        self._populate_mates(assembly)

        LOGGER.debug(
            f"Populated CAD: {len(self.instances)} instances, "
            f"{len(self.occurrences)} occurrences, {len(self.mates)} mates, "
            f"{len(self.patterns)} patterns, {len(self.parts)} parts"
        )

    def _populate_instances(self, assembly: Assembly) -> None:
        """
        Populate instances from root assembly and all subassemblies.

        This creates a flat registry of ALL instances (root + nested) mapped by absolute PathKeys.

        Args:
            assembly: Assembly from Onshape API
        """
        subassembly_lookup = {subassembly.uid: subassembly for subassembly in assembly.subAssemblies}

        def _populate_branch(
            branch_instances: list[Union[PartInstance, AssemblyInstance]],
            parent_key: Optional[PathKey],
        ) -> None:
            for instance in branch_instances:
                path_tuple = (instance.id,) if parent_key is None else (*parent_key.path, instance.id)
                key = self.keys_by_id.get(path_tuple)
                if not key:
                    scope = "root" if parent_key is None else "nested"
                    LOGGER.warning(f"No PathKey for {scope} instance {instance.id} ({instance.name})")
                    continue

                self.instances[key] = instance

                if isinstance(instance, AssemblyInstance):
                    subassembly = subassembly_lookup.get(instance.uid)
                    if not subassembly:
                        LOGGER.warning(
                            f"Missing SubAssembly definition for instance {instance.id} "
                            f"({instance.name}) with uid {instance.uid}"
                        )
                        continue
                    _populate_branch(subassembly.instances, key)

        _populate_branch(assembly.rootAssembly.instances, None)
        LOGGER.info(f"Populated {len(self.instances)} instances (including nested)")

    def _populate_occurrences(self, assembly: Assembly) -> None:
        """
        Populate occurrences from root assembly (includes all nested occurrences).

        Args:
            assembly: Assembly from Onshape API
        """
        for occurrence in assembly.rootAssembly.occurrences:
            path_tuple = tuple(occurrence.path)
            key = self.keys_by_id.get(path_tuple)
            if key:
                self.occurrences[key] = occurrence
            else:
                LOGGER.warning(f"No PathKey for occurrence {occurrence.path}")

        LOGGER.info(f"Populated {len(self.occurrences)} occurrences")

    def _populate_subassemblies(self, assembly: Assembly) -> None:
        """
        Populate subassemblies from the assembly JSON

        Args:
            assembly: Assembly from Onshape API
        """
        # Build a multimap from subassembly UID -> list[PathKey] (one per occurrence)
        uid_to_pathkeys: dict[str, list[PathKey]] = {}
        for key, inst in self.instances.items():
            if isinstance(inst, AssemblyInstance):
                uid_to_pathkeys.setdefault(inst.uid, []).append(key)

        total_defs = len(assembly.subAssemblies)
        total_occurrences = 0
        rigid_marked = 0

        for subassembly in assembly.subAssemblies:
            pathkeys = uid_to_pathkeys.get(subassembly.uid)
            if not pathkeys:
                LOGGER.warning(
                    "SubAssembly definition uid=%s has no matching AssemblyInstance occurrences", subassembly.uid
                )
                continue

            # One JSON definition can correspond to multiple occurrences (patterned / reused)
            for pathkey in pathkeys:
                # Mark rigidity for this occurrence if depth logic requires it.
                if pathkey.depth >= self.max_depth:
                    subassembly.isRigid = True
                    inst = self.instances.get(pathkey)
                    if isinstance(inst, AssemblyInstance):
                        inst.isRigid = True
                        rigid_marked += 1

                self.subassemblies[pathkey] = subassembly
                total_occurrences += 1

        LOGGER.info(
            "Populated %d subassembly occurrences from %d definitions (rigid=%d, max_depth=%d)",
            total_occurrences,
            total_defs,
            rigid_marked,
            self.max_depth,
        )

    def _populate_patterns(self, assembly: Assembly) -> None:
        """
        Populate patterns from root assembly and all subassemblies.

        Args:
            assembly: Assembly from Onshape API
        """
        # Root patterns
        for pattern in assembly.rootAssembly.patterns:
            if not pattern.suppressed:
                self.patterns[pattern.id] = pattern

        # Subassembly patterns (only from flexible subassemblies)
        for subassembly in self.subassemblies.values():
            if subassembly.isRigid:
                continue
            for pattern in subassembly.patterns:
                if not pattern.suppressed:
                    self.patterns[pattern.id] = pattern

        LOGGER.debug(f"Populated {len(self.patterns)} patterns")

    def _remap_mate_for_rigid_assemblies(
        self, parent_key: PathKey, child_key: PathKey, mate_data: MateFeatureData
    ) -> tuple[PathKey, PathKey, MateFeatureData]:
        """
        Remap mate if either entity is inside a rigid assembly.

        If a mated entity is buried inside a rigid assembly:
        1. Find the rigid assembly root
        2. Update PathKey to point to rigid assembly
        3. Update matedCS transform to be relative to rigid assembly
        4. Update matedOccurrence to match new PathKey

        Args:
            parent_key: Original parent PathKey
            child_key: Original child PathKey
            mate_data: Original mate data

        Returns:
            Tuple of (new_parent_key, new_child_key, updated_mate_data)
        """
        # Make a copy to avoid mutating the original
        import copy

        mate_data = copy.deepcopy(mate_data)

        LOGGER.debug(f"Checking mate '{mate_data.name}' for remapping: {parent_key} <-> {child_key}")

        # Check parent
        parent_rigid_root = self.get_rigid_assembly_root(parent_key)
        if parent_rigid_root and parent_rigid_root != parent_key:
            # Parent is inside a rigid assembly, remap it
            LOGGER.debug(f"Remapping parent from {parent_key} to rigid root {parent_rigid_root}")

            # Get original part_to_mate transform, rigid assembly to part transform, and mate's part_to_mate transform
            og_part_tf = np.array(self.occurrences.get(parent_key).tf).reshape(4, 4)
            og_part_to_mate_tf = np.array(mate_data.matedEntities[PARENT].matedCS.part_to_mate_tf).reshape(4, 4)
            rigid_sub_to_part_tf = np.array(self.parts.get(parent_key).rigidAssemblyToPartTF.part_to_mate_tf).reshape(
                4, 4
            )

            # Compute new transform relative to rigid assembly
            new_tf = rigid_sub_to_part_tf @ og_part_tf @ og_part_to_mate_tf
            # Update mate data with new MatedCS from transform
            mate_data.matedEntities[PARENT].matedCS = MatedCS.from_tf(new_tf)
            mate_data.matedEntities[PARENT].matedOccurrence = list(parent_rigid_root.path)

            # Update PathKey
            parent_key = parent_rigid_root

        # Check child
        child_rigid_root = self.get_rigid_assembly_root(child_key)
        if child_rigid_root and child_rigid_root != child_key:
            # Child is inside a rigid assembly, remap it
            LOGGER.debug(f"Remapping child from {child_key} to rigid root {child_rigid_root}")

            # Get original part_to_mate transform, rigid assembly to part transform, and mate's part_to_mate transform
            og_part_tf = np.array(self.occurrences.get(child_key).tf).reshape(4, 4)
            og_part_to_mate_tf = np.array(mate_data.matedEntities[CHILD].matedCS.part_to_mate_tf).reshape(4, 4)
            rigid_sub_to_part_tf = np.array(self.parts.get(child_key).rigidAssemblyToPartTF.part_to_mate_tf).reshape(
                4, 4
            )

            # Compute new transform relative to rigid assembly
            new_tf = rigid_sub_to_part_tf @ og_part_tf @ og_part_to_mate_tf

            # Update mate data with new MatedCS from transform
            mate_data.matedEntities[CHILD].matedCS = MatedCS.from_tf(new_tf)
            mate_data.matedEntities[CHILD].matedOccurrence = list(child_rigid_root.path)

            # Update PathKey
            child_key = child_rigid_root

        return parent_key, child_key, mate_data

    def _populate_mates(self, assembly: Assembly) -> None:
        """
        Populate mates from root and all subassemblies with assembly provenance.

        Root mates: Key = (None, parent, child)
        Subassembly mates: Key = (sub_key, parent, child)

        For mates referencing parts inside rigid assemblies:
        - Remap PathKey to rigid assembly root
        - Update matedCS transform to be relative to rigid assembly

        Args:
            assembly: Assembly from Onshape API
        """
        # Process root assembly mates
        for feature in assembly.rootAssembly.features:
            if feature.featureType == AssemblyFeatureType.MATE and isinstance(feature.featureData, MateFeatureData):
                mate_data = feature.featureData
                if len(mate_data.matedEntities) >= 2:
                    parent_path = tuple(mate_data.matedEntities[PARENT].matedOccurrence)
                    child_path = tuple(mate_data.matedEntities[CHILD].matedOccurrence)

                    parent_key = self.keys_by_id.get(parent_path)
                    child_key = self.keys_by_id.get(child_path)

                    if parent_key and child_key:
                        # Remap if inside rigid assembly
                        parent_key, child_key, mate_data = self._remap_mate_for_rigid_assemblies(
                            parent_key, child_key, mate_data
                        )

                        # Filter out internal mates (both entities in same rigid assembly)
                        if parent_key == child_key:
                            LOGGER.debug(
                                f"Filtering internal mate '{mate_data.name}' within rigid assembly {parent_key}"
                            )
                            continue

                        # Store with None as assembly key (root)
                        self.mates[(None, parent_key, child_key)] = mate_data
                    else:
                        LOGGER.warning("Missing PathKey for root mate")

        # Process subassembly mates
        for subassembly in assembly.subAssemblies:
            # Find the AssemblyInstance(s) that correspond to this subassembly
            # Look in the flat instances registry (includes all nested instances)
            matching_keys = [
                key
                for key, inst in self.instances.items()
                if isinstance(inst, AssemblyInstance) and inst.uid == subassembly.uid
            ]

            if not matching_keys:
                LOGGER.warning(f"Could not find instance for subassembly {subassembly.uid}")
                continue

            # Process mates for each instance of this subassembly
            for sub_key in matching_keys:
                # Skip mates from rigid subassemblies (depth > max_depth)
                # Rigid subassemblies are treated as single bodies, so their internal mates are not relevant
                if self.max_depth == 0 or sub_key.depth > self.max_depth:
                    LOGGER.debug(
                        f"Skipping mates from rigid subassembly {sub_key} at depth "
                        f"{sub_key.depth} (max_depth={self.max_depth})"
                    )
                    continue

                # Process mates from this subassembly
                for feature in subassembly.features:
                    if feature.featureType == AssemblyFeatureType.MATE and isinstance(
                        feature.featureData, MateFeatureData
                    ):
                        mate_data = feature.featureData
                        if len(mate_data.matedEntities) >= 2:
                            # Relative paths from subassembly JSON
                            parent_rel_path = tuple(mate_data.matedEntities[PARENT].matedOccurrence)
                            child_rel_path = tuple(mate_data.matedEntities[CHILD].matedOccurrence)

                            # Convert to absolute by prepending subassembly path
                            parent_abs_path = sub_key.path + parent_rel_path
                            child_abs_path = sub_key.path + child_rel_path

                            parent_key = self.keys_by_id.get(parent_abs_path)
                            child_key = self.keys_by_id.get(child_abs_path)

                            if parent_key and child_key:
                                # Remap if inside rigid assembly
                                parent_key, child_key, mate_data = self._remap_mate_for_rigid_assemblies(
                                    parent_key, child_key, mate_data
                                )

                                # Filter out internal mates (both entities in same rigid assembly)
                                if parent_key == child_key:
                                    LOGGER.debug(
                                        f"Filtering internal mate '{mate_data.name}' within rigid assembly {parent_key}"
                                    )
                                    continue

                                # Store with subassembly key
                                self.mates[(sub_key, parent_key, child_key)] = mate_data
                            else:
                                LOGGER.warning(f"Missing PathKey for subassembly {sub_key} mate")

        LOGGER.debug(f"Populated {len(self.mates)} mates")

    def _populate_parts(self, assembly: Assembly) -> None:
        """
        Populate parts from assembly.parts list.

        Maps Part objects to PathKeys by matching instance UIDs.
        Creates synthetic Part objects for rigid assemblies.
        Mass properties remain None (populated later via API calls).

        Args:
            assembly: Assembly from Onshape API
        """
        # Build Part lookup by UID
        # Build a multimap from part UID -> list[PathKey] (one per occurrence)
        uid_to_pathkeys: dict[str, list[PathKey]] = {}
        for key, inst in self.instances.items():
            if isinstance(inst, PartInstance):
                uid_to_pathkeys.setdefault(inst.uid, []).append(key)

        for part in assembly.parts:
            pathkeys = uid_to_pathkeys.get(part.uid)
            if not pathkeys:
                LOGGER.warning("Part definition uid=%s has no matching PartInstance", part.uid)
                continue

            for pathkey in pathkeys:
                # TODO: find if this part is within a rigid assembly
                # and set its rigidAssemblyWorkspaceId and rigidAssemblyToPartTF
                # For now, just store the Part as-is
                self.parts[pathkey] = part

        # Create synthetic Part objects for rigid assemblies
        for key, subassembly in self.subassemblies.items():
            if subassembly.isRigid:
                subassembly_instance = self.instances.get(key)
                # Rigid assembly: create synthetic Part object
                self.parts[key] = Part(
                    isStandardContent=False,
                    partId=subassembly_instance.elementId,  # Use element ID as part ID
                    bodyType="assembly",
                    documentId=subassembly_instance.documentId,
                    elementId=subassembly_instance.elementId,
                    documentMicroversion=subassembly_instance.documentMicroversion,
                    configuration=subassembly_instance.configuration,
                    fullConfiguration=subassembly_instance.fullConfiguration,
                    documentVersion=subassembly_instance.documentVersion,
                    isRigidAssembly=True,
                    rigidAssemblyWorkspaceId=self.workspace_id,
                    rigidAssemblyToPartTF=None,
                    MassProperty=None,  # Populated later via mass property fetch
                )

        LOGGER.info(f"Populated {len(self.parts)} parts from assembly")

    def _compute_rigid_part_transforms(self) -> None:
        """
        Cache transforms from rigid assemblies to their descendant parts.

        For each part instance that lives inside a rigid assembly, compute the transform
        that maps from the rigid assembly frame to the part frame. This mapping is stored
        for later use when populating Part models.
        """
        computed = 0

        for key, instance in self.instances.items():
            if not isinstance(instance, PartInstance):
                continue

            rigid_root = self.get_rigid_assembly_root(key)
            if not rigid_root:
                continue

            rigid_tf = self.get_transform(rigid_root)
            part_tf = self.get_transform(key)

            if rigid_tf is None or part_tf is None:
                LOGGER.warning(
                    "Cannot compute rigid assembly transform for part %s: rigid_tf=%s, part_tf=%s",
                    key,
                    rigid_tf is not None,
                    part_tf is not None,
                )
                continue

            # Transform taking the rigid assembly frame to the part frame
            rigid_to_part_tf = np.linalg.inv(rigid_tf) @ part_tf
            self.rigid_part_transforms[key] = MatedCS.from_tf(rigid_to_part_tf)

            computed += 1

        if computed:
            LOGGER.info("Computed rigid assembly transforms for %d parts", computed)

    def _attach_rigid_transform_to_part(self, key: PathKey) -> None:
        """Apply cached rigid assembly transforms to the Part model."""
        part_model = self.parts.get(key)
        if not part_model:
            return

        rigid_to_part_tf = self.rigid_part_transforms.get(key)

        if rigid_to_part_tf is not None:
            part_model.rigidAssemblyToPartTF = rigid_to_part_tf
        elif key.depth > self.max_depth:
            LOGGER.warning(f"No rigid assembly transform found for part {key}")
            exit(1)


# ============================================================================
# HELPER FUNCTIONS FOR PathKey-based Robot Construction
# ============================================================================


def fetch_mass_properties_for_kinematic_parts(
    cad: CAD,
    kinematic_graph: "KinematicGraph",
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
            if hasattr(result, "mass"):  # Verify it's a MassProperties object
                part.MassProperty = cast(MassProperties, result)
                LOGGER.debug(f"Updated mass properties for {key}")
            else:
                LOGGER.error(f"Invalid mass properties result for {key}: {result}")

    LOGGER.info("Mass properties fetching complete")


async def _fetch_rigid_subassemblies_async(cad: CAD, client: Client, assembly: Assembly):
    """
    Fetch and populate all subassembly occurrences

    Args:
        cad (CAD): _description_
        client (Client): _description_
    """
    _subassemblies = {}

    # subassemblies can only be found by comparing UID with instances??
