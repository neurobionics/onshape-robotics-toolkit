"""
This module contains functions that provide a way to traverse the assembly structure, extract information about parts,
subassemblies, instances, and mates, and generate a hierarchical representation of the assembly.

"""

import asyncio
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional, Union

if TYPE_CHECKING:
    pass

import copy

import numpy as np
from loguru import logger

from onshape_robotics_toolkit.config import (
    record_cad_config,
    record_mate_name,
    record_part_name,
    update_mate_limits,
)
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.models.assembly import (
    Assembly,
    AssemblyFeature,
    AssemblyFeatureType,
    AssemblyInstance,
    BodyType,
    MatedCS,
    MateFeatureData,
    MateGroupFeatureData,
    MateType,
    Occurrence,
    Part,
    PartInstance,
    Pattern,
    RootAssembly,
    SubAssembly,
)
from onshape_robotics_toolkit.models.document import Document, WorkspaceType, parse_url
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name, parse_onshape_expression

# Path and entity constants
ROOT_PATH_NAME = "root"
SUBASSEMBLY_PATH_PREFIX = "subassembly"

# Assembly classification constants
# Feature types that indicate a rigid assembly when they're the ONLY features present.
# Mate groups are organizational features that group parts together without defining
# kinematic relationships. When an assembly contains ONLY mate groups (or no features),
# it signals the designer's intent for those parts to move as a single rigid body.
RIGID_ASSEMBLY_ONLY_FEATURE_TYPES = {AssemblyFeatureType.MATEGROUP}

# Entity relationship constants
CHILD = 0
PARENT = 1

RELATION_CHILD = 1
RELATION_PARENT = 0


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
    def depth(self) -> int:
        """Get depth in hierarchy (0 = root level)."""
        return len(self._path) - 1

    def __repr__(self) -> str:
        return f"PathKey({self._path})"

    def __str__(self) -> str:
        """String representation showing the name path structure."""
        return "_".join(self._name_path) if self._name_path else "(empty)"

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
        try:
            name_tuple = tuple(id_to_name[instance_id] for instance_id in id_tuple)
        except KeyError as e:
            raise KeyError(f"Instance ID {e} not found in id_to_name mapping") from e

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
        keys_by_id: Canonical PathKey index by ID (ID path tuple → PathKey)
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
    name: Optional[str]
    max_depth: int

    # Core data (flat dicts with absolute PathKeys)
    keys_by_id: dict[tuple[str, ...], PathKey]  # ID path tuple -> PathKey (canonical index)
    keys_by_name: dict[tuple[str, ...], PathKey]  # Name path tuple -> PathKey (reverse index)
    instances: dict[PathKey, Union[PartInstance, AssemblyInstance]]
    occurrences: dict[PathKey, Occurrence]
    subassemblies: dict[PathKey, SubAssembly]
    mates: dict[tuple[Optional[PathKey], PathKey, PathKey], MateFeatureData]  # (assembly, parent, child)
    patterns: dict[str, Pattern]
    parts: dict[PathKey, Part]  # Populated eagerly from assembly.parts

    def __init__(
        self,
        document_id: str,
        element_id: str,
        wtype: str,
        workspace_id: str,
        document_microversion: str,
        name: Optional[str] = "cad",
        max_depth: int = 0,
        client: Optional[Client] = None,
    ):
        """
        Initialize an empty CAD document.

        Args:
            document_id: Onshape document ID
            element_id: Onshape element (assembly) ID
            wtype: Workspace type of the document
            workspace_id: Onshape workspace ID
            document_microversion: Onshape document microversion
            name: name of the Onshape document (not element)
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

        self._client = client

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
            return self.mates.get((assembly, parent, child))
        else:
            mate = self.mates.get((None, parent, child))
            if mate:
                return mate

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

    def estimate_api_calls(
        self,
        fetch_mass_properties: bool = True,
        fetch_mate_properties: bool = True,
        download_meshes: bool = True,
    ) -> dict[str, int]:
        """
        Estimate the number of REMAINING API calls needed to process this CAD assembly.

        This method analyzes the parsed assembly structure and calculates how many
        additional API calls will be required to fetch mass properties and download meshes.

        Note: This does NOT include the initial get_assembly() call that was already
        made to create this CAD object.

        Args:
            fetch_mass_properties: Whether mass properties will be fetched
            download_meshes: Whether mesh files will be downloaded

        Returns:
            Dictionary containing breakdown of estimated REMAINING API calls:
                - 'subassemblies': Calls for fetching rigid subassembly data
                - 'mass_properties': Calls for fetching mass properties
                - 'meshes': Calls for downloading mesh files
                - 'total': Total estimated REMAINING API calls

        Examples:
            >>> cad = CAD.from_assembly(assembly, max_depth=1)
            >>> estimation = cad.estimate_api_calls(
            ...     fetch_mass_properties=True,
            ...     download_meshes=True
            ... )
            >>> print(f"Estimated remaining API calls: {estimation['total']}")
            Estimated remaining API calls: 24
        """
        num_kinematic_parts = sum(
            1
            for key, part in self.parts.items()
            if part.rigidAssemblyToPartTF is None and not part.isRigidAssembly and not self.instances[key].suppressed
        )

        num_rigid_subassemblies = sum(
            1 for key, sub in self.subassemblies.items() if sub.isRigid and not self.instances[key].suppressed
        )
        num_flexible_subassemblies = sum(
            1 for key, sub in self.subassemblies.items() if not self.instances[key].suppressed
        )

        # Each rigid subassembly needs get_root_assembly to fetch occurrence data
        subassembly_calls = num_rigid_subassemblies

        mass_property_calls = 0
        if fetch_mass_properties:
            mass_property_calls = num_kinematic_parts + num_rigid_subassemblies

        mate_property_calls = 0
        if fetch_mate_properties:
            mate_property_calls = 1 + num_flexible_subassemblies - num_rigid_subassemblies

        mesh_download_calls = 0
        if download_meshes:
            mesh_download_calls = num_kinematic_parts + num_rigid_subassemblies

        total_calls = subassembly_calls + mass_property_calls + mesh_download_calls + mate_property_calls

        return {
            "subassemblies": subassembly_calls,
            "mass_properties": mass_property_calls,
            "mate_properties": mate_property_calls,
            "meshes": mesh_download_calls,
            "total": total_calls,
        }

    def __repr__(self) -> str:
        # TODO: would be nice to have the CAD tree print here
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

    @classmethod
    def from_assembly(
        cls,
        assembly: Assembly,
        max_depth: int = 0,
        client: Optional[Client] = None,
        fetch_mass_properties: bool = True,
        fetch_mate_properties: bool = True,
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
            client=client,
        )

        # Build id_to_name mapping (needed for PathKey creation)
        id_to_name = cls._build_id_to_name_map(assembly)

        # Build PathKeys from occurrences (single source of truth)
        cad.keys_by_id, cad.keys_by_name = cls._build_path_keys_from_occurrences(
            assembly.rootAssembly.occurrences, id_to_name
        )

        # Populate all data using the pre-built PathKeys
        cad._populate_from_assembly(assembly)

        if fetch_mate_properties:
            cad.fetch_mate_limits(client)

        logger.info(f"Created {cad}")
        record_cad_config(max_depth=max_depth)

        if client is not None:
            estimation = cad.estimate_api_calls(
                fetch_mass_properties=fetch_mass_properties,
                fetch_mate_properties=fetch_mate_properties,
                download_meshes=True,
            )
            logger.info(
                f"Estimated API calls for CAD to Robot conversion → Total: {estimation['total']} "
                f"(subassemblies: {estimation['subassemblies']}, "
                f"mass_properties: {estimation['mass_properties']}, "
                f"mate_properties: {estimation['mate_properties']}, "
                f"meshes: {estimation['meshes']})"
            )

        return cad

    @classmethod
    def from_url(
        cls,
        url: str,
        *,
        client: Client,
        max_depth: int = 0,
        configuration: str = "default",
        log_response: bool = True,
        with_meta_data: bool = True,
    ) -> "CAD":
        """Create a CAD instance directly from an Onshape document URL."""
        base_url, did, wtype, wid, eid = parse_url(url)
        if client is None:
            raise ValueError("client must be provided to load CAD from URL")

        document = Document(base_url=base_url, did=did, wtype=wtype, wid=wid, eid=eid, url=url)

        assembly = client.get_assembly(
            document.did,
            document.wtype,
            document.wid,
            document.eid,
            configuration=configuration,
            log_response=log_response,
            with_meta_data=with_meta_data,
        )

        return cls.from_assembly(assembly, max_depth=max_depth, client=client)

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
        logger.debug("Building PathKeys from occurrences...")

        keys_by_id: dict[tuple[str, ...], PathKey] = {}
        keys_by_name: dict[tuple[str, ...], PathKey] = {}

        for occurrence in occurrences:
            # Create PathKey with both ID and name paths
            key = PathKey.from_path(occurrence.path, id_to_name)

            # Index by ID path tuple (immutable, hashable)
            keys_by_id[key.path] = key

            # Index by name path tuple for reverse lookup
            keys_by_name[key.name_path] = key

        logger.debug(f"Built {len(keys_by_id)} PathKeys from {len(occurrences)} occurrences")
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
        logger.debug("Building instance ID to name mapping...")

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

        logger.debug(f"Mapped {len(id_to_name)} instance IDs to names")
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

        # Step 4: Populate parts from assembly.parts (mass properties still None)
        self._populate_parts(assembly)

        # Step 6: Populate mates with assembly provenance
        self._populate_mates(assembly)

        # Step 5: Populate patterns (root + subassemblies)
        self._populate_patterns(assembly)

        logger.debug(
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
                    if instance.suppressed:
                        continue
                    # check if parent is suppressed
                    if parent_key and self.instances[parent_key].suppressed:
                        continue

                    scope = "root" if parent_key is None else "nested"
                    logger.warning(f"No PathKey for {scope} instance {instance.id} ({instance.name})")
                    continue

                self.instances[key] = copy.deepcopy(instance)

                if isinstance(instance, AssemblyInstance):
                    subassembly = subassembly_lookup.get(instance.uid)
                    if not subassembly:
                        logger.warning(
                            f"Missing SubAssembly definition for instance {instance.id} "
                            f"({instance.name}) with uid {instance.uid}"
                        )
                        continue
                    _populate_branch(subassembly.instances, key)
                elif isinstance(instance, PartInstance):
                    record_part_name(str(key), getattr(instance, "name", None))

        _populate_branch(assembly.rootAssembly.instances, None)
        logger.info(f"Populated {len(self.instances)} instances (including nested)")

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
                logger.warning(f"No PathKey for occurrence {occurrence.path}")

        logger.info(f"Populated {len(self.occurrences)} occurrences")

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
        rigid_by_depth = 0
        rigid_by_features = 0

        for subassembly in assembly.subAssemblies:
            pathkeys = uid_to_pathkeys.get(subassembly.uid)
            if not pathkeys:
                logger.warning(
                    "SubAssembly definition uid=%s has no matching AssemblyInstance occurrences", subassembly.uid
                )
                continue

            # Analyze features to determine if this subassembly should be rigid
            # An assembly is rigid by features if:
            # 1. It has no features (empty assembly), OR
            # 2. It has features AND all features are mate groups (organizational only)
            is_rigid_by_mate_groups = self._is_rigid_by_features(subassembly)

            for pathkey in pathkeys:
                # Create a deep copy for this occurrence
                subassembly_copy = copy.deepcopy(subassembly)

                # Mark rigidity based on depth or feature analysis
                is_rigid_by_depth_check = pathkey.depth >= self.max_depth

                if is_rigid_by_mate_groups:
                    subassembly_copy.isRigid = True
                    rigid_by_features += 1
                    logger.debug(
                        f"Subassembly {pathkey} marked as rigid (features analysis: "
                        f"{len(subassembly.features)} features, all mate groups)"
                    )
                elif is_rigid_by_depth_check:
                    subassembly_copy.isRigid = True
                    rigid_by_depth += 1
                    logger.debug(
                        f"Subassembly {pathkey} marked as rigid (depth {pathkey.depth} >= max_depth {self.max_depth})"
                    )

                # Update the corresponding instance
                inst = self.instances[pathkey]
                if isinstance(inst, AssemblyInstance):
                    inst.isRigid = subassembly_copy.isRigid

                self.subassemblies[pathkey] = subassembly_copy
                total_occurrences += 1

        logger.info(
            f"Populated {total_occurrences} subassembly occurrences from {total_defs} definitions "
            f"(rigid_by_depth={rigid_by_depth}, rigid_by_features={rigid_by_features}, max_depth={self.max_depth})",
        )

    def _is_rigid_by_features(self, subassembly: SubAssembly) -> bool:
        """
        Determine if a subassembly should be treated as rigid based on its features.

        A subassembly is considered rigid if:
        1. It has no features at all (empty assembly), OR
        2. It has features but ALL of them are mate groups

        Mate groups are organizational features that group parts together without
        defining kinematic relationships. When an assembly contains only mate groups,
        it indicates the designer intends for those parts to move as a single rigid body.

        Args:
            subassembly: SubAssembly to analyze

        Returns:
            True if subassembly should be rigid, False otherwise

        Examples:
            >>> # Empty assembly -> rigid
            >>> sub = SubAssembly(features=[])
            >>> _is_rigid_by_features(sub)
            True

            >>> # Only mate groups -> rigid
            >>> sub = SubAssembly(features=[MateGroup1, MateGroup2])
            >>> _is_rigid_by_features(sub)
            True

            >>> # Mix of mate groups and mates -> flexible
            >>> sub = SubAssembly(features=[MateGroup1, RevoluteMate])
            >>> _is_rigid_by_features(sub)
            False
        """
        if len(subassembly.features) == 0:
            return True

        # Check if all non-suppressed features are mate groups
        non_suppressed_features = [f for f in subassembly.features if not f.suppressed]
        if len(non_suppressed_features) == 0:
            return True

        return all(feature.featureType in RIGID_ASSEMBLY_ONLY_FEATURE_TYPES for feature in non_suppressed_features)

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

        def _process_feature(
            feature: AssemblyFeature,
            assembly_key: Optional[PathKey],
            path_prefix: Optional[tuple[str, ...]],
        ) -> None:
            """Internal helper to process a single assembly feature.

            Handles both mate features (which create kinematic relationships) and
            mate groups (which are organizational only and don't create edges).

            Args:
                feature: Feature object which may contain MateFeatureData or MateGroupFeatureData
                assembly_key: None for root, or PathKey of subassembly provenance
                path_prefix: Tuple prefix to prepend for relative subassembly paths
            """
            if feature.suppressed:
                return

            # Handle mate groups (organizational only - no kinematic relationship)
            if feature.featureType == AssemblyFeatureType.MATEGROUP:
                if isinstance(feature.featureData, MateGroupFeatureData):
                    mate_group_data: MateGroupFeatureData = feature.featureData
                    scope = "root" if assembly_key is None else f"subassembly {assembly_key}"
                    logger.debug(
                        f"Found mate group '{mate_group_data.name}' in {scope} with "
                        f"{len(mate_group_data.occurrences)} occurrences (organizational only, no mate edges created)"
                    )
                return

            # Handle mate connectors (not yet implemented)
            if feature.featureType == AssemblyFeatureType.MATECONNECTOR:
                # TODO: add support for mate connectors
                return

            # Handle regular mates (create kinematic relationships)
            if feature.featureType != AssemblyFeatureType.MATE or not isinstance(feature.featureData, MateFeatureData):
                return

            mate_data: MateFeatureData = copy.deepcopy(feature.featureData)

            # TODO: Onshape mate feature data always has two entities (parent/child). If origin mates ever
            # appear differently, this is the place to update handling.
            try:
                parent_occ = mate_data.matedEntities[PARENT].matedOccurrence
                child_occ = mate_data.matedEntities[CHILD].matedOccurrence
            except Exception:
                logger.warning(f"Malformed mate feature {mate_data.name}")
                return

            parent_path = tuple(parent_occ)
            child_path = tuple(child_occ)

            if path_prefix:
                parent_path = path_prefix + parent_path
                child_path = path_prefix + child_path

            parent_key = self.keys_by_id.get(parent_path)
            child_key = self.keys_by_id.get(child_path)

            # NOTE: reorient the mated entities to match this parent, child order
            # TODO: add tests to make sure this convention is preserved
            # We create indices for parent and child, get the occurrences,
            # remap the mate data to always be parent -> child
            mate_data.matedEntities = [
                mate_data.matedEntities[PARENT],
                mate_data.matedEntities[CHILD],
            ]

            sanitized_mate_name = get_sanitized_name(mate_data.name)
            record_mate_name(sanitized_mate_name, mate_data.name, mate_data.limits)

            if parent_key and child_key:
                self.mates[(assembly_key, parent_key, child_key)] = mate_data
            else:
                scope = "root" if assembly_key is None else f"subassembly {assembly_key}"
                logger.warning(
                    "Missing PathKey for %s mate: %s (parent_found=%s, child_found=%s)",
                    scope,
                    mate_data.name,
                    bool(parent_key),
                    bool(child_key),
                )

        # Process root assembly features (absolute paths)
        for feature in assembly.rootAssembly.features:
            _process_feature(feature, None, None)

        # Process subassembly features (relative paths)
        for sub_key, subassembly in self.subassemblies.items():
            if subassembly.isRigid:
                continue

            for feature in subassembly.features:
                _process_feature(feature=feature, assembly_key=sub_key, path_prefix=sub_key.path)

        logger.debug(f"Populated {len(self.mates)} mates (root + flexible subassemblies)")

    def _populate_patterns(self, assembly: Assembly) -> None:
        """
        Populate patterns from root assembly and all subassemblies.

        Args:
            assembly: Assembly from Onshape API
        """

        def _add_pattern(pattern: Pattern, path_prefix: Optional[tuple[str, ...]]) -> None:
            if pattern.suppressed:
                return

            mutated_seed_pattern_instances: dict[tuple[str, ...], list[tuple[str, ...]]] = {}
            for seed_id, instance_ids in pattern.seedToPatternInstances.items():
                # since seed_id can be both a single string or a tuple of strings,
                # we need to make it a tuple for consistent keying
                seed_id_tuple: tuple[str, ...] = (seed_id,) if isinstance(seed_id, str) else seed_id

                mutated_id = seed_id_tuple if path_prefix is None else (*path_prefix, *seed_id_tuple)

                for instance_id in instance_ids:
                    # since instance_id can be both a single string or a tuple of strings,
                    # we need to make it a tuple for consistent keying
                    instance_id_tuple: tuple[str, ...]
                    if isinstance(instance_id, str):
                        instance_id_tuple = (instance_id,)
                    elif isinstance(instance_id, list):
                        instance_id_tuple = tuple(instance_id)
                    else:
                        instance_id_tuple = instance_id

                    mutated_instance_id = (
                        instance_id_tuple if path_prefix is None else (*path_prefix, *instance_id_tuple)
                    )
                    mutated_seed_pattern_instances.setdefault(mutated_id, []).append(mutated_instance_id)

            pattern.seedToPatternInstances = mutated_seed_pattern_instances  # type: ignore[assignment]
            self.patterns[pattern.id] = pattern

        # Root patterns
        for pattern in assembly.rootAssembly.patterns:
            _add_pattern(pattern, None)

        # Subassembly patterns (only from flexible subassemblies)
        for subassembly_key, subassembly in self.subassemblies.items():
            if subassembly.isRigid:
                continue

            for pattern in subassembly.patterns:
                _add_pattern(pattern, subassembly_key.path)

        self._flatten_patterns()

        logger.debug(f"Populated {len(self.patterns)} patterns")

    def _flatten_patterns(
        self,
    ) -> None:
        def _add_mate(
            seed_mate: MateFeatureData,
            seed_key: PathKey,
            instance_key: PathKey,
            other_entity_key: PathKey,
            mate_key: tuple[Optional[PathKey], PathKey, PathKey],
            is_seed_parent: bool = True,
        ) -> None:
            new_mate = copy.deepcopy(seed_mate)
            # NOTE: seed index needs to be replaced by the instance
            # since we reordered the populated mates to always have parent->child order,
            # we should not use the global constants (PARENT, CHILD) here
            oe_index = 1 if is_seed_parent else 0
            oe_mated_cs = new_mate.matedEntities[oe_index].matedCS

            # NOTE: compute new matedCS for the other entity, this is crucial because
            # this governs where this new instance will be placed in the robot structure
            seed_pose_wrt_world = self.occurrences[seed_key].tf
            instance_pose_wrt_world = self.occurrences[instance_key].tf
            oe_pose_wrt_world = self.occurrences[other_entity_key].tf

            # NOTE: find the other entity's mated CS wrt to the instance part
            # First, find the other entity's mated CS wrt seed part, then transform it
            # to the instance global coordinates, then find this wrt to the other entity
            oe_mated_cs_wrt_world = oe_pose_wrt_world @ oe_mated_cs.to_tf
            oe_mated_cs_wrt_seed = np.linalg.inv(seed_pose_wrt_world) @ oe_mated_cs_wrt_world
            oe_mated_cs_for_instance = instance_pose_wrt_world @ oe_mated_cs_wrt_seed
            oe_mated_cs_tf = np.linalg.inv(oe_pose_wrt_world) @ oe_mated_cs_for_instance

            new_mate.matedEntities[oe_index].matedCS = MatedCS.from_tf(oe_mated_cs_tf)
            new_mate.matedEntities[1 - oe_index].matedOccurrence = list(instance_key.path)
            self.mates[mate_key] = new_mate

        seed_instance_to_pattern_instances: dict[tuple[str, PathKey], list[PathKey]] = {}

        for pattern_id, pattern in self.patterns.items():
            if pattern.suppressed:
                continue

            for seed_id, instance_ids in pattern.seedToPatternInstances.items():
                # Ensure seed_id is a tuple for dictionary lookup
                seed_id_tuple = seed_id if isinstance(seed_id, tuple) else (seed_id,)
                seed_path_key = self.keys_by_id[seed_id_tuple]
                key = (pattern_id, seed_path_key)

                if key not in seed_instance_to_pattern_instances:
                    seed_instance_to_pattern_instances[key] = []

                for instance_id in instance_ids:
                    # NOTE: instance_id is a list of strings (path)
                    instance_path_key = self.keys_by_id[tuple(instance_id)]
                    seed_instance_to_pattern_instances[key].append(instance_path_key)

        # find all mates referencing this part
        original_mates = copy.deepcopy(self.mates)
        for (assembly_key, parent_key, child_key), mate in original_mates.items():
            for (_, seed_key), instance_keys in seed_instance_to_pattern_instances.items():
                if parent_key == seed_key:
                    for instance_key in instance_keys:
                        _add_mate(
                            seed_mate=mate,
                            seed_key=parent_key,
                            instance_key=instance_key,
                            other_entity_key=child_key,
                            mate_key=(assembly_key, instance_key, child_key),
                            is_seed_parent=True,
                        )
                if child_key == seed_key:
                    for instance_key in instance_keys:
                        _add_mate(
                            seed_mate=mate,
                            seed_key=child_key,
                            instance_key=instance_key,
                            other_entity_key=parent_key,
                            mate_key=(assembly_key, parent_key, instance_key),
                            is_seed_parent=False,
                        )

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
                logger.warning("Part definition uid=%s has no matching PartInstance", part.uid)
                continue

            for pathkey in pathkeys:
                self.parts[pathkey] = copy.deepcopy(part)  # Avoid mutating original data
                self.parts[pathkey].worldToPartTF = MatedCS.from_tf(self.occurrences[pathkey].tf)

                # Check if this part belongs to a rigid assembly
                # This can happen either because:
                # 1. The part's depth exceeds max_depth (depth-based rigidity), OR
                # 2. The part is inside an assembly that was marked rigid by feature analysis (mate groups only)
                rigid_root = self.get_rigid_assembly_root(pathkey)
                if rigid_root is not None:
                    self.parts[pathkey].rigidAssemblyKey = rigid_root
                    self.parts[pathkey].rigidAssemblyWorkspaceId = self.workspace_id

                    # NOTE: Using the root occurrences of the rigid assembly directly from Onshape API
                    # instead of computing it from the root assembly's global occurrences because
                    # Onshape's occurrence TF for subassemblies are not what we expect it to be, the occurrence TF
                    # does not reflect the pose of the subassembly in world frame, will Onshape potentially fix this?
                    if self.subassemblies[rigid_root].RootOccurrences is None:
                        if self._client is None:
                            logger.warning(
                                f"At max_depth of {self.max_depth}, we require Client to "
                                "fetch all root occurrences of a subassembly."
                            )
                            logger.warning("These root occurrences are used to remap parts inside rigid assemblies.")
                            logger.warning(
                                f"Skipping setting rigidAssemblyToPartTF for part {pathkey} "
                                f"inside rigid assembly {rigid_root}."
                            )
                            logger.warning(
                                "This will result in malformed joints that have refer to parts within rigid assemblies."
                            )
                            continue

                        asyncio.run(self.fetch_occurrences_for_subassemblies(self._client))

                    part_pose_wrt_rigid_root = self.subassemblies[rigid_root].RootOccurrences[pathkey]  # type: ignore[index]
                    self.parts[pathkey].rigidAssemblyToPartTF = MatedCS.from_tf(tf=part_pose_wrt_rigid_root.tf)

                    logger.debug(f"Set rigidAssemblyToPartTF for {pathkey}, with rigid assembly {rigid_root}")

        # Create synthetic Part objects for rigid assemblies
        for key, subassembly in self.subassemblies.items():
            if subassembly.isRigid:
                subassembly_instance = self.instances[key]
                # Rigid assembly: create synthetic Part object
                self.parts[key] = Part(
                    isStandardContent=False,
                    partId=subassembly_instance.elementId,  # Use element ID as part ID
                    bodyType=BodyType.SOLID,
                    documentId=subassembly_instance.documentId,
                    elementId=subassembly_instance.elementId,
                    documentMicroversion=subassembly_instance.documentMicroversion,
                    configuration=subassembly_instance.configuration,
                    fullConfiguration=subassembly_instance.fullConfiguration,
                    documentVersion=subassembly_instance.documentVersion,
                    isRigidAssembly=True,
                    rigidAssemblyKey=None,  # Not applicable for rigid assembly itself
                    rigidAssemblyWorkspaceId=self.workspace_id,
                    rigidAssemblyToPartTF=None,
                    worldToPartTF=MatedCS.from_tf(self.occurrences[key].tf),
                    MassProperty=None,  # Populated later via mass property fetch
                )

        logger.info(f"Populated {len(self.parts)} parts from assembly")

    async def fetch_mass_properties_for_parts(self, client: Client) -> None:
        async def _fetch_mass_properties(key: PathKey, part: Part, client: Client) -> None:
            try:
                if part.isRigidAssembly:
                    logger.debug(f"Fetching mass properties for rigid assembly: {key}")
                    part.MassProperty = await asyncio.to_thread(
                        client.get_assembly_mass_properties,
                        did=part.documentId,
                        wtype=WorkspaceType.W.value,
                        wid=part.rigidAssemblyWorkspaceId,  # type: ignore[arg-type]
                        eid=part.elementId,
                    )
                else:
                    logger.debug(f"Fetching mass properties for part: {key}")
                    part.MassProperty = await asyncio.to_thread(
                        client.get_mass_property,
                        did=part.documentId,
                        wtype=WorkspaceType.M.value,
                        wid=part.documentMicroversion if part.documentMicroversion else self.document_microversion,
                        eid=part.elementId,
                        partID=part.partId,
                    )
            except Exception as e:
                logger.error(f"Failed to fetch mass properties for part {key}: {e}")

        tasks = []
        for key, part in self.parts.items():
            if part.bodyType == BodyType.SHEET.value:
                continue  # skip surface bodies

            if self.instances[key].suppressed:
                continue

            if part.MassProperty is not None:
                logger.debug(f"Part {key} already has mass properties, skipping")
                continue

            if part.rigidAssemblyToPartTF is not None:
                # this part belongs to a rigid subassembly, skip fetching mass properties
                continue

            tasks.append(_fetch_mass_properties(key, part, client))

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def fetch_occurrences_for_subassemblies(self, client: Client) -> None:
        async def _fetch_rootassembly(key: PathKey, subassembly: SubAssembly, client: Client) -> None:
            try:
                logger.debug(f"Fetching root assembly for subassembly: {key}")
                _subassembly_data: RootAssembly = await asyncio.to_thread(
                    client.get_root_assembly,
                    did=subassembly.documentId,
                    wtype=WorkspaceType.M.value,
                    wid=subassembly.documentMicroversion,
                    eid=subassembly.elementId,
                    with_mass_properties=True,
                    log_response=False,
                )
                _subassembly_occurrences = _subassembly_data.occurrences
                for occurrence in _subassembly_occurrences:
                    # NOTE: add sub-assembly key as prefix to get absolute path
                    path_tuple = tuple(key.path) + tuple(occurrence.path)
                    occ_key = self.keys_by_id.get(path_tuple)
                    if occ_key:
                        if subassembly.RootOccurrences is None:
                            subassembly.RootOccurrences = {}
                        subassembly.RootOccurrences[occ_key] = occurrence
                    else:
                        # Check if any parent in the hierarchy is suppressed (causing missing PathKey)
                        # Walk up the path and check if any instance is suppressed
                        is_suppressed_parent = False
                        for i in range(len(path_tuple), 0, -1):
                            partial_path = path_tuple[:i]
                            partial_key = self.keys_by_id.get(partial_path)
                            if (
                                partial_key
                                and self.instances.get(partial_key, None)
                                and self.instances[partial_key].suppressed
                            ):
                                is_suppressed_parent = True
                                break

                        if not is_suppressed_parent:
                            logger.warning(f"No PathKey for subassembly occurrence {occurrence.path} in {key}")

            except Exception as e:
                logger.error(f"Failed to fetch root assembly for subassembly {key}: {e}")

        tasks = []
        for key, subassembly in self.subassemblies.items():
            if self.instances[key].suppressed:
                continue

            if subassembly.RootOccurrences is not None:
                logger.debug(f"Subassembly {key} already has RootOccurrences, skipping")
                continue

            if not subassembly.isRigid:
                # dont fetch root occurrences for flexible subassemblies
                continue

            tasks.append(_fetch_rootassembly(key, subassembly, client))

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    def fetch_mate_limits(self, client: Optional[Client]) -> None:
        """
        Fetch joint limits from Onshape features and populate mate data.

        This method fetches features for the root assembly and all flexible subassemblies,
        extracts joint limit parameters from mate features, and stores them in the
        corresponding MateFeatureData objects.

        Args:
            client: Onshape API client for fetching features

        Note:
            - Only processes mates with limitsEnabled=true
            - Parses limit expressions (e.g., "90 deg", "0.5 m") to numerical values
            - Stores limits as {'min': float, 'max': float} dict in MateFeatureData.limits
            - For revolute joints: uses axial Z limits
            - For prismatic/slider joints: uses Z limits
        """
        if client is None:
            logger.warning("No client provided for fetching mate limits, skipping")
            return

        logger.info("Fetching mate limits from assembly features")

        # Build list of assemblies to fetch features from (root + flexible subassemblies)
        assemblies_to_fetch: list[tuple[Optional[PathKey], str, str, str, str]] = []
        assemblies_to_fetch.append((None, self.document_id, self.wtype, self.workspace_id, self.element_id))

        # Add flexible subassemblies
        for sub_key, subassembly in self.subassemblies.items():
            if self.instances[sub_key].suppressed:
                continue

            if not subassembly.isRigid:
                assemblies_to_fetch.append((
                    sub_key,
                    subassembly.documentId,
                    WorkspaceType.M.value,
                    subassembly.documentMicroversion,
                    subassembly.elementId,
                ))

        # Fetch features for each assembly
        limits_found_count = 0
        for assembly_key, did, wtype, wid, eid in assemblies_to_fetch:
            try:
                logger.debug(f"Fetching features for assembly: {assembly_key or 'root'}")
                features = client.get_features(did=did, wtype=wtype, wid=wid, eid=eid)

                # Process each feature to extract limits
                for feature in features.features:
                    if feature.message.featureType != "mate":
                        continue

                    feature_id = feature.message.featureId

                    # Find the corresponding mate in our mates dict
                    mate_data: Optional[MateFeatureData] = None
                    for _, mate in self.mates.items():
                        if mate.id == feature_id:
                            mate_data = mate
                            break

                    if mate_data is None:
                        logger.debug(f"Could not find mate data for feature {feature_id}")
                        continue

                    # Check if limits are enabled
                    params = feature.message.parameter_dict()
                    limits_enabled = params.get("limitsEnabled")
                    if limits_enabled is None or not limits_enabled.get("message", {}).get("value", False):
                        continue

                    # Determine which limit parameters to use based on mate type
                    is_axial = mate_data.mateType in (MateType.REVOLUTE, MateType.CYLINDRICAL)

                    # Extract limit expressions
                    if is_axial:
                        min_param_name = "limitAxialZMin"
                        max_param_name = "limitAxialZMax"
                    else:
                        min_param_name = "limitZMin"
                        max_param_name = "limitZMax"

                    # Extract parameter expressions
                    def get_param_expression(param_dict: dict[str, Any], param_name: str) -> str | None:
                        param = param_dict.get(param_name)
                        if not isinstance(param, dict):
                            return None
                        # Handle BTMParameterNullableQuantity type
                        if param.get("typeName") == "BTMParameterNullableQuantity":
                            message = param.get("message", {})
                            if not isinstance(message, dict) or message.get("isNull", True):
                                return None
                            expression = message.get("expression")
                            return expression if isinstance(expression, str) else None
                        return None

                    min_expression = get_param_expression(params, min_param_name)
                    max_expression = get_param_expression(params, max_param_name)

                    # Parse expressions to numerical values
                    min_value = parse_onshape_expression(min_expression)
                    max_value = parse_onshape_expression(max_expression)

                    # Only store if we got valid values
                    if min_value is not None and max_value is not None:
                        limits = {"min": min_value, "max": max_value}
                        mate_data.limits = limits
                        sanitized_mate_name = get_sanitized_name(mate_data.name)
                        update_mate_limits(sanitized_mate_name, limits)
                        limits_found_count += 1
                        logger.debug(
                            f"Set limits for mate '{mate_data.name}' ({mate_data.mateType}): "
                            f"min={min_value:.4f}, max={max_value:.4f}"
                        )

            except Exception as e:
                logger.warning(f"Failed to fetch features for assembly {assembly_key or 'root'}: {e}")
                continue

        logger.info(f"Fetched limits for {limits_found_count} mates out of {len(self.mates)} total mates")
