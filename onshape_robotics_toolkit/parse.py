"""
This module contains functions that provide a way to traverse the assembly structure, extract information about parts,
subassemblies, instances, and mates, and generate a hierarchical representation of the assembly.

"""

import asyncio
import copy
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Optional, Union, cast

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
    Occurrence,
    Part,
    PartInstance,
    Pattern,
    RootAssembly,
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

    The path is a tuple of instance IDs that represents the hierarchical position
    in the assembly tree, exactly as it appears in occurrence.path and
    matedOccurrence from the JSON API response.

    Examples:
        # Root-level part instance
        PathKey(("MqRDHdbA0tAm2ygBR",))

        # Nested part in subassembly
        PathKey(("MoN/4FhyvQ92+I8TU", "MZHBlAU4IxmX6u6A0", "MrpOYQ6mQsyqwPVz0"))

        # Direct creation from JSON occurrence path
        PathKey.from_occurrence_path(["MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG"])
    """

    _path: tuple[str, ...]

    def __init__(self, path: tuple[str, ...]):
        """
        Create a PathKey from a tuple of instance IDs.

        Args:
            path: Tuple of Onshape instance IDs representing hierarchical position
        """
        object.__setattr__(self, "_path", path)

    def to_name(self, id_to_name: dict[str, str]) -> "PathKey":
        """
        Convert PathKey IDs to sanitized names using provided mapping.

        Args:
            id_to_name: Mapping from instance ID to sanitized name
        Returns:
            New PathKey with names instead of IDs

        """
        try:
            named_path = tuple(id_to_name.get(instance_id) for instance_id in self._path)
        except KeyError as e:
            LOGGER.error(f"ID {e} not found in id_to_name mapping")
            exit(1)

        return PathKey(named_path)

    @property
    def path(self) -> tuple[str, ...]:
        """Get the immutable path tuple."""
        return self._path

    @property
    def leaf(self) -> str:
        """Get the last ID in the path (the actual entity)."""
        return self._path[-1] if self._path else ""

    @property
    def parent(self) -> Optional["PathKey"]:
        """Get parent PathKey by trimming last element."""
        if len(self._path) <= 1:
            return None
        return PathKey(self._path[:-1])

    @property
    def root(self) -> Optional[str]:
        """Get the root ID (first element in path)."""
        return self._path[0] if self._path else None

    @property
    def depth(self) -> int:
        """Get depth in hierarchy (0 = root level)."""
        return len(self._path)

    def __repr__(self) -> str:
        return f"PathKey({self._path})"

    def __str__(self) -> str:
        """String representation showing the path structure."""
        return " > ".join(self._path) if self._path else "(empty)"

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
    def from_path(cls, path: Union[list[str], str]) -> "PathKey":
        """
        Create PathKey from a path (list) or single instance ID (string).

        This handles both:
        - occurrence.path from JSON (list of IDs)
        - matedOccurrence from mate features (list of IDs)
        - single instance ID for root-level instances (string)

        Args:
            path: Either a list of instance IDs or a single instance ID string

        Returns:
            PathKey with the path as an immutable tuple

        Examples:
            # From occurrence JSON (list)
            occ_key = PathKey.from_path(occurrence.path)
            # PathKey(("MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG"))

            # From mate JSON (list)
            parent_key = PathKey.from_path(mate.matedEntities[0].matedOccurrence)
            # PathKey(("MoN/4FhyvQ92+I8TU", "MZHBlAU4IxmX6u6A0", "MrpOYQ6mQsyqwPVz0"))

            # From single instance ID (string)
            part_key = PathKey.from_path("MqRDHdbA0tAm2ygBR")
            # PathKey(("MqRDHdbA0tAm2ygBR",))
        """
        if isinstance(path, str):
            # Single instance ID -> single-element tuple
            return cls((path,))
        else:
            # List of IDs -> tuple
            return cls(tuple(path))


@dataclass
class InstanceRegistry:
    """
    Registry for all instances (parts and assemblies) using path-based keys.

    This replaces the complex TypedKey system with a simpler approach that uses
    Onshape's natural path structure directly.

    Rigid vs Flexible classification is determined dynamically from path depth:
    - is_rigid = (key.depth > max_depth)
    - No need for separate storage!

    Attributes:
        parts: Maps PathKey to PartInstance
        assemblies: Maps PathKey to AssemblyInstance
        id_to_name: Maps instance ID to sanitized name
        max_depth: Maximum depth before assemblies become rigid
    """

    # Primary storage: PathKey -> Instance
    parts: dict[PathKey, PartInstance]
    assemblies: dict[PathKey, AssemblyInstance]

    # Name mapping (ID -> sanitized name)
    id_to_name: dict[str, str]

    # Reverse lookup: (sanitized_name, depth) -> PathKey
    # We use depth to disambiguate same-named instances at different levels
    _name_to_keys: dict[tuple[str, int], list[PathKey]]

    # Configuration
    max_depth: int

    def __init__(self, max_depth: int = 0):
        """
        Initialize empty registry.

        Args:
            max_depth: Maximum depth for flexible assemblies (0 = all rigid, ∞ = all flexible)
        """
        self.parts = {}
        self.assemblies = {}
        self.id_to_name = {}
        self._name_to_keys = {}
        self.max_depth = max_depth

    def add_part(self, key: PathKey, instance: PartInstance) -> None:
        """
        Add a part instance to the registry.

        Args:
            key: PathKey for this part
            instance: PartInstance from JSON
        """
        self.parts[key] = instance
        self._index_name(key, instance.name)

    def add_assembly(self, key: PathKey, instance: AssemblyInstance) -> None:
        """
        Add an assembly instance to the registry.

        Rigid vs flexible classification is determined dynamically from depth.

        Args:
            key: PathKey for this assembly
            instance: AssemblyInstance from JSON
        """
        self.assemblies[key] = instance
        self._index_name(key, instance.name)

    def _index_name(self, key: PathKey, raw_name: str) -> None:
        """
        Index name for reverse lookup.

        Args:
            key: PathKey to index
            raw_name: Raw name from JSON (e.g., "Part 1 <1>")
        """
        sanitized = get_sanitized_name(raw_name)
        lookup_key = (sanitized, key.depth)

        if lookup_key not in self._name_to_keys:
            self._name_to_keys[lookup_key] = []

        self._name_to_keys[lookup_key].append(key)

    def get_part(self, key: PathKey) -> Optional[PartInstance]:
        """Get part by PathKey."""
        return self.parts.get(key)

    def get_assembly(self, key: PathKey) -> Optional[AssemblyInstance]:
        """Get assembly by PathKey."""
        return self.assemblies.get(key)

    def get_instance(self, key: PathKey) -> Optional[Union[PartInstance, AssemblyInstance]]:
        """Get any instance (part or assembly) by PathKey."""
        return self.parts.get(key) or self.assemblies.get(key)

    def is_rigid_assembly(self, key: PathKey) -> bool:
        """
        Check if assembly is rigid.

        Args:
            key: PathKey to check

        Returns:
            True if the assembly instance is marked as rigid

        Example:
            # Assemblies beyond max_depth are marked rigid during population
            # max_depth = 2
            # PathKey depth 0-2 -> flexible (isRigid=False)
            # PathKey depth 3+   -> rigid (isRigid=True)
        """
        assembly = self.assemblies.get(key)
        if not assembly:
            return False
        return isinstance(assembly, AssemblyInstance) and assembly.isRigid

    def is_flexible_assembly(self, key: PathKey) -> bool:
        """
        Check if assembly is flexible.

        Args:
            key: PathKey to check

        Returns:
            True if the assembly instance is marked as flexible (not rigid)
        """
        assembly = self.assemblies.get(key)
        if not assembly:
            return False
        return isinstance(assembly, AssemblyInstance) and not assembly.isRigid

    def lookup_by_name(self, name: str, depth: Optional[int] = None) -> list[PathKey]:
        """
        Look up PathKeys by sanitized name.

        Args:
            name: Sanitized name (e.g., "Part_1_1")
            depth: Optional depth filter (0 = root level)

        Returns:
            List of PathKeys with this name (may be multiple if duplicates exist)

        Example:
            # Find all parts named "Part_1_1" at any depth
            keys = registry.lookup_by_name("Part_1_1")

            # Find only root-level parts named "Part_1_1"
            keys = registry.lookup_by_name("Part_1_1", depth=0)
        """
        if depth is not None:
            return self._name_to_keys.get((name, depth), [])
        else:
            # Return all keys with this name regardless of depth
            all_keys = []
            for (n, _d), keys in self._name_to_keys.items():
                if n == name:
                    all_keys.extend(keys)
            return all_keys

    def get_hierarchical_name(self, key: PathKey, separator: str = "-") -> str:
        """
        Build hierarchical name from path.

        Args:
            key: PathKey to build name for
            separator: String to join names (default ":")

        Returns:
            Hierarchical name (e.g., "Assembly_1:Subassembly_2:Part_3")

        Example:
            # For PathKey(("MoN/4FhyvQ92+I8TU", "MZHBlAU4IxmX6u6A0", "MrpOYQ6mQsyqwPVz0"))
            # Returns: "Assembly_1:double_wheel_1:wheel_part_1"
        """
        names = []
        for instance_id in key.path:
            name = self.id_to_name.get(instance_id, instance_id)
            names.append(name)

        return separator.join(names)

    def build_from_assembly(self, assembly: Assembly) -> None:
        """
        Build registry from Assembly JSON in a single traversal.

        Args:
            assembly: Assembly from Onshape API

        This processes:
        1. Root-level instances
        2. Builds ID-to-name mapping from all subassemblies

        Note: Rigid vs flexible is determined dynamically from depth, not stored!
        """
        # Build ID to name mapping first (used by all instances)
        self._build_id_to_name_map(assembly)

        # Process root-level instances
        for instance in assembly.rootAssembly.instances:
            key = PathKey.from_path(instance.id)  # Single ID -> single-element tuple

            if instance.type == InstanceType.PART:
                self.add_part(key, cast(PartInstance, instance))
            elif instance.type == InstanceType.ASSEMBLY:
                self.add_assembly(key, cast(AssemblyInstance, instance))

    def _build_id_to_name_map(self, assembly: Assembly) -> None:
        """
        Build mapping from instance ID to sanitized name.

        Args:
            assembly: Assembly containing all instances

        This processes root assembly and all subassemblies to build complete mapping.
        """
        LOGGER.debug("Building instance ID to name mapping...")

        # Root assembly instances
        for instance in assembly.rootAssembly.instances:
            sanitized = get_sanitized_name(instance.name)
            self.id_to_name[instance.id] = sanitized

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
                self.id_to_name[instance.id] = sanitized

        LOGGER.debug(f"Mapped {len(self.id_to_name)} instance IDs to names")

    def __len__(self) -> int:
        """Total number of instances."""
        return len(self.parts) + len(self.assemblies)

    @property
    def rigid_count(self) -> int:
        """Count rigid assemblies dynamically."""
        return sum(1 for key in self.assemblies if self.is_rigid_assembly(key))

    @property
    def flexible_count(self) -> int:
        """Count flexible assemblies dynamically."""
        return sum(1 for key in self.assemblies if self.is_flexible_assembly(key))

    def __repr__(self) -> str:
        return (
            f"InstanceRegistry("
            f"parts={len(self.parts)}, "
            f"assemblies={len(self.assemblies)}, "
            f"max_depth={self.max_depth}, "
            f"rigid={self.rigid_count}, "
            f"flexible={self.flexible_count})"
        )


@dataclass
class OccurrenceRegistry:
    """
    Registry for occurrences using path-based keys.

    Occurrences represent the actual placement (transform) of instances in the assembly.
    Each occurrence has a path that matches its instance path, making lookups trivial.

    Attributes:
        occurrences: Maps PathKey to Occurrence
        id_to_name: Reference to InstanceRegistry's ID mapping (shared)
    """

    # Primary storage: PathKey -> Occurrence
    occurrences: dict[PathKey, Occurrence]

    # Shared name mapping (from InstanceRegistry)
    id_to_name: dict[str, str]

    def __init__(self, id_to_name: dict[str, str]):
        """
        Initialize occurrence registry.

        Args:
            id_to_name: Shared ID-to-name mapping from InstanceRegistry
        """
        self.occurrences = {}
        self.id_to_name = id_to_name

    def add_occurrence(self, occurrence: Occurrence) -> PathKey:
        """
        Add an occurrence to the registry.

        Args:
            occurrence: Occurrence from JSON

        Returns:
            PathKey created from occurrence.path

        Example:
            # From JSON: {"path": ["MoN/4FhyvQ92+I8TU", "MM10pxoGk/3TUSoYG"], "transform": [...]}
            key = registry.add_occurrence(occurrence)
        """
        key = PathKey.from_path(occurrence.path)
        self.occurrences[key] = occurrence
        return key

    def get_occurrence(self, key: PathKey) -> Optional[Occurrence]:
        """
        Get occurrence by PathKey.

        Args:
            key: PathKey to lookup

        Returns:
            Occurrence if found, None otherwise
        """
        return self.occurrences.get(key)

    def get_transform(self, key: PathKey) -> Optional[np.ndarray]:
        """
        Get 4x4 transform matrix for an occurrence.

        Args:
            key: PathKey to lookup

        Returns:
            4x4 numpy array, or None if not found

        Example:
            tf = registry.get_transform(key)
            # Returns: np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
        """
        occ = self.occurrences.get(key)
        if occ:
            return np.array(occ.transform).reshape(4, 4)
        return None

    def build_transform_chain(self, key: PathKey) -> np.ndarray:
        """
        Build complete transform by walking up the path hierarchy.

        This multiplies transforms from the occurrence up to the root,
        giving the final world-space transform.

        Args:
            key: PathKey to build transform for

        Returns:
            4x4 numpy array representing cumulative transform

        Example:
            # For key = PathKey(("ASM_1", "ASM_2", "PART_3"))
            # Multiplies: root_to_ASM_1 @ ASM_1_to_ASM_2 @ ASM_2_to_PART_3
            world_tf = registry.build_transform_chain(key)
        """
        transforms = []

        # Walk up the path from full to root
        current_path = key.path
        for i in range(len(current_path), 0, -1):
            sub_path = current_path[:i]
            sub_key = PathKey(sub_path)
            occ = self.occurrences.get(sub_key)

            if occ:
                transforms.append(np.array(occ.transform).reshape(4, 4))

        # Multiply transforms in order (child to parent)
        result = np.eye(4)
        for tf in transforms:
            result = tf @ result

        return result

    def get_hierarchical_name(self, key: PathKey, separator: str = ":") -> str:
        """
        Build hierarchical name from path.

        Args:
            key: PathKey to build name for
            separator: String to join names (default ":")

        Returns:
            Hierarchical name (e.g., "Assembly_1:Subassembly_2:Part_3")
        """
        names = []
        for instance_id in key.path:
            name = self.id_to_name.get(instance_id, instance_id)
            names.append(name)

        return separator.join(names)

    def build_from_root_assembly(self, root_assembly: RootAssembly) -> None:
        """
        Build occurrence registry from root assembly occurrences.

        Args:
            root_assembly: RootAssembly containing occurrences

        Example:
            registry = OccurrenceRegistry(instance_registry.id_to_name)
            registry.build_from_root_assembly(assembly.rootAssembly)
        """
        for occurrence in root_assembly.occurrences:
            self.add_occurrence(occurrence)

        LOGGER.debug(f"Added {len(self.occurrences)} occurrences from root assembly")

    def __len__(self) -> int:
        """Total number of occurrences."""
        return len(self.occurrences)

    def __repr__(self) -> str:
        return f"OccurrenceRegistry(occurrences={len(self.occurrences)})"


@dataclass
class MateRegistry:
    """
    Registry for mate features using path-based keys.

    Mates connect two parts/assemblies together, each referenced by their occurrence paths.
    We use a tuple of (parent_key, child_key) as the dictionary key for O(1) lookup.

    Attributes:
        mates: Maps (parent_PathKey, child_PathKey) tuple to MateFeatureData
        feature_id_to_mate: Maps feature ID to mate data for reverse lookup
        id_to_name: Reference to InstanceRegistry's ID mapping (shared)
    """

    # Primary storage: (PathKey, PathKey) -> MateFeatureData
    mates: dict[tuple[PathKey, PathKey], MateFeatureData]

    # Reverse lookup: feature ID -> MateFeatureData
    feature_id_to_mate: dict[str, MateFeatureData]

    # Shared name mapping (from InstanceRegistry)
    id_to_name: dict[str, str]

    def __init__(self, id_to_name: dict[str, str]):
        """
        Initialize mate registry.

        Args:
            id_to_name: Shared ID-to-name mapping from InstanceRegistry
        """
        self.mates = {}
        self.feature_id_to_mate = {}
        self.id_to_name = id_to_name

    def add_mate(self, feature: AssemblyFeature) -> Optional[tuple[PathKey, PathKey]]:
        """
        Add a mate feature to the registry.

        Args:
            feature: AssemblyFeature with featureType = MATE

        Returns:
            Tuple of (parent_key, child_key) if mate is valid, None otherwise

        Example:
            # From JSON assembly feature
            mate_keys = registry.add_mate(feature)
            # Returns: (PathKey(("PART_1",)), PathKey(("PART_2",)))
        """
        if feature.featureType != AssemblyFeatureType.MATE:
            LOGGER.warning(f"Feature {feature.id} is not a mate, skipping")
            return None

        if not isinstance(feature.featureData, MateFeatureData):
            LOGGER.warning(f"Feature {feature.id} has invalid mate data, skipping")
            return None

        mate_data = feature.featureData

        # Extract parent and child occurrence paths from mate
        if len(mate_data.matedEntities) < 2:
            LOGGER.warning(f"Mate {feature.id} has insufficient mated entities, skipping")
            return None

        parent_path = mate_data.matedEntities[PARENT].matedOccurrence
        child_path = mate_data.matedEntities[CHILD].matedOccurrence

        # Create PathKeys directly from occurrence paths
        parent_key = PathKey.from_path(parent_path)
        child_key = PathKey.from_path(child_path)

        # Store mate by key pair
        mate_key = (parent_key, child_key)
        self.mates[mate_key] = mate_data

        # Store reverse lookup by feature ID
        self.feature_id_to_mate[feature.id] = mate_data

        return mate_key

    def get_mate(self, parent_key: PathKey, child_key: PathKey) -> Optional[MateFeatureData]:
        """
        Get mate between two parts/assemblies.

        Args:
            parent_key: PathKey of parent
            child_key: PathKey of child

        Returns:
            MateFeatureData if found, None otherwise
        """
        return self.mates.get((parent_key, child_key))

    def get_mate_by_feature_id(self, feature_id: str) -> Optional[MateFeatureData]:
        """
        Get mate by feature ID (reverse lookup).

        Args:
            feature_id: Assembly feature ID

        Returns:
            MateFeatureData if found, None otherwise
        """
        return self.feature_id_to_mate.get(feature_id)

    def get_mates_for_part(self, key: PathKey) -> list[tuple[PathKey, MateFeatureData]]:
        """
        Get all mates involving a specific part (as parent or child).

        Args:
            key: PathKey to search for

        Returns:
            List of (other_key, mate_data) tuples

        Example:
            # Find all parts connected to PART_1
            mates = registry.get_mates_for_part(PathKey.from_path("PART_1"))
            # Returns: [(PathKey("PART_2"), mate_data), (PathKey("PART_3"), mate_data)]
        """
        result = []

        for (parent_key, child_key), mate_data in self.mates.items():
            if parent_key == key:
                result.append((child_key, mate_data))
            elif child_key == key:
                result.append((parent_key, mate_data))

        return result

    def build_from_features(self, features: list[AssemblyFeature]) -> None:
        """
        Build mate registry from assembly features.

        Args:
            features: List of AssemblyFeatures from RootAssembly or SubAssembly

        Example:
            registry = MateRegistry(instance_registry.id_to_name)
            registry.build_from_features(assembly.rootAssembly.features)
        """
        mate_count = 0
        for feature in features:
            if feature.featureType == AssemblyFeatureType.MATE and self.add_mate(feature):
                mate_count += 1

        LOGGER.debug(f"Added {mate_count} mates from {len(features)} features")

    def __len__(self) -> int:
        """Total number of mates."""
        return len(self.mates)

    def __repr__(self) -> str:
        return f"MateRegistry(mates={len(self.mates)})"


@dataclass
class PatternRegistry:
    """
    Registry for assembly patterns (circular, linear, mirror).

    Patterns create multiple instances from a seed instance. Each pattern maps
    seed instance IDs to lists of patterned instance IDs.

    Attributes:
        patterns: Maps pattern ID to Pattern
        seed_to_pattern: Maps seed instance ID to pattern ID (for reverse lookup)
        id_to_name: Reference to InstanceRegistry's ID mapping (shared)
    """

    # Primary storage: pattern ID -> Pattern
    patterns: dict[str, Pattern]

    # Reverse lookup: seed instance ID -> pattern ID
    seed_to_pattern: dict[str, str]

    # Shared name mapping (from InstanceRegistry)
    id_to_name: dict[str, str]

    def __init__(self, id_to_name: dict[str, str]):
        """
        Initialize pattern registry.

        Args:
            id_to_name: Shared ID-to-name mapping from InstanceRegistry
        """
        self.patterns = {}
        self.seed_to_pattern = {}
        self.id_to_name = id_to_name

    def add_pattern(self, pattern: Pattern) -> str:
        """
        Add a pattern to the registry.

        Args:
            pattern: Pattern from JSON

        Returns:
            Pattern ID

        Example:
            # From JSON: {"id": "PATTERN_1", "seedToPatternInstances": {"SEED_1": ["INST_1", "INST_2"]}}
            pattern_id = registry.add_pattern(pattern)
        """
        self.patterns[pattern.id] = pattern

        # Build reverse lookup: seed -> pattern
        for seed_id in pattern.seedToPatternInstances:
            self.seed_to_pattern[seed_id] = pattern.id

        return pattern.id

    def get_pattern(self, pattern_id: str) -> Optional[Pattern]:
        """Get pattern by ID."""
        return self.patterns.get(pattern_id)

    def get_pattern_for_seed(self, seed_id: str) -> Optional[Pattern]:
        """
        Get pattern containing a specific seed instance.

        Args:
            seed_id: Instance ID of seed

        Returns:
            Pattern if found, None otherwise
        """
        pattern_id = self.seed_to_pattern.get(seed_id)
        if pattern_id:
            return self.patterns.get(pattern_id)
        return None

    def get_patterned_instances(self, seed_id: str) -> list[str]:
        """
        Get list of patterned instance IDs for a seed.

        Args:
            seed_id: Instance ID of seed

        Returns:
            List of patterned instance IDs, or empty list if not found

        Example:
            # Seed "PART_1" creates pattern instances ["PART_2", "PART_3", "PART_4"]
            instances = registry.get_patterned_instances("PART_1")
            # Returns: ["PART_2", "PART_3", "PART_4"]
        """
        pattern = self.get_pattern_for_seed(seed_id)
        if pattern and seed_id in pattern.seedToPatternInstances:
            return pattern.seedToPatternInstances[seed_id]
        return []

    def is_seed_instance(self, instance_id: str) -> bool:
        """Check if an instance is a seed for a pattern."""
        return instance_id in self.seed_to_pattern

    def build_from_patterns(self, patterns: list[Pattern]) -> None:
        """
        Build pattern registry from assembly patterns.

        Args:
            patterns: List of Patterns from RootAssembly or SubAssembly

        Example:
            registry = PatternRegistry(instance_registry.id_to_name)
            registry.build_from_patterns(assembly.rootAssembly.patterns)
        """
        for pattern in patterns:
            if not pattern.suppressed:  # Only add active patterns
                self.add_pattern(pattern)

        LOGGER.debug(f"Added {len(self.patterns)} patterns")

    def __len__(self) -> int:
        """Total number of patterns."""
        return len(self.patterns)

    def __repr__(self) -> str:
        return f"PatternRegistry(patterns={len(self.patterns)})"


# ============================================================================
# Assembly Data Classes (Registry-Based System)
# ============================================================================


@dataclass
class AssemblyData:
    """
    Base assembly data with registries.

    Shared by both root assemblies and subassemblies.
    Contains instances, mates, and patterns.

    Note: Occurrences are NOT included because subassemblies
    in the JSON response don't have occurrences - only instances.

    Attributes:
        instances: Registry for all part and assembly instances
        mates: Registry for mate connections
        patterns: Registry for assembly patterns
    """

    instances: InstanceRegistry
    mates: MateRegistry
    patterns: PatternRegistry

    def __init__(self, max_depth: int = 0):
        """
        Initialize assembly data with empty registries.

        Args:
            max_depth: Maximum depth for flexible assembly classification
        """
        self.instances = InstanceRegistry(max_depth=max_depth)
        self.mates = MateRegistry(id_to_name=self.instances.id_to_name)
        self.patterns = PatternRegistry(id_to_name=self.instances.id_to_name)

    def populate_from_assembly(self, assembly: Assembly) -> None:
        """
        Populate registries from an Assembly object.

        Args:
            assembly: Assembly from Onshape API
        """
        self.instances.build_from_assembly(assembly=assembly)
        # Add mates
        if assembly.rootAssembly.features:
            for feature in assembly.rootAssembly.features:
                if feature.featureType == AssemblyFeatureType.MATE:
                    self.mates.add_mate(feature)

        # Add patterns
        if assembly.rootAssembly.patterns:
            for pattern in assembly.rootAssembly.patterns:
                self.patterns.add_pattern(pattern)

        LOGGER.debug(
            f"Populated AssemblyData: {len(self.instances.parts)} parts, "
            f"{len(self.instances.assemblies)} assemblies, "
            f"{len(self.mates.mates)} mates, "
            f"{len(self.patterns.patterns)} patterns"
        )

    def populate_from_subassembly(self, subassembly: SubAssembly, occurrence_registry: "OccurrenceRegistry") -> None:
        """
        Populate registries from a SubAssembly object.

        Note: SubAssemblies don't have occurrences in the JSON response, but we can
        use the occurrence registry to find the full paths for instances.

        Args:
            subassembly: SubAssembly from Onshape API
            occurrence_registry: Occurrence registry to find full paths
        """
        # Build a mapping of leaf IDs to full paths from occurrences
        leaf_id_to_paths: dict[str, list[PathKey]] = {}
        for path_key in occurrence_registry.occurrences:
            leaf_id = path_key.leaf
            if leaf_id not in leaf_id_to_paths:
                leaf_id_to_paths[leaf_id] = []
            leaf_id_to_paths[leaf_id].append(path_key)

        # Add instances using their full paths from occurrences
        for instance in subassembly.instances:
            # instance.id is just the leaf ID, we need to find the full path(s)
            matching_paths = leaf_id_to_paths.get(instance.id, [])

            for key in matching_paths:
                if instance.type == InstanceType.PART:
                    if isinstance(instance, PartInstance):
                        self.instances.add_part(key, instance)
                elif instance.type == InstanceType.ASSEMBLY and isinstance(instance, AssemblyInstance):
                    self.instances.add_assembly(key, instance)

        # Add mates
        if subassembly.features:
            for feature in subassembly.features:
                if feature.featureType == AssemblyFeatureType.MATE:
                    self.mates.add_mate(feature)

        # Add patterns
        if subassembly.patterns:
            for pattern in subassembly.patterns:
                self.patterns.add_pattern(pattern)

        LOGGER.debug(
            f"Populated SubAssembly: {len(self.instances.parts)} parts, "
            f"{len(self.instances.assemblies)} assemblies, "
            f"{len(self.mates.mates)} mates, "
            f"{len(self.patterns.patterns)} patterns"
        )

    def __repr__(self) -> str:
        return (
            f"AssemblyData("
            f"parts={len(self.instances.parts)}, "
            f"assemblies={len(self.instances.assemblies)}, "
            f"mates={len(self.mates.mates)}, "
            f"patterns={len(self.patterns.patterns)})"
        )


@dataclass
class RootAssemblyData(AssemblyData):
    """
    Root assembly data with occurrences.

    Only root assemblies (and subassemblies when fetched separately as their own assembly)
    have occurrence transforms. Subassemblies in the JSON response don't have occurrences.

    Attributes:
        occurrences: Registry for occurrence transforms
    """

    occurrences: OccurrenceRegistry

    def __init__(self, max_depth: int = 0):
        """
        Initialize root assembly data with empty registries.

        Args:
            max_depth: Maximum depth for flexible assembly classification
        """
        super().__init__(max_depth=max_depth)
        self.occurrences = OccurrenceRegistry(id_to_name=self.instances.id_to_name)

    def populate_from_assembly(self, assembly: Assembly) -> None:
        """
        Populate registries from a RootAssembly object (includes occurrences).

        Args:
            root_assembly: RootAssembly from Onshape API
        """
        # Populate base registries (instances, mates, patterns)
        super().populate_from_assembly(assembly)

        # Add occurrences (only root assemblies have these)
        for occurrence in assembly.rootAssembly.occurrences:
            self.occurrences.add_occurrence(occurrence)

        LOGGER.debug(f"Added {len(self.occurrences.occurrences)} occurrences to RootAssemblyData")

    def __repr__(self) -> str:
        return (
            f"RootAssemblyData("
            f"parts={len(self.instances.parts)}, "
            f"assemblies={len(self.instances.assemblies)}, "
            f"occurrences={len(self.occurrences.occurrences)}, "
            f"mates={len(self.mates.mates)}, "
            f"patterns={len(self.patterns.patterns)})"
        )


@dataclass
class CAD:
    """
    Top-level CAD document container.

    Represents a complete Onshape assembly with:
    - Root assembly data (instances, occurrences, mates, patterns) from rootAssembly only
    - Parts dictionary (shared across entire document)
    - Subassemblies from the JSON (stored with their own registries)
    - Fetched flexible subassemblies (recursively fetched as separate CAD documents)

    This structure maps to Onshape's assembly JSON schema:
    {
      "parts": [...],                    -> parts
      "rootAssembly": {...},             -> root_assembly
      "subAssemblies": [...]             -> sub_assemblies (keyed by PathKey)
    }

    Attributes:
        document_id: Onshape document ID
        element_id: Onshape element (assembly) ID
        document_microversion: Onshape document microversion (workspace ID)
        root_assembly: Root assembly data (only rootAssembly, not subassemblies)
        parts: Part definitions (shared across document)
        sub_assemblies: AssemblyData for each subassembly from JSON (keyed by PathKey)
        fetched_subassemblies: Nested subassembly CAD documents (fetched via API)
        max_depth: Maximum depth for flexible assemblies
        current_depth: Current depth in hierarchy (0 = root)
    """

    document_id: str
    element_id: str
    workspace_id: str
    document_microversion: str

    keys: dict[PathKey, Union[PartInstance, AssemblyInstance]]

    root_assembly: RootAssemblyData
    parts: dict[PathKey, Part]
    sub_assemblies: dict[PathKey, AssemblyData]
    fetched_subassemblies: dict[PathKey, "CAD"]
    max_depth: int
    current_depth: int

    def __init__(
        self,
        document_id: str,
        element_id: str,
        workspace_id: str,
        document_microversion: str,
        root_assembly: RootAssemblyData,
        parts: dict[PathKey, Part],
        max_depth: int = 0,
        current_depth: int = 0,
    ):
        """
        Initialize a CAD document.

        Args:
            document_id: Onshape document ID
            element_id: Onshape element (assembly) ID
            document_microversion: Onshape document microversion (workspace ID)
            root_assembly: Root assembly data
            parts: Part definitions keyed by PathKey
            max_depth: Maximum depth for flexible assemblies
            current_depth: Current depth in hierarchy
        """
        self.document_id = document_id
        self.element_id = element_id
        self.workspace_id = workspace_id
        self.document_microversion = document_microversion
        self.root_assembly = root_assembly
        self.parts = parts
        self.sub_assemblies = {}
        self.fetched_subassemblies = {}
        self.max_depth = max_depth
        self.current_depth = current_depth

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
            CAD with populated registries
        """
        # Create root assembly data and populate ONLY from rootAssembly
        root_assembly_data = RootAssemblyData(max_depth=max_depth)
        root_assembly_data.populate_from_assembly(assembly)

        # Create CAD instance (parts will be populated later)
        cad = cls(
            document_id=assembly.rootAssembly.documentId,
            element_id=assembly.rootAssembly.elementId,
            workspace_id=assembly.document.wid,
            document_microversion=assembly.rootAssembly.documentMicroversion,
            root_assembly=root_assembly_data,
            parts={},  # Will be populated below
            max_depth=max_depth,
            current_depth=current_depth,
        )

        # Populate subassemblies separately (adds instances to root_assembly)
        # We need the occurrence registry to find full paths for subassembly instances
        for subassembly in assembly.subAssemblies:
            cad._populate_subassembly(subassembly)

        # Populate parts (both regular parts and rigid subassemblies)
        cad._populate_parts(assembly)

        return cad

    def _populate_subassembly(self, subassembly: SubAssembly) -> None:
        """
        Populate a subassembly's data into both root instances and sub_assemblies registry.

        This does two things:
        1. Adds instances to the root_assembly.instances (for flat access to all instances)
        2. Creates an AssemblyData with mates/patterns for the subassembly (hierarchical structure)

        Args:
            subassembly: SubAssembly from Onshape JSON
        """
        # Create AssemblyData for this subassembly (for mates/patterns)
        assembly_data = AssemblyData(max_depth=self.max_depth)

        # Build a mapping of leaf IDs to full paths from occurrences
        leaf_id_to_paths: dict[str, list[PathKey]] = {}
        for path_key in self.root_assembly.occurrences.occurrences:
            leaf_id = path_key.leaf
            if leaf_id not in leaf_id_to_paths:
                leaf_id_to_paths[leaf_id] = []
            leaf_id_to_paths[leaf_id].append(path_key)

        # Add instances to BOTH registries
        for instance in subassembly.instances:
            # instance.id is just the leaf ID, we need to find the full path(s)
            matching_paths = leaf_id_to_paths.get(instance.id, [])

            for key in matching_paths:
                if instance.type == InstanceType.PART:
                    if isinstance(instance, PartInstance):
                        # Add to root assembly instances (flat registry)
                        self.root_assembly.instances.add_part(key, instance)
                        # Also add to subassembly's own registry
                        assembly_data.instances.add_part(key, instance)
                elif instance.type == InstanceType.ASSEMBLY and isinstance(instance, AssemblyInstance):
                    # Mark as rigid if depth exceeds max_depth
                    if key.depth > self.max_depth:
                        instance.isRigid = True
                        LOGGER.debug(f"Marked {instance.name} at {key} (depth={key.depth}) as rigid")

                    # Add to root assembly instances (flat registry)
                    self.root_assembly.instances.add_assembly(key, instance)
                    # Also add to subassembly's own registry
                    assembly_data.instances.add_assembly(key, instance)

        # Add mates (only to subassembly, not root)
        if subassembly.features:
            for feature in subassembly.features:
                if feature.featureType == AssemblyFeatureType.MATE:
                    assembly_data.mates.add_mate(feature)

        # Add patterns (only to subassembly, not root)
        if subassembly.patterns:
            for pattern in subassembly.patterns:
                assembly_data.patterns.add_pattern(pattern)

        # Find the PathKey(s) for this subassembly in the assembly instances
        # The subassembly's elementId should match an assembly instance's elementId
        for key, instance in self.root_assembly.instances.assemblies.items():
            if instance.elementId == subassembly.elementId:
                self.sub_assemblies[key] = assembly_data
                LOGGER.debug(
                    f"Populated subassembly {instance.name} at {key}: "
                    f"{len(assembly_data.instances.parts)} parts, "
                    f"{len(assembly_data.instances.assemblies)} assemblies, "
                    f"{len(assembly_data.mates.mates)} mates"
                )

    def _populate_parts(self, assembly: Assembly) -> None:
        """
        Populate the parts dictionary with both regular parts and rigid subassemblies.

        Rigid subassemblies are treated as parts because they are downloaded as single
        STL files, just like regular parts.

        Args:
            assembly: Assembly from Onshape JSON
        """
        # Populate regular parts keyed by PathKey
        # Match part instances with their Part definitions
        part_id_to_part = {part.partId: part for part in assembly.parts}
        for key, instance in self.root_assembly.instances.parts.items():
            if instance.partId in part_id_to_part:
                self.parts[key] = part_id_to_part[instance.partId]

        # Also create Part objects for rigid subassemblies
        # Rigid subassemblies are downloaded as single STL files, just like parts
        rigid_count = 0
        for key, assembly_instance in self.root_assembly.instances.assemblies.items():
            if isinstance(assembly_instance, AssemblyInstance) and assembly_instance.isRigid:
                # Find the matching subassembly data
                matching_subassembly = None
                for subassembly in assembly.subAssemblies:
                    if subassembly.elementId == assembly_instance.elementId:
                        matching_subassembly = subassembly
                        break

                if matching_subassembly:
                    # Create a Part object for this rigid subassembly
                    # Use the subassembly's data (where assembly is defined)
                    # not the instance's data (where instance was created)
                    #
                    # For rigidAssemblyWorkspaceId: We use documentMicroversion as workspace ID
                    # because SubAssembly doesn't include documentMetaData. This works for
                    # assemblies in the same document. For cross-document assemblies, we would
                    # need to fetch the full RootAssembly to get the actual workspace ID.
                    self.parts[key] = Part(
                        isStandardContent=False,
                        fullConfiguration=matching_subassembly.fullConfiguration or "",
                        configuration=matching_subassembly.configuration or "",
                        documentId=matching_subassembly.documentId,
                        elementId=matching_subassembly.elementId,
                        documentMicroversion=matching_subassembly.documentMicroversion,
                        documentVersion="",
                        partId="",  # Rigid assemblies don't have a partId
                        bodyType="",
                        MassProperty=None,
                        isRigidAssembly=True,
                        rigidAssemblyToPartTF={},
                        rigidAssemblyWorkspaceId=self.workspace_id,
                    )
                    rigid_count += 1
                    LOGGER.debug(f"Created Part object for rigid subassembly {assembly_instance.name} at {key}")

        if rigid_count > 0:
            LOGGER.debug(f"Created {rigid_count} Part objects for rigid subassemblies")

    async def fetch_subassemblies(self, client: Client) -> None:
        """
        Recursively fetch all flexible subassembly CAD documents.

        This method:
        1. Identifies flexible subassembly instances
        2. Fetches their assembly data from Onshape API
        3. Creates nested CAD objects
        4. Stores them in self.fetched_subassemblies keyed by PathKey

        Args:
            client: Onshape API client for fetching assemblies
        """
        if self.current_depth >= self.max_depth:
            LOGGER.debug(f"Max depth {self.max_depth} reached, skipping subassembly fetch")
            return

        # Get flexible assembly instances (ones we want to fetch)
        flexible_assemblies = [
            (key, instance)
            for key, instance in self.root_assembly.instances.assemblies.items()
            if self.root_assembly.instances.is_flexible_assembly(key)
        ]

        if not flexible_assemblies:
            LOGGER.debug("No flexible subassemblies to fetch")
            return

        LOGGER.info(f"Fetching {len(flexible_assemblies)} subassemblies at depth {self.current_depth}")

        # Fetch each subassembly
        tasks = []
        for key, instance in flexible_assemblies:
            tasks.append(self._fetch_single_subassembly(client, key, instance))

        # Wait for all fetches to complete
        await asyncio.gather(*tasks)

        LOGGER.info(f"Fetched {len(self.fetched_subassemblies)} subassemblies at depth {self.current_depth}")

    async def _fetch_single_subassembly(self, client: Client, key: PathKey, instance: AssemblyInstance) -> None:
        """
        Fetch a single subassembly.

        Args:
            client: Onshape API client
            key: PathKey of the subassembly instance
            instance: AssemblyInstance to fetch
        """
        try:
            LOGGER.debug(f"Fetching subassembly {instance.name} (depth {self.current_depth + 1})")

            # Determine workspace type and fetch assembly
            if instance.documentVersion:
                assembly_data = client.get_assembly(
                    did=instance.documentId,
                    wtype=WorkspaceType.V,
                    wid=instance.documentVersion,
                    eid=instance.elementId,
                )
            elif instance.documentMicroversion:
                assembly_data = client.get_assembly(
                    did=instance.documentId,
                    wtype=WorkspaceType.M,
                    wid=instance.documentMicroversion,
                    eid=instance.elementId,
                )
            else:
                LOGGER.warning(f"Subassembly {instance.name} has no version info, skipping")
                return

            # Create nested CAD
            subassembly_cad = CAD.from_assembly(
                assembly=assembly_data,
                max_depth=self.max_depth,
                current_depth=self.current_depth + 1,
            )

            # Recursively fetch nested subassemblies
            await subassembly_cad.fetch_subassemblies(client)

            # Store in fetched_subassemblies dict
            self.fetched_subassemblies[key] = subassembly_cad

            LOGGER.debug(f"Successfully fetched subassembly {instance.name}")

        except Exception as e:
            LOGGER.error(f"Failed to fetch subassembly {instance.name}: {e}")

    def get_subassembly(self, key: PathKey) -> Optional["CAD"]:
        """
        Get a fetched subassembly CAD document by PathKey.

        Args:
            key: PathKey of the subassembly instance

        Returns:
            CAD if found, None otherwise
        """
        return self.fetched_subassemblies.get(key)

    def get_all_subassemblies(self, recursive: bool = False) -> dict[PathKey, "CAD"]:
        """
        Get all fetched subassemblies.

        Args:
            recursive: If True, includes nested subassemblies recursively

        Returns:
            Dictionary of PathKey -> CAD
        """
        if not recursive:
            return self.fetched_subassemblies

        # Recursively collect all subassemblies
        all_subs = dict(self.fetched_subassemblies)
        for sub_cad in self.fetched_subassemblies.values():
            nested = sub_cad.get_all_subassemblies(recursive=True)
            all_subs.update(nested)

        return all_subs

    def get_part(self, key: PathKey) -> Optional[Part]:
        """
        Get a part definition by PathKey.

        Args:
            key: PathKey of the part instance

        Returns:
            Part if found, None otherwise
        """
        return self.parts.get(key)

    def get_part_by_id(self, part_id: str) -> list[tuple[PathKey, Part]]:
        """
        Get all part instances with a given partId.

        Since the same part can appear multiple times (different instances),
        this returns a list of (PathKey, Part) tuples.

        Args:
            part_id: Onshape part ID

        Returns:
            List of (PathKey, Part) tuples for matching parts
        """
        return [(key, part) for key, part in self.parts.items() if part.partId == part_id]

    def get_part_instance(self, key: PathKey) -> Optional[PartInstance]:
        """
        Get a part instance by PathKey.

        Args:
            key: PathKey of the part instance

        Returns:
            PartInstance if found, None otherwise
        """
        return self.root_assembly.instances.get_part(key)

    def get_assembly_instance(self, key: PathKey) -> Optional[AssemblyInstance]:
        """
        Get an assembly instance by PathKey.

        Args:
            key: PathKey of the assembly instance

        Returns:
            AssemblyInstance if found, None otherwise
        """
        return self.root_assembly.instances.get_assembly(key)

    def get_occurrence(self, key: PathKey) -> Optional[Occurrence]:
        """
        Get an occurrence by PathKey.

        Args:
            key: PathKey of the occurrence

        Returns:
            Occurrence if found, None otherwise
        """
        return self.root_assembly.occurrences.get_occurrence(key)

    def get_transform(self, key: PathKey) -> Optional[np.ndarray]:
        """
        Get the 4x4 transform matrix for an occurrence.

        Args:
            key: PathKey of the occurrence

        Returns:
            4x4 numpy array if found, None otherwise
        """
        return self.root_assembly.occurrences.get_transform(key)

    def get_mate(self, parent_key: PathKey, child_key: PathKey) -> Optional[MateFeatureData]:
        """
        Get a mate between two parts/assemblies from root assembly only.

        Args:
            parent_key: PathKey of the parent entity
            child_key: PathKey of the child entity

        Returns:
            MateFeatureData if found, None otherwise
        """
        return self.root_assembly.mates.get_mate(parent_key, child_key)

    def get_all_mates(self, include_subassemblies: bool = True) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Get all mates from root assembly and optionally subassemblies.

        Args:
            include_subassemblies: If True, includes mates from all subassemblies

        Returns:
            Dictionary mapping (parent_key, child_key) tuples to MateFeatureData
        """
        all_mates = dict(self.root_assembly.mates.mates)

        if include_subassemblies:
            for assembly_data in self.sub_assemblies.values():
                all_mates.update(assembly_data.mates.mates)

        return all_mates

    def get_subassembly_data(self, key: PathKey) -> Optional[AssemblyData]:
        """
        Get the AssemblyData for a subassembly by PathKey.

        Args:
            key: PathKey of the subassembly instance

        Returns:
            AssemblyData if found, None otherwise
        """
        return self.sub_assemblies.get(key)

    def get_pattern(self, pattern_id: str) -> Optional[Pattern]:
        """
        Get a pattern by its ID.

        Args:
            pattern_id: Pattern ID

        Returns:
            Pattern if found, None otherwise
        """
        return self.root_assembly.patterns.get_pattern(pattern_id)

    def show(self) -> None:
        """Display the CAD assembly structure as a tree."""
        from collections import defaultdict

        # Build hierarchy: parent_path -> list of child keys
        children_by_path: dict[tuple[str, ...], list[PathKey]] = defaultdict(list)

        # Collect all instances
        for key in self.root_assembly.instances.parts:
            parent_path = key.path[:-1]
            children_by_path[parent_path].append(key)

        for key in self.root_assembly.instances.assemblies:
            parent_path = key.path[:-1]
            children_by_path[parent_path].append(key)

        def get_display_name(key: PathKey) -> str:
            """Get display name for a key."""
            part = self.root_assembly.instances.parts.get(key)
            if part:
                return f"{part.name} (part)"

            assembly = self.root_assembly.instances.assemblies.get(key)
            if assembly:
                assembly_type = "rigid" if self.root_assembly.instances.is_rigid_assembly(key) else "flexible"
                return f"{assembly.name} ({assembly_type})"

            return f"{key.leaf} (unknown)"

        def print_tree(path: tuple[str, ...], depth: int = 0) -> None:
            """Recursively print tree structure."""
            prefix = "    " * depth

            if depth == 0:
                print(f"{prefix}Assembly Root")

            # Get children at this level
            children = children_by_path.get(path, [])

            # Sort: parts first, then assemblies; within each group, sort by name
            sorted_children = sorted(
                children,
                key=lambda k: (
                    0 if k in self.root_assembly.instances.parts else 1,  # Parts first
                    (
                        part.name
                        if (part := self.root_assembly.instances.parts.get(k)) is not None
                        else (
                            asm.name if (asm := self.root_assembly.instances.assemblies.get(k)) is not None else k.leaf
                        )
                    ),
                ),
            )

            for key in sorted_children:
                print(f"{prefix}|-- {get_display_name(key)}")

                # Check if this key has a fetched subassembly
                if key in self.fetched_subassemblies:
                    sub_cad = self.fetched_subassemblies[key]
                    print(f"{prefix}|   +-- Fetched Subassembly Document (depth={sub_cad.current_depth})")

                # Recurse to children
                print_tree(key.path, depth + 1)

        print_tree(())

    def __repr__(self) -> str:
        return (
            f"CAD("
            f"depth={self.current_depth}, "
            f"parts={len(self.root_assembly.instances.parts)}, "
            f"assemblies={len(self.root_assembly.instances.assemblies)}, "
            f"sub_assemblies={len(self.sub_assemblies)}, "
            f"fetched_subassemblies={len(self.fetched_subassemblies)})"
        )

    def process_mates_and_relations(self) -> None:
        """
        Process all mates and relations in this CAD assembly.

        This method:
        1. Filters out mates that are completely within rigid subassemblies (beyond max_depth)
        2. Updates PathKeys and transforms for mates involving parts in rigid subassemblies
        3. Expands pattern instances to create mates for all pattern copies
        4. Ensures all mate names are unique for valid URDF generation

        The processing is done using PathKey-based lookups which eliminates the need
        for complex string concatenation and proxy mapping used in the legacy system.

        Important: Mates are processed in place, updating both:
        - The PathKeys in the mate registry (to point to rigid subassembly roots)
        - The MateFeatureData transforms (matedCS) to be in rigid subassembly coordinates
        """
        # Step 1: Filter and remap mates in root assembly
        self._filter_and_remap_mates(
            self.root_assembly.mates,
            self.root_assembly.instances,
            self.root_assembly.occurrences,
        )

        # Step 2: Filter and remap mates in each subassembly (only flexible ones)
        for sub_key, sub_assembly_data in self.sub_assemblies.items():
            # Only process flexible subassemblies (depth <= max_depth)
            if sub_key.depth <= self.max_depth:
                LOGGER.debug(f"Processing mates for flexible subassembly at {sub_key}")
                self._filter_and_remap_mates(
                    sub_assembly_data.mates,
                    sub_assembly_data.instances,
                    self.root_assembly.occurrences,  # Use root occurrences for transforms
                )

        # Step 3: Expand patterns for all assemblies
        self._expand_all_patterns()

        # Step 4: Ensure all mate names are unique across all assemblies
        self._ensure_unique_mate_names()

    def _filter_and_remap_mates(
        self,
        mate_registry: MateRegistry,
        instance_registry: InstanceRegistry,
        occurrence_registry: OccurrenceRegistry,
    ) -> None:
        """
        Filter and remap mates based on rigid subassembly boundaries.

        This method:
        1. Removes mates that are completely within rigid subassemblies (both entities buried)
        2. Remaps mate PathKeys to rigid subassembly roots when entities are buried
        3. Transforms mate coordinates to rigid subassembly coordinate systems

        Args:
            mate_registry: Mate registry to process (modified in place)
            instance_registry: Instance registry for rigid assembly detection
            occurrence_registry: Occurrence registry for transforms
        """
        # Identify all rigid assembly keys
        rigid_assembly_keys = {key for key in instance_registry.assemblies if instance_registry.is_rigid_assembly(key)}

        if not rigid_assembly_keys:
            LOGGER.debug("No rigid assemblies found, skipping mate filtering")
            return

        # Process mates: filter and remap
        mates_to_remove: list[tuple[PathKey, PathKey]] = []
        mates_to_add: dict[tuple[PathKey, PathKey], MateFeatureData] = {}

        for (parent_key, child_key), mate_data in list(mate_registry.mates.items()):
            # Find rigid assembly roots for parent and child
            parent_rigid_root = self._find_rigid_assembly_root(parent_key, rigid_assembly_keys)
            child_rigid_root = self._find_rigid_assembly_root(child_key, rigid_assembly_keys)

            # Case 1: Both entities are within the SAME rigid subassembly
            # These mates are internal to the rigid assembly and should be removed
            if parent_rigid_root and child_rigid_root and parent_rigid_root == child_rigid_root:
                LOGGER.debug(
                    f"Removing mate {mate_data.name}: both entities within rigid subassembly {parent_rigid_root}"
                )
                mates_to_remove.append((parent_key, child_key))
                continue

            # Case 2: One or both entities need to be remapped to rigid root
            new_parent_key = parent_rigid_root if parent_rigid_root and parent_key != parent_rigid_root else parent_key
            new_child_key = child_rigid_root if child_rigid_root and child_key != child_rigid_root else child_key

            # If keys changed, we need to remap the mate
            if new_parent_key != parent_key or new_child_key != child_key:
                # Transform mate coordinates to rigid subassembly coordinate system
                self._transform_mate_to_rigid_coords(
                    mate_data, parent_key, new_parent_key, child_key, new_child_key, occurrence_registry
                )

                # Remove old mate and add with new keys
                mates_to_remove.append((parent_key, child_key))
                mates_to_add[(new_parent_key, new_child_key)] = mate_data

                LOGGER.debug(
                    f"Remapped mate {mate_data.name}: "
                    f"({parent_key}, {child_key}) -> ({new_parent_key}, {new_child_key})"
                )

        # Apply changes to registry
        for key in mates_to_remove:
            if key in mate_registry.mates:
                del mate_registry.mates[key]

        mate_registry.mates.update(mates_to_add)

        LOGGER.debug(f"Filtered {len(mates_to_remove) - len(mates_to_add)} mates, remapped {len(mates_to_add)} mates")

    def _transform_mate_to_rigid_coords(
        self,
        mate_data: MateFeatureData,
        old_parent_key: PathKey,
        new_parent_key: PathKey,
        old_child_key: PathKey,
        new_child_key: PathKey,
        occurrence_registry: OccurrenceRegistry,
    ) -> None:
        """
        Transform mate coordinate systems when entities are remapped to rigid subassembly roots.

        Args:
            mate_data: Mate feature data to modify
            old_parent_key: Original parent PathKey (possibly buried in rigid assembly)
            new_parent_key: New parent PathKey (rigid assembly root)
            old_child_key: Original child PathKey (possibly buried in rigid assembly)
            new_child_key: New child PathKey (rigid assembly root)
            occurrence_registry: Occurrence registry for transforms
        """
        # Transform parent if needed (PARENT is at index 1)
        if old_parent_key != new_parent_key:
            # Build transform from rigid root to buried part
            hierarchical_tf = np.matrix(
                np.eye(4)
            )  # self._build_hierarchical_transform(old_parent_key, new_parent_key, occurrence_registry)
            if hierarchical_tf is not None:
                # Get current mate CS
                current_mate_cs = mate_data.matedEntities[PARENT].matedCS

                # Transform: new_mate_tf = hierarchical_tf @ current_mate_tf
                new_mate_tf = hierarchical_tf @ current_mate_cs.part_to_mate_tf

                # Update mate CS
                mate_data.matedEntities[PARENT].matedCS = MatedCS.from_tf(new_mate_tf)

            # Update occurrence path to reflect new key (use full path, not just leaf)
            mate_data.matedEntities[PARENT].matedOccurrence = list(new_parent_key.path)

        # Transform child if needed (CHILD is at index 0)
        if old_child_key != new_child_key:
            # Build transform from rigid root to buried part
            hierarchical_tf = np.matrix(
                np.eye(4)
            )  # self._build_hierarchical_transform(old_child_key, new_child_key, occurrence_registry)
            if hierarchical_tf is not None:
                # Get current mate CS
                current_mate_cs = mate_data.matedEntities[CHILD].matedCS

                # Transform: new_mate_tf = hierarchical_tf @ current_mate_tf
                new_mate_tf = hierarchical_tf @ current_mate_cs.part_to_mate_tf

                # Update mate CS
                mate_data.matedEntities[CHILD].matedCS = MatedCS.from_tf(new_mate_tf)

            # Update occurrence path to reflect new key (use full path, not just leaf)
            mate_data.matedEntities[CHILD].matedOccurrence = list(new_child_key.path)

    def _expand_all_patterns(self) -> None:
        """Expand patterns for root assembly and all flexible subassemblies."""
        # Expand root assembly patterns
        self._expand_pattern_mates(
            self.root_assembly.mates,
            self.root_assembly.patterns,
            self.root_assembly.instances,
            self.root_assembly.occurrences,
        )

        # Expand subassembly patterns (only flexible ones)
        for sub_key, sub_assembly_data in self.sub_assemblies.items():
            if sub_key.depth <= self.max_depth:
                self._expand_pattern_mates(
                    sub_assembly_data.mates,
                    sub_assembly_data.patterns,
                    sub_assembly_data.instances,
                    self.root_assembly.occurrences,
                )

    def _expand_pattern_mates(
        self,
        mate_registry: MateRegistry,
        pattern_registry: PatternRegistry,
        instance_registry: InstanceRegistry,
        occurrence_registry: OccurrenceRegistry,
    ) -> None:
        """
        Expand mates for pattern instances.

        For each mate that connects to a patterned entity, creates additional mates
        for all pattern instances with properly calculated transforms.

        Args:
            mate_registry: Mate registry to update
            pattern_registry: Pattern registry with seed-to-pattern mappings
            instance_registry: Instance registry for lookups
            occurrence_registry: Occurrence registry for transforms
        """
        # Collect new mates to add (can't modify dict during iteration)
        new_mates: dict[tuple[PathKey, PathKey], MateFeatureData] = {}

        # Iterate over existing mates
        for (parent_key, child_key), mate_data in list(mate_registry.mates.items()):
            # Check if either parent or child is a seed in a pattern
            parent_leaf_id = parent_key.leaf
            child_leaf_id = child_key.leaf

            parent_pattern = pattern_registry.get_pattern_for_seed(parent_leaf_id)
            child_pattern = pattern_registry.get_pattern_for_seed(child_leaf_id)

            # Skip if neither is in a pattern
            if not parent_pattern and not child_pattern:
                continue

            # Skip if both are in patterns (unsupported scenario)
            if parent_pattern and child_pattern:
                LOGGER.warning(
                    f"Mate {mate_data.name} has both parent and child in patterns. "
                    f"This scenario is not supported. Skipping pattern expansion."
                )
                continue

            # Determine which entity is patterned
            if child_pattern:
                self._expand_mate_for_pattern(
                    mate_data, child_key, parent_key, child_pattern, CHILD, occurrence_registry, new_mates
                )
            elif parent_pattern:
                self._expand_mate_for_pattern(
                    mate_data, parent_key, child_key, parent_pattern, PARENT, occurrence_registry, new_mates
                )

        # Add all new mates to the registry
        mate_registry.mates.update(new_mates)
        LOGGER.debug(f"Expanded {len(new_mates)} pattern mates")

    def _expand_mate_for_pattern(
        self,
        seed_mate: MateFeatureData,
        pattern_entity_key: PathKey,
        other_entity_key: PathKey,
        pattern: Pattern,
        pattern_entity_index: int,
        occurrence_registry: OccurrenceRegistry,
        output_mates: dict[tuple[PathKey, PathKey], MateFeatureData],
    ) -> None:
        """
        Expand a single mate for all instances in a pattern.

        Args:
            seed_mate: Original mate feature data
            pattern_entity_key: PathKey of the patterned entity (seed)
            other_entity_key: PathKey of the non-patterned entity
            pattern: Pattern object containing instance IDs
            pattern_entity_index: Index of patterned entity (CHILD or PARENT)
            occurrence_registry: Occurrence registry for transforms
            output_mates: Dictionary to store new mates
        """

        # Get pattern instances for this seed
        seed_leaf_id = pattern_entity_key.leaf
        pattern_instance_ids = pattern.seedToPatternInstances.get(seed_leaf_id, [])

        if not pattern_instance_ids:
            return

        # Get transforms
        seed_tf = occurrence_registry.get_transform(pattern_entity_key)
        other_tf = occurrence_registry.get_transform(other_entity_key)

        if seed_tf is None or other_tf is None:
            LOGGER.warning(
                f"Missing occurrence for pattern expansion: seed={pattern_entity_key}, other={other_entity_key}"
            )
            return

        # Get original mate coordinate system
        original_mate_cs = seed_mate.matedEntities[pattern_entity_index].matedCS

        # Create a mate for each pattern instance
        for pattern_instance_id in pattern_instance_ids:
            # Find the PathKey for this pattern instance
            # Pattern instance IDs are leaf IDs, need to find full path
            pattern_instance_key = self._find_instance_key_by_leaf_id(pattern_instance_id, self.root_assembly.instances)

            if not pattern_instance_key:
                LOGGER.warning(f"Could not find PathKey for pattern instance {pattern_instance_id}")
                continue

            # Get pattern instance transform
            pattern_instance_tf = occurrence_registry.get_transform(pattern_instance_key)
            if pattern_instance_tf is None:
                LOGGER.warning(f"No occurrence found for pattern instance {pattern_instance_key}")
                continue

            # Calculate relative transform: seed → pattern_instance
            seed_to_pattern_tf = pattern_instance_tf @ np.linalg.inv(seed_tf)

            # Clean up floating point errors
            tolerance = 1e-10
            seed_to_pattern_tf[np.abs(seed_to_pattern_tf) < tolerance] = 0.0

            # Transform mate coordinates: local → world → pattern_relative → local
            other_mate_world_tf = other_tf @ original_mate_cs.part_to_mate_tf
            pattern_transformed_world_tf = seed_to_pattern_tf @ other_mate_world_tf
            final_mate_tf = np.linalg.inv(other_tf) @ pattern_transformed_world_tf

            # Clean up final transform
            final_mate_tf[np.abs(final_mate_tf) < tolerance] = 0.0

            # Create new mate feature data
            new_mate = copy.deepcopy(seed_mate)

            # Update the patterned entity's mate CS
            new_mate.matedEntities[pattern_entity_index].matedCS = MatedCS.from_tf(final_mate_tf)
            new_mate.matedEntities[pattern_entity_index].matedOccurrence = [pattern_instance_id]

            # Create mate key with correct parent/child order
            if pattern_entity_index == CHILD:
                new_mate_key = (other_entity_key, pattern_instance_key)
            else:  # pattern_entity_index == PARENT
                new_mate_key = (pattern_instance_key, other_entity_key)

            output_mates[new_mate_key] = new_mate

    def _find_instance_key_by_leaf_id(self, leaf_id: str, instance_registry: InstanceRegistry) -> Optional[PathKey]:
        """Find PathKey for an instance by its leaf ID."""
        # Check parts
        for key in instance_registry.parts:
            if key.leaf == leaf_id:
                return key
        # Check assemblies
        for key in instance_registry.assemblies:
            if key.leaf == leaf_id:
                return key
        return None

    def _find_rigid_assembly_root(self, key: PathKey, rigid_assembly_keys: set[PathKey]) -> Optional[PathKey]:
        """
        Find the rigid assembly root for a given key.

        Checks if any ancestor of the key is a rigid assembly.

        Args:
            key: PathKey to check
            rigid_assembly_keys: Set of all rigid assembly PathKeys

        Returns:
            PathKey of the rigid assembly root if found, None otherwise
        """
        # Check each ancestor path
        for i in range(1, len(key.path)):
            ancestor_key = PathKey(key.path[:i])
            if ancestor_key in rigid_assembly_keys:
                return ancestor_key
        return None

    def _build_hierarchical_transform(
        self,
        part_key: PathKey,
        rigid_root_key: PathKey,
        occurrence_registry: OccurrenceRegistry,
    ) -> Optional[np.matrix]:
        """
        Build hierarchical transform from rigid assembly root to part.

        Args:
            part_key: PathKey of the part
            rigid_root_key: PathKey of the rigid assembly root
            occurrence_registry: Occurrence registry for transforms

        Returns:
            4x4 transform matrix (np.matrix) or None if transforms not found
        """
        # Start with identity
        parent_tf = np.eye(4)

        # Build path from rigid root to part
        # rigid_root_key.path is like ('rigid_sub',)
        # part_key.path is like ('rigid_sub', 'level1', 'level2', 'part')
        # We need to multiply transforms for 'level1', 'level2' (exclude rigid root and final part)

        rigid_depth = len(rigid_root_key.path)

        # Iterate through intermediate levels (excluding the part itself)
        for i in range(rigid_depth, len(part_key.path)):
            level_key = PathKey(part_key.path[: i + 1])
            level_tf = occurrence_registry.get_transform(level_key)

            if level_tf is not None:
                parent_tf = parent_tf @ level_tf

        # Convert to matrix and return if not identity
        if not np.allclose(parent_tf, np.eye(4)):
            return np.matrix(parent_tf)
        return None

    def _ensure_unique_mate_names(self) -> None:
        """
        Ensure all mate names are unique across root and subassemblies.

        Pattern expansion can create duplicate mate names, which makes URDF invalid.
        This function resolves naming conflicts by renaming duplicates.
        """
        # Collect all unique mate objects (deduplicate by object ID)
        # Some mates may be shared across assemblies, so we need to dedupe
        seen_ids: set[int] = set()
        unique_mates: list[MateFeatureData] = []

        # Add root assembly mates
        for mate_data in self.root_assembly.mates.mates.values():
            mate_id = id(mate_data)
            if mate_id not in seen_ids:
                unique_mates.append(mate_data)
                seen_ids.add(mate_id)

        # Add subassembly mates
        for sub_data in self.sub_assemblies.values():
            for mate_data in sub_data.mates.mates.values():
                mate_id = id(mate_data)
                if mate_id not in seen_ids:
                    unique_mates.append(mate_data)
                    seen_ids.add(mate_id)

        # Now ensure all unique mate names are unique
        used_names: set[str] = set()
        renamed_count = 0

        for mate in unique_mates:
            if mate.name in used_names:
                # This name is already used, find a unique variant
                base_name = mate.name
                counter = 2
                new_name = f"{base_name}_{counter}"

                while new_name in used_names:
                    counter += 1
                    new_name = f"{base_name}_{counter}"

                mate.name = new_name
                renamed_count += 1

            used_names.add(mate.name)

        if renamed_count > 0:
            LOGGER.debug(f"Renamed {renamed_count} mates to ensure uniqueness")


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
