"""
This module contains functions that provide a way to traverse the assembly structure, extract information about parts,
subassemblies, instances, and mates, and generate a hierarchical representation of the assembly.

"""

import asyncio
import copy
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional, Union, cast

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

    @property
    def path(self) -> tuple[str, ...]:
        """Get the immutable path tuple."""
        return self._path

    @property
    def leaf_id(self) -> str:
        """Get the last ID in the path (the actual entity)."""
        return self._path[-1] if self._path else ""

    @property
    def parent_key(self) -> Optional["PathKey"]:
        """Get parent PathKey by trimming last element."""
        if len(self._path) <= 1:
            return None
        return PathKey(self._path[:-1])

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

    def get_hierarchical_name(self, key: PathKey, separator: str = ":") -> str:
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

    def populate_from_root_assembly(self, root_assembly: RootAssembly) -> None:
        """
        Populate registries from a RootAssembly object.

        Args:
            root_assembly: RootAssembly from Onshape API
        """
        # Add instances
        for instance in root_assembly.instances:
            key = PathKey.from_path(instance.id)

            if instance.type == InstanceType.PART:
                if isinstance(instance, PartInstance):
                    self.instances.add_part(key, instance)
            elif instance.type == InstanceType.ASSEMBLY and isinstance(instance, AssemblyInstance):
                # Mark as rigid if depth exceeds max_depth
                if key.depth > self.instances.max_depth:
                    instance.isRigid = True
                    LOGGER.debug(f"Marked {instance.name} at {key} (depth={key.depth}) as rigid")
                self.instances.add_assembly(key, instance)

        # Add mates
        if root_assembly.features:
            for feature in root_assembly.features:
                if feature.featureType == AssemblyFeatureType.MATE:
                    self.mates.add_mate(feature)

        # Add patterns
        if root_assembly.patterns:
            for pattern in root_assembly.patterns:
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
            leaf_id = path_key.leaf_id
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

    def populate_from_root_assembly(self, root_assembly: RootAssembly) -> None:
        """
        Populate registries from a RootAssembly object (includes occurrences).

        Args:
            root_assembly: RootAssembly from Onshape API
        """
        # Populate base registries (instances, mates, patterns)
        super().populate_from_root_assembly(root_assembly)

        # Add occurrences (only root assemblies have these)
        for occurrence in root_assembly.occurrences:
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
        root_assembly: Root assembly data (only rootAssembly, not subassemblies)
        parts: Part definitions (shared across document)
        sub_assemblies: AssemblyData for each subassembly from JSON (keyed by PathKey)
        fetched_subassemblies: Nested subassembly CAD documents (fetched via API)
        max_depth: Maximum depth for flexible assemblies
        current_depth: Current depth in hierarchy (0 = root)
    """

    document_id: str
    element_id: str
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
            root_assembly: Root assembly data
            parts: Part definitions keyed by PathKey
            max_depth: Maximum depth for flexible assemblies
            current_depth: Current depth in hierarchy
        """
        self.document_id = document_id
        self.element_id = element_id
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
        root_assembly_data.populate_from_root_assembly(assembly.rootAssembly)

        # Create CAD instance (parts will be populated later)
        cad = cls(
            document_id=assembly.rootAssembly.documentId,
            element_id=assembly.rootAssembly.elementId,
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
            leaf_id = path_key.leaf_id
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
                    # Note: Mass properties should be fetched separately via fetch_mass_properties
                    self.parts[key] = Part(
                        isStandardContent=False,
                        fullConfiguration=matching_subassembly.fullConfiguration or "",
                        configuration=matching_subassembly.configuration or "",
                        documentId=assembly_instance.documentId,
                        elementId=assembly_instance.elementId,
                        documentMicroversion=assembly_instance.documentMicroversion or "",
                        documentVersion=assembly_instance.documentVersion,
                        partId="",  # Rigid assemblies don't have a partId
                        bodyType="",
                        MassProperty=None,
                        isRigidAssembly=True,
                        rigidAssemblyToPartTF=None,
                        rigidAssemblyWorkspaceId=None,
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

            return f"{key.leaf_id} (unknown)"

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
                            asm.name
                            if (asm := self.root_assembly.instances.assemblies.get(k)) is not None
                            else k.leaf_id
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
            hierarchical_tf = self._build_hierarchical_transform(old_parent_key, new_parent_key, occurrence_registry)
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
            hierarchical_tf = self._build_hierarchical_transform(old_child_key, new_child_key, occurrence_registry)
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
            parent_leaf_id = parent_key.leaf_id
            child_leaf_id = child_key.leaf_id

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
        import copy

        # Get pattern instances for this seed
        seed_leaf_id = pattern_entity_key.leaf_id
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
            if key.leaf_id == leaf_id:
                return key
        # Check assemblies
        for key in instance_registry.assemblies:
            if key.leaf_id == leaf_id:
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
# LEGACY FUNCTIONS (string-based keys, deprecated)
# ============================================================================


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

        if instance.type == InstanceType.ASSEMBLY and isinstance(instance, AssemblyInstance):
            cast(AssemblyInstance, instance).isRigid = isRigid

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


def get_instances_legacy(
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


def get_instances_sync_legacy(
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
        instance_map: dict[str, Union[PartInstance, AssemblyInstance]] = {}
        id_to_name_map: dict[str, str] = {}

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
) -> None:
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
    subassembly_instance_map: dict[str, list[str]] = {}
    rigid_subassembly_instance_map: dict[str, list[str]] = {}

    for instance_key, instance in instance_map.items():
        if instance.type == InstanceType.ASSEMBLY and isinstance(instance, AssemblyInstance):
            if cast(AssemblyInstance, instance).isRigid:
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
                feature.featureType in RIGID_ASSEMBLY_ONLY_FEATURE_TYPES for feature in subassembly.features
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
) -> None:
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
            rigidAssemblyWorkspaceId=rigid_subassembly.documentMetaData.defaultWorkspace.id
            if rigid_subassembly.documentMetaData and rigid_subassembly.documentMetaData.defaultWorkspace
            else None,
            rigidAssemblyToPartTF={},
        )

    return part_map


def get_parts(
    assembly: Assembly,
    rigid_subassemblies: dict[str, RootAssembly],
    client: Client,
    instances: dict[str, Union[PartInstance, AssemblyInstance]],
    graph: Optional[nx.Graph] = None,
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
        LOGGER.warning(
            "Calling get_parts() without a graph parameter uses the legacy workflow that fetches "
            "mass properties for ALL parts, which can be very slow for complex assemblies. "
            "Consider passing a graph parameter.",
        )
        # Return empty dict for legacy workflow without graph
        return {}


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
    graph: Optional[nx.Graph] = None,
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
    if graph is not None:
        get_parts_with_selective_mass_properties(
            parts, graph, client, rigid_subassemblies, include_rigid_subassembly_parts
        )

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
    LOGGER.debug(
        f"DEBUG: build_hierarchical_transform_for_rigid_subassembly called with occurrences_list={occurrences_list}"
    )

    if len(occurrences_list) < 2:
        LOGGER.debug("DEBUG: Short occurrences_list, returning identity matrix")
        return np.matrix(np.eye(4))

    rigid_sub_name = occurrences_list[0]
    if rigid_sub_name not in rigid_subassembly_occurrence_map:
        LOGGER.warning(f"Rigid subassembly {rigid_sub_name} not found in occurrence map")
        return np.matrix(np.eye(4))

    LOGGER.debug(
        f"DEBUG: Found rigid subassembly {rigid_sub_name}, \
            available keys: {list(rigid_subassembly_occurrence_map[rigid_sub_name].keys())}"
    )

    parent_tf = np.matrix(np.eye(4))
    rigid_sub_occurrences = rigid_subassembly_occurrence_map[rigid_sub_name]

    # Traverse through each sub-assembly level (excluding the final part)
    for i in range(len(occurrences_list) - 1):
        # Build key relative to this rigid subassembly (exclude the rigid subassembly name itself)
        hierarchical_key = SUBASSEMBLY_JOINER.join(occurrences_list[1 : i + 2])
        LOGGER.debug(f"DEBUG: Looking for hierarchical_key: {hierarchical_key}")
        _occurrence_data = rigid_sub_occurrences.get(hierarchical_key)
        if _occurrence_data is None:
            LOGGER.warning(f"No occurrence data found for hierarchical key: {hierarchical_key}")
            continue
        _occurrence_tf = np.matrix(_occurrence_data.transform).reshape(4, 4)
        LOGGER.debug(f"DEBUG: Found occurrence transform for {hierarchical_key}: {_occurrence_tf}")
        parent_tf = np.matrix(parent_tf @ _occurrence_tf)
        LOGGER.debug(f"DEBUG: Cumulative parent_tf after {hierarchical_key}: {parent_tf}")

    LOGGER.debug(f"DEBUG: Final hierarchical transform: {parent_tf}")
    return parent_tf


def build_hierarchical_transform_with_flexible_parents(
    occurrences_list: list[str],
    rigid_subassembly_occurrence_map: dict[str, dict[str, Occurrence]],
    flexible_occurrences: dict[str, Occurrence],
    rigid_subassemblies: dict[str, RootAssembly],
    subassembly_prefix: Optional[str] = None,
) -> np.matrix:
    """
    Build hierarchical transform chain for a part that may be within a rigid subassembly
    that itself is nested within flexible parent components.

    This handles the mixed flexible-rigid scenario that occurs at intermediate max_depth values.

    Args:
        occurrences_list: List like [flexible_parent, rigid_sub_name, level1, part_name]
        rigid_subassembly_occurrence_map: Map of rigid subassembly occurrences
        flexible_occurrences: Map of flexible assembly occurrences with transforms
        rigid_subassemblies: Map of rigid subassembly instances
        subassembly_prefix: Optional prefix for subassembly keys

    Returns:
        Transform matrix from assembly root to the part's coordinate system
    """
    if len(occurrences_list) < 2:
        return np.matrix(np.eye(4))

    # Find the first rigid subassembly in the occurrence path
    rigid_start_idx = None
    for i, occurrence_name in enumerate(occurrences_list):
        if occurrence_name in rigid_subassemblies:
            rigid_start_idx = i
            break

    if rigid_start_idx is None:
        # No rigid subassembly found, shouldn't happen but handle gracefully
        LOGGER.warning(f"No rigid subassembly found in occurrences: {occurrences_list}")
        return np.matrix(np.eye(4))

    # Build transform from root to the rigid subassembly through flexible parents
    flexible_to_rigid_tf = np.matrix(np.eye(4))

    # Apply transforms for flexible parents leading to the rigid subassembly
    for i in range(rigid_start_idx):
        # Build the occurrence key for flexible components
        if subassembly_prefix:
            occurrence_key = f"{subassembly_prefix}_25B5_{occurrences_list[i]}"
        else:
            occurrence_key = occurrences_list[i]

        if occurrence_key in flexible_occurrences:
            flexible_tf = np.matrix(flexible_occurrences[occurrence_key].transform).reshape(4, 4)
            flexible_to_rigid_tf = np.matrix(flexible_to_rigid_tf @ flexible_tf)
        else:
            LOGGER.warning(f"Flexible occurrence {occurrence_key} not found in occurrences map")

    # Build transform within the rigid subassembly
    rigid_occurrences_sublist = occurrences_list[rigid_start_idx:]
    rigid_internal_tf = build_hierarchical_transform_for_rigid_subassembly(
        rigid_occurrences_sublist, rigid_subassembly_occurrence_map
    )

    # Combine the transforms: flexible_to_rigid @ rigid_internal
    total_tf = np.matrix(flexible_to_rigid_tf @ rigid_internal_tf)

    return total_tf


def build_hierarchical_transform_with_flexible_parents_v2(
    occurrences_list: list[str],
    rigid_subassembly_occurrence_map: dict[str, dict[str, Occurrence]],
    flexible_occurrences: dict[str, Occurrence],
    rigid_subassembly_sanitized_to_prefixed: dict[str, str],
    subassembly_prefix: Optional[str] = None,
) -> np.matrix:
    """
    Build hierarchical transform chain for a part that may be within a rigid subassembly
    that itself is nested within flexible parent components.

    This handles the mixed flexible-rigid scenario that occurs at intermediate max_depth values.
    Updated version that uses sanitized-to-prefixed mapping for proper name resolution.

    Args:
        occurrences_list: List like [flexible_parent, rigid_sub_name, level1, part_name]
        rigid_subassembly_occurrence_map: Map of rigid subassembly occurrences
        flexible_occurrences: Map of flexible assembly occurrences with transforms
        rigid_subassembly_sanitized_to_prefixed: Mapping from sanitized to prefixed names
        subassembly_prefix: Optional prefix for subassembly keys

    Returns:
        Transform matrix from assembly root to the part's coordinate system
    """
    if len(occurrences_list) < 2:
        return np.matrix(np.eye(4))

    # Find the first rigid subassembly in the occurrence path
    rigid_start_idx = None
    for i, occurrence_name in enumerate(occurrences_list):
        if occurrence_name in rigid_subassembly_sanitized_to_prefixed:
            rigid_start_idx = i
            break

    if rigid_start_idx is None:
        # No rigid subassembly found, shouldn't happen but handle gracefully
        LOGGER.warning(f"No rigid subassembly found in occurrences: {occurrences_list}")
        return np.matrix(np.eye(4))

    # Build transform from root to the rigid subassembly through flexible parents
    flexible_to_rigid_tf = np.matrix(np.eye(4))

    # Apply transforms for flexible parents leading to the rigid subassembly
    for i in range(rigid_start_idx):
        # Build the occurrence key for flexible components
        if subassembly_prefix:
            occurrence_key = f"{subassembly_prefix}_25B5_{occurrences_list[i]}"
        else:
            occurrence_key = occurrences_list[i]

        if occurrence_key in flexible_occurrences:
            flexible_tf = np.matrix(flexible_occurrences[occurrence_key].transform).reshape(4, 4)
            flexible_to_rigid_tf = np.matrix(flexible_to_rigid_tf @ flexible_tf)
        else:
            LOGGER.warning(f"Flexible occurrence {occurrence_key} not found in occurrences map")

    # Build transform within the rigid subassembly
    rigid_occurrences_sublist = occurrences_list[rigid_start_idx:]
    # Update the first occurrence to use the prefixed name
    rigid_sanitized_name = rigid_occurrences_sublist[0]
    rigid_prefixed_name = rigid_subassembly_sanitized_to_prefixed[rigid_sanitized_name]
    prefixed_rigid_occurrences_sublist = [rigid_prefixed_name] + rigid_occurrences_sublist[1:]
    rigid_internal_tf = build_hierarchical_transform_for_rigid_subassembly(
        prefixed_rigid_occurrences_sublist, rigid_subassembly_occurrence_map
    )

    # Combine the transforms: flexible_to_rigid @ rigid_internal
    total_tf = np.matrix(flexible_to_rigid_tf @ rigid_internal_tf)

    return total_tf


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

        if feature.featureType == AssemblyFeatureType.MATE and isinstance(feature.featureData, MateFeatureData):
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

            LOGGER.debug(f"Processing mate: parent_occurrences={parent_occurrences}")
            LOGGER.debug(f"rigid_subassemblies keys: {list(rigid_subassemblies.keys())}")

            # Create a mapping from sanitized names to prefixed names for rigid subassemblies
            rigid_subassembly_sanitized_to_prefixed = {}
            for prefixed_key in rigid_subassemblies:
                # Extract the sanitized name from the prefixed key
                sanitized_name = prefixed_key.split(SUBASSEMBLY_JOINER)[-1]
                rigid_subassembly_sanitized_to_prefixed[sanitized_name] = prefixed_key

            LOGGER.debug(f"Rigid subassembly name mapping: {rigid_subassembly_sanitized_to_prefixed}")
            LOGGER.debug(
                f"Parent[0] in rigid_subassemblies: {parent_occurrences[0] in rigid_subassembly_sanitized_to_prefixed}"
            )
            LOGGER.debug(f"Len parent_occurrences: {len(parent_occurrences)}")
            if len(parent_occurrences) > 1:
                LOGGER.debug(
                    f"Any parent occ in rigid_subassemblies: \
                        {any(occ in rigid_subassembly_sanitized_to_prefixed for occ in parent_occurrences)}"
                )

            # Handle rigid subassemblies
            if parent_occurrences[0] in rigid_subassembly_sanitized_to_prefixed:
                # Get the prefixed name for rigid subassembly lookups
                prefixed_parent_name = rigid_subassembly_sanitized_to_prefixed[parent_occurrences[0]]

                # Build hierarchical transform chain from rigid subassembly root to the part
                # Update the occurrence list to use prefixed names for rigid subassembly lookup
                prefixed_parent_occurrences = [prefixed_parent_name] + parent_occurrences[1:]
                parent_tf = build_hierarchical_transform_for_rigid_subassembly(
                    prefixed_parent_occurrences, rigid_subassembly_occurrence_map
                )
                parent_parentCS = MatedCS.from_tf(parent_tf)
                part_dict = parts[prefixed_parent_name].rigidAssemblyToPartTF
                if part_dict is None:
                    part_dict = {}
                    parts[prefixed_parent_name].rigidAssemblyToPartTF = part_dict
                part_dict[parent_occurrences[1]] = parent_parentCS
                feature.featureData.matedEntities[PARENT].parentCS = parent_parentCS

                # Use the prefixed name for the mate connection
                parent_occurrences = [prefixed_parent_name]
            elif len(parent_occurrences) > 1 and any(
                occ in rigid_subassembly_sanitized_to_prefixed for occ in parent_occurrences
            ):
                # Handle nested rigid subassemblies within flexible parents
                # Pass the sanitized-to-prefixed mapping instead of using direct rigid_subassemblies check
                parent_tf = build_hierarchical_transform_with_flexible_parents_v2(
                    parent_occurrences,
                    rigid_subassembly_occurrence_map,
                    occurrences,
                    rigid_subassembly_sanitized_to_prefixed,
                    subassembly_prefix,
                )
                parent_parentCS = MatedCS.from_tf(parent_tf)
                # Find the rigid subassembly in the path and get its prefixed name
                rigid_idx = next(
                    i for i, occ in enumerate(parent_occurrences) if occ in rigid_subassembly_sanitized_to_prefixed
                )
                prefixed_rigid_name = rigid_subassembly_sanitized_to_prefixed[parent_occurrences[rigid_idx]]
                part_dict = parts[prefixed_rigid_name].rigidAssemblyToPartTF
                if part_dict is None:
                    part_dict = {}
                    parts[prefixed_rigid_name].rigidAssemblyToPartTF = part_dict
                part_dict[parent_occurrences[-1]] = parent_parentCS
                feature.featureData.matedEntities[PARENT].parentCS = parent_parentCS

                # Use the prefixed name for the mate connection
                parent_occurrences = [prefixed_rigid_name]

            LOGGER.debug(f"Processing mate: child_occurrences={child_occurrences}")
            LOGGER.debug(
                f"Child[0] in rigid_subassemblies: {child_occurrences[0] in rigid_subassembly_sanitized_to_prefixed}"
            )
            LOGGER.debug(f"Len child_occurrences: {len(child_occurrences)}")
            if len(child_occurrences) > 1:
                LOGGER.debug(
                    f"Any child occ in rigid_subassemblies: \
                        {any(occ in rigid_subassembly_sanitized_to_prefixed for occ in child_occurrences)}"
                )

            if child_occurrences[0] in rigid_subassembly_sanitized_to_prefixed:
                # Get the prefixed name for rigid subassembly lookups
                prefixed_child_name = rigid_subassembly_sanitized_to_prefixed[child_occurrences[0]]

                # Build hierarchical transform chain from rigid subassembly root to the part
                # Update the occurrence list to use prefixed names for rigid subassembly lookup
                prefixed_child_occurrences = [prefixed_child_name] + child_occurrences[1:]
                parent_tf = build_hierarchical_transform_for_rigid_subassembly(
                    prefixed_child_occurrences, rigid_subassembly_occurrence_map
                )
                child_parentCS = MatedCS.from_tf(parent_tf)
                part_dict = parts[prefixed_child_name].rigidAssemblyToPartTF
                if part_dict is None:
                    part_dict = {}
                    parts[prefixed_child_name].rigidAssemblyToPartTF = part_dict
                part_dict[child_occurrences[1]] = child_parentCS
                feature.featureData.matedEntities[CHILD].parentCS = child_parentCS

                # Use the prefixed name for the mate connection
                child_occurrences = [prefixed_child_name]
            elif len(child_occurrences) > 1 and any(
                occ in rigid_subassembly_sanitized_to_prefixed for occ in child_occurrences
            ):
                # Handle nested rigid subassemblies within flexible parents
                # Pass the sanitized-to-prefixed mapping instead of using direct rigid_subassemblies check
                parent_tf = build_hierarchical_transform_with_flexible_parents_v2(
                    child_occurrences,
                    rigid_subassembly_occurrence_map,
                    occurrences,
                    rigid_subassembly_sanitized_to_prefixed,
                    subassembly_prefix,
                )
                child_parentCS = MatedCS.from_tf(parent_tf)
                # Find the rigid subassembly in the path and get its prefixed name
                rigid_idx = next(
                    i for i, occ in enumerate(child_occurrences) if occ in rigid_subassembly_sanitized_to_prefixed
                )
                prefixed_rigid_name = rigid_subassembly_sanitized_to_prefixed[child_occurrences[rigid_idx]]
                part_dict = parts[prefixed_rigid_name].rigidAssemblyToPartTF
                if part_dict is None:
                    part_dict = {}
                    parts[prefixed_rigid_name].rigidAssemblyToPartTF = part_dict
                part_dict[child_occurrences[-1]] = child_parentCS
                feature.featureData.matedEntities[CHILD].parentCS = child_parentCS

                # Use the prefixed name for the mate connection
                child_occurrences = [prefixed_rigid_name]
            else:
                # Check if child is in a flexible assembly that contains rigid subassemblies
                # This handles the case where child parts are siblings of rigid subassemblies
                # within the same flexible assembly and need coordinate system adjustments

                # Look for a parent occurrence that contains both the child and rigid subassemblies
                child_needs_coordinate_adjustment = False
                parent_assembly_prefix = None

                # Check if any parent occurrences contain rigid subassemblies as siblings
                if any(occ in rigid_subassembly_sanitized_to_prefixed for occ in parent_occurrences):  # noqa: SIM102
                    # The parent side has rigid subassemblies, check if child is a sibling
                    if len(parent_occurrences) > 1:
                        # Parent is like ['wheel_1', 'double-wheel_1', 'single-wheel_X', 'Part_5_X']
                        # Child is like ['Part_3_X'] - they're siblings in wheel_1 assembly
                        parent_assembly_prefix = parent_occurrences[0]  # 'wheel_1'
                        child_needs_coordinate_adjustment = True

                        LOGGER.debug(
                            f"DEBUG: Child {child_occurrences} needs coordinate adjustment \
                                relative to parent assembly {parent_assembly_prefix}"
                        )

                if child_needs_coordinate_adjustment and parent_assembly_prefix:
                    # Build the child occurrence path relative to the parent assembly
                    # For child 'Part_3_1' in assembly 'wheel_1', need to get its transform relative to wheel_1
                    child_occurrence_key = child_occurrences[0]

                    # Build the full occurrence key - need to match the pattern from rigid subassemblies
                    # Looking at rigid keys like 'wheel_1_EF24_double-wheel_1', the pattern is:
                    # {parent_assembly}_{JOINER}_{child_name}
                    if subassembly_prefix:
                        full_child_key = f"{subassembly_prefix}{SUBASSEMBLY_JOINER} \
                            {parent_assembly_prefix}{SUBASSEMBLY_JOINER}{child_occurrence_key}"
                    else:
                        # Find the joiner from rigid subassembly keys to use consistent naming
                        # Look at any rigid subassembly key to extract the joiner pattern
                        sample_rigid_key = next(iter(rigid_subassembly_sanitized_to_prefixed.values()))
                        # Extract joiner pattern: wheel_1_XXXX_double-wheel_1 -> XXXX is the joiner
                        parts_of_key = sample_rigid_key.split(SUBASSEMBLY_JOINER)
                        if len(parts_of_key) >= 2:
                            # Extract the middle part which is the joiner
                            key_joiner = parts_of_key[1].split("_")[0] if "_" in parts_of_key[1] else parts_of_key[1]
                            full_child_key = (
                                f"{parent_assembly_prefix}_{key_joiner}{SUBASSEMBLY_JOINER}{child_occurrence_key}"
                            )
                        else:
                            full_child_key = f"{parent_assembly_prefix}{SUBASSEMBLY_JOINER}{child_occurrence_key}"

                    LOGGER.debug(f"DEBUG: Looking for child occurrence key: {full_child_key}")
                    LOGGER.debug(f"DEBUG: Available occurrence keys: {list(occurrences.keys())}")

                    # Get the child's transform relative to the parent assembly
                    if full_child_key in occurrences:
                        child_occurrence_tf = np.matrix(np.array(occurrences[full_child_key].transform).reshape(4, 4))
                        child_parentCS = MatedCS.from_tf(child_occurrence_tf)
                        feature.featureData.matedEntities[CHILD].parentCS = child_parentCS
                        LOGGER.debug(f"DEBUG: Set child parentCS for {child_occurrence_key}: {child_occurrence_tf}")
                    else:
                        LOGGER.debug(f"DEBUG: Child occurrence key {full_child_key} not found in occurrences")

            mates_map[
                join_mate_occurrences_with_proxy(
                    parent=parent_occurrences,
                    child=child_occurrences,
                    instance_proxy_map=instance_proxy_map,
                    prefix=subassembly_prefix,
                )
            ] = feature.featureData

        elif feature.featureType == AssemblyFeatureType.MATERELATION and isinstance(
            feature.featureData, MateRelationFeatureData
        ):
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
                    part_dict = parts[parent_occurrences[0]].rigidAssemblyToPartTF
                    if part_dict is None:
                        part_dict = {}
                        parts[parent_occurrences[0]].rigidAssemblyToPartTF = part_dict
                    part_dict[parent_occurrences[1]] = parent_parentCS
                    pattern_mate.matedEntities[PARENT].parentCS = parent_parentCS
                    parent_occurrences = [parent_occurrences[0]]

                if child_occurrences[0] in rigid_subassemblies:
                    parent_tf = build_hierarchical_transform_for_rigid_subassembly(
                        child_occurrences, rigid_subassembly_occurrence_map
                    )
                    child_parentCS = MatedCS.from_tf(parent_tf)
                    part_dict = parts[child_occurrences[0]].rigidAssemblyToPartTF
                    if part_dict is None:
                        part_dict = {}
                        parts[child_occurrences[0]].rigidAssemblyToPartTF = part_dict
                    part_dict[child_occurrences[1]] = child_parentCS
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


def ensure_unique_mate_names(mates: dict[str, MateFeatureData]) -> None:
    """
    Ensure all mate names are unique by appending suffixes to duplicates.

    This function modifies the mate names in-place to ensure URDF joint name uniqueness.
    Pattern-expanded mates and regular mates might have conflicting names, so we need
    to detect and resolve these conflicts iteratively until all names are unique.

    Args:
        mates: Dictionary mapping mate keys to mate feature data

    Examples:
        >>> mates = {
        ...     "key1": MateFeatureData(name="joint1", ...),
        ...     "key2": MateFeatureData(name="joint1", ...),  # duplicate
        ...     "key3": MateFeatureData(name="joint1_2", ...),  # conflicts with renamed
        ... }
        >>> ensure_unique_mate_names(mates)
        >>> # Result: "joint1", "joint1_2", "joint1_3"
    """
    max_iterations = 10  # Prevent infinite loops
    iteration = 0

    while iteration < max_iterations:
        name_counts: dict[str, int] = {}
        mate_keys_by_name: dict[str, list[Any]] = {}

        # Count occurrences of each current name
        for mate_key, mate in mates.items():
            name = mate.name
            name_counts[name] = name_counts.get(name, 0) + 1
            if name not in mate_keys_by_name:
                mate_keys_by_name[name] = []
            mate_keys_by_name[name].append(mate_key)

        # Check if we have any duplicates
        duplicates_found = False
        for name, count in name_counts.items():
            if count > 1:
                duplicates_found = True
                mate_keys = mate_keys_by_name[name]

                # Generate unique names for all duplicates except the first
                for i, mate_key in enumerate(mate_keys[1:], start=2):
                    # Find a unique name by trying sequential suffixes
                    base_name = name
                    suffix = i
                    new_name = f"{base_name}_{suffix}"

                    # Keep incrementing suffix until we find a unique name
                    while new_name in name_counts:
                        suffix += 1
                        new_name = f"{base_name}_{suffix}"

                    mates[mate_key].name = new_name
                    LOGGER.debug(f"Renamed duplicate mate '{name}' to '{new_name}' for key {mate_key}")

        if not duplicates_found:
            break

        iteration += 1

    if iteration >= max_iterations:
        LOGGER.warning(f"Could not resolve all mate name conflicts after {max_iterations} iterations")


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
    mates, relations = asyncio.run(
        get_mates_and_relations_async(
            assembly, instance_proxy_map, occurrences, subassemblies, rigid_subassemblies, id_to_name_map, parts
        )
    )

    # Ensure all mate names are unique for valid URDF generation
    ensure_unique_mate_names(mates)

    return mates, relations
