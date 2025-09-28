"""
This module contains functions that provide a way to traverse the assembly structure, extract information about parts,
subassemblies, instances, and mates, and generate a hierarchical representation of the assembly.

"""

import asyncio
import copy
import uuid
from collections import deque
from dataclasses import dataclass
from enum import Enum
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
from onshape_robotics_toolkit.utilities.helpers import clean_name_for_urdf, get_sanitized_name

SUBASSEMBLY_JOINER = f"_{uuid.uuid4().hex[:4].upper()}_"
MATE_JOINER = f"_{uuid.uuid4().hex[:4].upper()}_"

LOGGER.info(f"Generated joiners - SUBASSEMBLY: {SUBASSEMBLY_JOINER}, MATE: {MATE_JOINER}")

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
# ENHANCED UNIFIED ASSEMBLY PARSING SYSTEM
# ============================================================================


class EntityType(Enum):
    """Type of entity in the assembly hierarchy."""

    PART = "PART"
    RIGID_ASSEMBLY = "RIGID_ASSEMBLY"
    FLEXIBLE_ASSEMBLY = "FLEXIBLE_ASSEMBLY"
    FEATURE = "FEATURE"
    OCCURRENCE = "OCCURRENCE"


@dataclass(frozen=True)
class TypedKey:
    """Type-safe key with entity context and hierarchical path information."""

    path: tuple[str, ...]  # Hierarchical path using instance IDs
    entity_type: EntityType
    entity_id: str  # The actual ID of the entity

    def __str__(self) -> str:
        """String representation for debugging."""
        path_str = ":".join(self.path) if self.path else ROOT_PATH_NAME
        return f"{self.entity_type.value}:{path_str}:{self.entity_id}"

    def __repr__(self) -> str:
        return f"TypedKey({self.entity_type.value}, {self.path}, {self.entity_id})"

    @property
    def parent_path(self) -> tuple[str, ...]:
        """Get parent path."""
        return self.path[:-1] if self.path else ()

    @property
    def depth(self) -> int:
        """Get depth in hierarchy."""
        return len(self.path)

    @classmethod
    def for_part(cls, path: tuple[str, ...], part_id: str) -> "TypedKey":
        """Create a TypedKey for a part."""
        return cls(path=path, entity_type=EntityType.PART, entity_id=part_id)

    @classmethod
    def for_rigid_assembly(cls, path: tuple[str, ...], assembly_id: str) -> "TypedKey":
        """Create a TypedKey for a rigid assembly."""
        return cls(path=path, entity_type=EntityType.RIGID_ASSEMBLY, entity_id=assembly_id)

    @classmethod
    def for_flexible_assembly(cls, path: tuple[str, ...], assembly_id: str) -> "TypedKey":
        """Create a TypedKey for a flexible assembly."""
        return cls(path=path, entity_type=EntityType.FLEXIBLE_ASSEMBLY, entity_id=assembly_id)

    @classmethod
    def for_feature(cls, path: tuple[str, ...], feature_id: str) -> "TypedKey":
        """Create a TypedKey for a feature."""
        return cls(path=path, entity_type=EntityType.FEATURE, entity_id=feature_id)

    @classmethod
    def for_occurrence(cls, path: tuple[str, ...], occurrence_id: str) -> "TypedKey":
        """Create a TypedKey for an occurrence."""
        return cls(path=path, entity_type=EntityType.OCCURRENCE, entity_id=occurrence_id)


class IdToNameResolver:
    """Centralized ID to name resolution with efficient lookups."""

    def __init__(self, assembly: Assembly):
        self.id_to_name: dict[str, str] = {}
        self.id_to_instance: dict[str, Union[PartInstance, AssemblyInstance]] = {}
        self._build_mappings(assembly)

    def _build_mappings(self, assembly: Assembly) -> None:
        """Build ID mappings in single pass."""
        LOGGER.debug("Building ID to name resolver mappings...")

        # Root assembly instances
        for instance in assembly.rootAssembly.instances:
            sanitized_name = get_sanitized_name(instance.name)
            self.id_to_name[instance.id] = sanitized_name
            self.id_to_instance[instance.id] = instance

        # Traverse all subassemblies to build complete mapping
        subassembly_deque: deque[SubAssembly] = deque(assembly.subAssemblies)
        visited_uids: set[str] = set()

        while subassembly_deque:
            subassembly = subassembly_deque.popleft()

            # Avoid processing the same subassembly multiple times
            if subassembly.uid in visited_uids:
                continue
            visited_uids.add(subassembly.uid)

            # Process instances in this subassembly
            for instance in subassembly.instances:
                sanitized_name = get_sanitized_name(instance.name)
                self.id_to_name[instance.id] = sanitized_name
                self.id_to_instance[instance.id] = instance

        LOGGER.debug(f"Resolved {len(self.id_to_name)} instance ID to name mappings")

    def get_name(self, instance_id: str) -> str:
        """Get sanitized name for an instance ID."""
        return self.id_to_name.get(instance_id, instance_id)

    def get_instance(self, instance_id: str) -> Optional[Union[PartInstance, AssemblyInstance]]:
        """Get instance for an instance ID."""
        return self.id_to_instance.get(instance_id)

    def has_id(self, instance_id: str) -> bool:
        """Check if an instance ID exists."""
        return instance_id in self.id_to_name


class KeyNamer:
    """Type-aware key naming with consistent output."""

    def __init__(self, id_resolver: IdToNameResolver, joiner: str = ":"):
        self.id_resolver = id_resolver
        self.joiner = joiner
        # Separate caches by entity type to handle same names for different entity types
        self._reverse_cache_by_type: dict[EntityType, dict[str, TypedKey]] = {}
        self._prefixed_reverse_cache_by_type: dict[str, dict[EntityType, dict[str, TypedKey]]] = {}

    def get_name(self, key: TypedKey, prefix: Optional[str] = None) -> str:
        """Get clean name for any typed key."""
        # Build path names from IDs
        path_names = [self.id_resolver.get_name(path_id) for path_id in key.path]

        # Add the entity name if it's different from the last path element
        if key.path and key.entity_id != key.path[-1]:
            entity_name = self.id_resolver.get_name(key.entity_id)
            if entity_name != key.entity_id:  # Only add if we have a meaningful name
                path_names.append(entity_name)

        name = self.joiner.join(path_names) if path_names else key.entity_id
        name = clean_name_for_urdf(name)

        if prefix:
            return f"{prefix}{self.joiner}{name}"
        return name

    def lookup_key(self, name: str, entity_type: EntityType, prefix: Optional[str] = None) -> Optional[TypedKey]:
        """
        Find a key by its generated name (for reverse lookup).

        Args:
            name: The generated name to look up
            entity_type: Entity type to look up (ensures type safety)
            prefix: Optional prefix scope for the lookup

        Returns:
            The TypedKey that generates this name, or None if not found
        """
        if prefix:
            # Handle prefixed lookups
            if prefix not in self._prefixed_reverse_cache_by_type:
                return None  # Cache not built for this prefix yet

            type_caches = self._prefixed_reverse_cache_by_type[prefix]
            if entity_type not in type_caches:
                return None  # Cache not built for this entity type yet

            return type_caches[entity_type].get(name)
        else:
            # Handle non-prefixed lookups
            if entity_type not in self._reverse_cache_by_type:
                return None  # Cache not built for this entity type yet

            return self._reverse_cache_by_type[entity_type].get(name)

    def build_reverse_cache(self, keys: list[TypedKey], prefix: Optional[str] = None) -> None:
        """
        Build reverse mapping cache for efficient name-to-key lookups.

        Args:
            keys: List of TypedKeys to build cache for
            prefix: Optional prefix for scoped cache (e.g., subassembly context)
        """
        # Group keys by entity type to avoid name collisions
        keys_by_type: dict[EntityType, list[TypedKey]] = {}
        for key in keys:
            if key.entity_type not in keys_by_type:
                keys_by_type[key.entity_type] = []
            keys_by_type[key.entity_type].append(key)

        # Build separate cache for each entity type
        total_entries = 0
        for entity_type, type_keys in keys_by_type.items():
            type_cache: dict[str, TypedKey] = {}

            for key in type_keys:
                generated_name = self.get_name(key, prefix)
                if generated_name in type_cache:
                    LOGGER.warning(
                        f"Duplicate name '{generated_name}' found in {entity_type.value}: keeping first occurrence"
                    )
                    continue
                type_cache[generated_name] = key

            # Store the type-specific cache
            if prefix:
                if prefix not in self._prefixed_reverse_cache_by_type:
                    self._prefixed_reverse_cache_by_type[prefix] = {}
                self._prefixed_reverse_cache_by_type[prefix][entity_type] = type_cache
            else:
                self._reverse_cache_by_type[entity_type] = type_cache

            total_entries += len(type_cache)

        LOGGER.debug(
            f"Built reverse cache with {total_entries} total entries across {len(keys_by_type)} entity types"
            + (f" for prefix '{prefix}'" if prefix else "")
        )

    def clear_cache(self, prefix: Optional[str] = None, entity_type: Optional[EntityType] = None) -> None:
        """
        Clear reverse mapping cache.

        Args:
            prefix: If provided, clears only the prefixed cache for this scope.
                   If None, clears the main cache.
            entity_type: If provided, clears only the cache for this entity type.
                        If None, clears all entity type caches.
        """
        if prefix:
            if entity_type:
                # Clear specific entity type cache for specific prefix
                if prefix in self._prefixed_reverse_cache_by_type:
                    self._prefixed_reverse_cache_by_type[prefix].pop(entity_type, None)
            else:
                # Clear all entity type caches for specific prefix
                self._prefixed_reverse_cache_by_type.pop(prefix, None)
        else:
            if entity_type:
                # Clear specific entity type cache
                self._reverse_cache_by_type.pop(entity_type, None)
            else:
                # Clear all entity type caches
                self._reverse_cache_by_type.clear()


@dataclass
class CAD:
    """Container for all parsed CAD assembly data with TypedKey system."""

    assembly: Assembly
    instances: dict[TypedKey, Union[PartInstance, AssemblyInstance]]
    parts: dict[TypedKey, PartInstance]
    rigid_assemblies: dict[TypedKey, AssemblyInstance]
    flexible_assemblies: dict[TypedKey, AssemblyInstance]
    occurrences: dict[TypedKey, Occurrence]
    features: dict[TypedKey, AssemblyFeature]
    mates: dict[TypedKey, MateFeatureData]
    relations: dict[TypedKey, MateRelationFeatureData]
    id_resolver: IdToNameResolver
    key_namer: KeyNamer
    subassemblies: dict[TypedKey, SubAssembly]
    rigid_subassemblies: dict[TypedKey, RootAssembly]
    max_depth: int

    @classmethod
    def from_assembly(cls, assembly: Assembly, client, max_depth: int = 0) -> "CAD":
        """
        Create CAD from an Onshape assembly with full subassembly parsing.

        Args:
            assembly: The assembly to parse
            client: Onshape client for fetching subassemblies
            max_depth: Maximum depth for traversal

        Returns:
            CAD with all data parsed, subassemblies fetched, and reverse cache built
        """
        builder = AssemblyStructureBuilder(assembly, max_depth)
        cad = builder.build()

        # Fetch rigid subassemblies if any exist
        if cad.rigid_assemblies:
            cad.fetch_rigid_subassemblies(client)

        # Build reverse cache for lookups
        cad.build_reverse_cache()

        return cad

    @property
    def part_keys(self) -> list[TypedKey]:
        """Get all part keys."""
        return list(self.parts.keys())

    @property
    def assembly_keys(self) -> list[TypedKey]:
        """Get all assembly keys (both rigid and flexible)."""
        return list(self.rigid_assemblies.keys()) + list(self.flexible_assemblies.keys())

    @property
    def rigid_assembly_keys(self) -> list[TypedKey]:
        """Get all rigid assembly keys."""
        return list(self.rigid_assemblies.keys())

    @property
    def flexible_assembly_keys(self) -> list[TypedKey]:
        """Get all flexible assembly keys."""
        return list(self.flexible_assemblies.keys())

    @property
    def mate_keys(self) -> list[TypedKey]:
        """Get all mate keys."""
        return list(self.mates.keys())

    def build_reverse_cache(self, prefix: Optional[str] = None) -> None:
        """
        Build reverse mapping cache for all entity types.

        Args:
            prefix: Optional prefix for scoped cache
        """
        all_keys = (
            list(self.instances.keys())  # Already includes all parts and assemblies
            + list(self.features.keys())
            + list(self.occurrences.keys())
            + list(self.subassemblies.keys())
            + list(self.rigid_subassemblies.keys())
            # Note: rigid_assemblies and flexible_assemblies keys already included in instances
        )
        self.key_namer.build_reverse_cache(all_keys, prefix)

    def lookup_part(self, name: str, prefix: Optional[str] = None) -> Optional[PartInstance]:
        """Look up a part by its generated name."""
        key = self.key_namer.lookup_key(name, EntityType.PART, prefix)
        return self.parts.get(key) if key else None

    def lookup_assembly(self, name: str, prefix: Optional[str] = None) -> Optional[AssemblyInstance]:
        """Look up an assembly by its generated name (searches both rigid and flexible)."""
        # Try rigid first
        key = self.key_namer.lookup_key(name, EntityType.RIGID_ASSEMBLY, prefix)
        result = self.rigid_assemblies.get(key) if key else None
        if result:
            return result

        # Try flexible
        key = self.key_namer.lookup_key(name, EntityType.FLEXIBLE_ASSEMBLY, prefix)
        return self.flexible_assemblies.get(key) if key else None

    def lookup_rigid_assembly(self, name: str, prefix: Optional[str] = None) -> Optional[AssemblyInstance]:
        """Look up a rigid assembly by its generated name."""
        key = self.key_namer.lookup_key(name, EntityType.RIGID_ASSEMBLY, prefix)
        return self.rigid_assemblies.get(key) if key else None

    def lookup_flexible_assembly(self, name: str, prefix: Optional[str] = None) -> Optional[AssemblyInstance]:
        """Look up a flexible assembly by its generated name."""
        key = self.key_namer.lookup_key(name, EntityType.FLEXIBLE_ASSEMBLY, prefix)
        return self.flexible_assemblies.get(key) if key else None

    def lookup_mate(self, name: str, prefix: Optional[str] = None) -> Optional[MateFeatureData]:
        """Look up a mate by its generated name."""
        key = self.key_namer.lookup_key(name, EntityType.FEATURE, prefix)
        return self.mates.get(key) if key else None

    def lookup_subassembly(self, name: str, prefix: Optional[str] = None) -> Optional[SubAssembly]:
        """Look up a flexible subassembly by its generated name."""
        key = self.key_namer.lookup_key(name, EntityType.FLEXIBLE_ASSEMBLY, prefix)
        return self.subassemblies.get(key) if key else None

    def lookup_rigid_subassembly(self, name: str, prefix: Optional[str] = None) -> Optional[RootAssembly]:
        """Look up a rigid subassembly by its generated name."""
        key = self.key_namer.lookup_key(name, EntityType.RIGID_ASSEMBLY, prefix)
        return self.rigid_subassemblies.get(key) if key else None

    @property
    def subassembly_keys(self) -> list[TypedKey]:
        """Get all flexible subassembly keys."""
        return list(self.subassemblies.keys())

    @property
    def rigid_subassembly_keys(self) -> list[TypedKey]:
        """Get all rigid subassembly keys."""
        return list(self.rigid_subassemblies.keys())

    async def fetch_rigid_subassemblies_async(self, client) -> None:
        """
        Asynchronously fetch rigid subassemblies from Onshape API.

        Args:
            client: Onshape client for API calls
        """

        # Find assembly instances that should have rigid subassemblies
        # Fetch each rigid instance separately (no UID-based deduplication)
        # This is needed because different instances may have different occurrence transforms
        rigid_tasks = []

        for key, assembly_instance in self.rigid_assemblies.items():
            # Find the corresponding subassembly
            subassembly = None
            for sub in self.assembly.subAssemblies:
                if sub.uid == assembly_instance.uid:
                    subassembly = sub
                    break

            if subassembly:
                task = self._fetch_single_rigid_subassembly(client, key, subassembly)
                rigid_tasks.append(task)

        if rigid_tasks:
            rigid_instance_count = len(self.rigid_assemblies)
            LOGGER.info(f"Fetching {len(rigid_tasks)} unique rigid assemblies (from {rigid_instance_count} instances)")
            await asyncio.gather(*rigid_tasks, return_exceptions=True)
            LOGGER.info(f"Completed fetching {len(self.rigid_subassemblies)} rigid subassemblies")

    async def _fetch_single_rigid_subassembly(self, client, key: TypedKey, subassembly: SubAssembly) -> None:
        """Fetch a single rigid subassembly."""
        try:
            root_assembly = await asyncio.to_thread(
                client.get_root_assembly,
                did=subassembly.documentId,
                wtype=WorkspaceType.M.value,
                wid=subassembly.documentMicroversion,
                eid=subassembly.elementId,
                with_mass_properties=True,
                log_response=False,
            )
            # Use the instance key instead of UID to distinguish different instances of the same subassembly
            # This ensures each rigid instance gets its own entry even if they share the same UID
            rigid_key = TypedKey.for_assembly(("rigid_subassembly", *key.path), subassembly.uid)
            self.rigid_subassemblies[rigid_key] = root_assembly
            LOGGER.debug(f"Successfully fetched rigid subassembly: {self.key_namer(key.path)}")
        except Exception as e:
            LOGGER.error(f"Failed to fetch rigid subassembly {self.key_namer(key.path)}: {e}")

    def fetch_rigid_subassemblies(self, client) -> None:
        """
        Synchronous wrapper for fetch_rigid_subassemblies_async.

        Args:
            client: Onshape client for API calls
        """
        asyncio.run(self.fetch_rigid_subassemblies_async(client))

    def show_tree(self) -> None:
        """Display the CAD assembly structure as a tree."""
        from collections import defaultdict

        # Build hierarchy from TypedKey paths
        children_by_path = defaultdict(list)
        all_paths = set()

        # Collect all instances with their paths
        for key in self.instances:
            all_paths.add(key.path)
            if key.path:
                parent_path = key.path[:-1]
                children_by_path[parent_path].append(key)
            else:
                # Root level
                children_by_path[()].append(key)

        def get_display_name(key: TypedKey) -> str:
            """Get display name for a key."""
            instance = self.instances.get(key)
            if instance:
                return f"{instance.name} ({key.entity_type.value})"
            return f"{key.entity_id} ({key.entity_type.value})"

        def print_tree(path: tuple[str, ...], depth: int = 0):
            """Recursively print tree structure."""
            prefix = "    " * depth

            if depth == 0:
                print(f"{prefix}Assembly Root")

            # Print children at this level, sorted by type (parts first, then assemblies) and then by name
            children = children_by_path[path]
            # Sort by: 1) entity type (parts first), 2) instance name
            sorted_children = sorted(
                children,
                key=lambda k: (
                    0 if k.entity_type.value == "PART" else 1,  # Parts first, then assemblies
                    self.instances.get(k, k).name if hasattr(self.instances.get(k, k), "name") else k.entity_id,
                ),
            )

            for key in sorted_children:
                print(f"{prefix}|-- {get_display_name(key)}")

                # Check for subassemblies/rigid subassemblies at this key
                sub_keys = [k for k in self.subassemblies if k.path == key.path]
                rigid_keys = [k for k in self.rigid_subassemblies if k.path == key.path]

                for sub_key in sub_keys:
                    sub = self.subassemblies[sub_key]
                    print(f"{prefix}|   +-- Flexible Subassembly: {sub.name}")

                for rigid_key in rigid_keys:
                    rigid = self.rigid_subassemblies[rigid_key]
                    print(f"{prefix}|   +-- Rigid Subassembly: {rigid.name}")

                # Recurse to children
                if key.path in children_by_path:
                    print_tree(key.path, depth + 1)

        print_tree(())


class AssemblyStructureBuilder:
    """Builds all assembly mappings in a single efficient traversal."""

    def __init__(self, assembly: Assembly, max_depth: int = 0):
        self.assembly = assembly
        self.max_depth = max_depth

        # Core resolver and namer
        self.id_resolver = IdToNameResolver(assembly)
        self.key_namer = KeyNamer(self.id_resolver)

        # All mappings built in single pass
        self.instances: dict[TypedKey, Union[PartInstance, AssemblyInstance]] = {}
        self.parts: dict[TypedKey, PartInstance] = {}
        self.rigid_assemblies: dict[TypedKey, AssemblyInstance] = {}
        self.flexible_assemblies: dict[TypedKey, AssemblyInstance] = {}
        self.occurrences: dict[TypedKey, Occurrence] = {}
        self.features: dict[TypedKey, AssemblyFeature] = {}
        self.mates: dict[TypedKey, MateFeatureData] = {}
        self.relations: dict[TypedKey, MateRelationFeatureData] = {}
        self.subassemblies: dict[TypedKey, SubAssembly] = {}
        self.rigid_subassemblies: dict[TypedKey, RootAssembly] = {}

    def build(self) -> CAD:
        """Build all assembly mappings in a single traversal."""
        LOGGER.info("Starting assembly parsing...")

        self._traverse_and_build()
        self._build_occurrences()

        LOGGER.info(
            f"Assembly parsing completed: {len(self.instances)} instances, "
            f"{len(self.parts)} parts, {len(self.rigid_assemblies)} rigid assemblies, "
            f"{len(self.flexible_assemblies)} flexible assemblies, "
            f"{len(self.subassemblies)} subassemblies, {len(self.rigid_subassemblies)} rigid subassemblies, "
            f"{len(self.mates)} mates, {len(self.occurrences)} occurrences"
        )

        return CAD(
            assembly=self.assembly,
            instances=self.instances,
            parts=self.parts,
            rigid_assemblies=self.rigid_assemblies,
            flexible_assemblies=self.flexible_assemblies,
            occurrences=self.occurrences,
            features=self.features,
            mates=self.mates,
            relations=self.relations,
            id_resolver=self.id_resolver,
            key_namer=self.key_namer,
            subassemblies=self.subassemblies,
            rigid_subassemblies=self.rigid_subassemblies,
            max_depth=self.max_depth,
        )

    def _traverse_and_build(self) -> None:
        """Single traversal that builds all instance and feature mappings."""
        LOGGER.debug("Building all mappings in single traversal...")

        # Process root assembly
        self._process_assembly_level(self.assembly.rootAssembly, path=())

        # Process subassemblies with proper hierarchy tracking
        uid_to_subassembly = {sub.uid: sub for sub in self.assembly.subAssemblies}

        subassembly_queue: deque[tuple[tuple[str, ...], AssemblyInstance, SubAssembly]] = deque()
        visited_paths: set[tuple[str, ...]] = set()

        # Find root-level assembly instances
        for instance in self.assembly.rootAssembly.instances:
            if instance.type == InstanceType.ASSEMBLY:
                instance_path = (instance.id,)
                if instance_path in visited_paths:
                    continue
                visited_paths.add(instance_path)

                if instance.uid in uid_to_subassembly:
                    subassembly = uid_to_subassembly[instance.uid]
                    subassembly_queue.append((instance_path, instance, subassembly))

        # Process subassemblies breadth-first
        while subassembly_queue:
            path, assembly_instance, subassembly = subassembly_queue.popleft()

            # Determine if this subassembly should be rigid
            # max_depth=0: all subassemblies are rigid
            # max_depth=1: root-level subassemblies flexible, deeper ones rigid
            # max_depth=N: subassemblies at depth <= N are flexible, deeper ones rigid
            is_rigid = True if self.max_depth == 0 else len(path) > self.max_depth

            # Process this subassembly level
            self._process_assembly_level(subassembly, path, is_rigid, subassembly)

            # Add nested assemblies to queue
            for instance in subassembly.instances:
                if instance.type == InstanceType.ASSEMBLY:
                    nested_path = (*path, instance.id)
                    if nested_path in visited_paths:
                        continue
                    visited_paths.add(nested_path)

                    if instance.uid in uid_to_subassembly:
                        nested_subassembly = uid_to_subassembly[instance.uid]
                        subassembly_queue.append((nested_path, instance, nested_subassembly))

    def _process_assembly_level(
        self,
        assembly_level: Union[RootAssembly, SubAssembly],
        path: tuple[str, ...],
        is_rigid: bool = False,
        subassembly_obj: Optional[SubAssembly] = None,
    ) -> None:
        """Process instances and features at a specific assembly level."""

        # Store subassembly if provided (using instance path as key to distinguish multiple instances)
        if subassembly_obj and path:
            # Use instance path instead of UID to properly distinguish multiple instances of the same subassembly
            if is_rigid:
                subassembly_key = TypedKey.for_rigid_assembly((SUBASSEMBLY_PATH_PREFIX, *path), subassembly_obj.uid)
                # Will be populated later when we fetch the RootAssembly
            else:
                subassembly_key = TypedKey.for_flexible_assembly((SUBASSEMBLY_PATH_PREFIX, *path), subassembly_obj.uid)
                self.subassemblies[subassembly_key] = subassembly_obj

        # Process instances
        for instance in assembly_level.instances:
            instance_path = (*path, instance.id)

            if instance.type == InstanceType.PART:
                key = TypedKey.for_part(instance_path, instance.id)
                self.instances[key] = instance
                self.parts[key] = instance
            elif instance.type == InstanceType.ASSEMBLY:
                # Determine if this assembly should be rigid based on depth
                instance_depth = len(instance_path)
                is_instance_rigid = True if self.max_depth == 0 else instance_depth > self.max_depth

                # Create typed key and store in appropriate collection
                if is_instance_rigid:
                    key = TypedKey.for_rigid_assembly(instance_path, instance.id)
                    self.rigid_assemblies[key] = instance
                    instance.isRigid = True  # Keep for backward compatibility
                else:
                    key = TypedKey.for_flexible_assembly(instance_path, instance.id)
                    self.flexible_assemblies[key] = instance
                    instance.isRigid = False  # Keep for backward compatibility

                self.instances[key] = instance

        # Process features
        for feature in assembly_level.features:
            feature_path = (*path, feature.id)
            key = TypedKey.for_feature(feature_path, feature.id)

            self.features[key] = feature

            # Classify feature types
            if hasattr(feature.featureData, "matedEntities"):
                self.mates[key] = feature.featureData
            elif hasattr(feature.featureData, "relationType"):
                self.relations[key] = feature.featureData

    def _build_occurrences(self) -> None:
        """Build occurrence mappings efficiently using ID resolver."""
        LOGGER.debug("Building occurrence mappings...")

        for occurrence in self.assembly.rootAssembly.occurrences:
            if not occurrence.path:
                continue

            # Use ID resolver for efficient lookup (no nested loops!)
            path_names = []
            all_ids_found = True

            for path_id in occurrence.path:
                if self.id_resolver.has_id(path_id):
                    path_names.append(path_id)
                else:
                    all_ids_found = False
                    break

            if all_ids_found and path_names:
                # Create occurrence key using the actual ID path
                occurrence_key = TypedKey.for_occurrence(path=tuple(occurrence.path), occurrence_id=occurrence.path[-1])
                self.occurrences[occurrence_key] = occurrence

        LOGGER.debug(f"Built {len(self.occurrences)} occurrence mappings")


# ============================================================================
# RECOMMENDED ENTRY POINT (CAD class with built-in subassembly parsing)
# ============================================================================

# Use CAD.from_assembly(assembly, client, max_depth) directly


def get_instances(
    assembly: Assembly, max_depth: int = 0
) -> tuple[dict[TypedKey, Union[PartInstance, AssemblyInstance]], dict[TypedKey, Occurrence], KeyNamer]:
    """
    Get instances function using TypedKey system.

    NOTE: This function only builds basic structure without subassembly fetching.
    For full CAD parsing with subassembly support, use CAD.from_assembly(assembly, client, max_depth).

    Returns:
        Tuple of (instances, occurrences, key_namer) using TypedKey system
    """
    builder = AssemblyStructureBuilder(assembly, max_depth)
    cad = builder.build()
    return cad.instances, cad.occurrences, cad.key_namer


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
        LOGGER.warning(
            "Calling get_parts() without a graph parameter uses the legacy workflow that fetches "
            "mass properties for ALL parts, which can be very slow for complex assemblies. "
            "Consider passing a graph parameter.",
        )


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
    LOGGER.debug(
        f"DEBUG: build_hierarchical_transform_for_rigid_subassembly called with occurrences_list={occurrences_list}"
    )

    if len(occurrences_list) < 2:
        LOGGER.debug("DEBUG: Short occurrences_list, returning identity matrix")
        return np.eye(4)

    rigid_sub_name = occurrences_list[0]
    if rigid_sub_name not in rigid_subassembly_occurrence_map:
        LOGGER.warning(f"Rigid subassembly {rigid_sub_name} not found in occurrence map")
        return np.eye(4)

    LOGGER.debug(
        f"DEBUG: Found rigid subassembly {rigid_sub_name}, \
            available keys: {list(rigid_subassembly_occurrence_map[rigid_sub_name].keys())}"
    )

    parent_tf = np.eye(4)
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
        parent_tf = parent_tf @ _occurrence_tf
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
        return np.eye(4)

    # Find the first rigid subassembly in the occurrence path
    rigid_start_idx = None
    for i, occurrence_name in enumerate(occurrences_list):
        if occurrence_name in rigid_subassemblies:
            rigid_start_idx = i
            break

    if rigid_start_idx is None:
        # No rigid subassembly found, shouldn't happen but handle gracefully
        LOGGER.warning(f"No rigid subassembly found in occurrences: {occurrences_list}")
        return np.eye(4)

    # Build transform from root to the rigid subassembly through flexible parents
    flexible_to_rigid_tf = np.eye(4)

    # Apply transforms for flexible parents leading to the rigid subassembly
    for i in range(rigid_start_idx):
        # Build the occurrence key for flexible components
        if subassembly_prefix:
            occurrence_key = f"{subassembly_prefix}_25B5_{occurrences_list[i]}"
        else:
            occurrence_key = occurrences_list[i]

        if occurrence_key in flexible_occurrences:
            flexible_tf = np.matrix(flexible_occurrences[occurrence_key].transform).reshape(4, 4)
            flexible_to_rigid_tf = flexible_to_rigid_tf @ flexible_tf
        else:
            LOGGER.warning(f"Flexible occurrence {occurrence_key} not found in occurrences map")

    # Build transform within the rigid subassembly
    rigid_occurrences_sublist = occurrences_list[rigid_start_idx:]
    rigid_internal_tf = build_hierarchical_transform_for_rigid_subassembly(
        rigid_occurrences_sublist, rigid_subassembly_occurrence_map
    )

    # Combine the transforms: flexible_to_rigid @ rigid_internal
    total_tf = flexible_to_rigid_tf @ rigid_internal_tf

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
        return np.eye(4)

    # Find the first rigid subassembly in the occurrence path
    rigid_start_idx = None
    for i, occurrence_name in enumerate(occurrences_list):
        if occurrence_name in rigid_subassembly_sanitized_to_prefixed:
            rigid_start_idx = i
            break

    if rigid_start_idx is None:
        # No rigid subassembly found, shouldn't happen but handle gracefully
        LOGGER.warning(f"No rigid subassembly found in occurrences: {occurrences_list}")
        return np.eye(4)

    # Build transform from root to the rigid subassembly through flexible parents
    flexible_to_rigid_tf = np.eye(4)

    # Apply transforms for flexible parents leading to the rigid subassembly
    for i in range(rigid_start_idx):
        # Build the occurrence key for flexible components
        if subassembly_prefix:
            occurrence_key = f"{subassembly_prefix}_25B5_{occurrences_list[i]}"
        else:
            occurrence_key = occurrences_list[i]

        if occurrence_key in flexible_occurrences:
            flexible_tf = np.matrix(flexible_occurrences[occurrence_key].transform).reshape(4, 4)
            flexible_to_rigid_tf = flexible_to_rigid_tf @ flexible_tf
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
    total_tf = flexible_to_rigid_tf @ rigid_internal_tf

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
                parts[prefixed_parent_name].rigidAssemblyToPartTF[parent_occurrences[1]] = parent_parentCS.part_tf
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
                parts[prefixed_rigid_name].rigidAssemblyToPartTF[parent_occurrences[-1]] = parent_parentCS.part_tf
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
                parts[prefixed_child_name].rigidAssemblyToPartTF[child_occurrences[1]] = child_parentCS.part_tf
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
                parts[prefixed_rigid_name].rigidAssemblyToPartTF[child_occurrences[-1]] = child_parentCS.part_tf
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
                        child_occurrence_tf = np.matrix(occurrences[full_child_key].transform).reshape(4, 4)
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
        name_counts = {}
        mate_keys_by_name = {}

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
